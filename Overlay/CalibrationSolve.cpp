#include "stdafx.h"
#include "CalibrationInternal.h"
#include "CalibrationDriver.h"
#include "CalibrationEngine.h"
#include "CalibrationSpace.h"
#include "Configuration.h"
#include "PoseMath.h"
#include "QualityBands.h"
#include "../common/MathConstants.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>

// Finishing a run: the solve, how a refused one is explained, and storing
// the result as the calibration or as a field anchor.

using namespace calibration_internal;
using questcal::ReadTrackedDeviceString;
using questcal::SynchronizeCalibrationDriver;

// Reading of a refused solve for the modal: what went wrong and what to
// change, one line each. The engine's own sentence stays in `message` for the
// log and the details toggle.
static StopReason DescribeSolveFailure(const questcal::EngineResult &result)
{
	using questcal::EngineFailure;
	StopReason reason;
	reason.detail = result.message;
	switch (result.failure)
	{
	case EngineFailure::NotEnoughRotation:
		reason.body = "The turns of the devices were too small.";
		reason.action = "Make larger turns. Keep the devices tightly together.";
		break;
	case EngineFailure::SingleAxis:
	case EngineFailure::TranslationUnobservable:
		reason.body = "The devices turned in only one direction.";
		reason.action = "Turn and tilt both devices together in different directions.";
		break;
	case EngineFailure::RotationResidual:
		reason.body = "The rotation values from the two devices were too different.";
		reason.action = "Try again. Hold the devices tightly together.";
		break;
	case EngineFailure::PositionResidual:
		reason.body = "The position values from the two devices were too different.";
		reason.action = "Try again. Move slowly, and keep both devices in clear view.";
		break;
	case EngineFailure::NotEnoughSamples:
		reason.body = "Nova Calibrator received almost no tracking data.";
		reason.action = "Make sure that both devices have tracking. Then try again.";
		break;
	case EngineFailure::InvalidSamples:
		reason.body = "Some tracking data was not correct.";
		reason.action = "Try again. If the problem occurs again, start SteamVR again.";
		break;
	case EngineFailure::ScaleNotIdentifiable:
		reason.body = "Nova Calibrator could not measure the playspace scale from this movement.";
		reason.action = "Move across a larger area when you try again. You can also set \xE2\x80\x9CMeasure "
			"playspace scale\xE2\x80\x9D to off in Settings.";
		break;
	case EngineFailure::NonFinite:
	case EngineFailure::OutOfRange:
	default:
		reason.body = "Nova Calibrator could not use the calibration result.";
		reason.action = "Try again with smooth movement around the room.";
		break;
	}
	return reason;
}

// Store an anchor solve into the field. The anchor keeps the absolute solved
// transform; its delta against the base calibration is derived at send time.
static void StoreFieldAnchor(CalibrationContext &ctx, const questcal::EngineResult &result,
                             const Eigen::Vector3d &targetCentroid)
{
	// Where the fitted data actually lives, in reference space.
	Eigen::Vector3d centroidRef = result.rotation * targetCentroid + result.translation;

	// Magnitude gate: the field corrects small SLAM-map deformation. A large
	// disagreement with the base calibration means the universe moved since
	// the base solve -- recalibrating the base is the honest fix, not a huge
	// local patch.
	double rotDeltaDeg = result.rotation.angularDistance(ctx.transform.rotation) * 180.0 / questcal::Pi;
	Eigen::Vector3d baseHere = ctx.transform.rotation * targetCentroid + ctx.transform.translationMeters;
	double posDeltaM = (centroidRef - baseHere).norm();

	char buf[256];
	if (rotDeltaDeg > 8.0 || posDeltaM > 0.30)
	{
		snprintf(buf, sizeof buf,
			"Anchor rejected: %.1f deg / %.1f cm from the base calibration",
			rotDeltaDeg, posDeltaM * 100.0);
		ctx.Outcome("Anchor not added",
			"This area is too far from your calibration for a small correction.",
			"Do a full calibration instead.", buf, CalibrationContext::Tone::Warn);
		return;
	}

	// Replace an anchor measured near this spot (horizontal distance, matching
	// the driver's XZ blend metric); otherwise append.
	CalibrationContext::FieldAnchor anchor{ centroidRef, result.rotation, result.translation };
	size_t slot = ctx.fieldAnchors.size();
	for (size_t i = 0; i < ctx.fieldAnchors.size(); ++i)
	{
		Eigen::Vector3d d = ctx.fieldAnchors[i].position - centroidRef;
		if (std::sqrt(d.x() * d.x() + d.z() * d.z()) < 1.0)
		{
			slot = i;
			break;
		}
	}

	// Decided before the transaction opens: a refused anchor is not a failed
	// save, so it must not bump the generation or roll anything back.
	bool append = slot == ctx.fieldAnchors.size();
	if (append && ctx.fieldAnchors.size() >= protocol::SetAlignmentField::MaxAnchors)
	{
		// The sentence is a translation key, so it names the limit itself.
		static_assert(protocol::SetAlignmentField::MaxAnchors == 8, "the anchor-limit message names 8");
		ctx.Outcome("Anchor not added", "You already have 8 anchors.",
			"To replace an anchor, go to a position less than 1 m from it. Then add a new anchor. You "
				"can also select \xE2\x80\x9C" "Clear anchors\xE2\x80\x9D in the Field anchors tool.",
			"Anchor limit reached (8)", CalibrationContext::Tone::Warn);
		return;
	}

	if (!SaveProfileFieldEdit(ctx,
		[&](questcal::ProfileRecord &candidate) {
			if (append)
				candidate.fieldAnchors.push_back(PersistedAnchor(anchor));
			else
				candidate.fieldAnchors[slot] = PersistedAnchor(anchor);
		},
		true))
	{
		ctx.Outcome("Anchor not added", "Nova Calibrator could not save the anchor.",
			"Start Nova Calibrator again. Then add the anchor again.",
			"Field anchor was not applied because the updated profile could not be saved",
			CalibrationContext::Tone::Warn);
		return;
	}
	SynchronizeCalibrationDriver(ctx);

	snprintf(buf, sizeof buf, "Field anchor %zu stored at (%.2f, %.2f): %.2f deg / %.1f cm from base\n",
		slot + 1, centroidRef.x(), centroidRef.z(), rotDeltaDeg, posDeltaM * 100.0);
	ctx.Log(buf);
	ctx.Outcome("Anchor added",
		"This area now has its own correction. The correction increases gradually as you go near the area.",
		"", "", CalibrationContext::Tone::Good, CalibrationContext::Event::Anchor,
		"The anchor is added. This area now has its own correction.");
}

void calibration_internal::FinishCalibration(CalibrationContext &ctx)
{
	auto &run = ctx.run;
	bool asAnchor = run.anchor && ctx.validProfile;

	questcal::EngineConfig config;
	// Scale stays global from the base solve; an anchor solve fits R,T on
	// target samples pre-scaled by it, so its absolute transform composes with
	// the driver's scale-then-transform application unchanged.
	config.solveScale = ctx.solveScale && !asAnchor;
	// A latency the correlation cannot measure this time is the last measured
	// one (continuous re-estimation refines it), or zero before any: the
	// residual gates still judge the solve. Both measured on this setup were
	// within 3.5 ms of zero, a third of a degree at a brisk head turn.
	config.fallbackTimeOffset = ctx.validProfile ? ctx.transform.timeOffset : 0.0;
	if (asAnchor && ctx.transform.scale != 1.0)
	{
		for (auto &s : run.targetSamples)
		{
			s.pos *= ctx.transform.scale;
			s.vel *= ctx.transform.scale;
		}
	}

	char buf[512];
	snprintf(buf, sizeof buf, "Collected %zu reference / %zu target samples, solving%s...\n",
		run.referenceSamples.size(), run.targetSamples.size(), asAnchor ? " (field anchor)" : "");
	ctx.Log(buf);

	// A lighthouse device that began a new solution mid-collection gave poses
	// from two solutions that can sit centimeters apart (4.4 deg / 33 cm live
	// on 2026-09-25). The solve would blend them or refuse them, and the
	// player would be told to move differently when the motion was fine.
	const bool referenceRestarted =
		LighthouseRestarts(ctx, run.referenceSerial) != run.referenceRestartsAtStart;
	if (referenceRestarted ||
		LighthouseRestarts(ctx, run.targetSerial) != run.targetRestartsAtStart)
	{
		AbortCalibration(ctx, {
			"The base station tracking of " + (referenceRestarted
				? DeviceName(ctx, run.referenceModel, run.referenceSerial, true)
				: DeviceName(ctx, run.targetModel, run.targetSerial, false)) +
				" started again during the measurement.",
			"Keep it in view of its base stations until the countdown stops.",
			"Lighthouse solution restarted during collection" });
		return;
	}
	if (run.toleratedGaps > 0)
	{
		snprintf(buf, sizeof buf, "raw collection rode through %llu short pose stream gap(s), %llu sample(s) lost",
			static_cast<unsigned long long>(run.toleratedGaps),
			static_cast<unsigned long long>(run.toleratedLoss));
		ctx.Diag(buf);
	}

	questcal::EngineResult result = questcal::CalibrationEngine::Solve(
		run.referenceSamples, run.targetSamples, config);
	if (result.timeOffsetFellBack)
	{
		snprintf(buf, sizeof buf,
			"Time offset not measured (%s; best correlation %.2f, peak margin %.4f); using %s %+.1f ms\n",
			result.timeOffsetFailure.c_str(), result.timeOffsetScore, result.timeOffsetPeakMargin,
			ctx.validProfile ? "the previous calibration's" : "the default",
			config.fallbackTimeOffset * 1000.0);
		ctx.Log(buf);
	}

	// Used only after a valid solve, which had at least 8 target samples.
	Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
	for (const auto &s : run.targetSamples)
		targetCentroid += s.pos;
	targetCentroid /= static_cast<double>(run.targetSamples.size());

	// Mount extrinsic for continuous calibration: a head-referenced base
	// solve's buffers directly measure the tracker's pose in the HMD frame, so
	// this runs before the buffers are cleared. The rigidity gate keeps a
	// hand-held calibration from arming continuous mode; on failure any
	// previously learned mount is kept. The outcome is folded into the run's
	// single outcome below, under the headline.
	struct
	{
		bool attempted = false;
		bool measured = false;
		bool tooFast = false;  // the failure was motion, so the modal shows the slow-down picture
		std::string note;
		std::string action;
	} mount;
	if (result.valid && !asAnchor && run.referenceId == vr::k_unTrackedDeviceIndex_Hmd)
	{
		questcal::MountExtrinsic extrinsic;
		if (questcal::ContinuousAlignment::DeriveMountExtrinsic(
			run.referenceSamples, run.targetSamples, result, extrinsic))
		{
			mount.attempted = true;
			std::string serial;
			if (ReadTrackedDeviceString(run.targetId,
				vr::Prop_SerialNumber_String, serial))
			{
				// The learned transform and physical-device identity are one
				// credential. Commit neither if the checked serial read failed.
				ctx.mountExtrinsic = extrinsic;
				ctx.continuousTrackerSerial = serial;
				snprintf(buf, sizeof buf,
					"Mount offset learned for continuous calibration (%.2f deg / %.1f mm spread, %zu pairs)\n",
					extrinsic.rotRmsDeg, extrinsic.posRmsM * 1000.0, extrinsic.pairs);
				ctx.Log(buf);
				mount.measured = true;
				mount.note = ctx.continuousEnabled
					? "The headset tracker setup is completed. Continuous calibration can now keep your trackers aligned."
					: "The headset tracker setup is completed. To use it, set \xE2\x80\x9C"
						"Continuous calibration\xE2\x80\x9D to on in Settings.";
			}
			else
			{
				mount.note = "Nova Calibrator could not identify the headset tracker. Nova Calibrator keeps the previous setup.";
				mount.action = "Calibrate again with the headset tracker. If the problem occurs again, start SteamVR again.";
			}
		}
		else if (ctx.continuousEnabled || run.targetSerial == ctx.continuousTrackerSerial)
		{
			// Said whenever the pick or the feature says this tracker is meant
			// to be on the headset. A strapped tracker does not move; an
			// inconsistent measurement means the motion was too fast for the
			// two systems' latency.
			mount.attempted = true;
			mount.tooFast = true;
			mount.note = ctx.mountExtrinsic.valid
				? "The measurements of the headset tracker position were too different. Nova Calibrator "
					"keeps the previous setup."
				: "The measurements of the headset tracker position were too different.";
			mount.action = "Calibrate again with the headset tracker. During the calibration, look around more slowly.";
		}
	}

	ctx.state = CalibrationState::None;
	if (!result.valid)
	{
		const StopReason reason = DescribeSolveFailure(result);
		ctx.lastRunPassed = false;
		ctx.Outcome("Calibration not completed", reason.body, reason.action, reason.detail,
			CalibrationContext::Tone::Warn);
		if (EndCalibrationRun(ctx))
			SynchronizeCalibrationDriver(ctx);
		return;
	}

	char offsetEvidence[64];
	if (result.timeOffsetFellBack)
		snprintf(offsetEvidence, sizeof offsetEvidence, "not measured");
	else
		snprintf(offsetEvidence, sizeof offsetEvidence, "correlation %.2f, peak margin %.4f",
			result.timeOffsetScore, result.timeOffsetPeakMargin);
	snprintf(buf, sizeof buf,
		"Rotation residual %.2f deg, position residual %.1f cm\n"
		"Time offset %+.1f ms (%s), axis spread %.3f, %zu pairs (%zu rejected), %zu samples gated\n",
		result.rotationRmsDeg, result.translationRmsMeters * 100.0,
		result.timeOffset * 1000.0, offsetEvidence, result.axisSpread,
		result.pairsUsed, result.pairsRejected, result.samplesGated);
	ctx.Log(buf);

	// Scale-artifact diagnostic: flat gains = genuine metric difference; fine
	// below gross = the reference stream smooths motion and the solved scale
	// is contaminated (the guard then takes scale from the gross band).
	if (result.motionGainValid)
	{
		const char *guardNote = "";
		switch (result.scaleGuard)
		{
		case questcal::ScaleGuard::FromGrossMotion:
			guardNote = " -- fine motion attenuated; scale taken from clean gross motion";
			break;
		case questcal::ScaleGuard::NeutralizedForSmoothing:
			guardNote = !result.scaleIdentifiable
				? " -- motion did not identify scale; held at neutral 1.0"
				: result.motionSmoothingDetected
					? " -- motion attenuation contaminated scale; held at neutral 1.0"
					: " -- frequency bands are physically inconsistent; scale held at neutral 1.0";
			break;
		case questcal::ScaleGuard::NotApplied:
			// The smoothing diagnostic is independent of the guard and fires
			// even with scale solving off, so it only speaks when it is alone.
			if (result.motionSmoothingDetected)
				guardNote = " -- streamed-pose smoothing detected";
			break;
		}
		snprintf(buf, sizeof buf,
			"Motion gain (reference vs target): %.3f gross / %.3f fine%s\n",
			result.motionGainLow, result.motionGainHigh, guardNote);
		ctx.Log(buf);
	}
	if (config.solveScale)
	{
		snprintf(buf, sizeof buf,
			"Scale confidence: %s (condition %.4f, one-sigma %.4f)\n",
			result.scaleIdentifiable ? "identifiable" : "insufficient",
			result.scaleCondition, result.scaleStdDev);
		ctx.Log(buf);
	}

	if (asAnchor)
	{
		// Hand back the devices this run neutralized now, as every other exit
		// does: StoreFieldAnchor resyncs only for an anchor it accepts and saves,
		// and a refused one would otherwise leave them uncalibrated until the
		// next idle scan.
		if (EndCalibrationRun(ctx))
			SynchronizeCalibrationDriver(ctx);
		ctx.lastRunPassed = true;
		StoreFieldAnchor(ctx, result, targetCentroid);
		return;
	}

	// The mount could not be re-measured and the old one stays: when it reads
	// these samples as consistently as a fresh measurement would have to, and
	// close to the solve, the calibration is what it reads, tilt included
	// (ContinuousAlignment::ReadWithMount). The loop would take the yaw and
	// position there anyway, and never the tilt.
	if (mount.tooFast && ctx.mountExtrinsic.valid && run.targetSerial == ctx.continuousTrackerSerial)
	{
		const auto reading = questcal::ContinuousAlignment::ReadWithMount(run.referenceSamples,
			run.targetSamples, ctx.mountExtrinsic, result.rotation, result.translation,
			result.scale, result.timeOffset);
		if (reading.valid)
		{
			snprintf(buf, sizeof buf,
				"Kept headset tracker mount against this solve: yaw %.2f deg, tilt %.2f deg, %.1f cm at the head "
				"(scatter %.2f deg / %.1f cm over %zu readings); %s\n",
				reading.fromSolve.yawDeg, reading.fromSolve.tiltDeg, reading.fromSolve.posM * 100.0,
				reading.scatterRotDeg, reading.scatterPosM * 100.0, reading.observations,
				reading.adopt ? "calibration taken from the mount" : "the solve stands");
			ctx.Log(buf);
			if (reading.adopt)
			{
				result.rotation = reading.rotation;
				result.translation = reading.translation;
			}
		}
	}

	// A live profile keeps its normalized target space. A fresh solve instead
	// follows the target's start frame to the frame in which collection ended.
	if (run.preserveTrackerFrames)
	{
		if (run.normalizationCaptured && !questcal::TrackerFrameCorrections::FollowedRun(
			run.targetNormalizationRotation, run.targetNormalizationTranslation,
			run.targetFrame.toStartRot, run.targetFrame.toStartTrans,
			ctx.trackerFrames.Snapshot()[run.targetId]))
		{
			AbortCalibration(ctx, {
				"The tracking of " + DeviceName(ctx, run.targetModel, run.targetSerial, false) +
					" started again during the measurement.",
				"Wait a few seconds until tracking is stable. Then start again.",
				"The target's frame correction did not follow every frame move of the measurement" });
			return;
		}
		if (!run.normalizationCaptured || !questcal::TrackerFrameCorrections::ExpressCalibration(run.targetNormalizationRotation,
			run.targetNormalizationTranslation, result.rotation, result.translation, result.scale))
		{
			AbortCalibration(ctx, { "Nova Calibrator could not use the calibration result.",
				"Try again with smooth movement around the room.", "Normalized calibration exceeded numeric bounds" });
			return;
		}
		ctx.Log("Recalibration retained the devices' relative lighthouse frame corrections\n");

		// Recalibrating is what players do when trackers look off, so it also
		// realigns every tracker reported against the target's base station:
		// they share SteamVR's geometry, so a correction differing from the
		// target's is a move it missed. Trackers in other stations' frames
		// keep their own.
		if (run.targetFrame.valid)
		{
			int aligned = 0;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				Eigen::Quaterniond frameRotation;
				Eigen::Vector3d frameTranslation;
				if (id != run.targetId && ctx.targetDeviceMask[id] &&
					FrameWatch.LastFrame(id, frameRotation, frameTranslation) &&
					!questcal::WorldFromDriverChanged(frameRotation, frameTranslation,
						run.targetFrame.wfdRot, run.targetFrame.wfdTrans) &&
					ctx.trackerFrames.Align(id, run.targetId))
					++aligned;
			}
			if (aligned > 0)
			{
				snprintf(buf, sizeof buf,
					"Recalibration realigned %d tracker(s) reported against the target's base station\n", aligned);
				ctx.Log(buf);
			}
		}
	}
	else if (run.targetFrame.moves > 0)
	{
		run.targetFrame.CarryCalibration(result.rotation, result.translation, result.scale);
		snprintf(buf, sizeof buf,
			"The target's lighthouse frame moved %d time(s) during the measurement; the calibration was "
			"measured in the frame it began in and carried to the one it ended in\n", run.targetFrame.moves);
		ctx.Log(buf);
	}

	// Only a successful base solve publishes its residuals: the advanced row
	// labels them "Last calibration" and ComputeCalibrationRating caps the
	// rating from them, so a deliberately short, spatially local anchor solve
	// (or a failure, which leaves the old profile live) must not replace them.
	ctx.lastResult = result;

	// Commit the tracking-system identity atomically with the successful base
	// solve. The UI's pending selection must never redirect an old transform.
	ctx.referenceTrackingSystem = run.referenceSystem;
	ctx.targetTrackingSystem = run.targetSystem;
	if (!run.preserveTrackerFrames)
		ctx.ResetTrackerFrames();
	ctx.SetCalibration(result.rotation, result.translation, result.scale);
	ctx.transform.timeOffset = result.timeOffset;
	ctx.validProfile = true;

	// A base recalibration re-measures the whole alignment; anchors solved
	// against the previous alignment would fight it. Start the field fresh.
	if (!ctx.fieldAnchors.empty())
	{
		ctx.fieldAnchors.clear();
		ctx.Log("Field anchors cleared (base recalibration)\n");
	}
	ctx.fieldGeneration++;

	// A fresh solve resets the staleness clock and all accumulated drift
	// evidence, and re-arms the one-shot notifications.
	ctx.calibrationUnixTime = static_cast<double>(std::time(nullptr));
	ctx.ClearDriftEvidence();
	ctx.driftScore = 0.0;
	ctx.alignment = CalibrationContext::AlignmentHealth::Fresh;
	Monitors.ReArmNotifications();
	ctx.lastAutoCorrectionUnixTime = 0.0;
	ctx.autoCorrectionsApplied = 0;
	ctx.continuousReanchors = 0;
	ctx.continuousReanchorsUndone = 0;
	ctx.continuousReanchorsAcrossSolutions = 0;
	DispatchStreamEvent(ctx, questcal::StreamEvent::Recalibrated);

	bool priorUniverseUnsafe = ctx.profileUniverseUnsafe;
	questcal::RebindCalibrationUniverse(
		ctx, run.hmdSerial, priorUniverseUnsafe);
	ctx.persistence.AdvanceRevision();
	ctx.persistence.MarkProfileAndSettings(ctx.timeLastTick);
	bool saved = SavePendingChanges(ctx);
	EndCalibrationRun(ctx);
	SynchronizeCalibrationDriver(ctx);

	if (result.scale != 1.0)
	{
		snprintf(buf, sizeof buf, "Playspace scale: %.4f\n", result.scale);
		ctx.Log(buf);
	}

	// One plain sentence on the quality band the rating uses; the residuals
	// behind it went to the detail lines above.
	ctx.lastRunPassed = true;
	const questcal::SolveQuality band =
		questcal::JudgeSolveQuality(result.rotationRmsDeg, result.translationRmsMeters);
	const bool good = band == questcal::SolveQuality::Good;
	const bool rough = band == questcal::SolveQuality::Poor;
	const std::string quality = good ? "In VR, make sure that your trackers align with your body."
		: rough ? "The trackers are not accurately aligned." : "Another calibration can align the trackers more accurately.";
	const std::string action = good ? ""
		: rough ? "Calibrate again with slower movement. Turn and tilt the devices in different directions."
		: "Examine the tracker positions in VR. If they are not correct, calibrate again.";
	const CalibrationContext::Tone tone = rough ? CalibrationContext::Tone::Warn
		: CalibrationContext::Tone::Good;
	// The feed names the pair it was made with.
	const std::string feed = run.referenceModel.empty() || run.targetModel.empty() ? "Nova Calibrator completed the calibration."
		: "The calibration used " + DeviceName(ctx, run.referenceModel, run.referenceSerial, true) + " and " +
			DeviceName(ctx, run.targetModel, run.targetSerial, false) + ".";
	const auto calibrated = CalibrationContext::Event::Calibrated;
	if (mount.attempted)
	{
		// A headset-tracker run is judged on what it was for.
		if (mount.measured)
			ctx.Outcome("Calibration completed", quality + " " + mount.note, action, "", tone, calibrated, feed);
		else
		{
			if (mount.tooFast)
				ctx.lastRunPassed = false;
			ctx.Outcome("Calibration completed, but the headset tracker setup did not change",
				quality + " " + mount.note, mount.action, "", CalibrationContext::Tone::Warn, calibrated);
		}
	}
	else
		ctx.Outcome("Calibration completed", quality, action, "", tone, calibrated, feed);
	if (!saved)
		ctx.Tell("Nova Calibrator uses the calibration for this session, but it could not save the "
			"calibration. After you start Nova Calibrator or SteamVR again, calibrate again.",
			CalibrationContext::Tone::Warn);
}
