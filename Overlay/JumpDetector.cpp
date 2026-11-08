// PCH-free on purpose: SolverTests compiles this file standalone.
#include "JumpDetector.h"

#include "PoseMath.h"
#include "../common/MathConstants.h"

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>

// vr:: constants come from whichever OpenVR header Protocol.h resolved
// (openvr.h when it is already loaded, openvr_driver.h otherwise) — both
// define the ones used here.

namespace
{

// Heading of a rotation, or the turn of a delta, as a scalar: what the
// heuristic path's line fit and the candidates' comparisons need. Through
// questcal::YawOnlyRotation, so both paths share one projection. A small
// delta reads small even when its quaternion came with w < 0; headings are
// only ever compared through WrapAngle or the fit's unwrapping.
double YawOf(const Eigen::Quaterniond &q)
{
	return questcal::SignedYawRadians(q);
}

double WrapAngle(double a)
{
	while (a > questcal::Pi) a -= 2.0 * questcal::Pi;
	while (a < -questcal::Pi) a += 2.0 * questcal::Pi;
	return a;
}

Eigen::Quaterniond YawQuat(double yaw)
{
	return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()));
}

// Least-squares line fit over one side of a candidate: position per axis and
// unwrapped heading against time, evaluated at the jump instant. A single
// frame pair would bake one frame of noise into the profile permanently, and
// extrapolating heading via angular velocity is biased when the device is
// pitched (the twist rate is not the world-Y angular velocity component).
struct WindowFit
{
	int n = 0;
	double sumT = 0, sumTT = 0;
	Eigen::Vector3d sumP = Eigen::Vector3d::Zero(), sumTP = Eigen::Vector3d::Zero();
	double sumY = 0, sumTY = 0;
	double prevYaw = 0;

	void Add(double t, const Eigen::Vector3d &pos, double heading, double t0)
	{
		double dt = t - t0;
		double yaw = n == 0 ? heading : prevYaw + WrapAngle(heading - prevYaw);   // unwrap
		prevYaw = yaw;
		sumT += dt; sumTT += dt * dt;
		sumP += pos; sumTP += dt * pos;
		sumY += yaw; sumTY += dt * yaw;
		n++;
	}

	// Evaluate the fitted line relative to the jump instant.
	bool At(Eigen::Vector3d &pos, double &yaw, double dt = 0.0) const
	{
		if (n < 3)
			return false;
		double denom = n * sumTT - sumT * sumT;
		if (std::abs(denom) < 1e-12)
			return false;
		Eigen::Vector3d slopeP = (n * sumTP - sumT * sumP) / denom;
		double slopeY = (n * sumTY - sumT * sumY) / denom;
		pos = (sumP - slopeP * sumT) / n + dt * slopeP;
		yaw = (sumY - slopeY * sumT) / n + dt * slopeY;
		return true;
	}
};

std::string Format(const char *fmt, ...)
{
	char buf[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof buf, fmt, args);
	va_end(args);
	return buf;
}

double MedianOf(std::vector<double> v)
{
	const size_t k = v.size() / 2;
	std::nth_element(v.begin(), v.begin() + k, v.end());
	return v[k];
}

} // namespace

void JumpDetector::Push(const protocol::DevicePoseSample &s)
{
	auto &dev = devices[s.deviceId];
	auto breakObservationContinuity = [&](const std::string &reason)
	{
		// An explicitly observed bad frame is stronger evidence than mere
		// absence. Neither the exact WFD path nor the composed-pose heuristic may
		// bridge it, even when the valid endpoints are less than gapSeconds apart.
		dev.wfdValid = false;
		dev.hist.clear();
		dev.repeatedPositions = 0;
		for (auto &candidate : candidates)
			if (candidate.deviceId == s.deviceId && candidate.kind == Kind::Heuristic)
				Drop(candidate, reason);
	};

	bool valid = s.poseIsValid && s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK);
	if (!valid || !IsUsableRingSample(s, qpcToSeconds))
	{
		// Reasons are only formatted for the detail log: an idle device sends
		// invalid frames continuously.
		breakObservationContinuity(!detailed ? std::string() : valid
			? std::string("a sample with unusable values")
			: Format("a sample not tracking (valid %d, tracking result %u)", s.poseIsValid ? 1 : 0, s.trackingResult));
		return;   // long gaps are still measured from the last valid sample
	}

	double t = RingSampleTime(s, qpcToSeconds);
	if (dev.lastValidTime >= 0.0 && t <= dev.lastValidTime)
	{
		// A composed-time inversion is a rejected observation too. Clear both
		// continuity paths before accepting a later monotonic sample.
		breakObservationContinuity(!detailed ? std::string() :
			Format("a sample %.1f ms out of order", (dev.lastValidTime - t) * 1000.0));
		return;
	}

	// A driver queue drop noted since this device's last sample (NoteStreamHole)
	// cost it a frame only if its own interval skipped. Then it breaks as the
	// old blanket rule broke every device: history and the exact WFD path go,
	// and its candidates still filling a fit window die; Ready ones stay.
	if (dev.holeNoted)
	{
		dev.holeNoted = false;
		if (dev.lastValidTime >= 0.0 && (dev.usualInterval <= 0.0 ||
			t - dev.lastValidTime > config.holeIntervalFactor * dev.usualInterval))
		{
			dev.wfdValid = false;
			dev.hist.clear();
			dev.repeatedPositions = 0;
			for (auto &candidate : candidates)
				if (candidate.deviceId == s.deviceId && candidate.PendingHeuristic())
					Drop(candidate, "a driver queue drop inside its fit windows");
		}
	}
	const double usualBefore = dev.usualInterval;
	if (dev.lastValidTime >= 0.0 && t - dev.lastValidTime < config.maxFrameGap)
	{
		const double interval = t - dev.lastValidTime;
		dev.usualInterval = dev.usualInterval <= 0.0 ? interval
			: dev.usualInterval + 0.05 * (interval - dev.usualInterval);
	}

	// A hard gap in the reference stream (disconnect, standby): the universe
	// may have moved with no observable frame pair. Never compensate across
	// it — reset baselines and report the event for staleness scoring.
	bool resumed = dev.lastValidTime < 0.0;
	if (dev.lastValidTime >= 0.0 && t - dev.lastValidTime > config.gapSeconds)
	{
		gaps.push_back({ s.deviceId, t - dev.lastValidTime, questcal::RingTime(t) });
		// Broken exactly like an observed bad frame, but this sample is still
		// processed.
		breakObservationContinuity(!detailed ? std::string() :
			Format("a %.1f s gap in the stream", t - dev.lastValidTime));
		resumed = true;
	}
	if (resumed)
		dev.streamResumeTime = t;

	RingSampleParts p = UnpackRingSample(s);
	Eigen::Vector3d driverVelocity(s.velocity[0], s.velocity[1], s.velocity[2]);
	Eigen::Vector3d driverAngularVelocity(
		s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]);
	ProbeAngularFrame(s.deviceId, dev, t, p.drvRot, driverAngularVelocity);
	const ringpose::DriverLocalPoseSample world = RingWorldPose(s, p, t);

	Hist h;
	h.t = t;
	h.sample = s.sampleTimeQpc;
	h.pos = world.position;
	h.vel = world.velocity;
	h.yaw = YawOf(world.rotation);
	h.yawRateWorld = (p.wfdRot * driverAngularVelocity).y();
	h.yawRateDevice = (world.rotation * driverAngularVelocity).y();

	bool rebased = false;
	if (dev.wfdValid)
	{
		if (questcal::WorldFromDriverChanged(dev.wfdRot, dev.wfdTrans, p.wfdRot, p.wfdTrans,
			config.wfdRotEpsRad, config.wfdTransEps))
		{
			// A WFD change is exact only when the driver-local pose remained on
			// its predicted trajectory. Some drivers can instead change WFD and
			// inversely re-express the local pose, leaving the composed world pose
			// continuous; compensating that bookkeeping change would create a
			// jump that never happened. Ambiguous changes fall through to the
			// composed-pose heuristic below.
			ringpose::DriverLocalPoseSample previous;
			previous.time = dev.lastValidTime;
			previous.rotation = dev.drvRot;
			previous.position = dev.drvPos;
			previous.velocity = dev.drvVel;
			ringpose::DriverLocalPoseSample current;
			current.time = t;
			current.rotation = p.drvRot;
			current.position = p.drvPos;
			current.velocity = driverVelocity;
			// In the frame the probe judged; until it has, either reading.
			auto continuousAs = [&](ringpose::AngularVelocityFrame frame)
			{
				previous.angularVelocity =
					ringpose::AngularVelocityInDriverWorld(dev.drvAngVel, dev.drvRot, frame);
				current.angularVelocity =
					ringpose::AngularVelocityInDriverWorld(driverAngularVelocity, p.drvRot, frame);
				return ringpose::IsDriverLocalPoseContinuous(
					previous, current, config.maxFrameGap,
					config.localContinuityPos, config.localContinuityRotRad);
			};
			bool localContinuous = dev.angularFrame == ringpose::AngularVelocityFrame::Unknown
				? continuousAs(ringpose::AngularVelocityFrame::DriverWorld) ||
					continuousAs(ringpose::AngularVelocityFrame::Device)
				: continuousAs(dev.angularFrame);

			if (localContinuous)
			{
				DetectWfdRebase(s.deviceId, dev, t, s.sampleTimeQpc, p.wfdRot, p.wfdTrans);
				rebased = true;
			}
			else
			{
				notes.push_back(Format(
					"worldFromDriver changed on device %u with discontinuous local pose; exact compensation ignored",
					s.deviceId));
			}
		}
	}

	if (rebased)
	{
		// The discontinuity is explained exactly; clear composed history so
		// the heuristic path cannot double-fire on the same event.
		dev.hist.clear();
	}
	else if (!dev.hist.empty())
	{
		DetectDiscontinuity(s.deviceId, dev, h, usualBefore);
	}

	if (!dev.hist.empty() && dev.hist.back().pos == h.pos)
		dev.repeatedPositions++;
	else
		dev.repeatedPositions = 0;
	dev.hist.push_back(h);
	while (h.t - dev.hist.front().t > 2.0 * config.window)
		dev.hist.pop_front();

	dev.wfdRot = p.wfdRot;
	dev.wfdTrans = p.wfdTrans;
	dev.drvRot = p.drvRot;
	dev.drvPos = p.drvPos;
	dev.drvVel = driverVelocity;
	dev.drvAngVel = driverAngularVelocity;
	dev.wfdValid = true;
	dev.lastValidTime = t;

	EvaluatePendingCandidates();
	TryAccept();
}

void JumpDetector::DetectWfdRebase(uint32_t id, DeviceState &dev, double t, int64_t sample,
                                   const Eigen::Quaterniond &newRot, const Eigen::Vector3d &newTrans)
{
	// Exact world delta: worlds relate by D = new_wfd ∘ old_wfd⁻¹.
	Eigen::Quaterniond dRot = (newRot * dev.wfdRot.conjugate()).normalized();

	// Constrain to yaw + translation (a recenter preserves gravity), through
	// the projection the chaperone re-anchor path shares: invariants 14/15
	// require the delta applied here and the standing-center re-anchor derived
	// in ChaperoneMath to be the same transform. The discarded swing is the
	// non-rigid residual, reported and never applied.
	double tilt = 0.0;
	Eigen::Quaterniond yawRot = questcal::YawOnlyRotation(dRot, &tilt);
	double yaw = YawOf(yawRot);

	// Compute the translation with the projected rotation so D stays
	// self-consistent with the yaw actually applied.
	Eigen::Vector3d dTrans = newTrans - yawRot * dev.wfdTrans;

	Candidate c;
	c.deviceId = id;
	c.t = t;
	c.sample = sample;
	c.kind = Kind::Exact;
	c.life = Life::Ready;   // the exact path has its delta immediately
	c.rot = yawRot;
	c.trans = dTrans;
	c.residualTiltRad = tilt;
	c.endpoint = ExactEndpoint{ newRot, newTrans, dev.wfdRot, dev.wfdTrans };
	candidates.push_back(c);

	notes.push_back(Format("worldFromDriver rebase on device %u: yaw %+.2f deg, shift %.3f m (tilt residual %.2f deg)",
		id, yaw * 180.0 / questcal::Pi, dTrans.norm(), tilt * 180.0 / questcal::Pi));
}

void JumpDetector::DetectDiscontinuity(uint32_t id, DeviceState &dev, const Hist &incoming,
                                       double usualInterval)
{
	const Hist &prev = dev.hist.back();
	double dt = incoming.t - prev.t;   // > 0: Push rejects out-of-order samples
	if (dt > config.maxFrameGap)
		return;

	double posErr = (incoming.pos - questcal::PredictPosition(prev.pos, prev.vel, incoming.vel, dt)).norm();
	// The yaw rate in the frame the probe judged. Until it has, a step must
	// exceed both readings: a Quest controller held at an angle turns fast
	// enough for the wrong reading alone to make a 2-4 degree "jump".
	const double yawErrWorld = std::abs(WrapAngle(incoming.yaw - prev.yaw - prev.yawRateWorld * dt));
	const double yawErrDevice = std::abs(WrapAngle(incoming.yaw - prev.yaw - prev.yawRateDevice * dt));
	const bool deviceFrame = dev.angularFrame == ringpose::AngularVelocityFrame::Device ||
		(dev.angularFrame == ringpose::AngularVelocityFrame::Unknown && yawErrDevice < yawErrWorld);
	double yawErr = deviceFrame ? yawErrDevice : yawErrWorld;
	const double yawRate = deviceFrame ? incoming.yawRateDevice : incoming.yawRateWorld;

	if (posErr < config.corroboratedPos && yawErr < config.corroboratedYawRad)
		return;

	// One pending candidate per device per window. A later jump inside it is
	// counted for the candidate's line.
	for (auto &c : candidates)
		if (c.deviceId == id && c.life != Life::Dead &&
			std::abs(c.t - incoming.t) < config.window)
		{
			c.jumpsInside++;
			c.largestInsidePos = (std::max)(c.largestInsidePos, posErr);
			c.largestInsideYawRad = (std::max)(c.largestInsideYawRad, yawErr);
			return;
		}

	Candidate c;
	c.deviceId = id;
	c.t = incoming.t;   // first post-jump sample time
	c.sample = incoming.sample;
	c.kind = Kind::Heuristic;
	c.life = Life::Pending;
	c.frameJumpPos = posErr;
	c.frameJumpYawRad = yawErr;
	c.frameInterval = dt;
	c.usualInterval = usualInterval;
	c.needsCorroboration = posErr < config.discontinuityPos && yawErr < config.discontinuityYawRad;
	c.followDeadline = c.t + config.window + config.controllerFollowSeconds;
	c.lastHoldCheck = c.t;
	// Keep only the pre-jump window.
	size_t first = 0;
	while (first < dev.hist.size() && c.t - dev.hist[first].t > config.window)
		++first;
	c.preWindow.assign(dev.hist.begin() + first, dev.hist.end());
	if (detailed)
		c.raisedText = Format("frame jump %.3f m / %.2f deg over %.1f ms, moving %.3f m/s, turning %.3f rad/s%s",
			posErr, yawErr * 180.0 / questcal::Pi, dt * 1000.0, incoming.vel.norm(), std::abs(yawRate),
			c.needsCorroboration ? "; under the solo floor, needs another device or a settled stream" : "");
	// For the log: the other devices whose own frames jumped with this one.
	for (auto &o : candidates)
		if (o.kind == Kind::Heuristic && o.life != Life::Dead && o.deviceId != id &&
			std::abs(o.t - c.t) <= config.agreeWindow)
		{
			if (id == vr::k_unTrackedDeviceIndex_Hmd)
				c.steppedWith.set(o.deviceId);
			else if (o.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
				o.steppedWith.set(id);
		}
	candidates.push_back(c);
}

void JumpDetector::EvaluatePendingCandidates()
{
	for (auto &c : candidates)
	{
		if (!c.PendingHeuristic())
			continue;
		double now = devices[c.deviceId].lastValidTime;
		if (now < c.t + config.window)
			continue;   // post window still filling

		const auto &dev = devices[c.deviceId];

		// Estimate the state at the jump instant from each side (see WindowFit).
		WindowFit pre, post;
		for (const auto &h : c.preWindow)
			pre.Add(h.t, h.pos, h.yaw, c.t);
		for (const auto &h : dev.hist)
		{
			if (h.t < c.t || h.t > c.t + config.window)
				continue;
			post.Add(h.t, h.pos, h.yaw, c.t);
		}

		Eigen::Vector3d prePos, postPos;
		double preYaw, postYaw;
		if (!pre.At(prePos, preYaw) || !post.At(postPos, postYaw))
		{
			Drop(c, Format("too few samples to fit (%d before, %d after)", pre.n, post.n));
			continue;
		}

		// Reject windows containing a transient glitch or a second unresolved
		// step. Their fitted endpoint is not evidence of a persistent rebase.
		struct FitResidual { double pos = 0.0, yaw = 0.0; int count = 0; };
		auto residual = [&](const auto &history, const WindowFit &fit, bool after)
		{
			double posSq = 0.0, yawSq = 0.0;
			FitResidual r;
			for (const auto &h : history)
			{
				if (after && (h.t < c.t || h.t > c.t + config.window))
					continue;
				Eigen::Vector3d predicted;
				double yaw = 0.0;
				fit.At(predicted, yaw, h.t - c.t);   // the fit succeeded above
				posSq += (h.pos - predicted).squaredNorm();
				double yawError = WrapAngle(h.yaw - yaw);
				yawSq += yawError * yawError;
				++r.count;
			}
			// count is the fit's own n, at least 3.
			r.pos = std::sqrt(posSq / r.count);
			r.yaw = std::sqrt(yawSq / r.count);
			return r;
		};
		const FitResidual before = residual(c.preWindow, pre, false), after = residual(dev.hist, post, true);
		auto clean = [&](const FitResidual &r)
		{
			return r.pos <= config.fitPositionRms && r.yaw <= config.fitYawRmsRad;
		};
		const std::string fitText = detailed ? Format(
			"fit rms before %.1f mm / %.3f deg over %d samples, after %.1f mm / %.3f deg over %d (limits %.1f mm / %.3f deg)",
			before.pos * 1000.0, before.yaw * 180.0 / questcal::Pi, before.count,
			after.pos * 1000.0, after.yaw * 180.0 / questcal::Pi, after.count,
			config.fitPositionRms * 1000.0, config.fitYawRmsRad * 180.0 / questcal::Pi) : std::string();
		if (!clean(before) || !clean(after))
		{
			Drop(c, "fit not clean, a transient or a second step: " + fitText);
			continue;
		}
		if (detailed)
		{
			WriteCandidateLine(c, Format(
				"fitted: before (%.4f, %.4f, %.4f) m yaw %.3f deg, after (%.4f, %.4f, %.4f) m yaw %.3f deg, %.3f m from the stream origin horizontally; ",
				prePos.x(), prePos.y(), prePos.z(), preYaw * 180.0 / questcal::Pi,
				postPos.x(), postPos.y(), postPos.z(), postYaw * 180.0 / questcal::Pi,
				std::hypot(prePos.x(), prePos.z())) + fitText);
			// The raw evidence, so the step can be refitted offline.
			std::string samples = Format("samples around the step on device %u (ms from it, x y z m, yaw deg):", c.deviceId);
			auto add = [&](const Hist &h)
			{
				samples += Format(" %+.1f %.4f %.4f %.4f %.3f;", (h.t - c.t) * 1000.0,
					h.pos.x(), h.pos.y(), h.pos.z(), h.yaw * 180.0 / questcal::Pi);
			};
			for (const auto &h : c.preWindow)
				add(h);
			for (const auto &h : dev.hist)
				if (h.t >= c.t && h.t <= c.t + config.window)
					add(h);
			details.push_back(samples);
		}

		double dYaw = WrapAngle(postYaw - preYaw);
		c.rot = YawQuat(dYaw);
		c.trans = postPos - c.rot * prePos;
		c.life = Life::Ready;
		if (AcceptedStep *followed = OwedFollowUp(c))
		{
			followed->owed.reset(c.deviceId);
			Drop(c, Format("the follow-up of the step accepted at %.3f s: its evidence is used", followed->t));
			continue;
		}

		// The shift is the step's translation about the stream origin; how far
		// the device itself moved tells a turn about it (the heading alone
		// stepping) from a turn about a distant point, which the shift alone
		// cannot: both scale with the device's distance from the origin.
		const double moved = (postPos - prePos).norm();
		std::string note = Format("pose discontinuity on device %u: yaw %+.2f deg, shift %.3f m, device moved %.3f m (frame jump %.3f m / %.2f deg)",
			c.deviceId, dYaw * 180.0 / questcal::Pi, c.trans.norm(), moved,
			c.frameJumpPos, c.frameJumpYawRad * 180.0 / questcal::Pi);
		const double resumeAge = ResumeAge(dev, c.t);
		if (resumeAge <= config.recentResumeSeconds)
			note += Format(" (%.1f s after the stream resumed)", resumeAge);

		// Decide the headset-only acceptance once, here; TryAccept only reads
		// the verdict. Refusals are logged to tell a gated step from a small one.
		if (c.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
		{
			// The 3DoF fallback on either side of the step. Before it, the step
			// is the catch-up onto resumed tracking. From it on, the fit after
			// the step reads a position the head has left, so the step it gives
			// is the hold's, not the frame's: entering 3DoF also applies the
			// engine's pending correction at once (weekly sweep, seed 1442538:
			// a 3.7 cm frame step read as 5.9 cm under a head walking at
			// 0.9 m/s, and even the exact step left more error than none).
			std::vector<Hist> postWindow;
			for (const auto &h : dev.hist)
				if (h.t >= c.t && h.t <= c.t + config.window)
					postWindow.push_back(h);
			const bool heldBefore = HeldPositionSignature(c.preWindow);
			const bool heldAfter = !heldBefore && HeldPositionSignature(postWindow);
			c.heldPosition = heldBefore || heldAfter;
			const bool thresholdSized =
				std::abs(c.trans.norm() - config.driftCatchUpPos) <= config.driftCatchUpPosBand ||
				std::abs(std::abs(dYaw) - config.driftCatchUpYawRad) <= config.driftCatchUpYawBandRad;
			// A heading that moved less than the fit can resolve did not turn:
			// that step is a small displacement, ignored and totalled like any
			// other (live 2026-10-24: a -0.08 deg, 8 mm step was called a turn
			// about the head and left out of the ignored total).
			const bool turnAboutHead =
				std::abs(dYaw) >= config.fitYawRmsRad &&
				std::abs(dYaw) <= config.driftCatchUpYawRad + config.driftCatchUpYawBandRad &&
				moved <= config.driftCatchUpHeadShift;
			c.driftCatchUp = (thresholdSized || turnAboutHead) && !driftFollowed;
			c.catchUpShaped = thresholdSized || turnAboutHead;
			bool aboveFloor = c.trans.norm() > config.soloSettledPos ||
				std::abs(dYaw) > config.soloSettledYawRad;
			bool steady = resumeAge >= config.soloSettledSeconds;
			// The frame carrying the step came late: the stream stalled across
			// it (see soloSettled* in Config).
			const bool lateFrame = c.usualInterval > 0.0 &&
				c.frameInterval > config.holeIntervalFactor * c.usualInterval;
			c.settledSolo = aboveFloor && steady && !c.heldPosition && !c.driftCatchUp && !lateFrame;
			// Past the solo floor TryAccept applies it alone, steady or not
			// (bigSolo): the note said "not accepted alone" over a 0.72 m step
			// it then compensated (2026-10-15).
			const bool soloSized = c.trans.norm() > config.soloPosThreshold ||
				std::abs(dYaw) > config.soloYawThresholdRad;
			if (heldBefore)
				note += "; position was held before the step (3DoF), not accepted alone";
			else if (heldAfter)
				note += "; position held from the step on (3DoF), not accepted alone";
			else if ((thresholdSized || turnAboutHead) && driftFollowed)
				note += "; shaped like a drift catch-up, but continuous alignment has followed the drift, so treated as a frame change";
			else if (thresholdSized)
				note += "; the size of the engine's reset threshold (a drift catch-up), not accepted alone";
			else if (turnAboutHead)
				note += "; a turn about the head (a drift catch-up), not accepted alone";
			else if (aboveFloor && !steady && !soloSized)
				note += Format("; stream steady under %.0f s, not accepted alone", config.soloSettledSeconds);
			else if (aboveFloor && lateFrame && !soloSized)
				note += Format("; its frame came late (%.1f ms, usually %.1f ms), not accepted alone",
					c.frameInterval * 1000.0, c.usualInterval * 1000.0);
		}
		notes.push_back(note);
	}
}

bool JumpDetector::HeldPositionSignature(const std::vector<Hist> &window)
{
	for (size_t i = 1; i < window.size(); ++i)
		if (window[i].pos == window[i - 1].pos && window[i].yaw != window[i - 1].yaw)
			return true;
	return false;
}

bool JumpDetector::AwaitsFittedStep(const Candidate &c0, double now) const
{
	return std::any_of(candidates.begin(), candidates.end(), [&](const Candidate &o) {
		return o.deviceId != c0.deviceId && o.PendingHeuristic() &&
			o.t - c0.t >= -config.controllerLeadSeconds && o.t <= c0.followDeadline &&
			now <= o.t + config.window + config.maxFrameGap;
	});
}

JumpDetector::AcceptedStep *JumpDetector::OwedFollowUp(const Candidate &c)
{
	if (c.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
		return nullptr;
	const double yaw = YawOf(c.rot);
	for (auto &a : recentAccepts)
		if (a.owed.test(c.deviceId) && c.t - a.t >= -config.controllerLeadSeconds && c.t <= a.followDeadline &&
			(c.trans - a.translation).norm() < config.agreePosTol &&
			std::abs(WrapAngle(yaw - a.yaw)) < config.agreeYawTolRad)
			return &a;
	return nullptr;
}

void JumpDetector::TryAccept()
{
	// Accepted steps whose follow-ups no longer fall due: every owing device
	// has followed, or the windows have closed with no step of theirs still
	// being fitted inside them.
	const double streamNow = devices[vr::k_unTrackedDeviceIndex_Hmd].lastValidTime;
	recentAccepts.erase(
		std::remove_if(recentAccepts.begin(), recentAccepts.end(),
			[&](const AcceptedStep &a) {
				if (a.owed.none())
					return true;
				if (streamNow <= a.followDeadline + config.window)
					return false;
				return std::none_of(candidates.begin(), candidates.end(), [&](const Candidate &c) {
					return c.PendingHeuristic() && a.owed.test(c.deviceId) && c.t <= a.followDeadline;
				});
			}),
		recentAccepts.end());

	// Exact path: a worldFromDriver rebase on the HMD is authoritative (the
	// HMD defines the reference universe). Agreement from other devices is a
	// sanity check, not a requirement. Evaluate it before heuristic guards:
	// two distinct WFD transitions telescope even when closely spaced, so
	// dropping the second would leave the calibration one rebase behind.
	for (auto &c : candidates)
	{
		if (!c.LiveExact() || c.deviceId != vr::k_unTrackedDeviceIndex_Hmd)
			continue;

		UniverseDelta d;
		d.rotation = c.rot;
		d.translation = c.trans;
		d.time = questcal::RingTime(c.t);
		d.sample = c.sample;
		d.exact = true;
		d.devicesAgreeing = 1;
		d.residualTiltRad = c.residualTiltRad;
		d.worldFromDriverRotation = c.endpoint.rotation;
		d.worldFromDriverTranslation = c.endpoint.translation;
		d.previousWorldFromDriverRotation = c.endpoint.previousRotation;
		d.previousWorldFromDriverTranslation = c.endpoint.previousTranslation;
		d.secondsSinceStreamResume = ResumeAge(devices[c.deviceId], c.t);
		std::bitset<vr::k_unMaxTrackedDeviceCount> counted;
		counted.set(c.deviceId);

		for (const auto &o : candidates)
		{
			if (&o == &c || !o.LiveExact() || std::abs(o.t - c.t) > config.agreeWindow)
				continue;
			double spread = (o.trans - c.trans).norm();
			d.residualSpread = std::max(d.residualSpread, spread);
			if (spread < config.agreePosTol && !counted.test(o.deviceId))
			{
				counted.set(o.deviceId);
				d.devicesAgreeing++;
			}
		}

		accepted.push_back(d);
		lastAcceptTime = c.t;
		candidates.clear();
		return;
	}

	// Heuristic path: windowed multi-device agreement.
	for (auto &c0 : candidates)
	{
		// The HMD anchors agreement: two independently relocalizing
		// controllers cannot move its reference universe.
		if (!c0.ReadyHeuristic() || c0.deviceId != vr::k_unTrackedDeviceIndex_Hmd)
			continue;
		double now = devices[c0.deviceId].lastValidTime;
		// A new event needs an uncontaminated baseline after the previous
		// rebase; elapsed time alone cannot distinguish an echo from a reset.
		if (c0.preWindow.front().t <= lastAcceptTime)
		{
			Drop(c0, "its window before the step reaches back past the last applied step");
			continue;
		}

		// Candidates from one device can coexist across adjacent fit windows.
		// Each device gets one vote and one contribution to the averaged delta.
		std::vector<const Candidate *> agree{ &c0 };
		std::bitset<vr::k_unMaxTrackedDeviceCount> counted;
		counted.set(c0.deviceId);
		double spread = 0.0;
		double lag = 0.0;
		for (const auto &o : candidates)
		{
			if (!o.ReadyHeuristic() || counted.test(o.deviceId))
				continue;
			// Devices agree within agreeWindow either way round. A controller
			// may also follow the HMD until the candidate's follow-up
			// deadline, or lead it by up to the smoother's grace (see
			// controllerLeadSeconds).
			double oLag = o.t - c0.t;
			if (oLag < -config.controllerLeadSeconds || o.t > c0.followDeadline)
				continue;
			double dPos = (o.trans - c0.trans).norm();
			double dYaw = std::abs(WrapAngle(YawOf(o.rot) - YawOf(c0.rot)));
			bool strict = c0.needsCorroboration || o.needsCorroboration;
			if (dPos < (strict ? config.corroboratedPosTol : config.agreePosTol) &&
				dYaw < (strict ? config.corroboratedYawTolRad : config.agreeYawTolRad))
			{
				counted.set(o.deviceId);
				agree.push_back(&o);
				spread = std::max(spread, dPos);
				if (std::abs(oLag) > config.agreeWindow && std::abs(oLag) > std::abs(lag))
					lag = oLag;
			}
		}

		bool solo = ActiveDeviceCount(now) <= 1;
		// A large step after a held position is the catch-up onto resumed
		// tracking (a wake, a dark corner): the frame did not change, the
		// head did. A step into one is measured against a position the head
		// has left. No solo path applies either, whatever its size. Nor does one
		// apply a step of the smoother's reset threshold, the drift
		// catch-up; the 10 deg yaw threshold sits exactly on the solo floor.
		bool bigSolo = solo && !c0.needsCorroboration && !c0.heldPosition && !c0.driftCatchUp &&
			(c0.trans.norm() > config.soloPosThreshold ||
			 std::abs(YawOf(c0.rot)) > config.soloYawThresholdRad);
		// Below the solo floor, a headset-only setup applies the step on the
		// strength of the fit alone (see soloSettled* in Config).
		bool settledSolo = solo && !bigSolo && c0.settledSolo;
		if (settledSolo)
			notes.push_back(Format(
				"pose discontinuity on device %u accepted alone: stream steady for %.0f s and position never held",
				c0.deviceId, ResumeAge(devices[c0.deviceId], c0.t)));

		if (agree.size() >= 2 || bigSolo || settledSolo)
		{
			UniverseDelta d;
			d.time = questcal::RingTime(c0.t);
			d.sample = c0.sample;
			d.exact = false;
			d.devicesAgreeing = static_cast<int>(agree.size());
			d.residualSpread = spread;
			d.secondsSinceStreamResume = ResumeAge(devices[c0.deviceId], c0.t);
			d.confirmationLagSeconds = lag;

			Eigen::Vector3d trans = Eigen::Vector3d::Zero();
			double s = 0.0, cs = 0.0;
			for (const auto *a : agree)
			{
				trans += a->trans;
				double y = YawOf(a->rot);
				s += std::sin(y);
				cs += std::cos(y);
			}
			d.translation = trans / static_cast<double>(agree.size());
			d.rotation = YawQuat(std::atan2(s, cs));

			accepted.push_back(d);
			lastAcceptTime = c0.t;
			if (c0.catchUpShaped && agree.size() >= 2)
				notes.push_back(Format(
					"pose discontinuity on device %u, shaped like a drift catch-up, confirmed by devices that jumped with it: %s",
					c0.deviceId, StepCompanions(c0, now).c_str()));
			// Discard this step's evidence: its headset candidates, each
			// confirming device's steps up to the one it confirmed with (the same
			// event in adjacent fit windows), and the follow-up each device that
			// did not vote owes it, the first of its steps that matches this one,
			// made already or still to come. Every other step stays, whatever its
			// time: it may confirm a later headset step still waiting for its own
			// confirmation. Clearing everything lost the second of two held map
			// switches; clearing by time lost it too when the second switch's
			// confirmation came before the step that confirmed the first. A
			// device whose own step for this one was lost to a spoilt fit still
			// owes it one, so its next matching step inside the windows is taken
			// for that.
			const double stepTime = c0.t;
			const double followDeadline = c0.followDeadline;
			std::array<double, vr::k_unMaxTrackedDeviceCount> usedUntil;
			usedUntil.fill(-std::numeric_limits<double>::infinity());
			for (const auto *a : agree)
				usedUntil[a->deviceId] = std::max(usedUntil[a->deviceId], a->t);
			// Every device seen that took no part owes this step a follow-up,
			// already made or still to come (OwedFollowUp). A step still being
			// fitted is judged once it is: compared now on its unset delta it
			// could pass for a small accepted step.
			AcceptedStep step{ stepTime, followDeadline, d.translation, YawOf(d.rotation), {} };
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
				if (id != vr::k_unTrackedDeviceIndex_Hmd && devices[id].lastValidTime >= 0.0 &&
					usedUntil[id] == -std::numeric_limits<double>::infinity())
					step.owed.set(id);
			recentAccepts.push_back(step);
			// The candidates the step takes with it that never reached an
			// outcome of their own (the fitted ones have their line).
			for (auto &c : candidates)
				if (!c.lineWritten && c.life != Life::Dead &&
					((c.deviceId == vr::k_unTrackedDeviceIndex_Hmd &&
						(c.kind != Kind::Heuristic || c.t <= stepTime)) ||
						c.t <= usedUntil[c.deviceId]))
					WriteCandidateLine(c, "taken into the accepted step");
			candidates.erase(
				std::remove_if(candidates.begin(), candidates.end(),
					[&](const Candidate &c) {
						if (c.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
							return c.kind != Kind::Heuristic || c.t <= stepTime;
						if (c.t <= usedUntil[c.deviceId])
							return true;
						if (!c.ReadyHeuristic() || OwedFollowUp(c) != &recentAccepts.back())
							return false;
						recentAccepts.back().owed.reset(c.deviceId);
						return true;
					}),
				candidates.end());
			return;
		}

		if (now <= c0.t + config.window + config.agreeWindow)
			continue;   // immediate agreement can still arrive
		// While every other reference device is locked still it cannot
		// step, so that time does not count against the follow-up wait.
		if (!solo && OthersLockedStill(now))
		{
			c0.followDeadline += now - c0.lastHoldCheck;
			if (!c0.heldLocked)
			{
				c0.heldLocked = true;
				notes.push_back(Format(
					"pose discontinuity on device %u: the other devices are locked still; waiting for them to move",
					c0.deviceId));
			}
		}
		c0.lastHoldCheck = now;
		c0.awaitingFit = false;
		if (!solo && now <= c0.followDeadline)
		{
			// Other reference devices are tracking and stayed continuous: on a
			// Quest Pro that is what a headset map switch looks like until the
			// controllers follow. Keep the candidate for that follow-up.
			if (!c0.held)
			{
				c0.held = true;
				notes.push_back(Format(
					"pose discontinuity on device %u not confirmed at once; holding up to %.0f s for a controller to follow",
					c0.deviceId, config.controllerFollowSeconds));
			}
			continue;
		}
		// A step made in the wait's last fit window is fitted after the
		// deadline. Expired first, the switch was lost, and the step then
		// confirmed the headset's next step of the same size, each later
		// switch on the evidence of the one before (the jump detector's
		// model, three switches of one size).
		if (!solo && AwaitsFittedStep(c0, now))
		{
			c0.awaitingFit = true;
			continue;
		}
		c0.life = Life::Dead;
		// A refused drift catch-up restored the alignment; it is not drift a
		// recalibration would remove, so it stays out of the ignored total.
		if (c0.driftCatchUp)
		{
			driftCatchUps++;
			notes.push_back(Format(
				"pose discontinuity on device %u expired as a drift catch-up (%d this session); %s",
				c0.deviceId, driftCatchUps, StepCompanions(c0, now).c_str()));
			continue;
		}
		ignoredSteps++;
		ignoredYaw = WrapAngle(ignoredYaw + YawOf(c0.rot));
		ignoredTranslation += c0.trans;
		notes.push_back(Format(
			"unconfirmed pose discontinuity on device %u ignored (%d ignored this session: yaw %+.2f deg, shift %.3f m in total)",
			c0.deviceId, ignoredSteps, ignoredYaw * 180.0 / questcal::Pi, ignoredTranslation.norm()));
	}

	// Prune stale entries. A held HMD candidate lives until its follow-up
	// deadline, or past it while it waits for a step still being fitted; a
	// controller step is kept long enough to confirm an HMD step that trails
	// it by the lead window; everything else has no use past a few seconds.
	auto stale = [&](const Candidate &c) {
		double keep = c.kind == Kind::Heuristic && c.deviceId == vr::k_unTrackedDeviceIndex_Hmd
			? c.followDeadline - c.t
			: std::max(5.0, config.controllerLeadSeconds + config.window + config.agreeWindow);
		return c.life == Life::Dead ||
			(devices[c.deviceId].lastValidTime - c.t > keep && !c.awaitingFit);
	};
	for (auto &c : candidates)
		if (!c.lineWritten && c.life != Life::Dead && stale(c))
			WriteCandidateLine(c, "pruned with no outcome");
	candidates.erase(std::remove_if(candidates.begin(), candidates.end(), stale), candidates.end());
}

std::string JumpDetector::StepCompanions(const Candidate &c, double now) const
{
	const int jumped = static_cast<int>(c.steppedWith.count());
	const int others = (std::max)(0, ActiveDeviceCount(now) - 1);
	return Format("%d other device(s) jumped with it, %d stayed continuous", jumped, (std::max)(0, others - jumped));
}

void JumpDetector::ProbeAngularFrame(uint32_t id, DeviceState &dev, double t,
                                     const Eigen::Quaterniond &rot, const Eigen::Vector3d &rate)
{
	if (dev.angularFrameReported)
		return;
	// The middle of three samples spanning under 50 ms, where the device
	// turns and the rates in its own frame and in the world's differ enough
	// to tell which one the driver reports.
	if (dev.rateSamples >= 2 && t > dev.rateTime[0] && t - dev.rateTime[0] < 0.05)
	{
		const double span = t - dev.rateTime[0];
		const Eigen::AngleAxisd inDevice(dev.rateRot[0].conjugate() * rot), inWorld(rot * dev.rateRot[0].conjugate());
		const Eigen::Vector3d deviceRate = inDevice.axis() * (inDevice.angle() / span);
		const Eigen::Vector3d worldRate = inWorld.axis() * (inWorld.angle() / span);
		if ((deviceRate - worldRate).norm() > 0.2 && dev.rateReported.norm() > 0.3)
		{
			dev.deviceFrameResidual.push_back((dev.rateReported - deviceRate).norm());
			dev.worldFrameResidual.push_back((dev.rateReported - worldRate).norm());
		}
		if (dev.deviceFrameResidual.size() >= 120)
		{
			const double inDeviceFrame = MedianOf(dev.deviceFrameResidual), inWorldFrame = MedianOf(dev.worldFrameResidual);
			dev.angularFrame = inDeviceFrame < 0.5 * inWorldFrame ? ringpose::AngularVelocityFrame::Device :
				inWorldFrame < 0.5 * inDeviceFrame ? ringpose::AngularVelocityFrame::DriverWorld :
				ringpose::AngularVelocityFrame::Unknown;
			const char *frame = dev.angularFrame == ringpose::AngularVelocityFrame::Device ? "the device frame" :
				dev.angularFrame == ringpose::AngularVelocityFrame::DriverWorld ? "the world frame" :
				"neither frame clearly";
			notes.push_back(Format(
				"angular velocity on device %u is reported in %s: over %zu turning samples it differs from the orientation's own rate by %.3f rad/s (median) taken in the device frame, %.3f rad/s in the world frame",
				id, frame, dev.deviceFrameResidual.size(), inDeviceFrame, inWorldFrame));
			dev.angularFrameReported = true;
			std::vector<double>().swap(dev.deviceFrameResidual);
			std::vector<double>().swap(dev.worldFrameResidual);
			return;
		}
	}
	if (dev.rateSamples >= 1)
	{
		dev.rateTime[0] = dev.rateTime[1];
		dev.rateRot[0] = dev.rateRot[1];
	}
	dev.rateTime[1] = t;
	dev.rateRot[1] = rot;
	dev.rateReported = rate;
	dev.rateSamples = (std::min)(dev.rateSamples + 1, 2);
}

int JumpDetector::ActiveDeviceCount(double now) const
{
	int n = 0;
	for (const auto &dev : devices)
		if (dev.lastValidTime > now - 1.0)
			n++;
	return n;
}

bool JumpDetector::OthersLockedStill(double now) const
{
	int others = 0;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		const auto &dev = devices[id];
		if (id == vr::k_unTrackedDeviceIndex_Hmd || dev.lastValidTime <= now - 1.0)
			continue;
		++others;
		if (dev.repeatedPositions < 2)
			return false;
	}
	return others > 0;
}

void JumpDetector::Drop(Candidate &c, const std::string &reason)
{
	if (c.life == Life::Dead)
		return;
	c.life = Life::Dead;
	WriteCandidateLine(c, "dropped: " + reason);
}

void JumpDetector::WriteCandidateLine(Candidate &c, const std::string &outcome)
{
	if (!detailed)
		return;
	std::string line = Format("candidate on device %u at %.3f s", c.deviceId, c.t);
	const bool raisedPart = !c.lineWritten && !c.raisedText.empty();
	const bool insidePart = !c.lineWritten && c.jumpsInside > 0;
	if (raisedPart)
		line += ": " + c.raisedText;
	if (insidePart)
		line += Format("%s%d more frame jump(s) inside it, the largest %.3f m / %.2f deg",
			raisedPart ? "; " : ": ", c.jumpsInside, c.largestInsidePos,
			c.largestInsideYawRad * 180.0 / questcal::Pi);
	line += (raisedPart || insidePart ? "; " : " ") + outcome;
	details.push_back(line);
	c.lineWritten = true;
}

void JumpDetector::NoteStreamHole()
{
	// Judged per device by its next sample (Push): only a device whose own
	// stream skipped lost anything.
	for (auto &dev : devices)
		dev.holeNoted = true;
}

void JumpDetector::Reset()
{
	for (auto &dev : devices)
	{
		dev.wfdValid = false;
		dev.lastValidTime = -1.0;
		dev.streamResumeTime = -1.0;
		dev.repeatedPositions = 0;
		dev.hist.clear();
		dev.holeNoted = false;
	}
	candidates.clear();
	accepted.clear();
	recentAccepts.clear();
	gaps.clear();
	notes.clear();
	details.clear();
	lastAcceptTime = -1e9;
	ignoredSteps = 0;
	ignoredYaw = 0.0;
	ignoredTranslation.setZero();
}

bool JumpDetector::HasLiveHeadsetCandidate(int64_t sample) const
{
	for (const auto &c : candidates)
		if (c.deviceId == vr::k_unTrackedDeviceIndex_Hmd && c.kind == Kind::Heuristic &&
			c.life != Life::Dead && c.sample == sample)
			return true;
	return false;
}
