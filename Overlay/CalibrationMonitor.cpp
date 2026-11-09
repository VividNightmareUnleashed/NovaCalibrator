#include "stdafx.h"
#include "CalibrationInternal.h"
#include "CalibrationDriver.h"
#include "CalibrationSpace.h"
#include "LighthouseLog.h"
#include "PoseMath.h"
#include "QualityBands.h"
#include "RingPoseMath.h"
#include "TrackingStreamDigest.h"
#include "../common/MathConstants.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

// The monitors that run beside the calibration every tick: drift staleness
// (detected and notified, never corrected), base station visibility from
// SteamVR's lighthouse log, and the lighthouse frame moves the target
// trackers' corrections follow.

using namespace calibration_internal;
using questcal::ReadTrackedDeviceString;
using questcal::SynchronizeCalibrationDriver;

// Base station visibility from SteamVR's lighthouse log (LighthouseLog.h):
// followed at 4 Hz, folded into ctx.lighthouse, joined to the drift
// monitor's device ids through this serial table.
static std::unique_ptr<lighthouselog::Tailer> LighthouseTail;
static std::string DeviceSerials[vr::k_unMaxTrackedDeviceCount];
// Read in the same scan; the drift feed keeps base stations out by it.
static vr::ETrackedDeviceClass DeviceClasses[vr::k_unMaxTrackedDeviceCount] = {};
// Also from the scan: another driver's virtual trackers
// (IsVirtualTrackerSystem), which have no base station frame to watch.
static bool VirtualDevices[vr::k_unMaxTrackedDeviceCount] = {};
// Also from the scan: whose tracking system a device is in, which decides
// whether its frame moves are the target side's before the driver
// synchronization marks it a target device.
static std::string DeviceTrackingSystems[vr::k_unMaxTrackedDeviceCount];
static bool DeviceTrackingSystemKnown[vr::k_unMaxTrackedDeviceCount] = {};
static double LastLighthousePoll = -1e9;
static double LastSerialScan = -1e9;
static uint64_t LighthouseRotationsSeen = 0;
static bool LighthouseAnnounced = false;
static VisibilityDigest LighthouseDigest;
static TrackingStreamDigest StreamDigest;
static std::vector<protocol::DevicePoseSample> MonitorScratch;
static bool MonitorActive = false;
static std::string FrameObservedSerials[vr::k_unMaxTrackedDeviceCount];
static uint64_t FrameEpoch = 0;
static uint64_t FrameStreamBoundary = 0;

// A new driver session (the ring's writer restarted or its epoch moved): its
// tracker frame corrections start over, and the driver is asked for any it
// kept for the profile.
static void StartFrameSession(CalibrationContext &ctx, uint64_t boundary)
{
	FrameStreamBoundary = boundary;
	// Kept for a recovery from the same driver session (ApplyCompletion),
	// unless a recovery is already waiting: the corrections are reset then.
	if (!ctx.frameRecoveryPending)
		ctx.framesBeforeSession = ctx.trackerFrames.Keep(ctx.frameDriverSession, questcal::FrameProfileKey(
			ctx.referenceTrackingSystem, ctx.targetTrackingSystem, ctx.profileHmdSerial, ctx.calibrationUnixTime));
	ctx.frameDriverSession = 0;
	ctx.frameRecoveryPending = true;
	ctx.ResetTrackerFrames();
	questcal::SynchronizeCalibrationDriver(ctx);
}

// ---------------------------------------------------------------------------
// Runtime monitoring: drift staleness (detect + notify only, never corrects)

// Supplied by the app shell (see Calibration.h); absent under -uipreview.
std::function<void(const char *)> calibration_internal::ToastSink;

void SetToastSink(std::function<void(const char *)> sink)
{
	ToastSink = std::move(sink);
}

// One-shot player-facing line + optional VR toast; each caller owns its
// re-arm flag. The line goes through Tell so it reaches the main screen's
// activity feed as well as the log.
void calibration_internal::NotifyOnce(CalibrationContext &ctx, bool &notified, const char *line,
	CalibrationContext::Tone tone, const char *toast, bool showToast, CalibrationContext::Event event)
{
	if (notified)
		return;
	notified = true;

	// Always logged, even with toasts off or no sink.
	ctx.Tell(std::string(line) + "\n", tone, event);

	if (showToast && ToastSink)
		ToastSink(toast);
}

static void NotifyStaleAlignment(CalibrationContext &ctx)
{
	NotifyOnce(ctx, Monitors.staleNotified,
		"Your calibration is possibly not correct. Calibrate again.",
		CalibrationContext::Tone::Warn,
		"Nova Calibrator: your calibration is possibly not correct. Calibrate again.",
		ctx.notifyPoorCalibration);
}

static void UpdateDriftScore(CalibrationContext &ctx)
{
	// Age alone can only ever say "aging": it saturates at 0.5 after 24 h,
	// below the stale threshold. Observed evidence is what crosses the line.
	// A continuously maintained calibration is not aging: each auto-applied
	// correction re-measures the alignment, so age counts from the later of
	// the manual solve and the last correction.
	double ageScore = 0.0;
	double ageBase = std::max(ctx.calibrationUnixTime, ctx.lastAutoCorrectionUnixTime);
	if (ageBase > 0.0)
	{
		double ageHours = (static_cast<double>(std::time(nullptr)) - ageBase) / 3600.0;
		ageScore = std::min(std::max(ageHours, 0.0) / 24.0, 1.0) * 0.5;
	}

	// A large slide dominates; repeated small events accumulate. Loss glitches
	// are weaker evidence than observed slides.
	double slideScore = std::min(ctx.driftMaxSlideM / 0.04, 1.0);
	double repeatScore = std::min((ctx.driftSlideEvents + 0.5 * ctx.discontinuousLossEvents) / 8.0, 1.0);
	double evidence = std::max(slideScore, repeatScore);

	// Soft-OR: independent evidence compounds without exceeding 1.
	ctx.driftScore = 1.0 - (1.0 - ageScore) * (1.0 - evidence);

	// One slide window is evidence rather than a verdict. Cap the combined score
	// until another event corroborates it; persistent drift quickly lifts the cap.
	if (ctx.driftSlideEvents + ctx.discontinuousLossEvents <= 1)
		ctx.driftScore = std::min(ctx.driftScore, 0.5);

	ctx.alignment =
		ctx.driftScore >= questcal::DriftStaleScore ? CalibrationContext::AlignmentHealth::Stale :
		ctx.driftScore >= questcal::DriftAgingScore ? CalibrationContext::AlignmentHealth::Aging :
		CalibrationContext::AlignmentHealth::Fresh;

	// A healthy continuous loop re-measures the alignment constantly; the
	// rating already skips staleness for it, and toasting "quality looks poor"
	// while it is visibly being maintained is pure noise.
	bool maintained = ctx.ContinuousArmed() &&
		ctx.continuousState == questcal::ContinuousAlignment::State::Tracking;
	if (ctx.alignment == CalibrationContext::AlignmentHealth::Stale && !maintained)
		NotifyStaleAlignment(ctx);
}

// Follows the lighthouse driver's log and keeps the serial table the drift
// monitor needs to join its events onto it. The log is a convenience: when
// it is missing or unreadable everything downstream behaves as it did
// without it.
void calibration_internal::LighthouseTick(CalibrationContext &ctx, double time)
{
	if (!LighthouseTail)
	{
		LighthouseTail = std::make_unique<lighthouselog::Tailer>(lighthouselog::DefaultLogPath());
		ctx.lighthouseLogPath = LighthouseTail->Path();
	}

	// Serials are what the log names devices by. Property reads are
	// cross-process calls, so a slow scan; the set of devices changes rarely.
	if (time - LastSerialScan >= 2.0)
	{
		LastSerialScan = time;
		auto *system = vr::VRSystem();
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			DeviceClasses[id] = system->GetTrackedDeviceClass(id);
			if (DeviceClasses[id] == vr::TrackedDeviceClass_Invalid)
			{
				DeviceSerials[id].clear();
				VirtualDevices[id] = false;
				DeviceTrackingSystemKnown[id] = false;
				continue;
			}
			std::string trackingSystem;
			DeviceTrackingSystemKnown[id] = ReadTrackedDeviceString(id, vr::Prop_TrackingSystemName_String, trackingSystem);
			DeviceTrackingSystems[id] = trackingSystem;
			const bool virtualDevice = DeviceTrackingSystemKnown[id] && questcal::IsVirtualTrackerSystem(trackingSystem);
			if (virtualDevice && !VirtualDevices[id])
				FrameWatch.ForgetDevice(id);
			VirtualDevices[id] = virtualDevice;
			std::string serial;
			if (ReadTrackedDeviceString(id, vr::Prop_SerialNumber_String, serial))
			{
				DeviceSerials[id] = serial;
				if (ctx.trackerFrames.Bind(id, serial))
				{
					FrameWatch.ForgetDevice(id);
					FrameObservedThrough[id] = 0;
					SynchronizeCalibrationDriver(ctx);
				}
			}
		}
	}

	if (time - LastLighthousePoll < 0.25)
		return;
	LastLighthousePoll = time;

	static std::vector<lighthouselog::Event> events;
	events.clear();
	const bool wasAvailable = ctx.lighthouseLogAvailable;
	LighthouseTail->Poll(events);
	ctx.lighthouseLogAvailable = LighthouseTail->Available();
	if (!LighthouseAnnounced || ctx.lighthouseLogAvailable != wasAvailable)
	{
		LighthouseAnnounced = true;
		ctx.Log(ctx.lighthouseLogAvailable
			? "Reading base station visibility from " + ctx.lighthouseLogPath + "\n"
			: "SteamVR's log is not readable at " + ctx.lighthouseLogPath +
				"; base station visibility is unavailable\n");
	}
	// A rotated log is a new SteamVR session: the old sets and counts are
	// about devices that were since restarted.
	if (LighthouseTail->Rotations() != LighthouseRotationsSeen)
	{
		LighthouseRotationsSeen = LighthouseTail->Rotations();
		ctx.lighthouse.Reset();
		LighthouseDigest.Reset();
	}
	const std::string digest = LighthouseDigest.Flush(time);
	if (!digest.empty())
		ctx.Diag(digest);
	if (events.empty())
		return;

	// The lines carry wall-clock stamps; the monitors run on the ring clock.
	const double ringNow = QpcNowSeconds();
	const double unixNow = std::chrono::duration<double>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	for (const auto &e : events)
	{
		double ringTime = ringNow;
		if (e.timeKnown)
		{
			const double age = unixNow - e.unixTime;
			if (age > -5.0 && age < 3600.0)
				ringTime = ringNow - age;
		}
		std::string note = ctx.lighthouse.Apply(e, ringTime);
		if (e.kind == lighthouselog::Event::Kind::ServerStarted)
			LighthouseDigest.Reset();
		if (!e.historical && (e.kind == lighthouselog::Event::Kind::UniverseChosen ||
			e.kind == lighthouselog::Event::Kind::UniverseStopped))
		{
			const char *what = e.kind == lighthouselog::Event::Kind::UniverseStopped ? "stopped tracking with"
				: e.universeCreated ? "created" : "chose";
			ctx.Log("SteamVR " + std::string(what) + " lighthouse universe " + std::to_string(e.universeId) + "\n");
		}
		if (!note.empty() && e.serial.empty())
		{
			// SteamVR's own station moves and universe tilt: a frame move
			// logged after one is that re-solve. A station refined in place
			// (under the frame watch's bounds) goes to the detailed log.
			if (e.kind == lighthouselog::Event::Kind::StationMoved &&
				!LighthouseFrameWatch::NotableMove(e.moveMeters, e.moveDegrees, 0.0))
				ctx.Diag(note);
			else
				ctx.Log(note + "\n");
		}
		else if (!note.empty())
			ctx.Log(e.serial + " " + note + "\n");
		else if (!e.historical)
		{
			// The device's count, not the line's: a SECONDARY line names one
			// station and the set it joined is the model's.
			const LighthouseVisibility::Device *seen = ctx.lighthouse.Find(e.serial);
			if (seen && seen->visibleKnown)
				LighthouseDigest.Note(e.serial, seen->InView());
		}
	}
}

// A frame move's yaw and tilt in degrees, and whether it is worth a session
// log line by the frame watch's own bounds (smaller is a station refined in
// place).
static bool DescribeFrameMove(const LighthouseFrameWatch::Move &move, double &yawDeg, double &tiltDeg)
{
	double tilt = 0.0;
	yawDeg = std::abs(questcal::SignedYawRadians(move.rotation, &tilt)) * 180.0 / questcal::Pi;
	tiltDeg = tilt * 180.0 / questcal::Pi;
	return LighthouseFrameWatch::NotableMove(move.shiftM, yawDeg, tiltDeg);
}

// While SteamVR sets up its universe (LighthouseVisibility.h) a frame move
// places a station it guessed at startup: the calibration belongs to the
// universe, not to the guess, and is left where it is (live 2026-09-27: the
// calibration followed one such move 30.5 deg and 63 cm off). With no word from
// the log, the first seconds after the lighthouse devices start tracking
// stand in for it: the universe was chosen 1.1 and 4.6 s after the first
// new solution on 2026-09-26 and -27, and its stations placed within 5 s.
static constexpr double UniverseSetupFallbackSeconds = 15.0;

bool calibration_internal::DuringUniverseSetup(const CalibrationContext &ctx, double moveTime)
{
	switch (ctx.lighthouse.Universe(moveTime))
	{
	case LighthouseVisibility::UniverseSetup::SettingUp:
		return true;
	case LighthouseVisibility::UniverseSetup::Settled:
		return false;
	case LighthouseVisibility::UniverseSetup::Unknown:
		break;
	}
	return moveTime - FrameWatch.TrackingSince() < UniverseSetupFallbackSeconds;
}

// A base station's id from its device's serial ("LHB-04D47FB4"); 0 for any
// other serial.
static uint32_t StationIdOfSerial(const std::string &serial)
{
	if (serial.size() != 12 || serial.compare(0, 4, "LHB-") != 0)
		return 0;
	uint32_t id = 0;
	for (size_t i = 4; i < serial.size(); ++i)
	{
		const char c = serial[i];
		const int digit = c >= '0' && c <= '9' ? c - '0'
			: c >= 'A' && c <= 'F' ? c - 'A' + 10
			: c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
		if (digit < 0)
			return 0;
		id = (id << 4) | static_cast<uint32_t>(digit);
	}
	return id;
}

// SteamVR re-placing a station it found bumped corrects that station's pose
// (LighthouseVisibility::ReplacingBumped). Compensated as a frame move, as
// Nova did, the trackers in its frame stayed where the knock put them: on
// 2026-11-06 S-16's re-placement, 12.8 deg and 39 cm, split them from the
// other frames for the rest of the session, and each recalibration kept the
// split. A frame change is that station's when the station's own device is
// in the frame before or after it (one frame per station, at its pose; two
// stations' frames are metres apart, so the match can be loose).
static bool SameStationFrame(const Eigen::Quaterniond &aRot, const Eigen::Vector3d &aTrans,
	const Eigen::Quaterniond &bRot, const Eigen::Vector3d &bTrans)
{
	return aRot.angularDistance(bRot) < 3.0 * questcal::Pi / 180.0 && (aTrans - bTrans).norm() < 0.10;
}

// The id of the bumped station a frame change re-places, or 0.
static uint32_t BumpedStationReplacedBy(const CalibrationContext &ctx, double time,
	const Eigen::Quaterniond &fromRot, const Eigen::Vector3d &fromTrans,
	const Eigen::Quaterniond &toRot, const Eigen::Vector3d &toTrans)
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		const uint32_t station = StationIdOfSerial(DeviceSerials[id]);
		Eigen::Quaterniond rotation;
		Eigen::Vector3d translation;
		if (station != 0 && ctx.lighthouse.ReplacingBumped(station, time) &&
			FrameWatch.StationFrame(id, rotation, translation) &&
			(SameStationFrame(rotation, translation, fromRot, fromTrans) ||
				SameStationFrame(rotation, translation, toRot, toTrans)))
			return station;
	}
	return 0;
}

static bool ReplacesBumpedStation(const CalibrationContext &ctx, double time,
	const Eigen::Quaterniond &fromRot, const Eigen::Vector3d &fromTrans,
	const Eigen::Quaterniond &toRot, const Eigen::Vector3d &toTrans)
{
	return BumpedStationReplacedBy(ctx, time, fromRot, fromTrans, toRot, toTrans) != 0;
}

// A calibration completed with `targetId` as its target. When that device is
// in the frame of a station SteamVR found knocked and has not corrected yet,
// the calibration was measured on the poses the knock made
// (LighthouseVisibility::NoteCalibrationWhileBumped), and the correction is
// carried into it when it comes (CarryBumpCorrection).
void calibration_internal::NoteCalibrationForBumpedStations(CalibrationContext &ctx, uint32_t targetId)
{
	Eigen::Quaterniond targetRot;
	Eigen::Vector3d targetTrans;
	if (!FrameWatch.LastFrame(targetId, targetRot, targetTrans))
		return;
	const double ringNow = QpcNowSeconds();
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		const uint32_t station = StationIdOfSerial(DeviceSerials[id]);
		Eigen::Quaterniond rotation;
		Eigen::Vector3d translation;
		if (station != 0 && FrameWatch.StationFrame(id, rotation, translation) &&
			SameStationFrame(rotation, translation, targetRot, targetTrans) &&
			ctx.lighthouse.NoteCalibrationWhileBumped(station, ringNow))
			ctx.Log("The calibration was measured while base station " + ctx.lighthouse.StationNameById(station) +
				" waits for SteamVR's correction after a knock; the correction will be carried into it\n");
	}
}

// SteamVR's correction of a knocked station, left alone, puts the trackers in
// its frame back where they were before the knock. A calibration measured
// between the knock and the correction took the knocked poses for true, so
// the correction would move every tracker in the frame off by the knock (on
// 2026-11-06: calibrated 46 s after the first knock, corrected 15.6 min after
// it, 12 to 17 cm). The calibration takes the correction instead: the frame's
// trackers stay where it put them, and the other frames' trackers, which it
// had put off by the knock, come right. Once for a re-placement; the frame's
// other devices show the same move.
static void CarryBumpCorrection(CalibrationContext &ctx, const LighthouseFrameWatch::Move &move)
{
	const uint32_t station = BumpedStationReplacedBy(ctx, move.frameTime,
		move.fromRot, move.fromTrans, move.toRot, move.toTrans);
	if (station == 0 || !ctx.lighthouse.BumpCalibrated(station))
		return;
	ctx.lighthouse.BumpCarried(station);
	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;
	questcal::TrackerFrameCorrections::MoveInNormalizedSpace(ctx.trackerFrames.Snapshot()[move.id],
		move.rotation, move.translation, rotation, translation);
	const std::string name = ctx.lighthouse.StationNameById(station);
	if (!questcal::CarryTargetSpaceMove(ctx, rotation, translation, ctx.timeLastTick))
	{
		ctx.Log("SteamVR's correction of knocked base station " + name +
			" could not be carried into the calibration measured before it; calibrate again\n");
		return;
	}
	char line[256];
	snprintf(line, sizeof line,
		"SteamVR's correction of knocked base station %s was carried into the calibration measured before it: "
		"%.2f deg, %.1f cm at device %u\n",
		name.c_str(), rotation.angularDistance(Eigen::Quaterniond::Identity()) * 180.0 / questcal::Pi,
		move.shiftM * 100.0, move.id);
	ctx.Log(line);
	++ctx.bumpedStationCorrectionsCarried;
}

bool calibration_internal::FrameChangeLeftAlone(const CalibrationContext &ctx, double time,
	const Eigen::Quaterniond &fromRot, const Eigen::Vector3d &fromTrans,
	const Eigen::Quaterniond &toRot, const Eigen::Vector3d &toTrans)
{
	return DuringUniverseSetup(ctx, time) ||
		ReplacesBumpedStation(ctx, time, fromRot, fromTrans, toRot, toTrans);
}

// Whether a device's frame changes are the target side's. The driver
// synchronization marks a device a target one about a second after it
// appears, and its first samples come before that (live 2026-10-15: a
// controller's first-seen switch was dropped and it kept a correction 8 deg
// off the rest of its frame). Until the mark comes, a device whose tracking
// system is the target's, or not read yet, is waited for: its moves and
// switches are given back to the watch (LighthouseFrameWatch::Hold), and
// acted on once it is marked. One of another tracking system is not ours.
// The wait is bounded so a device that is never marked (one whose properties
// stay unreadable) cannot fill the move queue; what it held is then dropped
// with a line in the log.
using FrameMembership = questcal::TrackerFrameCorrections::Membership;
static constexpr double FrameMembershipWaitSeconds = 120.0;

static FrameMembership TargetMembership(const CalibrationContext &ctx, uint32_t id, double changeTime)
{
	if (id >= vr::k_unMaxTrackedDeviceCount)
		return FrameMembership::Other;
	if (ctx.targetDeviceMask[id])
		return FrameMembership::Target;
	if (DeviceTrackingSystemKnown[id] && DeviceTrackingSystems[id] != ctx.targetTrackingSystem)
		return FrameMembership::Other;
	return FrameWatch.LatestTime() - changeTime < FrameMembershipWaitSeconds
		? FrameMembership::Waiting : FrameMembership::Other;
}

static const char *SwitchCause(LighthouseFrameWatch::Switch::Cause cause)
{
	switch (cause)
	{
	case LighthouseFrameWatch::Switch::Cause::Appeared: return "first seen";
	case LighthouseFrameWatch::Switch::Cause::ReExpressed: return "re-expressed";
	case LighthouseFrameWatch::Switch::Cause::OwnJump: return "its own pose jumped too, no move leads there";
	case LighthouseFrameWatch::Switch::Cause::Returned: return "back from a tracking loss, no move leads there";
	}
	return "";
}

// How far a change of a device's correction moved it, at where it last was.
static void CorrectionChange(uint32_t id, const protocol::FrameCorrection &before,
	const protocol::FrameCorrection &after, double &shiftM, double &angleDeg)
{
	Eigen::Vector3d position;
	shiftM = 0.0;
	if (FrameWatch.LastPosition(id, position))
		shiftM = (questcal::TrackerFrameCorrections::Apply(after, position) -
			questcal::TrackerFrameCorrections::Apply(before, position)).norm();
	angleDeg = Eigen::Quaterniond(after.rotation.w, after.rotation.x, after.rotation.y, after.rotation.z)
		.angularDistance(Eigen::Quaterniond(before.rotation.w, before.rotation.x, before.rotation.y, before.rotation.z)) *
		180.0 / questcal::Pi;
}

static void LogFrameLine(CalibrationContext &ctx, const char *line, bool notable)
{
	if (notable && ctx.detailedLogging && !FrameCapturePending)
	{
		FrameCapturePending = true;
		FrameCaptureDue = (std::max)(ctx.timeLastTick + 0.15, LastFrameCapture + 2.0);
	}
	if (notable) ctx.Log(std::string(line) + "\n");
	else ctx.Diag(line);
}

// The shared calibration stays in one target space, and each lighthouse frame
// has one correction that the target devices in it take
// (TrackerFrameCorrections::Update): a device that showed a move takes the
// moved frame's, N_i' = N_i o F_i^-1 when that frame is derived from the one
// it left, and one that came into a frame with no move takes that frame's.
// The watch also handles hidden trackers and returns after tracking loss.
static void UpdateTrackerFrames(CalibrationContext &ctx)
{
	const bool moveOverflow = FrameWatch.TakeMoveOverflow();
	// A dropped switch leaves a device with the correction of a frame it is
	// no longer in, so it costs the calibration as a dropped move does.
	const bool switchOverflow = FrameWatch.TakeSwitchOverflow();
	auto moves = FrameWatch.TakeMoves();
	auto switches = FrameWatch.TakeSwitches();
	if (moveOverflow || switchOverflow)
	{
		EndCalibrationRun(ctx);
		ctx.enabled = false;
		ctx.profileUniverseUnsafe = true;
		ctx.frameMovesLost = true;
		ctx.persistence.MarkProfile(ctx.timeLastTick);
		ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::StreamGap);
		ctx.Log(moveOverflow
			? "Lighthouse frame move queue overflowed; calibration disabled until a new measurement\n"
			: "Lighthouse frame switch queue overflowed; calibration disabled until a new measurement\n");
		ctx.ReportError("SteamVR moved the base stations more often than Nova Calibrator could follow. The "
			"calibration is off until you calibrate again.\n");
		SynchronizeCalibrationDriver(ctx);
		return;
	}
	LighthouseFrameWatch::SortByTime(moves);
	const auto admitted = questcal::TrackerFrameCorrections::Admit(FrameWatch, moves, switches,
		[&ctx](uint32_t id, double time) { return TargetMembership(ctx, id, time); });
	for (const auto &move : admitted.waitingMoves)
		if (!move.held)
			ctx.Diag("lighthouse device " + std::to_string(move.id) + " frame moved before it is known "
				"to be a target device; the move waits until it is");
	for (const auto &move : admitted.droppedMoves)
		ctx.Log("Lighthouse: dropped a frame move held for device " + std::to_string(move.id) +
			": it did not become a target device\n");
	for (const auto &change : admitted.waitingSwitches)
		if (!change.held)
			ctx.Diag("lighthouse device " + std::to_string(change.id) + " " + DeviceSerials[change.id] +
				" changed frame (" + SwitchCause(change.cause) + ") before it is known to be a target "
				"device; the change waits until it is");
	for (const auto &change : admitted.droppedSwitches)
		ctx.Log("Lighthouse: dropped a frame change held for device " + std::to_string(change.id) +
			": it did not become a target device\n");
	const auto &targetMoves = admitted.moves;
	const auto &targetSwitches = admitted.switches;

	// A measurement keeps its target's correction as the run began with it but
	// for the moves it shows: the run checks at its end that the correction
	// followed its frame moves only.
	const uint32_t measured = ctx.state != CalibrationState::None ? ctx.run.targetId : vr::k_unTrackedDeviceIndexInvalid;
	auto calibrated = [&ctx](uint32_t id) { return id < vr::k_unMaxTrackedDeviceCount && ctx.targetDeviceMask[id]; };
	const size_t disagreements = ctx.trackerFrames.Disagreements();
	const size_t unsettled = ctx.trackerFrames.Unsettled();
	auto bumpCorrection = [&ctx](const LighthouseFrameWatch::Move &move)
	{
		return ReplacesBumpedStation(ctx, move.frameTime, move.fromRot, move.fromTrans, move.toRot, move.toTrans);
	};
	const auto result = ctx.trackerFrames.Update(FrameWatch, targetMoves, targetSwitches, measured, calibrated,
		[&ctx](double time, const Eigen::Quaterniond &fromRot, const Eigen::Vector3d &fromTrans,
			const Eigen::Quaterniond &toRot, const Eigen::Vector3d &toTrans)
		{
			return FrameChangeLeftAlone(ctx, time, fromRot, fromTrans, toRot, toTrans);
		});
	using How = questcal::TrackerFrameCorrections::Changed::How;
	auto changeOf = [&result](uint32_t id) -> const questcal::TrackerFrameCorrections::Changed *
	{
		for (const auto &c : result.changed)
			if (c.id == id)
				return &c;
		return nullptr;
	};
	for (const uint32_t id : result.refused)
		// Logged always, not only in detailed logging: the tracker stays off by
		// the move it reported, and the log has to say why.
		ctx.Log("Lighthouse: rejected an unsafe frame correction for device " + std::to_string(id) + "\n");

	for (const auto &move : targetMoves)
	{
		if (std::find(result.refused.begin(), result.refused.end(), move.id) != result.refused.end())
			continue;
		double yawDeg = 0.0, tiltDeg = 0.0;
		const bool notable = DescribeFrameMove(move, yawDeg, tiltDeg);
		// By when the frame moved, not when the device showed it: a tracker
		// switched off through the setup is back after it, its frame placed
		// during it (the 2026-09-27 placement replayed so: 68.7 cm followed at
		// the tracker).
		const bool setup = DuringUniverseSetup(ctx, move.frameTime);
		const bool bumped = !setup && bumpCorrection(move);
		const bool mounted = move.id == ctx.continuousTrackerId;
		if (setup)
			++ctx.frameMovesInSetup;
		else if (bumped)
			++ctx.bumpedStationCorrections;
		else
			++ctx.frameMovesFollowed;
		if (mounted)
		{
			// Observations and previous estimates already live in the
			// normalized space. Clear the window without rebasing them.
			ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::TargetFrameMoved);
			if (!setup && !bumped)
				++ctx.trackerFrameCompensations;
		}
		const auto &frame = ctx.trackerFrames.Snapshot()[move.id];
		char line[1024];
		snprintf(line, sizeof line,
			"lighthouse device %u %s frame moved%s%s: yaw %.2f deg, tilt %.2f deg, %.1f cm; %s; "
			"from q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f) "
			"to q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f); "
			"normalization q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f)",
			move.id, DeviceSerials[move.id].c_str(), move.returned ? " while off" : "",
			move.ownJump ? " with a local pose jump" : "", yawDeg, tiltDeg, move.shiftM * 100.0,
			setup ? "left alone during universe setup"
				: bumped ? "left alone: SteamVR corrected a base station it found knocked"
				: "compensated for this device",
			move.fromRot.w(), move.fromRot.x(), move.fromRot.y(), move.fromRot.z(),
			move.fromTrans.x(), move.fromTrans.y(), move.fromTrans.z(),
			move.toRot.w(), move.toRot.x(), move.toRot.y(), move.toRot.z(),
			move.toTrans.x(), move.toTrans.y(), move.toTrans.z(),
			frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z,
			frame.translation.v[0], frame.translation.v[1], frame.translation.v[2]);
		if (setup) ctx.lastFrameMoveInSetup = line;
		else if (bumped) ctx.lastBumpedStationCorrection = line;
		else
		{
			ctx.lastFrameMoveFollowed = line;
			if (mounted) ctx.lastTrackerFrameCompensation = line;
		}
		LogFrameLine(ctx, line, notable);
		if (bumped)
			CarryBumpCorrection(ctx, move);
	}

	for (const auto &change : targetSwitches)
	{
		char line[1024];
		const auto *c = changeOf(change.id);
		if (!c || c->how != How::Switched)
		{
			snprintf(line, sizeof line,
				"lighthouse device %u %s changed frame (%s); kept its frame correction: %s; "
				"from q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f) to q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f)",
				change.id, DeviceSerials[change.id].c_str(), SwitchCause(change.cause),
				change.id == measured ? "it is being measured" : "it is its frame's",
				change.fromRot.w(), change.fromRot.x(), change.fromRot.y(), change.fromRot.z(),
				change.fromTrans.x(), change.fromTrans.y(), change.fromTrans.z(),
				change.toRot.w(), change.toRot.x(), change.toRot.y(), change.toRot.z(),
				change.toTrans.x(), change.toTrans.y(), change.toTrans.z());
			ctx.Diag(line);
			continue;
		}
		const auto &after = ctx.trackerFrames.Snapshot()[change.id];
		double shiftM = 0.0, angleDeg = 0.0;
		CorrectionChange(change.id, c->before, after, shiftM, angleDeg);
		snprintf(line, sizeof line,
			"lighthouse device %u %s changed frame (%s); took its frame's correction: %.1f cm, %.2f deg at the "
			"device; normalization q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f)",
			change.id, DeviceSerials[change.id].c_str(), SwitchCause(change.cause), shiftM * 100.0, angleDeg,
			after.rotation.w, after.rotation.x, after.rotation.y, after.rotation.z,
			after.translation.v[0], after.translation.v[1], after.translation.v[2]);
		++ctx.frameJoins;
		ctx.lastFrameJoin = line;
		LogFrameLine(ctx, line, LighthouseFrameWatch::NotableMove(shiftM, angleDeg, 0.0));
	}

	for (const auto &c : result.changed)
	{
		// The headset tracker's correction is what the continuous loop holds to
		// the headset: whatever changed it, its window is in the old one.
		if (c.id == ctx.continuousTrackerId && c.how != How::Moved)
			ResetContinuousObservations(ctx, questcal::ContinuousAlignment::ResetReason::TargetFrameMoved);
		if (c.how != How::Replaced)
			continue;
		const auto &after = ctx.trackerFrames.Snapshot()[c.id];
		double shiftM = 0.0, angleDeg = 0.0;
		CorrectionChange(c.id, c.before, after, shiftM, angleDeg);
		char line[1024];
		snprintf(line, sizeof line,
			"lighthouse device %u %s took its frame's correction, which gave way to one the session start's "
			"moves or an older arrival derive: %.1f cm, %.2f deg at the device; "
			"normalization q(%.6f,%.6f,%.6f,%.6f) t(%.4f,%.4f,%.4f)",
			c.id, DeviceSerials[c.id].c_str(), shiftM * 100.0, angleDeg,
			after.rotation.w, after.rotation.x, after.rotation.y, after.rotation.z,
			after.translation.v[0], after.translation.v[1], after.translation.v[2]);
		++ctx.frameJoins;
		ctx.lastFrameJoin = line;
		LogFrameLine(ctx, line, LighthouseFrameWatch::NotableMove(shiftM, angleDeg, 0.0));
	}
	if (ctx.trackerFrames.Disagreements() > disagreements)
		ctx.Log("Lighthouse: two chains of recorded frame moves derived different corrections for one frame; "
			"the first one stays\n");
	if (ctx.trackerFrames.Unsettled() > unsettled)
		ctx.Log("Lighthouse: the frame corrections' derivation along the recorded moves stopped at its bound "
			"before it settled; the corrections stay as they were then\n");
	if (!result.changed.empty())
	{
		DispatchStreamEvent(ctx, questcal::StreamEvent::FrameMoved);
		SynchronizeCalibrationDriver(ctx);
	}
}

void calibration_internal::RuntimeMonitorTick(CalibrationContext &ctx, double now)
{
	const uint64_t boundary = PoseHub.StreamBoundaries();
	if (boundary != FrameStreamBoundary)
		StartFrameSession(ctx, boundary);
	// Frame corrections also start over from a recalibration or a profile
	// load, and come back from the driver's recovery: the epoch is how those
	// reach the windows kept here.
	if (FrameEpoch != ctx.trackerFrameEpoch)
	{
		FrameEpoch = ctx.trackerFrameEpoch;
		DispatchStreamEvent(ctx, questcal::StreamEvent::TrackerFramesReset);
	}
	// Score even while the monitors are parked so the UI's health readout
	// tracks calibration age from the moment a profile loads. Age alone
	// saturates below the stale threshold, so this can't notify by itself.
	if (ctx.validProfile)
		UpdateDriftScore(ctx);

	const bool idle = ctx.state == CalibrationState::None;
	// Frame normalization belongs to the driver session, including a manual
	// measurement. Drift and universe acceptance still run only while idle.
	bool shouldRun = !ctx.frameRecoveryPending && ctx.enabled && ctx.validProfile && PoseHub.RingOpen();

	if (!shouldRun)
	{
		if (MonitorActive)
		{
			// The frame watch keeps each device's last frame across the pause:
			// a station SteamVR re-solved meanwhile is still found from the
			// samples on either side of it, and corrections are kept for the
			// profile when it comes back. Forgetting them would leave a tracker
			// off by that move until the next fresh calibration.
			DispatchStreamEvent(ctx, questcal::StreamEvent::MonitorsParked);
			MonitorActive = false;
		}
		PoseHub.DiscardBacklog(MonitorConsumer);
		return;
	}

	if (!MonitorActive)
	{
		PoseHub.DiscardBacklog(MonitorConsumer);
		DispatchStreamEvent(ctx, questcal::StreamEvent::MonitorsResumed);
		MonitorActive = true;
	}

	// Judge the whole hole in front of the batch, from the drain itself: a
	// StreamBoundaries read before or after the drain can land on the wrong
	// side of a session boundary, and a hole split across two drains would
	// pass the leash share by share.
	PoseStreamHub::Hole hole;
	uint64_t dropped = PoseHub.Drain(MonitorConsumer, MonitorScratch, &hole);
	if (hole.sessionBoundary && PoseHub.StreamBoundaries() != FrameStreamBoundary)
	{
		StartFrameSession(ctx, PoseHub.StreamBoundaries());
		// The session boundary below drops the frame watch and its progress;
		// the continuous loop hears the boundary from its own drain.
		FrameEpoch = ctx.trackerFrameEpoch;
	}
	if (!ringpose::MonitorGapTolerable(hole.size, hole.sessionBoundary))
	{
		// We lost part of our own observation window; baselines across the
		// hole are unsafe. For the drift monitor a drain hole would read as a
		// tracking loss and could fake a discontinuity event.
		DispatchStreamEvent(ctx, hole.sessionBoundary
			? questcal::StreamEvent::SessionBoundary : questcal::StreamEvent::MonitorHole);
	}
	else if (dropped > 0)
	{
		// The driver's isolated contended-publish drops: one or two poses at a
		// time, every few seconds to minutes on a small rig and every 0.37 s on
		// a twenty-device one (2026-10-08). The drift monitor and the
		// watermarks work on sample times and lose nothing to a hole this
		// short; the jump detector drops a fit only for a device whose own
		// stream skipped there, and keeps the rest.
		DispatchStreamEvent(ctx, questcal::StreamEvent::MonitorDrop);
	}
	if (!ctx.detailedLogging)
		StreamDigest.Reset();
	else if (dropped > 0)
		StreamDigest.NoteDrops(dropped);

	for (const auto &s : MonitorScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		FrameObservedThrough[s.deviceId] = (std::max)(FrameObservedThrough[s.deviceId], s.sampleTimeQpc);
		// JumpDetector owns the observation-continuity policy and must see bad
		// frames as well as good ones. Drift/HMD caches below remain valid-only.
		if (ctx.referenceDeviceMask[s.deviceId])
		{
			if (idle) questcal::ObserveUniversePose(s);
			if (ctx.detailedLogging)
				StreamDigest.Note(s, QpcToSeconds);
		}
		else if (!VirtualDevices[s.deviceId])
		{
			const auto &serial = ctx.trackerFrames.Serial(s.deviceId);
			if (FrameObservedSerials[s.deviceId] != serial)
			{
				FrameWatch.ForgetDevice(s.deviceId);
				FrameObservedSerials[s.deviceId] = serial;
			}
			FrameWatch.Note(s, QpcToSeconds,
				DeviceClasses[s.deviceId] == vr::TrackedDeviceClass_TrackingReference,
				/*hiddenTracking=*/ctx.targetDeviceMask[s.deviceId]);
		}
	}
	const bool jumped = idle && questcal::FinishUniverseObservations(ctx, now);
	UpdateTrackerFrames(ctx);

	if (!idle)
	{
		DispatchStreamEvent(ctx, questcal::StreamEvent::Calibrating);
		return;
	}

	for (const auto &s : MonitorScratch)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		questcal::PoseSample sample;
		// A reference device's rate in the frame its driver reports it in, so
		// continuous alignment predicts its turns right (LocalPoseContinuity.h).
		const ringpose::AngularVelocityFrame angularFrame = ctx.referenceDeviceMask[s.deviceId]
			? questcal::ReferenceAngularVelocityFrame(s.deviceId)
			: ringpose::AngularVelocityFrame::Unknown;
		if (!TryComposeRingSample(s, QpcToSeconds, sample, ctx.targetDeviceMask[s.deviceId],
				angularFrame))
			continue;
		if (ctx.targetDeviceMask[s.deviceId] && !ctx.trackerFrames.Normalize(s.deviceId, sample))
			continue;
		// Per-device watermark across drains.
		double composedTime = sample.time;
		if (Monitors.hasComposedTime[s.deviceId] &&
			composedTime <= Monitors.lastComposedTime[s.deviceId])
			continue;
		Monitors.hasComposedTime[s.deviceId] = true;
		Monitors.lastComposedTime[s.deviceId] = composedTime;

		if (s.deviceId == vr::k_unTrackedDeviceIndex_Hmd &&
			ctx.referenceDeviceMask[s.deviceId])
		{
			Monitors.hmdRawPosition = sample.pos;
			Monitors.hmdRawTime = composedTime;
		}

		// AnchorsUniverse excludes devices whose normal movement could be
		// mistaken for playspace drift, including the mounted tracker.
		ringpose::DriftFeedCandidate candidate;
		candidate.deviceId = s.deviceId;
		candidate.deviceClass = DeviceClasses[s.deviceId];
		candidate.referenceSide = ctx.referenceDeviceMask[s.deviceId];
		candidate.targetSide = ctx.targetDeviceMask[s.deviceId];
		candidate.mountedTrackerId = ctx.continuousTrackerId;
		candidate.rawPosition = sample.pos;
		candidate.composedTime = composedTime;
		candidate.hmdRawPosition = Monitors.hmdRawPosition;
		candidate.hmdRawTime = Monitors.hmdRawTime;
		candidate.calibratedRotation = ctx.transform.rotation;
		candidate.calibratedTranslationMeters = ctx.transform.translationMeters;
		candidate.calibratedScale = ctx.transform.scale;

		if (ringpose::AnchorsUniverse(candidate))
			Drift->Push(s,
				ctx.targetDeviceMask[s.deviceId] ? ctx.transform.scale : 1.0,
				/*streamed=*/candidate.referenceSide);
	}

	if (ctx.detailedLogging)
		for (const auto &line : StreamDigest.Flush(now))
			ctx.Diag(line);

	// Lighthouse frame moves are rare and are the evidence a "the whole
	// calibration jumped" report needs, so one is always logged; a pose merely
	// re-expressed in another station's frame, or refined in place, only in
	// detail.
	for (const auto &report : FrameWatch.Flush())
	{
		const std::string line = LighthouseFrameWatch::Describe(report, ctx.continuousTrackerId);
		if (LighthouseFrameWatch::Notable(report))
		{
			ctx.lighthouseFrameMoves++;
			ctx.lastLighthouseFrameMove = line;
			ctx.Log(line + "\n");
		}
		else
		{
			ctx.Diag(line);
		}
	}

	// Jump acceptance lands before a slide window can conclude (~0.25 s vs
	// ~2.5 s), so dropping the drift window also discards any event queued
	// from the same discontinuity.
	if (jumped)
		DispatchStreamEvent(ctx, questcal::StreamEvent::UniverseJump);

	DriftMonitor::Event drift;
	while (Drift->PollEvent(drift))
	{
		char buf[256];
		// A lighthouse device that just lost or regained a base station is
		// reporting its own tracking, not the universes (LighthouseVisibility.h).
		const std::string &serial = DeviceSerials[drift.deviceId];
		if (!serial.empty() && ctx.lighthouse.Disturbed(serial, drift.time))
		{
			ctx.lighthouseAttributedEvents++;
			const LighthouseVisibility::Device *seen = ctx.lighthouse.Find(serial);
			snprintf(buf, sizeof buf,
				"Device %u %s %.1f cm after it %s -- its base stations, not drift\n",
				drift.deviceId,
				drift.type == DriftMonitor::Event::StationarySlide ? "slid" : "recovered",
				drift.magnitude * 100.0,
				seen && !seen->lastDisturbanceText.empty()
					? seen->lastDisturbanceText.c_str() : "changed base stations");
			ctx.Log(buf);
			continue;
		}
		// The same while SteamVR sets up or re-chooses its lighthouse universe:
		// it re-places its stations for up to a minute (LighthouseVisibility.h).
		if (drift.deviceId < vr::k_unMaxTrackedDeviceCount &&
			ctx.targetDeviceMask[drift.deviceId] &&
			ctx.lighthouse.UniverseQuiet(drift.time))
		{
			ctx.lighthouseAttributedEvents++;
			snprintf(buf, sizeof buf,
				"Device %u %s %.1f cm while SteamVR set up its lighthouse universe -- its base stations, not drift\n",
				drift.deviceId,
				drift.type == DriftMonitor::Event::StationarySlide ? "slid" : "recovered",
				drift.magnitude * 100.0);
			ctx.Log(buf);
			continue;
		}
		if (drift.type == DriftMonitor::Event::StationarySlide)
		{
			ctx.driftSlideEvents++;
			ctx.driftMaxSlideM = std::max(ctx.driftMaxSlideM, drift.magnitude);
			snprintf(buf, sizeof buf,
				"Stationary device %u slid %.1f cm -- drift evidence\n",
				drift.deviceId, drift.magnitude * 100.0);
		}
		else
		{
			ctx.discontinuousLossEvents++;
			snprintf(buf, sizeof buf,
				"Device %u recovered %.1f cm away from where tracking dropped -- possible re-localization\n",
				drift.deviceId, drift.magnitude * 100.0);
		}
		ctx.Log(buf);
	}

}
