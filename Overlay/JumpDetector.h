#pragma once

#include "Clocks.h"
#include "PoseMath.h"
#include "RingPoseMath.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <bitset>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

// Universe-jump detection over the reference system's pose stream.
//
// When the Quest runtime recenters or re-localizes, its whole universe shifts
// in one frame. Two detection paths:
//
//  - Exact: the reference driver re-bases worldFromDriver while the
//    driver-local pose stays continuous. The delta is then exactly
//    D = new_wfd ∘ old_wfd⁻¹, per device, no heuristics.
//  - Heuristic fallback: the composed world pose jumps inconsistently with
//    reported velocity. D is estimated from short velocity-compensated
//    windows on both sides of the discontinuity, never a single frame pair.
//
// Accepted deltas are constrained to yaw + translation (a recenter preserves
// gravity). The exact path reports the discarded tilt as residualTiltRad; the
// heuristic path regresses heading only and leaves it 0. Both residuals are
// diagnostics only. Times are ring-sample seconds (QPC * QpcToSeconds), not
// UI time, and a RingTime where they are handed out.
class JumpDetector
{
public:
	// Fixed detection policy.
	struct Config
	{
		// A wfd change that counts as a rebase, as the universe verdict counts it.
		double wfdRotEpsRad = questcal::WorldFromDriverChangeRadians;
		double wfdTransEps = questcal::WorldFromDriverChangeMeters;   // meters
		double discontinuityPos = 0.20;    // meters of unexplained motion in one frame pair
		double discontinuityYawRad = 5.0 * 3.14159265358979 / 180.0;
		// Smaller steps need a clean fit and tighter HMD/controller agreement.
		// They never qualify for the single-device fallback.
		double corroboratedPos = 0.05;
		double corroboratedYawRad = 2.0 * 3.14159265358979 / 180.0;
		double corroboratedPosTol = 0.02;
		double corroboratedYawTolRad = 1.0 * 3.14159265358979 / 180.0;
		double fitPositionRms = 0.01;
		double fitYawRmsRad = 0.5 * 3.14159265358979 / 180.0;
		// These limits stay absolute, also for a large step. Let grow with the
		// step (a tenth of it), they passed 50 false jumps of 10 deg and more in
		// 80 minutes of a headset moved harder than the limits below allow for,
		// as when it is taken off; held, they passed none (harness, 2026-11-14).
		// What a device can do between two samples, where the stream does not
		// show it. A head starts or reverses a turn with up to about 100 rad/s2
		// and moves with up to about 30 m/s2, so over an interval dt it can
		// leave the path its samples predict by dt^2 / 2 times that: 0.8 deg and
		// 4 mm at 60 Hz, 3.2 deg and 17 mm at 30 Hz, 7.2 deg and 38 mm at
		// 20 Hz, 19 deg and 10 cm over an 82 ms hole. A step no larger than
		// that is no evidence of a frame change: it is neither raised from a
		// frame pair nor kept from a fit whose windows hold such an interval.
		// Through Virtual Desktop a Quest's poses come at the game's frame
		// rate. A tester's session of 2026-11-14 ran at 20 to 38 Hz for minutes
		// at a time, and 52 head turns of 0.03 to 3.45 deg were compensated as
		// universe jumps: fitted as steps over three to five samples a side,
		// then confirmed by a controller's own false step. Of the 37 headset
		// steps that log kept, these limits pass the three that came at 60 Hz;
		// of the 460 on the controllers, 95.
		double hiddenAngularAcceleration = 100.0;   // rad/s^2
		double hiddenAcceleration = 30.0;           // m/s^2
		// The sparsest streams. Below about 15 Hz `window` holds fewer than
		// three samples a side, and frames more than maxFrameGap apart were
		// never compared, so a recenter there went unseen: of 60 recenters by a
		// player standing still, 12 at 20 Hz, 50 at 15 Hz and all at 12 Hz
		// and below (harness sweep, 2026-11-14). A candidate's windows now
		// widen until each holds three samples, to sparseWindow at most, and
		// frame pairs up to maxStepInterval apart are compared: 3 of the 60
		// are missed at 20 Hz and 25 at 15 Hz, each in windows the fit's limits
		// refuse. What an interval can hide grows with its square (38 deg and
		// 20 cm at 115 ms, 115 deg and 60 cm at 200 ms), so only a step the
		// size of a recenter passes there, and below about 7 Hz none does.
		double sparseWindow = 0.5;
		double maxStepInterval = 0.2;
		double maxFrameGap = ringpose::MaxAdjacentFrameSeconds;
		// After a driver queue drop, a device whose next interval is this many
		// times its usual one lost a frame of its own (one lost frame doubles
		// it; a 45 Hz stream's jitter stays under it).
		double holeIntervalFactor = 1.5;
		double localContinuityPos = ringpose::MaxLocalPositionErrorMeters;
		double localContinuityRotRad = ringpose::MaxLocalRotationErrorRadians;
		double window = 0.2;               // seconds of history on each side of a candidate
		double agreeWindow = 0.25;         // seconds within which devices must agree
		double agreePosTol = 0.10;         // meters between per-device delta translations
		double agreeYawTolRad = 3.0 * 3.14159265358979 / 180.0;
		// Each Quest Pro controller is its own tracking frontend on the shared
		// map and may keep its previous frame for up to 30 s after the headset
		// switches map, so an HMD candidate no device confirmed inside
		// agreeWindow is held this long for the controller's step; one no
		// controller follows (a 3DoF catch-up after a wake) still expires. The
		// clock pauses while every other reference device is locked still: the
		// engine's static prior freezes a resting controller (its position
		// repeats bit for bit), and it cannot step until the hand moves again.
		double controllerFollowSeconds = 30.0;
		// The controller follower snaps a delta above 5 cm at once while the
		// headset smoother slides for its 5 s grace, so the second of two close
		// map moves can reach the controllers up to 5 s before the headset. A
		// controller step that far ahead still confirms if the deltas match.
		double controllerLeadSeconds = 5.0;
		double soloPosThreshold = 0.30;    // single-device heuristic acceptance floor
		double soloYawThresholdRad = 10.0 * 3.14159265358979 / 180.0;
		// Headset-only setups (the primary population) never have a second
		// reference device to confirm a step; one session logged three 1-4 cm
		// same-sign steps in three minutes that a recalibration later removed.
		// A clean persistent HMD step is therefore accepted alone down to the
		// corroborated floor once the stream has been continuous for
		// soloSettledSeconds (a wake, a stream restart and Virtual Desktop's
		// reconnect re-zeroing all fall inside the first minute) and neither
		// fit window shows a held position: the 3DoF fallback holds position
		// bit for bit, which 6DoF never did (0 of 4,331 frames, 2026-09-11).
		// Nor when the frame carrying the step came late (holeIntervalFactor
		// times the usual interval): a stalled stream, whose fit straddles the
		// stall. Live 2026-10-24, Virtual Desktop: a -3.94 deg step on a 43.9 ms
		// frame of a ~13 ms stream, inside a compositor hand-off stall, was
		// applied alone and undone by the recalibration 26 s later.
		double soloSettledPos = 0.05;
		double soloSettledYawRad = 2.0 * 3.14159265358979 / 180.0;
		double soloSettledSeconds = 60.0;
		// The engine's correction smoother slides toward the map target while
		// the head moves and, once the remaining offset exceeds its reset
		// threshold (10 cm or 10 deg, compiled defaults of the examined build,
		// after a 5 s grace), snaps the rest in one step of almost exactly the
		// threshold. That snap restores the alignment; compensating it would put
		// the drift back. A headset step inside this band is never applied alone
		// unless continuous alignment has followed the drift (SetDriftFollowed);
		// a matching controller step still confirms it. A real frame change of
		// exactly the threshold is logged and left for the next correction.
		double driftCatchUpPos = 0.10;
		double driftCatchUpPosBand = 0.005;
		double driftCatchUpYawRad = 10.0 * 3.14159265358979 / 180.0;
		double driftCatchUpYawBandRad = 0.3 * 3.14159265358979 / 180.0;
		// The translation band only holds for a step without yaw: the smoother
		// measures about the map origin, which the stream does not show. Heading
		// drift accrues where the user turns slowly in place, so its catch-up
		// turns the frame about the head, while a frame change moves the head by
		// its distance from the pivot times the angle. A headset step within the
		// yaw threshold that moves the head no more than this is a catch-up too;
		// a frame change pivoting within ~30 cm of the head is the cost.
		double driftCatchUpHeadShift = 0.02;
		double gapSeconds = 2.0;           // reference stream gap => no compensation, event only
		// A discontinuity this soon after the device's stream (re)started is
		// annotated with its resume age: after a Quest Pro wakes (seen through
		// Virtual Desktop) poses resume in 3DoF, and the snap to 6DoF a few
		// seconds later looks exactly like a universe jump.
		double recentResumeSeconds = 60.0;
	};

	// A universe delta to fold into the calibration: newCal = D ∘ oldCal.
	struct UniverseDelta
	{
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };   // yaw-only
		Eigen::Vector3d translation{ 0, 0, 0 };
		questcal::RingTime time;           // at the jump
		// The HMD's jump sample, by its capture ticks: what the universe
		// verdict matches the worldFromDriver transition at it by.
		int64_t sample = 0;
		bool exact = false;                // wfd-rebase path vs heuristic
		int devicesAgreeing = 0;
		double residualTiltRad = 0.0;      // non-rigid part discarded by the yaw constraint
		double residualSpread = 0.0;       // meters; disagreement between devices (heuristic)
		// Seconds between the reporting device's most recent stream (re)start
		// (its first valid sample ever, or the first after a gap) and the
		// jump. Reported, never used to gate acceptance.
		double secondsSinceStreamResume = -1.0;
		// Heuristic path: seconds between the HMD's step and the confirming
		// device's step when the confirmation arrived outside agreeWindow:
		// positive for a controller following a map switch, negative for a
		// controller that stepped ahead of the headset (the smoother's
		// grace); 0 when devices agreed at once or the delta was accepted
		// solo. `time` stays the HMD's jump instant either way.
		double confirmationLagSeconds = 0.0;
		// Absolute HMD worldFromDriver endpoint captured by this exact sample.
		// `exact` is the validity discriminator; heuristic deltas leave identity.
		Eigen::Quaterniond worldFromDriverRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d worldFromDriverTranslation{ 0, 0, 0 };
		// The HMD worldFromDriver the exact rebase started from.
		Eigen::Quaterniond previousWorldFromDriverRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d previousWorldFromDriverTranslation{ 0, 0, 0 };
	};

	struct GapEvent
	{
		uint32_t deviceId = 0;
		double duration = 0.0;             // seconds without valid reference samples
		questcal::RingTime time;           // the sample that ended it

		// Only a gap in the headset's stream can hide a universe move: the
		// headset is what the universe is measured from. A controller that
		// sleeps or leaves the headset's view stops its own stream while the
		// headset goes on tracking (a tester's session had 42 such gaps of 43).
		bool OnHeadset() const { return deviceId == 0; }   // k_unTrackedDeviceIndex_Hmd
	};

	explicit JumpDetector(double qpcToSeconds) : qpcToSeconds(qpcToSeconds) { }
	// With another policy: the harness replays a logged step as the build of
	// its day judged it.
	JumpDetector(double qpcToSeconds, const Config &policy) : config(policy), qpcToSeconds(qpcToSeconds) { }

	// Feed one reference-system sample, in ring order and valid or not (a bad
	// frame breaks continuity). The caller bounds deviceId; non-reference
	// devices must not be pushed.
	void Push(const protocol::DevicePoseSample &sample);

	// Queued output, each drained by the caller until it returns false: an
	// accepted jump, a reference-stream gap, and human-readable notes (wfd
	// rebases seen, candidates expired, ...) for the calibration log.
	bool PollDelta(UniverseDelta &out) { return PopFront(accepted, out); }
	bool PollGap(GapEvent &out) { return PopFront(gaps, out); }
	bool PollNote(std::string &out) { return PopFront(notes, out); }

	// The frame the device's reported angular velocity is in, as the probe
	// judged it (ProbeAngularFrame); Unknown until it has.
	ringpose::AngularVelocityFrame AngularFrameOf(uint32_t id) const
	{
		return id < vr::k_unMaxTrackedDeviceCount
			? devices[id].angularFrame : ringpose::AngularVelocityFrame::Unknown;
	}

	// The caller's copy of the stream is missing a few samples (the driver
	// drops an isolated pose when two device threads contend for a queue
	// claim). The drop names no device, so each device's next sample says
	// whether it lost one: an interval past holeIntervalFactor times its usual
	// one breaks that device's continuity as an observed bad frame does, and a
	// candidate of it whose fit window was still filling dies; an ordinary
	// interval means the lost pose was another device's, and nothing changes.
	// Breaking every device for every drop made the detector useless where
	// drops are frequent (2026-10-08: one every 0.37 s on a twenty-device rig).
	// The resume clock, Ready candidates and the session totals stay either way.
	void NoteStreamHole();
	// Whether continuous alignment keeps the calibration on an independent
	// reference (a mounted tracker). The
	// calibration has then followed the drift, so a drift catch-up moves the
	// frame away from it and is compensated like any other step. The caller
	// refreshes this every tick; Reset leaves it alone.
	void SetDriftFollowed(bool followed) { driftFollowed = followed; }
	// Detailed logging: each candidate's evidence (the frame jump that raised
	// it, both fitted states and their residuals, the samples around the
	// step) and the reason any candidate is dropped, which otherwise happens
	// silently. Off, nothing is gathered. The caller refreshes it every tick.
	void SetDetailed(bool on)
	{
		detailed = on;
		if (!on)
			details.clear();
	}
	bool PollDetail(std::string &out) { return PopFront(details, out); }

	// Drop all per-device state (calibration started, monitors disabled, a
	// stall-sized hole or a driver session boundary in the stream, ...).
	void Reset();

	// Whether a headset heuristic candidate raised at `sample` (its capture
	// ticks) is still alive: its fit window is filling, or it waits for a
	// device to confirm it. The profile-universe verdict does not give up on
	// a worldFromDriver change such a candidate may yet explain.
	bool HasLiveHeadsetCandidate(int64_t sample) const;

private:
	struct Hist
	{
		double t;
		int64_t sample = 0;                // the ring sample's capture ticks: which sample it was
		Eigen::Vector3d pos;               // composed world position
		Eigen::Vector3d vel;               // world-rotated velocity
		double yaw;                        // composed world heading
		// The composed world rotation, and its rate in the raw universe with
		// the reported angular velocity read in the driver's world frame and
		// in the device's own frame. The jump check takes the reading the
		// probe judged, and both until it has. It predicts the whole rotation
		// and reads the heading off it (PredictedYaw): the heading of a tilted
		// device does not turn at the rate's vertical part in either reading.
		Eigen::Quaterniond rot;
		Eigen::Vector3d rateWorld;
		Eigen::Vector3d rateDevice;
	};

	// Which detection path produced a candidate, and where it is in its life.
	enum class Kind { Exact, Heuristic };
	enum class Life
	{
		Pending,   // heuristic: post-jump window still filling
		Ready,     // has a usable delta (exact candidates are born Ready)
		Dead,      // expired unconfirmed, or its observation continuity broke
	};

	// The worldFromDriver endpoints the exact path captured.
	struct ExactEndpoint
	{
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
		Eigen::Quaterniond previousRotation{ 1, 0, 0, 0 };
		Eigen::Vector3d previousTranslation{ 0, 0, 0 };
	};

	// An accepted step whose follow-ups are still owed. Each device that took
	// no part in it owes one, within its windows: that device's first step
	// there matching the accepted delta, judged once fitted, is its follow-up,
	// the same event's evidence, already used. A later step stays, as new
	// evidence (TryAccept, OwedFollowUp).
	struct AcceptedStep
	{
		double t = 0.0;
		double followDeadline = 0.0;
		Eigen::Vector3d translation{ 0, 0, 0 };
		double yaw = 0.0;
		std::bitset<vr::k_unMaxTrackedDeviceCount> owed;
	};

	struct Candidate
	{
		uint32_t deviceId = 0;
		double t = 0.0;
		int64_t sample = 0;   // the sample at `t` (Hist::sample)
		Kind kind = Kind::Heuristic;
		Life life = Life::Pending;
		// Heuristic candidates: the length of each fit window, `window` or
		// longer on a sparse stream (see sparseWindow in Config).
		double window = 0.0;
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
		double residualTiltRad = 0.0;
		ExactEndpoint endpoint;   // Kind::Exact only
		// Heuristic candidates: pre-jump window snapshot, evaluated once the
		// post-jump window has filled.
		std::vector<Hist> preWindow;
		// The velocity-compensated jump between the two frames that raised
		// the candidate. Against the fitted step it shows whether the pose
		// overshot within a frame and settled back.
		double frameJumpPos = 0.0, frameJumpYawRad = 0.0;
		// The interval between those two frames, and the device's usual one
		// before it (0 while unknown): a late frame is a stalled stream.
		double frameInterval = 0.0, usualInterval = 0.0;
		bool needsCorroboration = false;
		bool held = false;   // HMD heuristic: past agreeWindow, awaiting a controller follow-up
		// HMD heuristic: when the follow-up wait ends, pushed out while the
		// other reference devices are locked still; `heldLocked` marks that
		// the extension was logged.
		double followDeadline = 0.0;
		double lastHoldCheck = -1.0;
		bool heldLocked = false;
		// HMD heuristic: kept past its deadline for a step inside its windows
		// that is still being fitted (AwaitsFittedStep).
		bool awaitingFit = false;
		// HMD heuristic, decided once when the fit completes. `heldPosition`:
		// a fit window shows the 3DoF fallback: before the step, a catch-up
		// onto resumed tracking; from it on, a step measured against a held
		// position. `driftCatchUp`: shaped like the smoother's reset
		// (see driftCatchUp* in Config). No solo path applies either.
		// `settledSolo`: may be applied with no other reference device
		// tracking (see soloSettled*).
		bool heldPosition = false;
		bool driftCatchUp = false;
		bool settledSolo = false;
		// HMD heuristic: another reference device was tracking when the step
		// was raised. Such a step is that device's to confirm or to leave
		// unconfirmed: the device going quiet during the hold does not make
		// the step a headset-only one.
		bool othersAtStep = false;
		// HMD heuristic, for the log only: shaped like a drift catch-up
		// (threshold-sized or a turn about the head, followed drift or not),
		// and the other devices whose own frames jumped within agreeWindow of
		// it. Whether controllers step with such a step is whether they share
		// the headset's correction (Constellation-tracked ones do in the
		// engine; self-tracked ones follow their own map).
		bool catchUpShaped = false;
		std::bitset<vr::k_unMaxTrackedDeviceCount> steppedWith;
		// Detailed mode, for the candidate's one log line (WriteCandidateLine):
		// the jump that raised it, and how many of its device's later frame
		// jumps fell inside it. A line per such jump was 89% of a tester's
		// session log and pushed its middle out (SessionLogTrim.h).
		std::string raisedText;
		int jumpsInside = 0;
		double largestInsidePos = 0.0, largestInsideYawRad = 0.0;
		bool lineWritten = false;

		// The filters every consumer needs, once each.
		bool LiveExact() const { return kind == Kind::Exact && life != Life::Dead; }
		bool ReadyHeuristic() const { return kind == Kind::Heuristic && life == Life::Ready; }
		bool PendingHeuristic() const { return kind == Kind::Heuristic && life == Life::Pending; }
	};

	struct DeviceState
	{
		bool wfdValid = false;
		Eigen::Quaterniond wfdRot{ 1, 0, 0, 0 };
		Eigen::Vector3d wfdTrans{ 0, 0, 0 };
		Eigen::Quaterniond drvRot{ 1, 0, 0, 0 };
		Eigen::Vector3d drvPos{ 0, 0, 0 };
		Eigen::Vector3d drvVel{ 0, 0, 0 };
		Eigen::Vector3d drvAngVel{ 0, 0, 0 };
		double lastValidTime = -1.0;
		double streamResumeTime = -1.0;    // first valid sample ever, or first after a gap
		// Consecutive samples whose composed position repeated the previous
		// one bit for bit: the static prior's lock on a still controller.
		int repeatedPositions = 0;
		std::deque<Hist> hist;
		// A driver queue drop was noted since this device's last sample
		// (NoteStreamHole), and the device's usual interval between valid
		// samples (0 until two have arrived).
		bool holeNoted = false;
		double usualInterval = 0.0;
		// Which frame the reported angular velocity is in, judged once a
		// session (and kept across Reset): the two samples before the latest
		// (time and driver-local orientation, oldest first) with the middle
		// one's reported rate, and that rate's residuals against the
		// orientation's own rate taken in the device frame and in the world
		// frame. The jump check reads the rate in the judged frame.
		ringpose::AngularVelocityFrame angularFrame = ringpose::AngularVelocityFrame::Unknown;
		int rateSamples = 0;
		double rateTime[2] = { 0.0, 0.0 };
		Eigen::Quaterniond rateRot[2] = { Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity() };
		Eigen::Vector3d rateReported{ 0, 0, 0 };
		std::vector<double> deviceFrameResidual, worldFrameResidual;
		bool angularFrameReported = false;
	};

	// A candidate's device has always had a valid sample since the last
	// Reset, and a gap after the candidate kills it, so this is never negative.
	static double ResumeAge(const DeviceState &dev, double t)
	{
		return t - dev.streamResumeTime;
	}

	template <typename T>
	static bool PopFront(std::deque<T> &queue, T &out)
	{
		if (queue.empty())
			return false;
		out = queue.front();
		queue.pop_front();
		return true;
	}

	// The engine's 3DoF fallback: position held bit-for-bit between frames
	// while the orientation keeps integrating. Never seen in 6DoF.
	static bool HeldPositionSignature(const std::vector<Hist> &window);
	// The turn and the displacement a device can make unseen in `interval`
	// seconds (see hiddenAngularAcceleration in Config).
	double HiddenYaw(double interval) const { return 0.5 * config.hiddenAngularAcceleration * interval * interval; }
	double HiddenPos(double interval) const { return 0.5 * config.hiddenAcceleration * interval * interval; }

	void DetectWfdRebase(uint32_t id, DeviceState &dev, double t, int64_t sample,
	                     const Eigen::Quaterniond &newRot, const Eigen::Vector3d &newTrans);
	// `usualInterval` is the device's usual interval before `incoming` arrived.
	void DetectDiscontinuity(uint32_t id, DeviceState &dev, const Hist &incoming, double usualInterval);
	void EvaluatePendingCandidates();
	void TryAccept();
	int ActiveDeviceCount(double now) const;
	// For the log: which devices' frames jumped with an HMD candidate.
	std::string StepCompanions(const Candidate &c, double now) const;
	// Detailed mode: the candidate's line with its outcome ("dropped: ...",
	// "fitted: ..."). The first one carries how it was raised and the jumps
	// inside it; a later one, for a fitted candidate dropped afterwards, only
	// the outcome.
	void WriteCandidateLine(Candidate &c, const std::string &outcome);
	// The frame of a device's reported angular velocity, judged once and logged.
	void ProbeAngularFrame(uint32_t id, DeviceState &dev, double t,
	                       const Eigen::Quaterniond &rot, const Eigen::Vector3d &rate);
	// True when at least one other reference device is tracking and every
	// one of them is locked still (position repeated over the last samples).
	bool OthersLockedStill(double now) const;
	// A step of another device inside a held HMD candidate's windows that is
	// still being fitted, and still can be by `now` (its fit is due a window
	// after it).
	bool AwaitsFittedStep(const Candidate &c0, double now) const;

	Config config;
	double qpcToSeconds;

	DeviceState devices[vr::k_unMaxTrackedDeviceCount];
	std::vector<Candidate> candidates;
	std::deque<UniverseDelta> accepted;
	std::vector<AcceptedStep> recentAccepts;
	// The accepted step `c` is the owed follow-up of, if any.
	AcceptedStep *OwedFollowUp(const Candidate &c);
	std::deque<GapEvent> gaps;
	std::deque<std::string> notes;
	double lastAcceptTime = -1e9;
	bool driftFollowed = false;
	bool detailed = false;
	std::deque<std::string> details;
	// Marks a live candidate dead and, in detailed mode, says why.
	void Drop(Candidate &c, const std::string &reason);
	// HMD steps that expired unapplied since the last Reset, summed as one
	// transform so the log shows whether they add up to the drift a
	// recalibration later removes.
	int ignoredSteps = 0;
	double ignoredYaw = 0.0;
	Eigen::Vector3d ignoredTranslation{ 0, 0, 0 };
	// Drift catch-ups over the whole session, as their log line counts them;
	// Reset leaves it (a calibration or a stream break reset it mid-session,
	// so the line read 1, 1, 2, 3 on 2026-10-07).
	int driftCatchUps = 0;
};
