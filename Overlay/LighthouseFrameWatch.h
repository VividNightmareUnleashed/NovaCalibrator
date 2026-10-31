#pragma once

// RingPoseMath.h first: its Protocol.h settles which OpenVR header this TU
// has, and ChaperoneMath.h adds none once one is in.
#include "RingPoseMath.h"
#include "ChaperoneMath.h"
#include "../common/MathConstants.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <set>
#include <string>
#include <vector>

// Moves of the lighthouse side's frame, which nothing else watches: the jump
// detector follows the headset side only, and the vrserver.txt tailer reads
// per-device lines. The lighthouse driver reports a tracker's pose in a base
// station's frame, with that station's pose in the universe as the sample's
// worldFromDriver; a base station's own sample is its pose over an identity
// local pose. If SteamVR re-solves the stations, every lighthouse device's
// worldFromDriver changes together while its local pose stays on its
// trajectory, and each world pose moves by the same transform. A headset
// tracker moved that way looks exactly like the two universes moving apart
// (live 2026-09-24: 62 deg of yaw, 78.6 deg of tilt, 5 m at the head, rigid
// for an hour and gone after a SteamVR restart); this records whether it was.
//
// Each worldFromDriver change is classified from the device's own samples:
//   moved         the local pose stayed on its trajectory, or unchanged for a
//                 device at rest, so the world pose moved by the change;
//   re-expressed  the world pose stayed put: the same pose in another frame;
//   unclear       both moved, e.g. a tracker restarting at the same moment.
// Changes within groupSeconds of the first one form a report carrying the
// world delta, how far the moved devices' deltas disagree (not at all for a
// whole-universe move) and which devices moved.
//
// What moves a frame in practice is SteamVR re-solving where its base
// stations stand: "Moving base FD626122 111mm and 1.3 deg because of
// relationship with 4921060B ... Moving the base for tracking too, which
// might cause a pop" in vrserver.txt, or a new universe tilt when a device
// calibrates a station's pitch. Every device reported in that station's frame
// moves with it, the headset tracker included when it is one of them (live
// 2026-09-26: never in two hours, then, once a restart had put it in the
// frame of the station SteamVR kept moving, twelve times in 26 minutes, up to
// 2.1 deg of tilt and 6 to 12 cm each). A moved tracker's delta is also handed
// out on its own at once (TakeMoves), for the caller to follow in that
// device's frame correction before anything measures it.
//
// A device that cannot vouch for its own move gets one read off the others:
// one back from a disconnect (SteamVR switched it off after sitting still,
// and re-solved stations meanwhile: live 2026-09-26, eight moves and a
// universe tilt reset in the 2.5 minutes the headset tracker was off), or one
// whose own pose jumped as its frame changed. If the moves other devices
// reported, a base station's own samples included, lead from its old frame to
// its new one, that frame moved under it and the delta is exact; back in
// another station's frame it is not, and nothing is handed out.
//
// A device that came into a frame without a move its correction can follow
// (back in another station's frame, re-expressed, or first seen) is handed
// out as a switch instead (TakeSwitches): its correction belongs to the
// frame it left, and the caller gives it the new frame's
// (TrackerFrameCorrections::Update).
//
// A device another driver hides from applications is watched all the same:
// Standable hooks the body trackers it republishes as its own and passes the
// physical ones on as not connected while they track (live 2026-09-26: four
// trackers at 400 Hz, every sample "not connected", all of them in the frame
// of the station SteamVR kept moving). A target-system device whose pose says
// it tracks counts here whatever the connection flag says; only one that does
// not is a device gone. The caller says which devices those are: Standable's
// own unused slots also report not connected, valid and Running_OK, at the
// origin, and are not hidden trackers (live 2026-10-08).
//
// Every move is reported, the ones SteamVR makes while it sets up its
// universe included (LighthouseVisibility.h); those place a station guessed
// at startup where it stands, and the caller leaves them alone.
class LighthouseFrameWatch
{
public:
	struct Config
	{
		double groupSeconds = 1.0;
		// A device at rest (ringpose::RestPositionMeters and RestRotationRadians).
		double restPositionM = ringpose::RestPositionMeters;
		double restRotationRad = ringpose::RestRotationRadians;
		// How long such a device waits for the move to show on another
		// device: the station's own sample can come after it.
		double inferSeconds = 5.0;
		// How long a device's samples can still carry the frame a move left
		// after another device's carry the new one: the driver solves each
		// device on its own, so a move reaches them one at a time (live
		// 2026-10-15: a tracker one sample behind the rest lost a 17.8 cm move).
		double lagSeconds = 1.0;
	};

	enum class Kind { Moved, ReExpressed, Unclear };

	struct Report
	{
		double time = 0.0;
		int moved = 0, movedBaseStations = 0, reExpressed = 0, unclear = 0;
		std::vector<uint32_t> movedIds;
		// The world delta (new world = rotation * old world + translation) of
		// the first moved device, a base station's when one moved.
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
		double yawDeg = 0.0, tiltDeg = 0.0;
		double largestShiftM = 0.0;             // furthest a moved device's world pose jumped
		double spreadDeg = 0.0, spreadM = 0.0;  // the moved deltas' largest disagreement with it
	};

	// One device, not a base station, whose frame moved under it: its own
	// world delta (new world = rotation * old world + translation), exact
	// because it is read off the two worldFromDriver transforms.
	struct Move
	{
		uint32_t id = 0;
		double time = 0.0;
		// When the frame moved: the device's own sample for a move it showed,
		// and for one read off other devices' moves the time the move it stands
		// for first showed, before the device did (a tracker switched off
		// through SteamVR's universe setup is back after it, its frame moved
		// during it).
		double frameTime = 0.0;
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
		double shiftM = 0.0;   // how far the move carried the device
		// Read off other devices' moves: the device was disconnected when its
		// frame moved (returned), or its own pose jumped with the change.
		bool returned = false;
		bool ownJump = false;
		// The frame before and after (worldFromDriver): every device in the
		// frame reports the same pair.
		Eigen::Quaterniond fromRot{ 1, 0, 0, 0 };
		Eigen::Vector3d fromTrans{ 0, 0, 0 };
		Eigen::Quaterniond toRot{ 1, 0, 0, 0 };
		Eigen::Vector3d toTrans{ 0, 0, 0 };
		// Given back by the caller (Hold) at least once.
		bool held = false;
	};

	// A device, not a base station, in a frame its correction did not follow
	// it into: the same pose put in another station's frame, a frame change
	// its own pose jumped with or one it came back from a tracking loss in,
	// with no move reported anywhere leading there within inferSeconds (live
	// 2026-10-03: a body tracker bootstrapped from S-13 while SteamVR moved
	// S-3, then back into S-3's moved frame), or its first tracked sample
	// since the watch started, was reset or forgot it.
	struct Switch
	{
		enum class Cause { Appeared, ReExpressed, OwnJump, Returned };
		uint32_t id = 0;
		double time = 0.0;   // when its frame changed
		Cause cause = Cause::Appeared;
		// The frame before (identity for one that appeared) and after.
		Eigen::Quaterniond fromRot{ 1, 0, 0, 0 };
		Eigen::Vector3d fromTrans{ 0, 0, 0 };
		Eigen::Quaterniond toRot{ 1, 0, 0, 0 };
		Eigen::Vector3d toTrans{ 0, 0, 0 };
		// Given back by the caller (Hold) at least once.
		bool held = false;
	};

	// A frame some device, base station or not, moved with: `from` became `to`.
	struct Transition
	{
		uint64_t seq = 0;
		double time = 0.0;   // of the sample that first showed it
		Eigen::Quaterniond fromRot{ 1, 0, 0, 0 };
		Eigen::Vector3d fromTrans{ 0, 0, 0 };
		Eigen::Quaterniond toRot{ 1, 0, 0, 0 };
		Eigen::Vector3d toTrans{ 0, 0, 0 };
	};

	LighthouseFrameWatch() = default;
	explicit LighthouseFrameWatch(const Config &c) : config(c) {}

	// A sample from a device off the headset side. One that does not track is
	// skipped; a disconnected one that does not track is gone, and keeps its
	// last frame for a move that frame makes before it is back (OpenVR keeps a
	// device's index for the session, so whatever comes back at that index is
	// the same device). A target-system device (hiddenTracking) reported
	// disconnected while it tracks is hidden, not gone (see above); any other
	// device reported disconnected is gone.
	void Note(const protocol::DevicePoseSample &s, double qpcToSeconds, bool baseStation,
		bool hiddenTracking = true)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			return;
		Device &d = devices[s.deviceId];
		const bool tracking = (s.deviceIsConnected || hiddenTracking) && s.poseIsValid &&
			s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK) &&
			IsUsableRingSample(s, qpcToSeconds);
		if (!tracking)
		{
			if (!s.deviceIsConnected)
			{
				if (d.valid)
					d.away = true;
				d.valid = false;
			}
			return;
		}

		RingSampleParts p = UnpackRingSample(s);
		Device now;
		now.valid = true;
		now.baseStation = baseStation;
		now.time = RingSampleTime(s, qpcToSeconds);
		now.wfdRot = p.wfdRot;
		now.wfdTrans = p.wfdTrans;
		now.drvRot = p.drvRot;
		now.drvPos = p.drvPos;
		now.vel = Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]);
		now.angVel = Eigen::Vector3d(s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]);
		now.mark = nextTransition;

		latestTime = (std::max)(latestTime, now.time);
		if (!baseStation && trackingSince > 1e299)
			trackingSince = now.time;
		if (firstSample > 1e299)
			firstSample = now.time;
		if (!first[s.deviceId].seen)
			first[s.deviceId] = { true, baseStation, now.time, now.wfdRot, now.wfdTrans };
		const bool appeared = !d.valid && !d.away;
		const bool changed = (d.valid || d.away) &&
			questcal::WorldFromDriverChanged(d.wfdRot, d.wfdTrans, now.wfdRot, now.wfdTrans);
		if (changed && d.valid)
		{
			const Change c = Classify(s.deviceId, baseStation, d, now);
			if (c.kind == Kind::Moved)
			{
				if (!baseStation)
				{
					Move m{ c.id, c.time, c.time, c.rotation, c.translation, c.shiftM };
					SetFrames(m, d, now);
					Hand(m);
				}
				AddTransition(d, now);
			}
			else if (c.kind == Kind::Unclear && !baseStation)
			{
				Infer(s.deviceId, d, now, /*returned=*/false);
			}
			else if (c.kind == Kind::ReExpressed && !baseStation)
			{
				Switched(s.deviceId, now.time, Switch::Cause::ReExpressed,
					d.wfdRot, d.wfdTrans, now.wfdRot, now.wfdTrans);
			}
			Add(c);
		}
		else if (changed && !baseStation)
		{
			Infer(s.deviceId, d, now, /*returned=*/true);
		}
		else if (appeared && !baseStation)
		{
			Switched(s.deviceId, now.time, Switch::Cause::Appeared,
				Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), now.wfdRot, now.wfdTrans);
		}
		d = now;
		ExpireInferences();
	}

	// When the first device, base stations aside, tracked since the watch
	// started or was reset; far in the future until one has.
	double TrackingSince() const { return trackingSince; }

	// The same for any device, base stations included, and when the first
	// move was recorded: between the two, every frame a device shows is one
	// the watch's start saw unmoved.
	double FirstSampleTime() const { return firstSample; }
	double FirstMoveTime() const { return firstMove; }

	// The frame a device, base station or not, showed in its first tracked
	// sample since the watch started, was reset or forgot it, and when.
	bool FirstFrame(uint32_t id, Eigen::Quaterniond &rotation, Eigen::Vector3d &translation, double &time,
		bool &baseStation) const
	{
		if (id >= vr::k_unMaxTrackedDeviceCount || !first[id].seen)
			return false;
		rotation = first[id].rot;
		translation = first[id].trans;
		time = first[id].time;
		baseStation = first[id].baseStation;
		return true;
	}

	// The frame a device, base stations aside, was last seen tracking in,
	// kept while it is switched off; false for one not seen since the watch
	// started or was reset.
	bool LastFrame(uint32_t id, Eigen::Quaterniond &rotation, Eigen::Vector3d &translation) const
	{
		if (id >= vr::k_unMaxTrackedDeviceCount)
			return false;
		const Device &d = devices[id];
		if (!(d.valid || d.away) || d.baseStation)
			return false;
		rotation = d.wfdRot;
		translation = d.wfdTrans;
		return true;
	}

	// Whether the device's last frame (LastFrame) is the given one.
	bool InFrame(uint32_t id, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation) const
	{
		Eigen::Quaterniond r;
		Eigen::Vector3d t;
		return LastFrame(id, r, t) && !questcal::WorldFromDriverChanged(r, t, rotation, translation);
	}

	// The device has a last frame and no change of it is still waiting to be
	// read off other devices' moves: whatever it is handed next is about a
	// later change.
	bool Settled(uint32_t id) const
	{
		Eigen::Quaterniond r;
		Eigen::Vector3d t;
		if (!LastFrame(id, r, t))
			return false;
		for (const auto &f : inferences)
			if (f.move.id == id)
				return false;
		return true;
	}

	// Every move recorded since the watch started or was reset, oldest first
	// (at most MaxTransitions): whatever device showed it, a base station or a
	// device that is not a target one included.
	const std::vector<Transition> &Record() const { return transitions; }

	// The recorded moves lead from one frame to the other, step by step and
	// recorded in any order (Lead over the whole record).
	bool Leads(const Eigen::Quaterniond &fromRot, const Eigen::Vector3d &fromTrans,
		const Eigen::Quaterniond &toRot, const Eigen::Vector3d &toTrans) const
	{
		if (!questcal::WorldFromDriverChanged(fromRot, fromTrans, toRot, toTrans))
			return false;
		Inference f;
		f.since = 0;
		f.fromRot = fromRot;
		f.fromTrans = fromTrans;
		f.toRot = toRot;
		f.toTrans = toTrans;
		return !Lead(f).empty();
	}

	// Where the device last was: its local position in its last frame.
	bool LastPosition(uint32_t id, Eigen::Vector3d &position) const
	{
		Eigen::Quaterniond r;
		Eigen::Vector3d t;
		if (!LastFrame(id, r, t))
			return false;
		position = r * devices[id].drvPos + t;
		return true;
	}

	// The devices, base stations aside, last seen in the same frame share a
	// group, named by the lowest id among them; -1 for a device with no frame.
	int FrameGroup(uint32_t id) const
	{
		Eigen::Quaterniond r;
		Eigen::Vector3d t;
		if (!LastFrame(id, r, t))
			return -1;
		for (uint32_t other = 0; other < id; ++other)
			if (InFrame(other, r, t))
				return static_cast<int>(other);
		return static_cast<int>(id);
	}

	// The moved devices since the last call, in the order they moved, after
	// the ones given back (Hold). Bounded: once MaxPendingMoves wait untaken,
	// later ones are dropped and TakeMoveOverflow says so.
	std::vector<Move> TakeMoves()
	{
		std::vector<Move> out;
		out.swap(heldMoves);
		out.insert(out.end(), moves.begin(), moves.end());
		moves.clear();
		return out;
	}

	// The switches since the last call, in the order they were settled: at
	// once for a re-expressed or newly seen device, once no move led to its
	// new frame for one that jumped into it or came back in it. The ones
	// given back (Hold) come first, unless the device switched again since:
	// a join reads the frame the device is in now, so the newest switch
	// stands for both. Bounded like the moves: a switch past the bound is
	// dropped, and TakeSwitchOverflow says so.
	std::vector<Switch> TakeSwitches()
	{
		std::vector<Switch> out;
		for (const auto &held : heldSwitches)
			if (std::none_of(switches.begin(), switches.end(), [&](const Switch &s) { return s.id == held.id; }))
				out.push_back(held);
		heldSwitches.clear();
		out.insert(out.end(), switches.begin(), switches.end());
		switches.clear();
		return out;
	}

	// A move or switch the caller cannot act on yet, given back to be handed
	// out again with the next ones: the device's membership in the target
	// side is not known yet (it is decided after the device's first samples
	// arrive), or its join must wait for another change of its frame to be
	// read. Live 2026-10-15: a controller's first sample came before it was
	// known to be a target device, its switch was dropped, and it kept a
	// correction 8 deg off the rest of its frame. A held move counts toward
	// the move bound.
	void Hold(Move m)
	{
		m.held = true;
		if (heldMoves.size() + moves.size() < MaxPendingMoves)
			heldMoves.push_back(m);
		else
			moveOverflow = true;
	}

	void Hold(Switch s)
	{
		s.held = true;
		heldSwitches.erase(std::remove_if(heldSwitches.begin(), heldSwitches.end(),
			[&](const Switch &h) { return h.id == s.id; }), heldSwitches.end());
		heldSwitches.push_back(s);
	}

	// The time of the newest sample seen, -1e300 before any.
	double LatestTime() const { return latestTime; }

    // Overflow invalidates completeness: callers must stop applying the
    // profile until recalibration, rather than silently use partial frames.
    bool TakeMoveOverflow() { const bool out = moveOverflow; moveOverflow = false; return out; }
	// The same for the switches.
	bool TakeSwitchOverflow() { const bool out = switchOverflow; switchOverflow = false; return out; }

	// Reports whose group has closed: a sample arrived more than
	// groupSeconds after its first change.
	std::vector<Report> Flush()
	{
		if (!open.empty() && latestTime - open.front().time > config.groupSeconds)
			Close();
		std::vector<Report> out;
		out.swap(closed);
		return out;
	}

	void Reset()
	{
		for (auto &d : devices)
			d = Device{};
		open.clear();
		closed.clear();
		moves.clear();
		heldMoves.clear();
        moveOverflow = false;
		switches.clear();
		heldSwitches.clear();
		switchOverflow = false;
		transitions.clear();
		inferences.clear();
		latestTime = -1e300;
		trackingSince = 1e300;
		firstSample = 1e300;
		firstMove = 1e300;
		for (auto &f : first)
			f = First{};
	}

	// A reused slot must not bridge two physical devices. Other devices' frame
	// transitions remain useful to infer a returning tracker's frame.
	void ForgetDevice(uint32_t id)
	{
		if (id < vr::k_unMaxTrackedDeviceCount)
		{
			devices[id] = Device{};
			first[id] = First{};
		}
		for (auto *list : { &moves, &heldMoves })
			list->erase(std::remove_if(list->begin(), list->end(),
				[id](const Move &m) { return m.id == id; }), list->end());
		for (auto *list : { &switches, &heldSwitches })
			list->erase(std::remove_if(list->begin(), list->end(),
				[id](const Switch &s) { return s.id == id; }), list->end());
		inferences.erase(std::remove_if(inferences.begin(), inferences.end(),
			[id](const Inference &i) { return i.move.id == id; }), inferences.end());
	}

	// A move large enough for the session log: a centimeter at a device or a
	// tenth of a degree. Smaller ones (a station's pose refined in place)
	// belong in the detailed log with the re-expressions.
	static bool Notable(const Report &r)
	{
		return r.moved > 0 && NotableMove(r.largestShiftM, r.yawDeg, r.tiltDeg);
	}

	// The same bounds for a single move.
	static bool NotableMove(double shiftM, double yawDeg, double tiltDeg)
	{
		return shiftM >= 0.01 || yawDeg >= 0.1 || tiltDeg >= 0.1;
	}

	// One log line for a report.
	static std::string Describe(const Report &r, uint32_t headsetTrackerId)
	{
		char buf[320];
		std::string extra;
		if (r.moved > 0 && r.reExpressed > 0)
			extra += ", " + std::to_string(r.reExpressed) + " re-expressed without moving";
		if (r.unclear > 0)
			extra += ", " + std::to_string(r.unclear) + " whose own pose jumped as well";
		if (r.moved > 0)
		{
			const bool tracker = std::find(r.movedIds.begin(), r.movedIds.end(), headsetTrackerId) !=
				r.movedIds.end();
			char agree[96] = "";
			if (r.moved > 1)
				std::snprintf(agree, sizeof agree, "; their deltas agree within %.2f deg / %.1f cm",
					r.spreadDeg, r.spreadM * 100.0);
			std::snprintf(buf, sizeof buf,
				"lighthouse frame moved: %d device(s) (%d base station(s)%s), yaw %.2f deg, tilt %.2f deg, "
				"largest device shift %.1f cm%s%s",
				r.moved, r.movedBaseStations, tracker ? ", the headset tracker among them" : "",
				r.yawDeg, r.tiltDeg, r.largestShiftM * 100.0, agree, extra.c_str());
		}
		else if (r.unclear > 0)
		{
			std::snprintf(buf, sizeof buf,
				"lighthouse frame changed on %d device(s) whose own pose jumped as well, %d re-expressed without moving",
				r.unclear, r.reExpressed);
		}
		else
		{
			std::snprintf(buf, sizeof buf,
				"lighthouse: %d device(s) re-expressed their pose in another frame without moving", r.reExpressed);
		}
		return buf;
	}

private:
	struct Device
	{
		bool valid = false;
		bool baseStation = false;
		bool away = false;     // disconnected after a valid sample, which is kept
		uint64_t mark = 0;     // the next transition's sequence number at this sample
		double time = 0.0;
		Eigen::Quaterniond wfdRot{ 1, 0, 0, 0 };
		Eigen::Vector3d wfdTrans{ 0, 0, 0 };
		Eigen::Quaterniond drvRot{ 1, 0, 0, 0 };
		Eigen::Vector3d drvPos{ 0, 0, 0 };
		Eigen::Vector3d vel{ 0, 0, 0 }, angVel{ 0, 0, 0 };
	};

	struct Change
	{
		uint32_t id = 0;
		bool baseStation = false;
		Kind kind = Kind::Unclear;
		double time = 0.0;
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
		double shiftM = 0.0;
	};

	bool Unchanged(const Eigen::Quaterniond &r0, const Eigen::Vector3d &p0,
		const Eigen::Quaterniond &r1, const Eigen::Vector3d &p1) const
	{
		return (p1 - p0).norm() <= config.restPositionM &&
			r1.angularDistance(r0) <= config.restRotationRad;
	}

	Change Classify(uint32_t id, bool baseStation, const Device &was, const Device &is) const
	{
		ringpose::DriverLocalPoseSample localWas{ was.time, was.drvRot, was.drvPos, was.vel, was.angVel };
		ringpose::DriverLocalPoseSample localIs{ is.time, is.drvRot, is.drvPos, is.vel, is.angVel };
		const bool localKept = ringpose::IsDriverLocalPoseContinuous(localWas, localIs) ||
			Unchanged(was.drvRot, was.drvPos, is.drvRot, is.drvPos);

		const ringpose::DriverLocalPoseSample worldWas =
			ringpose::ComposeWithWorldFromDriver(localWas, was.wfdRot, was.wfdTrans);
		const ringpose::DriverLocalPoseSample worldIs =
			ringpose::ComposeWithWorldFromDriver(localIs, is.wfdRot, is.wfdTrans);
		const bool worldKept = ringpose::IsDriverLocalPoseContinuous(worldWas, worldIs) ||
			Unchanged(worldWas.rotation, worldWas.position, worldIs.rotation, worldIs.position);

		Change c;
		c.id = id;
		c.baseStation = baseStation;
		c.time = is.time;
		c.kind = worldKept ? Kind::ReExpressed : localKept ? Kind::Moved : Kind::Unclear;
		c.rotation = (is.wfdRot * was.wfdRot.conjugate()).normalized();
		c.translation = is.wfdTrans - c.rotation * was.wfdTrans;
		// The current local pose in both frames: how far the change alone
		// moved the device.
		c.shiftM = ((is.wfdRot * is.drvPos + is.wfdTrans) - (was.wfdRot * is.drvPos + was.wfdTrans)).norm();
		return c;
	}

	void Add(const Change &c)
	{
		if (!open.empty() && c.time - open.front().time > config.groupSeconds)
			Close();
		open.push_back(c);
	}

	void Close()
	{
		Report r;
		r.time = open.front().time;
		std::set<uint32_t> moved, movedStations, reExpressed, unclear;
		const Change *reference = nullptr;
		for (const auto &c : open)
		{
			if (c.kind == Kind::Moved)
			{
				moved.insert(c.id);
				if (c.baseStation)
					movedStations.insert(c.id);
				if (!reference || (c.baseStation && !reference->baseStation))
					reference = &c;
				r.largestShiftM = (std::max)(r.largestShiftM, c.shiftM);
			}
			else if (c.kind == Kind::ReExpressed)
				reExpressed.insert(c.id);
			else
				unclear.insert(c.id);
		}
		r.moved = static_cast<int>(moved.size());
		r.movedBaseStations = static_cast<int>(movedStations.size());
		r.reExpressed = static_cast<int>(reExpressed.size());
		r.unclear = static_cast<int>(unclear.size());
		r.movedIds.assign(moved.begin(), moved.end());
		if (reference)
		{
			r.rotation = reference->rotation;
			r.translation = reference->translation;
			double tilt = 0.0;
			r.yawDeg = std::abs(questcal::SignedYawRadians(r.rotation, &tilt)) * 180.0 / questcal::Pi;
			r.tiltDeg = tilt * 180.0 / questcal::Pi;
			for (const auto &c : open)
			{
				if (c.kind != Kind::Moved)
					continue;
				r.spreadDeg = (std::max)(r.spreadDeg,
					c.rotation.angularDistance(r.rotation) * 180.0 / 3.14159265358979323846);
				r.spreadM = (std::max)(r.spreadM, (c.translation - r.translation).norm());
			}
		}
		closed.push_back(r);
		open.clear();
	}

	// The first transition a change of `was`'s frame can have come from: the
	// ones recorded since its last sample, and the ones shown by other
	// devices up to lagSeconds before it, which it reached late.
	uint64_t Since(const Device &was) const
	{
		uint64_t since = was.mark;
		for (auto t = transitions.rbegin(); t != transitions.rend(); ++t)
		{
			if (t->seq >= since)
				continue;
			if (t->time < was.time - config.lagSeconds)
				break;
			since = t->seq;
		}
		return since;
	}

	// A device's frame change waiting for other devices to show it was a move.
	struct Inference
	{
		Move move;
		uint64_t since = 0;
		Eigen::Quaterniond fromRot{ 1, 0, 0, 0 };
		Eigen::Vector3d fromTrans{ 0, 0, 0 };
		Eigen::Quaterniond toRot{ 1, 0, 0, 0 };
		Eigen::Vector3d toTrans{ 0, 0, 0 };
		Eigen::Vector3d at{ 0, 0, 0 };   // the device's local position, for each move's shift
		// When its first change came: changes merged into it wait no longer
		// than that one would have.
		double firstChange = 0.0;
		// With changes merged into it, the newest one on its own (its frame
		// before, since and cause) and when the rest came and why: a whole no
		// move leads can still have its newest part led (Expire).
		bool merged = false;
		Eigen::Quaterniond newestFromRot{ 1, 0, 0, 0 };
		Eigen::Vector3d newestFromTrans{ 0, 0, 0 };
		uint64_t newestSince = 0;
		bool newestReturned = false;
		double restTime = 0.0;
		bool restReturned = false;
	};

	void AddTransition(const Device &was, const Device &is)
	{
		// Every device in the frame reports the same one.
		const size_t recent = (std::min)(transitions.size(), size_t(16));
		for (size_t i = transitions.size() - recent; i < transitions.size(); ++i)
		{
			const Transition &t = transitions[i];
			if (!questcal::WorldFromDriverChanged(t.fromRot, t.fromTrans, was.wfdRot, was.wfdTrans) &&
				!questcal::WorldFromDriverChanged(t.toRot, t.toTrans, is.wfdRot, is.wfdTrans))
				return;
		}
		if (transitions.size() >= MaxTransitions)
			transitions.erase(transitions.begin());
		transitions.push_back({ nextTransition++, is.time, was.wfdRot, was.wfdTrans, is.wfdRot, is.wfdTrans });
		firstMove = (std::min)(firstMove, is.time);

		for (size_t i = 0; i < inferences.size();)
		{
			const std::vector<Transition> lead = Lead(inferences[i]);
			if (!lead.empty())
			{
				HandLead(inferences[i], lead);
				inferences.erase(inferences.begin() + static_cast<std::ptrdiff_t>(i));
			}
			else
			{
				++i;
			}
		}
	}

	// The moves reported since `since` that lead from the old frame to the new
	// one, step by step (that frame moved, the station under it re-solved), or
	// none. Any chain of them counts, recorded in any order: another device can
	// show a frame between that the station's own samples skip, the station
	// move on past the new frame before the device shows it, and a device a
	// solve ahead show the second step before one behind shows the first (a
	// lead missed any of these ways settled an exact move as a switch).
	std::vector<Transition> Lead(const Inference &f) const
	{
		// Every frame the moves so far reach, with the move that first reached
		// it and the frame that move came from.
		struct Reached
		{
			Eigen::Quaterniond rot;
			Eigen::Vector3d trans;
			const Transition *by;
			size_t parent;
		};
		std::vector<Reached> reached{ { f.fromRot, f.fromTrans, nullptr, 0 } };
		auto find = [&reached](const Eigen::Quaterniond &rot, const Eigen::Vector3d &trans)
		{
			for (size_t i = 0; i < reached.size(); ++i)
				if (!questcal::WorldFromDriverChanged(reached[i].rot, reached[i].trans, rot, trans))
					return i;
			return reached.size();
		};
		// Pass over the record until a pass reaches no new frame: a step can be
		// recorded before the one it continues.
		for (bool grew = true; grew;)
		{
			grew = false;
			for (const auto &t : transitions)
			{
				if (t.seq < f.since)
					continue;
				const size_t from = find(t.fromRot, t.fromTrans);
				if (from == reached.size() || find(t.toRot, t.toTrans) != reached.size())
					continue;
				reached.push_back({ t.toRot, t.toTrans, &t, from });
				grew = true;
				if (questcal::WorldFromDriverChanged(t.toRot, t.toTrans, f.toRot, f.toTrans))
					continue;
				std::vector<Transition> chain;
				for (size_t i = reached.size() - 1; reached[i].by; i = reached[i].parent)
					chain.push_back(*reached[i].by);
				std::reverse(chain.begin(), chain.end());
				return chain;
			}
		}
		return {};
	}

	// The inferred change handed out move by move, each with the time it first
	// showed, so the caller judges each against SteamVR's universe setup on its
	// own. Followed in turn they compose to the device's whole change.
	void HandLead(const Inference &f, const std::vector<Transition> &lead)
	{
		for (const auto &t : lead)
		{
			Move m = f.move;
			m.frameTime = t.time;
			m.rotation = (t.toRot * t.fromRot.conjugate()).normalized();
			m.translation = t.toTrans - m.rotation * t.fromTrans;
			m.shiftM = ((t.toRot * f.at + t.toTrans) - (t.fromRot * f.at + t.fromTrans)).norm();
			m.fromRot = t.fromRot;
			m.fromTrans = t.fromTrans;
			m.toRot = t.toRot;
			m.toTrans = t.toTrans;
			Hand(m);
		}
	}

	void Infer(uint32_t id, const Device &was, const Device &is, bool returned)
	{
		Inference f;
		f.move.id = id;
		f.move.time = is.time;
		f.move.frameTime = is.time;
		f.move.returned = returned;
		f.since = Since(was);
		f.fromRot = was.wfdRot;
		f.fromTrans = was.wfdTrans;
		f.toRot = is.wfdRot;
		f.toTrans = is.wfdTrans;
		f.at = is.drvPos;
		f.firstChange = is.time;
		// A change of the same device still waiting for its lead is part of
		// this one: the two wait as one change, from the older one's frame and
		// no longer than the older one would have, and a lead for both lets the
		// device follow both. Settled on its own, the older one was a switch
		// whose join had the device give or take a correction short of the
		// move the lead for it, coming a moment later, would have followed (the
		// headset tracker after two re-solves in a row, replayed: 18 and 43 mm
		// at its frame's devices); dropped, it left the device's correction
		// short of it with nothing said (live 2026-10-15: a 17.8 cm move lost
		// this way, 4 s before it expired). One that ended in another frame was
		// left by a move the device followed since, or by a re-expression: a
		// lead for both would carry that move again (replayed: 24.4 mm at the
		// device), and a lead for the older one alone would come after the
		// newer move the device followed, which Follow refuses. It is settled,
		// as it was before changes waited together.
		for (const auto &g : inferences)
			if (g.move.id == id)
			{
				if (questcal::WorldFromDriverChanged(g.toRot, g.toTrans, was.wfdRot, was.wfdTrans))
				{
					Settle(g);
					continue;
				}
				f.merged = true;
				f.newestFromRot = was.wfdRot;
				f.newestFromTrans = was.wfdTrans;
				f.newestSince = Since(was);
				f.newestReturned = returned;
				f.restTime = g.move.time;
				f.restReturned = g.move.returned;
				f.since = (std::min)(f.since, g.since);
				f.fromRot = g.fromRot;
				f.fromTrans = g.fromTrans;
				f.firstChange = (std::min)(f.firstChange, g.firstChange);
				f.move.returned = f.move.returned || g.move.returned;
			}
		inferences.erase(std::remove_if(inferences.begin(), inferences.end(),
			[id](const Inference &g) { return g.move.id == id; }), inferences.end());
		f.move.ownJump = !f.move.returned;
		f.move.rotation = (f.toRot * f.fromRot.conjugate()).normalized();
		f.move.translation = f.toTrans - f.move.rotation * f.fromTrans;
		f.move.shiftM = ((f.toRot * f.at + f.toTrans) - (f.fromRot * f.at + f.fromTrans)).norm();
		f.move.fromRot = f.fromRot;
		f.move.fromTrans = f.fromTrans;
		f.move.toRot = f.toRot;
		f.move.toTrans = f.toTrans;
		const std::vector<Transition> lead = Lead(f);
		if (!lead.empty())
			HandLead(f, lead);
		else if (inferences.size() < MaxInferences)
			inferences.push_back(f);
		else
			Expire(f);
	}

	// No move led to the inferred change's new frame: the device switched.
	void Settle(const Inference &f)
	{
		Switched(f.move.id, f.move.time, f.move.returned ? Switch::Cause::Returned : Switch::Cause::OwnJump,
			f.fromRot, f.fromTrans, f.toRot, f.toTrans);
	}

	void Switched(uint32_t id, double time, Switch::Cause cause,
		const Eigen::Quaterniond &fromRot, const Eigen::Vector3d &fromTrans,
		const Eigen::Quaterniond &toRot, const Eigen::Vector3d &toTrans)
	{
		if (switches.size() >= MaxPendingMoves)
		{
			switchOverflow = true;
			return;
		}
		Switch s;
		s.id = id;
		s.time = time;
		s.cause = cause;
		s.fromRot = fromRot;
		s.fromTrans = fromTrans;
		s.toRot = toRot;
		s.toTrans = toTrans;
		switches.push_back(s);
	}

	static void SetFrames(Move &m, const Device &was, const Device &is)
	{
		m.fromRot = was.wfdRot;
		m.fromTrans = was.wfdTrans;
		m.toRot = is.wfdRot;
		m.toTrans = is.wfdTrans;
	}

	void Hand(const Move &m)
	{
		if (moves.size() + heldMoves.size() < MaxPendingMoves)
			moves.push_back(m);
        else
            moveOverflow = true;
	}

	void ExpireInferences()
	{
		const double cutoff = latestTime - config.inferSeconds;
		for (const auto &f : inferences)
			if (f.firstChange < cutoff)
				Expire(f);
		inferences.erase(std::remove_if(inferences.begin(), inferences.end(),
			[cutoff](const Inference &f) { return f.firstChange < cutoff; }), inferences.end());
	}

	// An inference out of time, or of room: settled as a switch, unless changes
	// were merged into it and the moves recorded lead the newest one alone.
	// That one is led, and the rest settled: an own jump into another station's
	// frame, which no move can lead, took the newest change's lead with it when
	// merged (replayed: the headset tracker's correction 55.2 mm short of a
	// re-solve it showed). Waiting the whole time first leaves the older
	// changes their own leads, which can come a moment later.
	void Expire(const Inference &f)
	{
		if (f.merged)
		{
			Inference newest = f;
			newest.merged = false;
			newest.since = f.newestSince;
			newest.fromRot = f.newestFromRot;
			newest.fromTrans = f.newestFromTrans;
			newest.move.returned = f.newestReturned;
			newest.move.ownJump = !f.newestReturned;
			const std::vector<Transition> lead = Lead(newest);
			if (!lead.empty())
			{
				Switched(f.move.id, f.restTime, f.restReturned ? Switch::Cause::Returned : Switch::Cause::OwnJump,
					f.fromRot, f.fromTrans, f.newestFromRot, f.newestFromTrans);
				HandLead(newest, lead);
				return;
			}
		}
		Settle(f);
	}

	static constexpr size_t MaxPendingMoves = 256;
	static constexpr size_t MaxTransitions = 512;
	static constexpr size_t MaxInferences = 64;

	Config config;
	Device devices[vr::k_unMaxTrackedDeviceCount];
	std::vector<Change> open;
	std::vector<Report> closed;
	std::vector<Move> moves, heldMoves;
    bool moveOverflow = false;
	std::vector<Switch> switches, heldSwitches;
	bool switchOverflow = false;
	std::vector<Transition> transitions;
	std::vector<Inference> inferences;
	uint64_t nextTransition = 0;
	double latestTime = -1e300;
	double trackingSince = 1e300;
	double firstSample = 1e300;
	double firstMove = 1e300;
	struct First
	{
		bool seen = false;
		bool baseStation = false;
		double time = 0.0;
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
	};
	First first[vr::k_unMaxTrackedDeviceCount];
};
