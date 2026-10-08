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
// frame it left, and the caller gives it the one of the devices already in
// the new frame (TrackerFrameCorrections::Join).
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
					Move m{ c.id, c.time, c.rotation, c.translation, c.shiftM };
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

	// The moved devices since the last call, in the order they moved. Bounded:
	// once MaxPendingMoves wait untaken, later ones are dropped.
	std::vector<Move> TakeMoves()
	{
		std::vector<Move> out;
		out.swap(moves);
		return out;
	}

	// The switches since the last call, in the order they were settled: at
	// once for a re-expressed or newly seen device, once no move led to its
	// new frame for one that jumped into it or came back in it. Bounded like
	// the moves; a switch past the bound is dropped, which leaves the device
	// with the correction it had, as before switches were reported.
	std::vector<Switch> TakeSwitches()
	{
		std::vector<Switch> out;
		out.swap(switches);
		return out;
	}

    // Overflow invalidates completeness: callers must stop applying the
    // profile until recalibration, rather than silently use partial frames.
    bool TakeMoveOverflow() { const bool out = moveOverflow; moveOverflow = false; return out; }

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
        moveOverflow = false;
		switches.clear();
		transitions.clear();
		inferences.clear();
		latestTime = -1e300;
		trackingSince = 1e300;
	}

	// A reused slot must not bridge two physical devices. Other devices' frame
	// transitions remain useful to infer a returning tracker's frame.
	void ForgetDevice(uint32_t id)
	{
		if (id < vr::k_unMaxTrackedDeviceCount)
			devices[id] = Device{};
		moves.erase(std::remove_if(moves.begin(), moves.end(),
			[id](const Move &m) { return m.id == id; }), moves.end());
		switches.erase(std::remove_if(switches.begin(), switches.end(),
			[id](const Switch &s) { return s.id == id; }), switches.end());
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

	// A frame some device, base station or not, moved with: `from` became `to`.
	struct Transition
	{
		uint64_t seq = 0;
		Eigen::Quaterniond fromRot{ 1, 0, 0, 0 };
		Eigen::Vector3d fromTrans{ 0, 0, 0 };
		Eigen::Quaterniond toRot{ 1, 0, 0, 0 };
		Eigen::Vector3d toTrans{ 0, 0, 0 };
	};

	// A device's frame change waiting for other devices to show it was a move.
	struct Inference
	{
		Move move;
		uint64_t since = 0;
		Eigen::Quaterniond fromRot{ 1, 0, 0, 0 };
		Eigen::Vector3d fromTrans{ 0, 0, 0 };
		Eigen::Quaterniond toRot{ 1, 0, 0, 0 };
		Eigen::Vector3d toTrans{ 0, 0, 0 };
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
		transitions.push_back({ nextTransition++, was.wfdRot, was.wfdTrans, is.wfdRot, is.wfdTrans });

		for (size_t i = 0; i < inferences.size();)
		{
			if (Leads(inferences[i]))
			{
				Hand(inferences[i].move);
				inferences.erase(inferences.begin() + static_cast<std::ptrdiff_t>(i));
			}
			else
			{
				++i;
			}
		}
	}

	// The moves reported since `since`, followed from the old frame, end on
	// the new one: that frame moved, the station under it re-solved.
	bool Leads(const Inference &f) const
	{
		Eigen::Quaterniond rot = f.fromRot;
		Eigen::Vector3d trans = f.fromTrans;
		bool moved = false;
		for (const auto &t : transitions)
		{
			if (t.seq < f.since ||
				questcal::WorldFromDriverChanged(t.fromRot, t.fromTrans, rot, trans))
				continue;
			rot = t.toRot;
			trans = t.toTrans;
			moved = true;
		}
		return moved && !questcal::WorldFromDriverChanged(rot, trans, f.toRot, f.toTrans);
	}

	void Infer(uint32_t id, const Device &was, const Device &is, bool returned)
	{
		Inference f;
		f.move.id = id;
		f.move.time = is.time;
		f.move.rotation = (is.wfdRot * was.wfdRot.conjugate()).normalized();
		f.move.translation = is.wfdTrans - f.move.rotation * was.wfdTrans;
		f.move.shiftM = ((is.wfdRot * is.drvPos + is.wfdTrans) - (was.wfdRot * is.drvPos + was.wfdTrans)).norm();
		f.move.returned = returned;
		f.move.ownJump = !returned;
		SetFrames(f.move, was, is);
		f.since = was.mark;
		f.fromRot = was.wfdRot;
		f.fromTrans = was.wfdTrans;
		f.toRot = is.wfdRot;
		f.toTrans = is.wfdTrans;
		// A newer change of the same device supersedes one still waiting.
		inferences.erase(std::remove_if(inferences.begin(), inferences.end(),
			[id](const Inference &g) { return g.move.id == id; }), inferences.end());
		if (Leads(f))
			Hand(f.move);
		else if (inferences.size() < MaxInferences)
			inferences.push_back(f);
		else
			Settle(f);
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
			return;
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
		if (moves.size() < MaxPendingMoves)
			moves.push_back(m);
        else
            moveOverflow = true;
	}

	void ExpireInferences()
	{
		const double cutoff = latestTime - config.inferSeconds;
		for (const auto &f : inferences)
			if (f.move.time < cutoff)
				Settle(f);
		inferences.erase(std::remove_if(inferences.begin(), inferences.end(),
			[cutoff](const Inference &f) { return f.move.time < cutoff; }), inferences.end());
	}

	static constexpr size_t MaxPendingMoves = 256;
	static constexpr size_t MaxTransitions = 512;
	static constexpr size_t MaxInferences = 64;

	Config config;
	Device devices[vr::k_unMaxTrackedDeviceCount];
	std::vector<Change> open;
	std::vector<Report> closed;
	std::vector<Move> moves;
    bool moveOverflow = false;
	std::vector<Switch> switches;
	std::vector<Transition> transitions;
	std::vector<Inference> inferences;
	uint64_t nextTransition = 0;
	double latestTime = -1e300;
	double trackingSince = 1e300;
};
