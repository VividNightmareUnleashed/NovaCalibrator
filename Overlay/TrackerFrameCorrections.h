#pragma once

#include "LighthouseFrameWatch.h"
#include "ProfileRecord.h"
#include "FrameRecovery.h"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace questcal
{

// Session-local normalization into the target world in which the profile was
// measured. It is independent of base/field generations: a shared calibration
// update must never overwrite one tracker's accumulated frame correction.
class TrackerFrameCorrections
{
public:
	using Frames = std::array<protocol::FrameCorrection, vr::k_unMaxTrackedDeviceCount>;

	void Reset() { *this = TrackerFrameCorrections{}; }
	// The watch starts over after a restore (the epoch it bumps): each device
	// shows its frame again, and the start anchors it with the restored
	// correction (Update).
	void Restore(const Frames &saved, const FrameSerialKeys &keys)
	{
		frames = saved;
		restoredKeys = keys;
		usableAfter.fill(-1e300);
		table.clear();
		fresh = false;
	}

	// The corrections a new frame session resets, with the driver session they
	// were synchronized with and their profile's key. A recovery answered by
	// that same driver session gives these back (Recover): they hold every
	// move followed, and the driver's checkpoint misses one whose
	// synchronization was still waiting when the recovery request replaced it
	// (replayed: 6.2 cm at the tracker).
	struct Kept
	{
		uint64_t driverSession = 0;
		uint64_t profileKey = 0;
		Frames frames{};
		FrameSerialKeys keys{};
	};
	Kept Keep(uint64_t driverSession, uint64_t profileKey) const
	{
		Kept kept;
		if (driverSession == 0 || profileKey == 0)
			return kept;
		kept.driverSession = driverSession;
		kept.profileKey = profileKey;
		kept.frames = frames;
		kept.keys = restoredKeys;
		return kept;
	}

	enum class Recovered { None, Kept, Checkpoint };
	// A recovery's answer: the kept corrections when it came from their driver
	// session for their profile, the driver's checkpoint when it has one for
	// the profile, or nothing (the corrections start over). A device bound
	// since to another identity than the correction's starts over too.
	Recovered Recover(const Kept &kept, uint64_t answerSession, uint64_t profileKey, bool checkpointFound,
		uint64_t checkpointProfileKey, const Frames &checkpoint, const FrameSerialKeys &checkpointKeys)
	{
		const bool own = kept.driverSession != 0 && kept.driverSession == answerSession && profileKey != 0 &&
			kept.profileKey == profileKey;
		if (!own && !(checkpointFound && checkpointProfileKey == profileKey))
			return Recovered::None;
		Frames recovered = own ? kept.frames : checkpoint;
		FrameSerialKeys keys = own ? kept.keys : checkpointKeys;
		for (uint32_t id = 0; id < recovered.size(); ++id)
			if (restoredKeys[id] != 0 && keys[id] != restoredKeys[id])
			{
				recovered[id] = {};
				keys[id] = restoredKeys[id];
			}
		Restore(recovered, keys);
		return own ? Recovered::Kept : Recovered::Checkpoint;
	}

	// OpenVR normally keeps indices for a server session. Still retire an old
	// correction if a known serial changes; a failed property read is not a
	// new identity and must not erase a returning tracker's correction.
	bool Bind(uint32_t id, const std::string &serial)
	{
		if (id >= frames.size() || serial.empty() || serials[id] == serial)
			return false;
		const uint64_t key = FrameIdentityKey(serial);
		const bool replaced = (!serials[id].empty() && serials[id] != serial) ||
			(restoredKeys[id] != 0 && restoredKeys[id] != key);
		bool relocated = false;
		for (uint32_t old = 0; old < frames.size(); ++old)
		{
			if (old == id || restoredKeys[old] != key) continue;
			frames[id] = frames[old];
			usableAfter[id] = usableAfter[old];
			frames[old] = {};
			usableAfter[old] = -1e300;
			serials[old].clear();
			restoredKeys[old] = 0;
			relocated = true;
			break;
		}
		serials[id] = serial;
		restoredKeys[id] = key;
		if (replaced && !relocated)
		{
			frames[id] = {};
			usableAfter[id] = -1e300;
		}
		return replaced || relocated;
	}

	// One device's correction carried across a move it showed, on its own: a
	// measurement's target between its frame's corrections (Update's `held`).
	bool Follow(const LighthouseFrameWatch::Move &move)
	{
		if (move.id >= frames.size() || !std::isfinite(move.time) || Parked(move.id) ||
			move.time < usableAfter[move.id] || !Compose(frames[move.id], move.rotation, move.translation))
			return false;
		// The monitor and solver drain separately. Discard samples preceding the
		// most recent move, including a move inferred after a tracker returned,
		// rather than applying the latest correction to the old frame.
		usableAfter[move.id] = move.time;
		return true;
	}

	// N' F = N: a correction carried across a frame move (new world =
	// rotation * old world + translation). Translation here is unscaled; C
	// applies its scale later.
	static bool Compose(protocol::FrameCorrection &frame, const Eigen::Quaterniond &moveRotation,
		const Eigen::Vector3d &moveTranslation)
	{
		if (!IsValidCalibrationTransform(moveRotation, moveTranslation, 1.0))
			return false;
		const Eigen::Quaterniond rotation(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z);
		const Eigen::Vector3d translation(frame.translation.v);
		const Eigen::Quaterniond nextRotation = (rotation * moveRotation.conjugate()).normalized();
		const Eigen::Vector3d nextTranslation = translation - nextRotation * moveTranslation;
		if (!IsValidCalibrationTransform(nextRotation, nextTranslation, 1.0))
			return false;
		frame.rotation = { nextRotation.w(), nextRotation.x(), nextRotation.y(), nextRotation.z() };
		for (int axis = 0; axis < 3; ++axis)
			frame.translation.v[axis] = nextTranslation(axis);
		return true;
	}

	bool Normalize(uint32_t id, PoseSample &sample) const
	{
		if (id >= frames.size() || sample.time < usableAfter[id])
			return false;
		ApplyToPose(id, sample);
		return true;
	}

	// For an already checked sample on another clock (runtime-pose collection).
	void ApplyToPose(uint32_t id, PoseSample &sample) const
	{
		if (id >= frames.size())
			return;
		const auto &frame = frames[id];
		const Eigen::Quaterniond rotation(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z);
		sample.pos = rotation * sample.pos + Eigen::Vector3d(frame.translation.v);
		sample.rot = (rotation * sample.rot).normalized();
		sample.vel = rotation * sample.vel;
		sample.angVel = rotation * sample.angVel;
	}

	const Frames &Snapshot() const { return frames; }
	// A solve C measured raw poses in one device's frame. Keep the shared
	// calibration in the existing normalized space: C_normalized = C N^-1.
	static bool ExpressCalibration(const Eigen::Quaterniond &n, const Eigen::Vector3d &t,
		Eigen::Quaterniond &rotation, Eigen::Vector3d &translation, double scale)
	{
		const Eigen::Quaterniond nextRotation = (rotation * n.conjugate()).normalized();
		const Eigen::Vector3d nextTranslation = translation - scale * (nextRotation * t);
		if (!IsValidCalibrationTransform(nextRotation, nextTranslation, scale)) return false;
		rotation = nextRotation;
		translation = nextTranslation;
		return true;
	}
	static bool ExpressCalibration(const protocol::FrameCorrection &frame,
		Eigen::Quaterniond &rotation, Eigen::Vector3d &translation, double scale)
	{
		return ExpressCalibration(Eigen::Quaterniond(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z),
			Eigen::Vector3d(frame.translation.v), rotation, translation, scale);
	}
	// ExpressCalibration with the N captured at a run's first sample holds only
	// if every move the run carried back to its start frame (toStart) was also
	// followed here: the target's N is then N_start o toStart. A move left alone
	// (SteamVR still placing its stations) or refused would put the result off
	// by that move.
	static bool FollowedRun(const Eigen::Quaterniond &startRotation, const Eigen::Vector3d &startTranslation,
		const Eigen::Quaterniond &toStartRotation, const Eigen::Vector3d &toStartTranslation,
		const protocol::FrameCorrection &now)
	{
		const Eigen::Quaterniond expectedRotation = (startRotation * toStartRotation).normalized();
		const Eigen::Vector3d expectedTranslation = startRotation * toStartTranslation + startTranslation;
		return !questcal::WorldFromDriverChanged(expectedRotation, expectedTranslation,
			Eigen::Quaterniond(now.rotation.w, now.rotation.x, now.rotation.y, now.rotation.z),
			Eigen::Vector3d(now.translation.v));
	}
	// Frame-anchored corrections. A correction belongs to a lighthouse frame,
	// not to a device: the devices reported against one base station share
	// SteamVR's geometry, so a difference between their corrections can only
	// be a move one of them missed. Each frame a calibrated device was seen in
	// has one entry, and every calibrated device takes its frame's. An entry is
	// anchored when it is derived through recorded moves from a frame whose
	// correction is known: a base station's at a fresh start, or a
	// recalibration's target's. It is provisional when a device carried its own
	// correction into a frame no anchored one reaches yet. An anchored entry is
	// never replaced but by a later recalibration (Anchor); a provisional one
	// gives way to an anchored one, or to one with an older root.
	//
	// The joins this replaces decided once, at a moment, whose correction a
	// device that came into a frame took, and whether the right one was there
	// by then depended on the order the devices arrived in. Each round of wait
	// and order rules left a narrower race (replayed on the last of them: two
	// devices that both moved on into a frame, 18.1 and 43.1 mm; a frame the
	// station's record steps over, 43.1 mm). An entry is derived from the
	// record whenever its moves show, in whatever order, so a device a solve
	// behind is corrected by the frame it still shows, and one that arrives
	// late finds the right correction there.
	struct Root
	{
		// Oldest first: the session start or a recalibration; a frame a device
		// carried its own correction into; a change no move leads (its pose
		// jumping with it, or back from a loss into a frame since moved).
		enum class Kind { Start, Carried, Unclear };
		Kind kind = Kind::Start;
		// Carried: the frame the correction was carried into.
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
	};
	struct Entry
	{
		// The frame (worldFromDriver): the newest one a device in it showed,
		// so a station refined by less than WorldFromDriverChanged's tolerance
		// stays the same frame, as it does for the watch.
		Eigen::Quaterniond rot{ 1, 0, 0, 0 };
		Eigen::Vector3d trans{ 0, 0, 0 };
		protocol::FrameCorrection correction;
		bool anchored = false;
		// The recalibration that anchored it (Anchor); 0 for the session start.
		uint64_t generation = 0;
		Root root;
		// A derivation that disagreed with this anchored entry was seen.
		bool disagreed = false;
	};

	// How long after the watch's first sample a frame shown for the first time
	// is still the session start's, unless a move was recorded before it.
	static constexpr double StartSeconds = 5.0;

	// A device whose correction an update changed.
	struct Changed
	{
		enum class How
		{
			Moved,      // it showed a move and took the new frame's correction
			Switched,   // it came into a frame with no move and took its correction
			Replaced,   // its frame's entry gave way to an anchored one or an older root's
			Anchored,   // a recalibration anchored its frame anew (Anchor)
		};
		uint32_t id = 0;
		How how = How::Replaced;
		protocol::FrameCorrection before;
	};
	struct UpdateResult
	{
		std::vector<Changed> changed;
		// Devices with a move no correction can follow (an unsafe composition).
		std::vector<uint32_t> refused;
	};

	// One drain of the watch: the target devices' moves and switches (the
	// caller holds back those of devices not yet known to be target ones), and
	// the watch's record of every move. `held` is a device whose correction
	// stays as a measurement began with it but for the moves it shows
	// (FollowedRun), `calibrated` says which devices are target ones, and
	// `setup(time)` whether a frame moved during SteamVR's universe setup: those
	// moves place a station it guessed, and a frame that moved so keeps the
	// correction it had.
	template <class Calibrated, class Setup>
	UpdateResult Update(const LighthouseFrameWatch &watch, const std::vector<LighthouseFrameWatch::Move> &moves,
		const std::vector<LighthouseFrameWatch::Switch> &switches, uint32_t held, Calibrated calibrated, Setup setup)
	{
		using Cause = LighthouseFrameWatch::Switch::Cause;
		UpdateResult result;
		const Frames before = frames;
		std::array<Changed::How, vr::k_unMaxTrackedDeviceCount> how;
		how.fill(Changed::How::Replaced);
		auto usable = [&](uint32_t id) { return id < frames.size() && calibrated(id) && !Parked(id); };

		// The base stations' frames the start saw are anchored, so the moves
		// they show anchor the frames they lead to, for a device switched on
		// after a move no tracker showed. A base station shows every re-solve
		// of its own frame, so the frame it shows at the start is the start's.
		// Only in a session that started fresh, with every correction the
		// identity: a recovered one knows a station's correction only from a
		// device in its frame.
		if (fresh)
			for (uint32_t id = 0; id < frames.size(); ++id)
			{
				Entry station;
				double time = 0.0;
				bool baseStation = false;
				if (watch.FirstFrame(id, station.rot, station.trans, time, baseStation) && baseStation &&
					AtStart(watch, time))
				{
					station.anchored = true;
					station.generation = generation;
					Offer(station, watch);
				}
			}

		for (const auto &m : moves)
		{
			if (!usable(m.id))
				continue;
			how[m.id] = Changed::How::Moved;
			const bool followed = !setup(m.frameTime);
			// The held device follows its moves on its own, and refuses one
			// older than a move it followed, as a measurement checks at its end
			// (FollowedRun).
			if (m.id == held)
			{
				if (followed && !Follow(m))
					result.refused.push_back(m.id);
				else if (!followed)
					usableAfter[m.id] = (std::max)(usableAfter[m.id], m.time);
				continue;
			}
			usableAfter[m.id] = (std::max)(usableAfter[m.id], m.time);
			// The move is in the watch's record, a lead read off other devices'
			// moves as a chain of them, so propagation derives the frame it led
			// to; the device takes it once settled there.
			protocol::FrameCorrection probe = frames[m.id];
			if (followed && !Compose(probe, m.rotation, m.translation))
				result.refused.push_back(m.id);
		}
		Propagate(watch, setup);

		for (const auto &s : switches)
		{
			if (!usable(s.id) || s.id == held)
				continue;
			usableAfter[s.id] = (std::max)(usableAfter[s.id], s.time);
			how[s.id] = Changed::How::Switched;
			// The correction the device brings is that of the frame it left (a
			// device that came into a frame and jumped out of it within one
			// drain has not taken it yet), else its own. A device first seen,
			// even in the watch's first samples, carries its own in, unanchored:
			// one first seen in a station's frame that SteamVR re-solved before
			// any device there showed it is already off by that re-solve, and
			// only a root the record orders before its own can put that right (a
			// station seen only through its trackers: 18.0 and 43.1 mm at its
			// two).
			const Entry *from = s.cause == Cause::Appeared ? nullptr : Find(s.fromRot, s.fromTrans);
			Entry next = from ? *from : Entry{};
			if (!from)
				next.correction = frames[s.id];
			next.rot = s.toRot;
			next.trans = s.toTrans;
			next.disagreed = false;
			// The same pose put in another frame: the correction that was right
			// in the frame it left is right in this one, from the same root.
			if (s.cause == Cause::ReExpressed && from)
				Offer(next, watch);
			else if (!Find(s.toRot, s.toTrans))
			{
				next.anchored = false;
				next.generation = 0;
				next.root.kind = s.cause == Cause::Appeared || s.cause == Cause::ReExpressed
					? Root::Kind::Carried : Root::Kind::Unclear;
				next.root.rot = s.toRot;
				next.root.trans = s.toTrans;
				Offer(next, watch);
			}
			Propagate(watch, setup);
		}

		Assign(watch, held, usable);
		for (uint32_t id = 0; id < frames.size(); ++id)
			if (!(frames[id] == before[id]))
				result.changed.push_back({ id, how[id], before[id] });
		return result;
	}

	// After a recalibration measured on `source`: recalibrating is what
	// players do when trackers look off, so its frame is anchored anew with
	// its correction, and so are the frames the recorded moves lead to from
	// it, replacing what they had. Every calibrated device there takes it;
	// the devices in the other stations' frames keep theirs.
	template <class Calibrated, class Setup>
	std::vector<Changed> Anchor(uint32_t source, const LighthouseFrameWatch &watch, Calibrated calibrated, Setup setup)
	{
		std::vector<Changed> out;
		Entry anchor;
		if (source >= frames.size() || Parked(source) || !watch.LastFrame(source, anchor.rot, anchor.trans))
			return out;
		const Frames before = frames;
		anchor.correction = frames[source];
		anchor.anchored = true;
		anchor.generation = ++generation;
		Offer(anchor, watch);
		Propagate(watch, setup);
		Assign(watch, vr::k_unTrackedDeviceIndexInvalid,
			[&](uint32_t id) { return id < frames.size() && calibrated(id) && !Parked(id); });
		for (uint32_t id = 0; id < frames.size(); ++id)
			if (!(frames[id] == before[id]))
				out.push_back({ id, Changed::How::Anchored, before[id] });
		return out;
	}

	// Anchored derivations of one frame that disagreed (beyond what counts as
	// one frame, Agree): two chains of moves that should compose alike did not.
	size_t Disagreements() const { return disagreements; }
	// Propagations stopped at their bound with entries still changing.
	size_t Unsettled() const { return unsettled; }
	const std::vector<Entry> &Table() const { return table; }

	// Whether a device's frame changes are the target side's: Target once the
	// caller marked it a calibrated device, Waiting while that can still come
	// (its moves and switches are given back to the watch until it does), and
	// Other for a device that is not ours.
	enum class Membership { Target, Waiting, Other };

	// One drain's moves and switches by their device's membership
	// (`membership(id, changeTime)`): a target device's are for Update, a
	// waiting one's are given back to the watch (LighthouseFrameWatch::Hold)
	// to come again, and one given back before, of a device that turned out
	// not ours, is dropped.
	struct Admitted
	{
		std::vector<LighthouseFrameWatch::Move> moves, waitingMoves, droppedMoves;
		std::vector<LighthouseFrameWatch::Switch> switches, waitingSwitches, droppedSwitches;
	};
	template <class Classify>
	static Admitted Admit(LighthouseFrameWatch &watch, const std::vector<LighthouseFrameWatch::Move> &moves,
		const std::vector<LighthouseFrameWatch::Switch> &switches, Classify membership)
	{
		Admitted out;
		for (const auto &m : moves)
		{
			const Membership kind = membership(m.id, m.time);
			if (kind == Membership::Target)
				out.moves.push_back(m);
			else if (kind == Membership::Waiting)
			{
				out.waitingMoves.push_back(m);
				watch.Hold(m);
			}
			else if (m.held)
				out.droppedMoves.push_back(m);
		}
		for (const auto &s : switches)
		{
			const Membership kind = membership(s.id, s.time);
			if (kind == Membership::Target)
				out.switches.push_back(s);
			else if (kind == Membership::Waiting)
			{
				out.waitingSwitches.push_back(s);
				watch.Hold(s);
			}
			else if (s.held)
				out.droppedSwitches.push_back(s);
		}
		return out;
	}

	const std::string &Serial(uint32_t id) const { return serials.at(id); }
	uint64_t IdentityKey(uint32_t id) const { return restoredKeys.at(id); }

	// Two corrections within what counts as one frame (WorldFromDriverChanged):
	// the same moves followed in another order differ by rounding alone.
	static bool Agree(const protocol::FrameCorrection &a, const protocol::FrameCorrection &b)
	{
		return !questcal::WorldFromDriverChanged(
			Eigen::Quaterniond(a.rotation.w, a.rotation.x, a.rotation.y, a.rotation.z), Eigen::Vector3d(a.translation.v),
			Eigen::Quaterniond(b.rotation.w, b.rotation.x, b.rotation.y, b.rotation.z), Eigen::Vector3d(b.translation.v));
	}

	// Where a correction puts a raw world position.
	static Eigen::Vector3d Apply(const protocol::FrameCorrection &frame, const Eigen::Vector3d &position)
	{
		return Eigen::Quaterniond(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z) * position +
			Eigen::Vector3d(frame.translation.v);
	}

private:
	// A recovered correction whose device has not been identified in its slot yet.
	bool Parked(uint32_t id) const { return restoredKeys[id] != 0 && serials[id].empty(); }

	// Between the watch's first sample and its first recorded move, and
	// within StartSeconds of the first: a frame first shown then is one the
	// session start had.
	static bool AtStart(const LighthouseFrameWatch &watch, double time)
	{
		return time >= watch.FirstSampleTime() &&
			time < (std::min)(watch.FirstMoveTime(), watch.FirstSampleTime() + StartSeconds);
	}

	// Root `a` is older than `b`: of the kinds, the session start's, then a
	// carried correction's, then an unclear change's; of two carried ones,
	// the frame the recorded moves lead from to the other's, and not back. The
	// code has no count of a station's re-solves, only its record of moves:
	// roots the record does not order are equals, and the entry there stays.
	static bool Older(const Root &a, const Root &b, const LighthouseFrameWatch &watch)
	{
		if (a.kind != b.kind)
			return a.kind < b.kind;
		return a.kind == Root::Kind::Carried && watch.Leads(a.rot, a.trans, b.rot, b.trans) &&
			!watch.Leads(b.rot, b.trans, a.rot, a.trans);
	}

	Entry *Find(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation)
	{
		for (auto &e : table)
			if (!questcal::WorldFromDriverChanged(e.rot, e.trans, rotation, translation))
				return &e;
		return nullptr;
	}
	const Entry *Find(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation) const
	{
		return const_cast<TrackerFrameCorrections *>(this)->Find(rotation, translation);
	}

	// Offers the frame `candidate` names its correction: a frame with no entry
	// takes it, a provisional entry when it is anchored or its root is older,
	// an anchored one only from a later recalibration. An anchored derivation
	// of the same anchoring that disagrees is counted, once per entry. True
	// when the entry changed.
	bool Offer(Entry candidate, const LighthouseFrameWatch &watch)
	{
		Entry *e = Find(candidate.rot, candidate.trans);
		if (!e)
		{
			if (table.size() >= MaxEntries)
				table.erase(table.begin());
			candidate.disagreed = false;
			table.push_back(candidate);
			stale = true;
			return true;
		}
		const bool take = e->anchored ? candidate.anchored && candidate.generation > e->generation
			: candidate.anchored || Older(candidate.root, e->root, watch);
		if (!take)
		{
			if (candidate.anchored && e->anchored && candidate.generation == e->generation && !e->disagreed &&
				!Agree(candidate.correction, e->correction))
			{
				e->disagreed = true;
				++disagreements;
			}
			return false;
		}
		candidate.rot = e->rot;
		candidate.trans = e->trans;
		candidate.disagreed = false;
		*e = candidate;
		stale = true;
		return true;
	}

	// Every recorded move from a frame with an entry offers the frame it led
	// to the entry carried across it (unchanged for one in the universe
	// setup), until no entry changes. Each change makes an entry new, anchored
	// or anchored later, or its root older, so this ends; a station re-solved
	// back to a frame offers an equal root, which nothing takes. Frames are
	// matched within a tolerance, though, which is not transitive: should two
	// roots ever each read as the older, the passes are bounded (a pass
	// carries every entry at least one recorded move further), and the stop
	// is counted rather than the tick hanging; the entries stay as they are
	// until a move or an entry changes.
	template <class Setup>
	void Propagate(const LighthouseFrameWatch &watch, Setup setup)
	{
		// Nothing to do while no entry changed and no move was recorded since.
		const uint64_t newest = watch.Record().empty() ? 0 : watch.Record().back().seq + 1;
		if (!stale && newest == propagatedThrough)
			return;
		propagatedThrough = newest;
		const size_t passes = watch.Record().size() + table.size() + 2;
		bool changed = true;
		for (size_t pass = 0; changed && pass < passes; ++pass)
		{
			changed = false;
			for (const auto &t : watch.Record())
			{
				const Entry *from = Find(t.fromRot, t.fromTrans);
				if (!from)
					continue;
				Entry next = *from;
				next.rot = t.toRot;
				next.trans = t.toTrans;
				if (!setup(t.time))
				{
					const Eigen::Quaterniond rotation = (t.toRot * t.fromRot.conjugate()).normalized();
					if (!Compose(next.correction, rotation, t.toTrans - rotation * t.fromTrans))
						continue;
				}
				changed = Offer(next, watch) || changed;
			}
		}
		if (changed)
			++unsettled;
		stale = false;
	}

	// Each calibrated device in a frame with an entry takes its correction,
	// once its frame is settled (a change of it still being read off other
	// devices' moves keeps what it has until it is). One in a frame with none,
	// which no switch has brought it into (its own was dropped before it was
	// known to be ours), carries its own in.
	template <class Usable>
	void Assign(const LighthouseFrameWatch &watch, uint32_t held, Usable usable)
	{
		for (uint32_t id = 0; id < frames.size(); ++id)
		{
			Eigen::Quaterniond rotation;
			Eigen::Vector3d translation;
			if (!usable(id) || id == held || !watch.Settled(id) || !watch.LastFrame(id, rotation, translation))
				continue;
			Entry *e = Find(rotation, translation);
			if (!e)
			{
				Entry own;
				own.rot = rotation;
				own.trans = translation;
				own.correction = frames[id];
				own.root = { Root::Kind::Carried, rotation, translation };
				Offer(own, watch);
				continue;
			}
			e->rot = rotation;
			e->trans = translation;
			frames[id] = e->correction;
		}
	}

	// Far more than the record's frames and the devices', so propagation never
	// drops an entry it is still deriving.
	static constexpr size_t MaxEntries = 2048;

	std::vector<Entry> table;
	uint64_t generation = 0;
	// No correction was restored since the reset: every one is the identity.
	bool fresh = true;
	size_t disagreements = 0;
	size_t unsettled = 0;
	// An entry changed since the last propagation, or the record grew past
	// propagatedThrough (the newest move's sequence number, plus one).
	bool stale = false;
	uint64_t propagatedThrough = 0;
	Frames frames{};
	FrameSerialKeys restoredKeys{};
	std::array<std::string, vr::k_unMaxTrackedDeviceCount> serials{};
	std::array<double, vr::k_unMaxTrackedDeviceCount> usableAfter = []
	{
		std::array<double, vr::k_unMaxTrackedDeviceCount> times{};
		times.fill(-1e300);
		return times;
	}();
};

} // namespace questcal
