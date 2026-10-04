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
	void Restore(const Frames &saved, const FrameSerialKeys &keys)
	{
		frames = saved;
		restoredKeys = keys;
		usableAfter.fill(-1e300);
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

	bool Follow(const LighthouseFrameWatch::Move &move)
	{
		if (move.id >= frames.size() || !std::isfinite(move.time) ||
			(restoredKeys[move.id] != 0 && serials[move.id].empty()) ||
			move.time < usableAfter[move.id] ||
			!IsValidCalibrationTransform(move.rotation, move.translation, 1.0))
			return false;
		const auto &old = frames[move.id];
		const Eigen::Quaterniond rotation(old.rotation.w, old.rotation.x, old.rotation.y, old.rotation.z);
		const Eigen::Vector3d translation(old.translation.v);
		// N' F = N. Translation here is unscaled; C applies its scale later.
		const Eigen::Quaterniond nextRotation = (rotation * move.rotation.conjugate()).normalized();
		const Eigen::Vector3d nextTranslation = translation - nextRotation * move.translation;
		if (!IsValidCalibrationTransform(nextRotation, nextTranslation, 1.0))
			return false;
		frames[move.id].rotation = { nextRotation.w(), nextRotation.x(), nextRotation.y(), nextRotation.z() };
		for (int axis = 0; axis < 3; ++axis)
			frames[move.id].translation.v[axis] = nextTranslation(axis);
		// The monitor and solver drain separately. Discard samples preceding the
		// most recent move, including a move inferred after a tracker returned,
		// rather than applying the latest correction to the old frame.
		usableAfter[move.id] = move.time;
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
	// After a recalibration measured on `source`, a device reported against the
	// same base station takes its correction: the two share SteamVR's geometry,
	// so a difference between them can only be a move one of them missed. A
	// parked recovery entry stays with its own device.
	bool Align(uint32_t id, uint32_t source)
	{
		if (id >= frames.size() || source >= frames.size() || id == source ||
			Parked(id) || frames[id] == frames[source])
			return false;
		frames[id] = frames[source];
		return true;
	}

	// One device Join gave another's correction.
	struct Joined
	{
		uint32_t id = 0;
		uint32_t source = 0;
		bool fromHeadsetTracker = false;
		// How many devices in the frame shared the correction it took.
		size_t sharedBy = 0;
		protocol::FrameCorrection before;
	};
	struct JoinResult
	{
		std::vector<Joined> joined;
		// Why nothing was taken, for the detailed log; null when something was.
		const char *kept = nullptr;
	};

	// The same rule between recalibrations. A device that came into a frame
	// with no move its correction followed (LighthouseFrameWatch::Switch)
	// carries a correction made for the frame it left: live 2026-10-03, a body
	// tracker bootstrapped from S-13 while SteamVR moved S-3 under every other
	// device, came back into S-3's frame 7.4 cm off them and stayed so for 92
	// minutes. It takes the correction of the calibrated devices already in its
	// frame: the headset tracker's when that is one of them, since the
	// continuous loop holds that one to the headset, otherwise theirs when they
	// all agree; alone, or among devices that disagree, it keeps its own. The
	// headset tracker coming into a frame gives its correction to the devices
	// there instead, and never takes one. `member` says which devices are
	// calibrated, `keep` is one whose correction stays put (a measurement's
	// target), and a change still being read off other devices' moves
	// (LighthouseFrameWatch::Settled) waits for its own switch.
	template <class Member>
	JoinResult Join(const LighthouseFrameWatch::Switch &change, const LighthouseFrameWatch &watch,
		uint32_t headsetTracker, uint32_t keep, Member member)
	{
		JoinResult result;
		const uint32_t id = change.id;
		Eigen::Quaterniond frameRotation;
		Eigen::Vector3d frameTranslation;
		if (id >= frames.size() || !member(id))
			result.kept = "it is not calibrated";
		else if (Parked(id))
			result.kept = "its recovered correction is not bound to it yet";
		else if (!watch.LastFrame(id, frameRotation, frameTranslation))
			result.kept = "it has no frame any more";
		else if (!watch.Settled(id))
			result.kept = "a later change of its frame is still being read";
		if (result.kept)
			return result;
		std::vector<uint32_t> mates;
		for (uint32_t other = 0; other < frames.size(); ++other)
			if (other != id && member(other) && !Parked(other) && watch.Settled(other) &&
				watch.InFrame(other, frameRotation, frameTranslation))
				mates.push_back(other);
		if (mates.empty())
		{
			result.kept = "no other calibrated device is in that frame";
			return result;
		}
		auto take = [&](uint32_t to, uint32_t from, bool fromHeadsetTracker, size_t sharedBy)
		{
			if (to == keep || Agree(frames[to], frames[from]))
				return;
			result.joined.push_back({ to, from, fromHeadsetTracker, sharedBy, frames[to] });
			frames[to] = frames[from];
		};
		if (id == headsetTracker)
		{
			for (const uint32_t mate : mates)
				take(mate, id, true, 1);
			if (result.joined.empty())
				result.kept = "the devices in that frame already have its correction";
			return result;
		}
		if (std::find(mates.begin(), mates.end(), headsetTracker) != mates.end())
		{
			take(id, headsetTracker, true, 1);
		}
		else
		{
			for (const uint32_t mate : mates)
				if (!Agree(frames[mate], frames[mates.front()]))
				{
					result.kept = "the devices in that frame disagree";
					return result;
				}
			take(id, mates.front(), false, mates.size());
		}
		if (result.joined.empty())
		{
			result.kept = id == keep ? "it is being measured" : "it already has their correction";
			return result;
		}
		// Samples from before the change are from the frame it left.
		usableAfter[id] = (std::max)(usableAfter[id], change.time);
		return result;
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
