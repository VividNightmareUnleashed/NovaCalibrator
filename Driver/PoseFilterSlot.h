#pragma once

#include "AtomicSnapshot.h"
#include "PoseFilter.h"
#include "../common/Protocol.h"
#include "../common/TransformLimits.h"

#include <openvr_driver.h>

// One device's pose filter inside the driver: the settings the IPC thread
// writes and the pose threads read, and the step that runs Driver/PoseFilter.h
// on a driver pose.
namespace questcal
{
namespace driverfilter
{

// The protocol's limits are the filter's own.
static_assert(protocol::limits::MinFilterCutoffHz == posefilter::limits::MinCutoffHz &&
	protocol::limits::MaxFilterCutoffHz == posefilter::limits::MaxCutoffHz &&
	protocol::limits::MaxFilterGain == posefilter::limits::MaxGain &&
	protocol::limits::MinFilterReleaseSeconds == posefilter::limits::MinReleaseSeconds &&
	protocol::limits::MaxFilterReleaseSeconds == posefilter::limits::MaxReleaseSeconds,
	"common/TransformLimits.h and Driver/PoseFilter.h disagree on the pose filter's limits");

// The settings are not under the runtime state's sequence counter. Each value
// is one lock-free atom, and every value the IPC thread stores has passed
// validation, so a pose that reads a setting mid-change filters with a mix of
// two good settings for that one pose, which Usable() below accepts or the
// pose leaves untouched.
struct AtomicSettings
{
	std::atomic<uint32_t> smooth{ 0 };
	std::atomic<uint32_t> releaseCorrections{ 0 };
	atomicsnapshot::Double minCutoffHz{ 1.0 };
	atomicsnapshot::Double speedGain{ 160.0 };
	atomicsnapshot::Double turnMinCutoffHz{ 1.0 };
	atomicsnapshot::Double turnGain{ 100.0 };
	atomicsnapshot::Double releaseSeconds{ 0.1 };
	atomicsnapshot::Double predictionScale{ 1.0 };

	// `filtered` is the device's bit in SetRuntimeState::filterMask.
	void Store(const protocol::PoseFilter &filter, bool filtered, double predictionCut) noexcept
	{
		smooth.store(filtered && filter.smooth ? 1u : 0u, std::memory_order_release);
		releaseCorrections.store(filtered && filter.releaseCorrections ? 1u : 0u, std::memory_order_release);
		minCutoffHz.store(filter.minCutoffHz, std::memory_order_release);
		speedGain.store(filter.speedGain, std::memory_order_release);
		turnMinCutoffHz.store(filter.turnMinCutoffHz, std::memory_order_release);
		turnGain.store(filter.turnGain, std::memory_order_release);
		releaseSeconds.store(filter.releaseSeconds, std::memory_order_release);
		predictionScale.store(1.0 - predictionCut, std::memory_order_release);
	}

	posefilter::Settings Load() const noexcept
	{
		posefilter::Settings settings;
		settings.smooth = smooth.load(std::memory_order_acquire) != 0;
		settings.releaseCorrections = releaseCorrections.load(std::memory_order_acquire) != 0;
		settings.minCutoffHz = minCutoffHz.load(std::memory_order_acquire);
		settings.speedGain = speedGain.load(std::memory_order_acquire);
		settings.turnMinCutoffHz = turnMinCutoffHz.load(std::memory_order_acquire);
		settings.turnGain = turnGain.load(std::memory_order_acquire);
		settings.releaseSeconds = releaseSeconds.load(std::memory_order_acquire);
		settings.predictionScale = predictionScale.load(std::memory_order_acquire);
		return settings;
	}
};

// The settings make the filter act on a device's poses: the device is one
// this driver post-processes.
inline bool Acts(const posefilter::Settings &settings) noexcept
{
	return !posefilter::ChangesNothing(settings) && posefilter::Usable(settings);
}

// Gives `pose` the motion of `own`, the same pose as the device's own driver
// sent it, and returns whether that changed it. `pose` is what another
// module's code ahead of the pose hook made of it (OpenVR-SmoothTracking
// smooths it and takes its velocities down), so this takes that code's
// post-processing off again. Everything that is not motion stays as that code
// left it: whether the pose is valid and the device connected (Standable hides
// a tracker that way), the tracking result, and the space the device is in.
inline bool TakeOwnMotion(vr::DriverPose_t &pose, const vr::DriverPose_t &own) noexcept
{
	bool changed = pose.poseTimeOffset != own.poseTimeOffset ||
		pose.qRotation.w != own.qRotation.w || pose.qRotation.x != own.qRotation.x ||
		pose.qRotation.y != own.qRotation.y || pose.qRotation.z != own.qRotation.z;
	for (int i = 0; i < 3; ++i)
		changed = changed || pose.vecPosition[i] != own.vecPosition[i] ||
			pose.vecVelocity[i] != own.vecVelocity[i] || pose.vecAcceleration[i] != own.vecAcceleration[i] ||
			pose.vecAngularVelocity[i] != own.vecAngularVelocity[i] ||
			pose.vecAngularAcceleration[i] != own.vecAngularAcceleration[i];
	if (!changed)
		return false;
	pose.poseTimeOffset = own.poseTimeOffset;
	pose.qRotation = own.qRotation;
	for (int i = 0; i < 3; ++i)
	{
		pose.vecPosition[i] = own.vecPosition[i];
		pose.vecVelocity[i] = own.vecVelocity[i];
		pose.vecAcceleration[i] = own.vecAcceleration[i];
		pose.vecAngularVelocity[i] = own.vecAngularVelocity[i];
		pose.vecAngularAcceleration[i] = own.vecAngularAcceleration[i];
	}
	return true;
}

// Filters one driver pose in place with `settings`, one Load() of the device's
// slot. `state` belongs to the device and is the caller's to guard;
// `captureSeconds` is the driver's clock at the pose's arrival. poseTimeOffset
// is read and left as it is.
//
// A pose counts as usable when its driver calls it valid. Whether the device
// is "connected" does not: Standable hides physical trackers by reporting
// them disconnected while their poses stay valid and in use.
inline void Apply(const posefilter::Settings &settings, posefilter::State &state, double captureSeconds,
	vr::DriverPose_t &pose)
{
	if (!Acts(settings))
	{
		if (state.primed || state.captureIsPoseTime)
			posefilter::Reset(state);
		return;
	}

	posefilter::Pose filtered;
	filtered.timeOffset = pose.poseTimeOffset;
	filtered.rotation[0] = pose.qRotation.w;
	filtered.rotation[1] = pose.qRotation.x;
	filtered.rotation[2] = pose.qRotation.y;
	filtered.rotation[3] = pose.qRotation.z;
	for (int i = 0; i < 3; ++i)
	{
		filtered.position[i] = pose.vecPosition[i];
		filtered.velocity[i] = pose.vecVelocity[i];
		filtered.angularVelocity[i] = pose.vecAngularVelocity[i];
		filtered.angularAcceleration[i] = pose.vecAngularAcceleration[i];
	}
	posefilter::Apply(state, settings, captureSeconds, pose.poseIsValid, filtered);
	pose.qRotation.w = filtered.rotation[0];
	pose.qRotation.x = filtered.rotation[1];
	pose.qRotation.y = filtered.rotation[2];
	pose.qRotation.z = filtered.rotation[3];
	for (int i = 0; i < 3; ++i)
	{
		pose.vecPosition[i] = filtered.position[i];
		pose.vecVelocity[i] = filtered.velocity[i];
		pose.vecAngularVelocity[i] = filtered.angularVelocity[i];
		pose.vecAngularAcceleration[i] = filtered.angularAcceleration[i];
	}
}

} // namespace driverfilter
} // namespace questcal
