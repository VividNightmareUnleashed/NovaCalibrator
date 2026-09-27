#pragma once

// Whether a driver-local pose stayed on its reported trajectory from one
// sample to the next: what tells a lighthouse frame that moved under a device
// from a pose that jumped (LighthouseFrameWatch.h, CalibrationRun.h). Kept
// free of OpenVR headers so a header any translation unit includes can use
// it; RingPoseMath.h carries it for everything that reads the pose ring.

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ringpose
{

constexpr double MaxAdjacentFrameSeconds = 0.1;
constexpr double MaxLocalPositionErrorMeters = 0.005;
constexpr double MaxLocalRotationErrorRadians =
	1.0 * 3.14159265358979323846 / 180.0;

struct DriverLocalPoseSample
{
	double time = 0.0;
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d position{ 0, 0, 0 };
	Eigen::Vector3d velocity{ 0, 0, 0 };
	Eigen::Vector3d angularVelocity{ 0, 0, 0 };
};

// A worldFromDriver transition describes a real raw-universe rebase only if
// adjacent driver-local poses remain on their reported trajectory. An inverse
// local-pose rewrite is bookkeeping that leaves the composed raw pose still.
// Both samples come from trusted ring samples (IsUsableRingSample).
inline bool IsDriverLocalPoseContinuous(
	const DriverLocalPoseSample &previous,
	const DriverLocalPoseSample &current,
	double maxFrameSeconds = MaxAdjacentFrameSeconds,
	double maxPositionErrorMeters = MaxLocalPositionErrorMeters,
	double maxRotationErrorRadians = MaxLocalRotationErrorRadians)
{
	double dt = current.time - previous.time;
	if (dt <= 0.0 || dt > maxFrameSeconds)
		return false;

	Eigen::Vector3d predictedPosition = previous.position +
		0.5 * (previous.velocity + current.velocity) * dt;
	Eigen::Quaterniond predictedRotation = previous.rotation.normalized();
	Eigen::Vector3d meanAngularVelocity =
		0.5 * (previous.angularVelocity + current.angularVelocity);
	double predictedAngle = meanAngularVelocity.norm() * dt;
	if (predictedAngle > 1e-12)
	{
		predictedRotation = Eigen::Quaterniond(Eigen::AngleAxisd(
			predictedAngle, meanAngularVelocity.normalized())) * predictedRotation;
	}

	return (current.position - predictedPosition).norm() <=
			maxPositionErrorMeters &&
		current.rotation.normalized().angularDistance(predictedRotation) <=
			maxRotationErrorRadians;
}

} // namespace ringpose
