#pragma once

#include "LocalPoseContinuity.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>

class PoseStreamHub;
struct CalibrationContext;
namespace protocol { struct DevicePoseSample; }

namespace questcal
{

struct WorldFromDriver;

void StartCalibrationSpace(PoseStreamHub &poseHub, double qpcToSeconds);
void StopCalibrationSpace();

// Maintains the persisted profile universe and protected chaperone from the
// dedicated HMD raw-pose stream.
void CalibrationSpaceTick(CalibrationContext &ctx, double now);
void CheckProtectedChaperone(CalibrationContext &ctx);

// Universe-jump observation is fed from the shared runtime-monitor stream so
// its accepted deltas land before drift and continuous-calibration decisions.
void ObserveUniversePose(const protocol::DevicePoseSample &sample);
// A few samples went missing from the monitor's drain (JumpDetector::NoteStreamHole).
void NoteUniverseStreamHole();
// The frame a reference device's driver reports angular velocity in, as the
// jump detector's probe judged it; Unknown until it has.
ringpose::AngularVelocityFrame ReferenceAngularVelocityFrame(uint32_t id);
void ResetUniverseObservations(CalibrationContext &ctx);
bool FinishUniverseObservations(CalibrationContext &ctx, double now);

// Applies a reference-space delta to every profile-owned spatial value.
// moveChaperone: the reference space itself moved (a headset re-center), so
// the protected chaperone moves with it; a correction or a re-anchor moves
// only the target side and leaves it.
bool ApplyCalibrationDelta(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation,
	bool snap, double now, bool moveChaperone);

// The target side's normalized space moved under a calibration measured
// before the move: a pose there is now `rotation * old + translation`. Keeps
// every target device where that calibration put it (C' = C M^-1), for the
// base calibration and each field anchor's solve. The reference space did not
// move, so the anchors' positions and the protected chaperone stay. False,
// with nothing changed, without a calibration or when a result would be out
// of bounds.
bool CarryTargetSpaceMove(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation, double now);

// Rebinds physical-HMD and raw-universe ownership after a successful base
// calibration and disarms a room snapshot that no longer belongs to it.
void RebindCalibrationUniverse(CalibrationContext &ctx,
	const std::string &hmdSerial, const std::string &hmdModel,
	bool priorUniverseUnsafe);

// The profile-universe verdict found the profile's WFD change followed: the
// profile and the protected chaperone take on `adopted` as their WFD, saved
// at once.
void AdoptProfileUniverse(CalibrationContext &ctx, const WorldFromDriver &adopted, double now);

} // namespace questcal
