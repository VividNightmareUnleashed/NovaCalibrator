#pragma once

#include "AlignmentField.h"
#include "InterfaceHookInjector.h"
#include "RuntimeSnapshot.h"
#include "PoseFilterSlot.h"
#include "IPCServer.h"
#include "../common/PoseRingWriter.h"
#include "../common/PoseStageTrace.h"

#include <openvr_driver.h>

#include <atomic>
#include <cstdint>
#include <mutex>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() override;

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	// False when the request is refused, and then `reason` (if given) says why.
	bool TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform,
		protocol::RejectReason *reason = nullptr);
	bool TrySetRuntimeState(const protocol::SetRuntimeState &newState,
		protocol::RejectReason *reason = nullptr);
	// `host` is how the device's own driver reached the hook; a pose without
	// one is never re-sent (ResendQuietPoses). `publish` false leaves the pose
	// out of the pose ring: an observation hook already recorded it raw. `own`
	// is then that raw pose, as the device's own driver sent it, while `pose`
	// is what another module's code ahead of the pose hook made of it; a device
	// that is post-processed here gets its own motion back from it
	// (PoseFilterSlot.h, TakeOwnMotion).
	void HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose,
		const PoseHostCall &host = {}, bool publish = true, const vr::DriverPose_t *own = nullptr);
	// Poses that got their own motion back since this provider was made
	// (protocol::HookStatus::restoredPoseUpdates).
	uint32_t OwnMotionRestored() const { return ownMotionRestored.load(std::memory_order_relaxed); }
	// Records a raw pose for the pose ring and nothing else: the observation
	// hook's part, ahead of another module's code (InterfaceHookInjector.cpp).
	void PublishRawPose(uint32_t openVRID, const vr::DriverPose_t &pose);
	// IPC-thread owned checkpoint, separate from temporary slot neutralization.
	void GetRuntimeState(protocol::Response &response) const
	{
		response.driverSessionId = driverSessionId;
		response.runtimeState = recoveryState;
	}

#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	void SetSessionForTest(uint64_t id) { driverSessionId = id; recoveryState = {}; }
	void SetPoseTimeForTest(double seconds) { poseTimeForTest = seconds; }
	// `trackingSystem` stands for the name SteamVR has for the device's
	// tracking system: any number but zero, the same for devices of one system.
	void SetDeviceClassForTest(uint32_t id, vr::ETrackedDeviceClass deviceClass, uint64_t trackingSystem = 1)
	{
		deviceClassForTest[id] = deviceClass;
		trackingSystems[id].store(trackingSystem, std::memory_order_relaxed);
	}
	// RunFrame's re-send pass, without its pose ring retry.
	void ResendQuietPosesForTest() { ResendQuietPoses(); }
	bool StartTraceForTest(const char *name)
	{
		const bool ready = traceRing.Create(name);
		traceActive.store(ready);
		return ready;
	}
	// Every sample built for the pose ring, counted even with no ring open;
	// the last one under its device's pose mutex.
	uint32_t PublishedForTest() const { return publishedForTest.load(std::memory_order_relaxed); }
	protocol::DevicePoseSample LastPublishedForTest() const { return lastPublishedForTest; }
#endif

private:
	// The one unwind path shared by Cleanup and every Init failure. Init must not
	// leave detours enabled behind a failure return, and the ring must not be
	// unmapped while a pose thread can still be inside the publish path.
	void Teardown();
	vr::EVRInitError FailInit(vr::EVRInitError error);

	using DeviceTransform = questcal::runtimesnapshot::DeviceTransform;
    using TransformSlot = questcal::runtimesnapshot::TransformSlot;
	// Bounded reads fall back to the last consistent base/field pair.
	void ReadRuntimeState(uint32_t openVRID, DeviceTransform &transform,
		protocol::SetAlignmentField &field);

	// One IPC transaction publishes the base and field together. A reader must
	// never compose a base from one transaction with a field from another.
	std::atomic<uint32_t> runtimeSequence{ 0 };
	TransformSlot transforms[vr::k_unMaxTrackedDeviceCount];
	// Different devices remain fully concurrent. Same-device callbacks share
	// lastGood and both slew states, so serialize that narrow ownership domain.
	std::mutex poseMutexes[vr::k_unMaxTrackedDeviceCount];

	// Per-device fallback when a read keeps racing a write, under the matching
	// pose mutex.
	questcal::runtimesnapshot::Snapshot lastGood[vr::k_unMaxTrackedDeviceCount];

	using AtomicAlignmentField = questcal::runtimesnapshot::AtomicAlignmentField;

	AtomicAlignmentField alignmentField;

	// Per-device slew state for the field and the base transform, under the
	// matching pose mutex.
	alignfield::EvalState fieldState[vr::k_unMaxTrackedDeviceCount];
	alignfield::EvalState baseState[vr::k_unMaxTrackedDeviceCount];

	// Post-processing of each device's forwarded poses (PoseFilterSlot.h). The
	// IPC thread writes the settings and the pose callbacks read them; each
	// device's filter state is under the matching pose mutex.
	questcal::driverfilter::AtomicSettings poseFilters[vr::k_unMaxTrackedDeviceCount];
	questcal::posefilter::State poseFilterStates[vr::k_unMaxTrackedDeviceCount];
	// Poses of post-processed devices whose motion another module's code had
	// changed, and that got their own driver's motion back.
	std::atomic<uint32_t> ownMotionRestored{ 0 };
	protocol::FrameCorrection lastLoggedFrame[vr::k_unMaxTrackedDeviceCount];

	// A device's pose right after its frame correction changed, recorded by
	// its pose callback and written to the log by the IPC thread
	// (FlushFrameLog), so a pose callback never waits on the log file. The flag
	// is set only while a record waits to be written.
	struct FrameLogRecord
	{
		long long qpc = 0;
		int connected = 0;
		uint32_t generation = 0;
		vr::HmdVector3d_t inputPosition{};
		vr::HmdQuaternion_t inputRotation{};
		vr::HmdVector3d_t outputPosition{};
		vr::HmdQuaternion_t outputRotation{};
	};
	FrameLogRecord frameLog[vr::k_unMaxTrackedDeviceCount];
	std::atomic<bool> frameLogReady[vr::k_unMaxTrackedDeviceCount] = {};
	void FlushFrameLog();

	// A base station's driver sends its pose only when it solves a new one (two
	// exports: last poses 27 minutes to 2.6 hours old), so the runtime would
	// keep it where the calibration stood at that pose while every correction
	// since moves the trackers around it. RunFrame sends its last pose again,
	// through its own driver's host call, whenever the calibration would now
	// put it elsewhere.
	void ResendQuietPoses();
	double NowSeconds() const;
	// Cached once known. RunFrame only.
	vr::ETrackedDeviceClass DeviceClass(uint32_t openVRID);

	// What the runtime last received for each device, under the matching pose
	// mutex.
	struct RuntimePoseRecord
	{
		bool seen = false;
		vr::DriverPose_t raw{};     // as the device's own driver sent it
		vr::DriverPose_t sent{};    // what the runtime received for it, from either
		PoseHostCall host;          // how that driver reached the hook
		double receivedAt = 0.0;    // when that driver last sent a pose
		uint32_t received = 0;      // counts that driver's poses
	};
	RuntimePoseRecord runtimePoses[vr::k_unMaxTrackedDeviceCount];
	// "Soften jumps" holds back more of the correction a tracker or controller
	// makes as its tracking comes back, when the other devices of its tracking
	// system say that nothing else moved (posefilter::MaxRecoveryHeldMeters). For
	// that, each such device keeps two of its recent world positions, taken a
	// sixtieth of a second apart under its pose mutex, so that a driver with
	// 500 poses a second is compared over the same time as one with 60.
	struct PeerMotion
	{
		double at = -1.0, before = -1.0;
		vr::HmdVector3d_t position{}, previous{};
		bool targetSpace = false;
	};
	PeerMotion peerMotion[vr::k_unMaxTrackedDeviceCount];
	// The trackers and controllers, and a number for the name of each one's
	// tracking system ("lighthouse", "oculus"), zero while SteamVR has none:
	// the name the overlay pairs devices by. Two systems' raw spaces can hold
	// the same numbers for different places, so only devices of one system
	// are compared. RunFrame reads both.
	std::atomic<uint64_t> bodyDevices{ 0 };
	std::atomic<uint64_t> trackingSystems[vr::k_unMaxTrackedDeviceCount]{};
	uint64_t trackingSystemsRead = 0;   // RunFrame's own
	// A device has "Soften jumps" on, so the positions above are kept.
	std::atomic<bool> observeRecoveryPeers{ false };
	// What the other devices of a device's tracking system say of a correction
	// it made:
	// at least two near it were steady, one moved or lost tracking with it, or
	// too few could be asked.
	enum class Peers { Steady, Disturbed, Unknown };
	Peers PeersNear(uint32_t id, const vr::DriverPose_t &pose, bool targetSpace, double now);
	bool RecoveredAlone(uint32_t id, const vr::DriverPose_t &raw, bool targetSpace, double now);
	void NotePeerMotion(uint32_t id, const vr::DriverPose_t &raw, bool targetSpace, double now);
	void ReadTrackingSystem(uint32_t id);
	// Devices with a record, so the pass skips the rest without a lock.
	std::atomic<uint64_t> recordedDevices{ 0 };
	// RunFrame's own.
	vr::ETrackedDeviceClass deviceClasses[vr::k_unMaxTrackedDeviceCount] = {};
	// The device's own pose came through while its re-send was in flight and
	// may have reached the runtime first: send again from the newest.
	bool resendOwed[vr::k_unMaxTrackedDeviceCount] = {};
	double lastResendPass = -1e300;
	// Re-sends since the last log line, written by the IPC thread
	// (FlushFrameLog) at most once a minute.
	std::atomic<uint32_t> resentPoses{ 0 };
	std::atomic<uint64_t> resentDevices{ 0 };
	std::atomic<uint32_t> largestResendMoveMicrometers{ 0 };
	uint64_t lastResendLogMs = 0;   // IPC thread

	uint64_t driverSessionId = 0;
	protocol::SetRuntimeState recoveryState;
	double qpcToSeconds = 0.0;
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	std::atomic<uint32_t> publishedForTest{ 0 };
	protocol::DevicePoseSample lastPublishedForTest;
	double poseTimeForTest = -1.0;
	vr::ETrackedDeviceClass deviceClassForTest[vr::k_unMaxTrackedDeviceCount] = {};
#endif

	// Under the device's pose mutex.
	void PublishSample(uint32_t openVRID, const vr::DriverPose_t &pose, int64_t qpc);
	// Publishes every raw (pre-transform) pose for the overlay's solver.
	protocol::PoseRingWriter poseRing;
	protocol::PoseRingWriter traceRing;
	std::atomic<bool> traceActive{ false };
	uint64_t lastTracePoll = 0;
	void PollTraceRequest();
	void TracePose(uint32_t id, questcal::stagetrace::Stage stage, uint64_t sequence,
		const vr::DriverPose_t &pose, int64_t qpc);
	// Publishes a completed Create (in Init or a RunFrame retry) to the pose
	// threads. Publish runs only while this is set.
	std::atomic<bool> poseRingReady{ false };
	uint64_t lastPoseRingCreateAttemptMs = 0;

	// Declared last so it is destroyed first: ~IPCServer joins the pipe thread,
	// which writes into the members above, in case the DLL unloads without
	// Cleanup().
	IPCServer server;
};
