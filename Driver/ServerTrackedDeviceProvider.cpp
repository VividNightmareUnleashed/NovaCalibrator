#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "RuntimePose.h"
#include "ProtocolValidation.h"

#include <algorithm>
#include <cmath>
#include <utility>

// Vertical displacement applied to a hidden device's forwarded pose.
static constexpr double HiddenPoseOffsetY = 1000.0;   // meters
static constexpr uint64_t PoseRingRetryIntervalMs = 1000;
// How often RunFrame looks for a quiet device to re-send (ResendQuietPoses).
static constexpr double ResendPassSeconds = 0.1;
// A device whose own driver sent a pose this recently gets the calibration
// with its next one.
static constexpr double ResendQuietSeconds = 0.5;
// Smaller differences are the slew's rounding, not a calibration change.
static constexpr double ResendMinMoveMeters = 1e-5;
static constexpr double ResendMinTurnRadians = 1e-5;
static constexpr uint64_t ResendLogIntervalMs = 60000;

namespace
{
struct WorldPose
{
	vr::HmdVector3d_t position;
	vr::HmdQuaternion_t rotation;
};

// Where the runtime puts a driver pose.
WorldPose World(const vr::DriverPose_t &pose)
{
	const auto rotated = questcal::driverpose::RotateVector(pose.qWorldFromDriverRotation, pose.vecPosition);
	return { questcal::driverpose::Add(rotated.v, pose.vecWorldFromDriverTranslation),
		questcal::driverpose::Multiply(pose.qWorldFromDriverRotation, pose.qRotation) };
}

double Distance(const vr::HmdVector3d_t &a, const vr::HmdVector3d_t &b)
{
	double sum = 0.0;
	for (int i = 0; i < 3; ++i)
		sum += (a.v[i] - b.v[i]) * (a.v[i] - b.v[i]);
	return std::sqrt(sum);
}

double Turn(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b)
{
	const double dot = std::abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
	return 2.0 * std::acos(std::min(dot, 1.0));
}
} // namespace

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	// Pose callbacks can begin on other driver threads as soon as the global
	// host detour is enabled. Initialize every callback-visible member first.
	LARGE_INTEGER started;
	QueryPerformanceCounter(&started);
	driverSessionId = static_cast<uint64_t>(started.QuadPart);
	recoveryState = {};
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);   // never fails on XP or later
	qpcToSeconds = 1.0 / static_cast<double>(freq.QuadPart);

	bool poseRingCreated = poseRing.Create(QUESTCALIBRATOR_SHMEM_NAME);
	poseRingReady.store(poseRingCreated, std::memory_order_release);
	lastPoseRingCreateAttemptMs = GetTickCount64();
	if (!poseRingCreated)
	{
		// Non-fatal: calibration transforms still apply, but the overlay will
		// fall back to runtime-predicted poses instead of raw driver poses.
		LOG("Failed to create pose ring shared memory (error %u)", GetLastError());
	}

	// Install the context detour before OpenVR initializes its cached interfaces.
	// InitServerDriverContext requests IVRServerDriverHost_006 through the
	// detour, so the check below proves a pose hook exists, not that every
	// device driver is routed through it: a driver that resolved its host
	// interface before this detour existed forwards poses untouched. Only the
	// `01novacalibrator` manifest name orders this driver first.
	if (!InjectHooks(this, pDriverContext))
		return FailInit(vr::VRInitError_Driver_Failed);

	vr::EVRInitError contextError = vr::InitServerDriverContext(pDriverContext);
	if (contextError != vr::VRInitError_None)
		return FailInit(contextError);

	if (!IsPoseUpdateHookInstalled())
	{
		LOG("No supported IVRServerDriverHost pose hook was installed");
		return FailInit(vr::VRInitError_Driver_Failed);
	}

	IPCServer::RequestSink sink;
	sink.setDeviceTransform = [this](const protocol::SetDeviceTransform &transform)
	{
		protocol::RejectReason reason = protocol::RejectReason::None;
		TrySetDeviceTransform(transform, &reason);
		return reason;
	};
	sink.setRuntimeState = [this](const protocol::SetRuntimeState &state)
	{
		protocol::RejectReason reason = protocol::RejectReason::None;
		TrySetRuntimeState(state, &reason);
		return reason;
	};
	sink.poseHookMask = [] { return PoseUpdateHookMask(); };
	sink.hookStatus = [] { return PoseHookStatus(); };
	sink.getRuntimeState = [this](protocol::Response &response) { GetRuntimeState(response); };
	if (!server.Run(std::move(sink)))
	{
		LOG("IPC server could not establish its control listener");
		return FailInit(vr::VRInitError_Driver_Failed);
	}

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Teardown()
{
	server.Stop();
	// Hooks may be live even when Init failed (InjectHooks refuses while a
	// previous Init's hooks remain). Wait for the detours to leave the driver
	// before the ring goes away, or a pose thread still publishing would write
	// into an unmapped view.
	const bool released = DisableHooks();
	poseRingReady.store(false, std::memory_order_release);
	// The IPC thread has stopped; write what it would have.
	FlushFrameLog();
	if (released)
		poseRing.Close();
	else
	{
		// A pose thread may still reach the ring, so leak the mapping; the next
		// writer's PID + creation-time liveness proof covers an owner that never
		// closes.
		LOG("The pose ring mapping is retained for the pose callback still inside the driver");
	}
}

vr::EVRInitError ServerTrackedDeviceProvider::FailInit(vr::EVRInitError error)
{
	Teardown();
	vr::CleanupDriverContext();
	return error;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	Teardown();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void ServerTrackedDeviceProvider::RunFrame()
{
	ResendQuietPoses();

	if (poseRingReady.load(std::memory_order_acquire))
		return;

	uint64_t now = GetTickCount64();
	if (now - lastPoseRingCreateAttemptMs < PoseRingRetryIntervalMs)
		return;
	// Zero wait budget: this is vrserver's driver frame loop, shared with every
	// other driver.
	if (poseRing.Create(QUESTCALIBRATOR_SHMEM_NAME, 0))
	{
		poseRingReady.store(true, std::memory_order_release);
		LOG("Pose ring shared memory became available after retry");
	}
	// Stamped after the attempt so a slow failure does not eat its own interval.
	lastPoseRingCreateAttemptMs = GetTickCount64();
}

void ServerTrackedDeviceProvider::FlushFrameLog()
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		if (!frameLogReady[id].load(std::memory_order_acquire))
			continue;
		const FrameLogRecord record = frameLog[id];
		frameLogReady[id].store(false, std::memory_order_release);
		if (!LogFile)
			continue;
		LOG("frame applied device %u QPC %lld connected %d: raw p %.6f %.6f %.6f q %.6f %.6f %.6f %.6f; "
			"output before hiding p %.6f %.6f %.6f q %.6f %.6f %.6f %.6f (base generation %u)",
			id, record.qpc, record.connected,
			record.inputPosition.v[0], record.inputPosition.v[1], record.inputPosition.v[2],
			record.inputRotation.w, record.inputRotation.x, record.inputRotation.y, record.inputRotation.z,
			record.outputPosition.v[0], record.outputPosition.v[1], record.outputPosition.v[2],
			record.outputRotation.w, record.outputRotation.x, record.outputRotation.y, record.outputRotation.z,
			record.generation);
	}

	// At most a line a minute, so a long session's corrections leave the log's
	// tail to everything else.
	const uint64_t nowMs = GetTickCount64();
	if (resentPoses.load(std::memory_order_acquire) != 0 &&
		(lastResendLogMs == 0 || nowMs - lastResendLogMs >= ResendLogIntervalMs))
	{
		lastResendLogMs = nowMs;
		const uint32_t resent = resentPoses.exchange(0, std::memory_order_acq_rel);
		const uint64_t devices = resentDevices.exchange(0, std::memory_order_acq_rel);
		const uint32_t largest = largestResendMoveMicrometers.exchange(0, std::memory_order_acq_rel);
		if (LogFile)
			LOG("base station poses re-sent as the calibration moved: %u (device mask %llx, largest move %.3f mm)",
				resent, static_cast<unsigned long long>(devices), largest / 1000.0);
	}
}

double ServerTrackedDeviceProvider::NowSeconds() const
{
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	if (poseTimeForTest >= 0.0)
		return poseTimeForTest;
#endif
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	return static_cast<double>(now.QuadPart) * qpcToSeconds;
}

vr::ETrackedDeviceClass ServerTrackedDeviceProvider::DeviceClass(uint32_t openVRID)
{
	if (deviceClasses[openVRID] != vr::TrackedDeviceClass_Invalid)
		return deviceClasses[openVRID];
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	deviceClasses[openVRID] = deviceClassForTest[openVRID];
#else
	if (vr::CVRPropertyHelpers *properties = vr::VRProperties())
	{
		vr::ETrackedPropertyError error = vr::TrackedProp_Success;
		const int32_t value = properties->GetInt32Property(
			properties->TrackedDeviceToPropertyContainer(openVRID), vr::Prop_DeviceClass_Int32, &error);
		if (error == vr::TrackedProp_Success)
			deviceClasses[openVRID] = static_cast<vr::ETrackedDeviceClass>(value);
	}
#endif
	return deviceClasses[openVRID];
}

void ServerTrackedDeviceProvider::ResendQuietPoses()
{
	const double passTime = NowSeconds();
	if (passTime >= lastResendPass && passTime - lastResendPass < ResendPassSeconds)
		return;
	lastResendPass = passTime;

	const uint64_t recorded = recordedDevices.load(std::memory_order_acquire);
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		// Base stations alone: every other device sends poses many times a
		// second, each taking the calibration as it stands.
		if ((recorded & (uint64_t{ 1 } << id)) == 0 ||
			DeviceClass(id) != vr::TrackedDeviceClass_TrackingReference)
			continue;

		vr::DriverPose_t pose{};
		PoseHostCall host;
		uint32_t received = 0;
		{
			// Never wait on a pose thread from SteamVR's frame loop: a device
			// whose own pose is going through gets the calibration with it.
			std::unique_lock<std::mutex> lock(poseMutexes[id], std::try_to_lock);
			if (!lock.owns_lock())
				continue;
			RuntimePoseRecord &record = runtimePoses[id];
			const double now = NowSeconds();
			if (!record.host.host || !record.host.original ||
				!record.raw.poseIsValid || !record.raw.deviceIsConnected ||
				(!resendOwed[id] && now - record.receivedAt < ResendQuietSeconds))
				continue;

			// As HandleDevicePoseUpdated transforms it, from the device's own
			// newest pose; nothing goes to the pose ring.
			pose = record.raw;
			DeviceTransform tf;
			protocol::SetAlignmentField field;
			ReadRuntimeState(id, tf, field);
			questcal::driverpose::ApplyRuntimePose(pose, tf.calibration, tf.frame,
				field, now, baseState[id], fieldState[id]);
			if (tf.control.hidden)
				pose.vecWorldFromDriverTranslation[1] += HiddenPoseOffsetY;

			const WorldPose before = World(record.sent);
			const WorldPose after = World(pose);
			const double moved = Distance(before.position, after.position);
			if (!resendOwed[id] && !(moved > ResendMinMoveMeters) &&
				!(Turn(before.rotation, after.rotation) > ResendMinTurnRadians))
				continue;

			record.sent = pose;
			host = record.host;
			received = record.received;
			const uint32_t micrometers = static_cast<uint32_t>(std::min(moved * 1e6, 4e9));
			uint32_t largest = largestResendMoveMicrometers.load(std::memory_order_relaxed);
			while (largest < micrometers &&
				!largestResendMoveMicrometers.compare_exchange_weak(largest, micrometers,
					std::memory_order_relaxed))
			{
			}
		}

		ResendPose(host, id, pose);
		resentDevices.fetch_or(uint64_t{ 1 } << id, std::memory_order_relaxed);
		resentPoses.fetch_add(1, std::memory_order_release);

		// The device's own pose may have come through meanwhile and reached the
		// runtime before this older one: send again, from the newest, next pass.
		std::unique_lock<std::mutex> lock(poseMutexes[id], std::try_to_lock);
		resendOwed[id] = !lock.owns_lock() || runtimePoses[id].received != received;
	}
}

bool ServerTrackedDeviceProvider::TrySetDeviceTransform(const protocol::SetDeviceTransform &newTransform,
	protocol::RejectReason *reason)
{
	FlushFrameLog();
	protocol::SetDeviceTransform sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newTransform, sanitized))
	{
		LOG("SetDeviceTransform: rejected invalid transform for device id %u", newTransform.openVRID);
		if (reason)
			*reason = protocol::RejectReason::InvalidValues;
		return false;
	}
	if (reason)
		*reason = protocol::RejectReason::None;

	auto &slot = transforms[sanitized.openVRID];
	// The IPC thread is the single writer. Odd means a coherent generation is
	// in flight; payload stores are atomic so a failed reader attempt is safe.
	runtimeSequence.fetch_add(1, std::memory_order_acq_rel);
	slot.Store(sanitized);
	runtimeSequence.fetch_add(1, std::memory_order_release);
	return true;
}

bool ServerTrackedDeviceProvider::TrySetRuntimeState(const protocol::SetRuntimeState &newState,
	protocol::RejectReason *reason)
{
	// The overlay sends its complete state about once a second, so a frame
	// change reaches the log within about that.
	FlushFrameLog();
	if (newState.expectedSessionId != 0 && newState.expectedSessionId != driverSessionId)
	{
		if (reason)
			*reason = protocol::RejectReason::StaleSession;
		return false;
	}
	protocol::SetRuntimeState sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(newState, sanitized))
	{
		LOG("SetRuntimeState: rejected invalid state (enabled=%llx, hidden=%llx, anchors=%u)",
			static_cast<unsigned long long>(newState.enabledMask),
			static_cast<unsigned long long>(newState.hiddenMask),
			newState.field.anchorCount);
		if (reason)
			*reason = protocol::RejectReason::InvalidValues;
		return false;
	}
	if (reason)
		*reason = protocol::RejectReason::None;

	if (sanitized.enabledMask != 0 && sanitized.frameProfileKey != 0)
		recoveryState = sanitized;
	runtimeSequence.fetch_add(1, std::memory_order_acq_rel);
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		protocol::SetDeviceTransform transform = sanitized.transform;
		transform.openVRID = id;
		transform.enabled = (sanitized.enabledMask >> id) & 1;
		transform.hidden = (sanitized.hiddenMask >> id) & 1;
		auto &slot = transforms[id];
		slot.Store(transform, sanitized.frames[id]);
	}

	alignmentField.Store(sanitized.field);
	runtimeSequence.fetch_add(1, std::memory_order_release);
	return true;
}

void ServerTrackedDeviceProvider::ReadRuntimeState(uint32_t openVRID,
	DeviceTransform &out, protocol::SetAlignmentField &field)
{
    const auto snapshot = questcal::runtimesnapshot::Read(runtimeSequence,
        transforms[openVRID], alignmentField, lastGood[openVRID]);
    out = snapshot.transform;
    field = snapshot.field;
}

void ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose,
	const PoseHostCall &host)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;
	std::lock_guard<std::mutex> poseLock(poseMutexes[openVRID]);

	// Publish the raw driver-space pose for the solver, stamped at capture.
	//
	// INVARIANT: this publish must stay ABOVE every pose mutation below,
	// including the poseTimeOffset shift. If the ring recorded rewritten poses,
	// the next solve would find ~zero residual offset and the full-replace
	// transform would erase the latency correction on every recalibration.
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	if (poseRingReady.load(std::memory_order_acquire))
	{
		protocol::DevicePoseSample sample;
		sample.sampleTimeQpc = now.QuadPart;
		sample.deviceId = openVRID;
		sample.trackingResult = static_cast<uint32_t>(pose.result);
		sample.poseIsValid = pose.poseIsValid;
		sample.deviceIsConnected = pose.deviceIsConnected;
		sample.poseTimeOffset = pose.poseTimeOffset;
		sample.worldFromDriverRotation = pose.qWorldFromDriverRotation;
		sample.rotation = pose.qRotation;
		for (int i = 0; i < 3; ++i)
		{
			sample.worldFromDriverTranslation[i] = pose.vecWorldFromDriverTranslation[i];
			sample.position[i] = pose.vecPosition[i];
			sample.velocity[i] = pose.vecVelocity[i];
			sample.angularVelocity[i] = pose.vecAngularVelocity[i];
		}
		poseRing.Publish(sample);
	}

	const vr::DriverPose_t raw = pose;
	DeviceTransform tf;
	protocol::SetAlignmentField field;
	ReadRuntimeState(openVRID, tf, field);
	const bool logFrame = LogFile && tf.control.enabled && pose.poseIsValid &&
		!(lastLoggedFrame[openVRID] == tf.frame) &&
		!frameLogReady[openVRID].load(std::memory_order_acquire);
	vr::HmdVector3d_t inputPosition{};
	vr::HmdQuaternion_t inputRotation{};
	if (logFrame)
	{
		const auto rotated = questcal::driverpose::RotateVector(pose.qWorldFromDriverRotation, pose.vecPosition);
		inputPosition = questcal::driverpose::Add(rotated.v, pose.vecWorldFromDriverTranslation);
		inputRotation = questcal::driverpose::Multiply(pose.qWorldFromDriverRotation, pose.qRotation);
	}

	double nowSeconds = static_cast<double>(now.QuadPart) * qpcToSeconds;
#ifdef QUESTCAL_DRIVER_PROVIDER_TEST_SEAM
	if (poseTimeForTest >= 0.0)
		nowSeconds = poseTimeForTest;
#endif

	questcal::driverpose::ApplyRuntimePose(pose, tf.calibration, tf.frame,
        field, nowSeconds, baseState[openVRID], fieldState[openVRID]);

	if (logFrame)
	{
		// Written by the IPC thread on its next request (FlushFrameLog).
		auto &record = frameLog[openVRID];
		const auto rotated = questcal::driverpose::RotateVector(pose.qWorldFromDriverRotation, pose.vecPosition);
		record.qpc = static_cast<long long>(now.QuadPart);
		record.connected = pose.deviceIsConnected;
		record.generation = tf.control.generation;
		record.inputPosition = inputPosition;
		record.inputRotation = inputRotation;
		record.outputPosition = questcal::driverpose::Add(rotated.v, pose.vecWorldFromDriverTranslation);
		record.outputRotation = questcal::driverpose::Multiply(pose.qWorldFromDriverRotation, pose.qRotation);
		frameLogReady[openVRID].store(true, std::memory_order_release);
		lastLoggedFrame[openVRID] = tf.frame;
	}

	// Hide the HMD-mounted continuous-calibration tracker from applications:
	// it keeps a valid pose (an invalid one would make SteamVR flag it lost) but
	// far above the play area, out of reach of games' tracker auto-assignment.
	if (tf.control.hidden)
		pose.vecWorldFromDriverTranslation[1] += HiddenPoseOffsetY;

	RuntimePoseRecord &record = runtimePoses[openVRID];
	record.raw = raw;
	record.sent = pose;
	record.host = host;
	record.receivedAt = nowSeconds;
	++record.received;
	if (!record.seen)
	{
		record.seen = true;
		recordedDevices.fetch_or(uint64_t{ 1 } << openVRID, std::memory_order_release);
	}
}
