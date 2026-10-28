#include "../Driver/ServerTrackedDeviceProvider.h"
#include "../Driver/ProtocolValidation.h"
#include "../Driver/PoseTransform.h"
#include "../Driver/Logging.h"
#include "../Overlay/TrackerFrameCorrections.h"
#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/DriverSession.h"
#include "../Overlay/DriverSyncTracker.h"
#include "../Overlay/DriverWorker.h"
#include "../Overlay/CalibrationRun.h"
#include "../Overlay/CollectionSource.h"
#include "../Overlay/LighthouseVisibility.h"
#include "../common/MathConstants.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using Check = void (*)(const char *, bool, const char *);
using Q = Eigen::Quaterniond;
using V = Eigen::Vector3d;
constexpr double TickScale = 1e-6;

protocol::DevicePoseSample Sample(uint32_t id, double time, const V &local,
	const Q &frameRotation = Q::Identity(), const V &frameTranslation = V::Zero())
{
	protocol::DevicePoseSample s;
	s.deviceId = id;
	s.sampleTimeQpc = static_cast<int64_t>(std::llround(time / TickScale));
	s.poseIsValid = true;
	s.deviceIsConnected = id == 9; // Four Standable-hidden body trackers.
	s.trackingResult = vr::TrackingResult_Running_OK;
	s.worldFromDriverRotation = questcal::WireQuaternion(frameRotation);
	for (int i = 0; i < 3; ++i)
	{
		s.position[i] = local(i);
		s.worldFromDriverTranslation[i] = frameTranslation(i);
	}
	return s;
}

vr::DriverPose_t DriverPose(const protocol::DevicePoseSample &s)
{
	vr::DriverPose_t pose{};
	pose.poseIsValid = s.poseIsValid;
	pose.deviceIsConnected = s.deviceIsConnected;
	pose.result = static_cast<vr::ETrackingResult>(s.trackingResult);
	pose.qRotation = s.rotation;
	pose.qWorldFromDriverRotation = s.worldFromDriverRotation;
	pose.qDriverFromHeadRotation.w = 1;
	for (int i = 0; i < 3; ++i)
	{
		pose.vecPosition[i] = s.position[i];
		pose.vecWorldFromDriverTranslation[i] = s.worldFromDriverTranslation[i];
		pose.vecVelocity[i] = s.velocity[i];
		pose.vecAngularVelocity[i] = s.angularVelocity[i];
	}
	return pose;
}

V World(const vr::DriverPose_t &pose)
{
	const auto &q = pose.qWorldFromDriverRotation;
	return Q(q.w, q.x, q.y, q.z) * V(pose.vecPosition) + V(pose.vecWorldFromDriverTranslation);
}

Q WorldRotation(const vr::DriverPose_t &pose)
{
	const auto q = questcal::driverpose::Multiply(pose.qWorldFromDriverRotation, pose.qRotation);
	return Q(q.w, q.x, q.y, q.z);
}

questcal::PoseSample Composed(const protocol::DevicePoseSample &s)
{
	// Hidden poses intentionally bypass the connected-only solver gate in
	// driver assertions. The mounted tracker itself is connected.
	auto connected = s;
	connected.deviceIsConnected = true;
	questcal::PoseSample pose;
	TryComposeRingSample(connected, TickScale, pose);
	return pose;
}

struct Fixture
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections corrections;
	std::unique_ptr<ServerTrackedDeviceProvider> driver = std::make_unique<ServerTrackedDeviceProvider>();
	std::unique_ptr<ServerTrackedDeviceProvider> control = std::make_unique<ServerTrackedDeviceProvider>();
	std::array<protocol::DevicePoseSample, 5> raw;
	std::array<protocol::DevicePoseSample, 5> original;
	protocol::SetRuntimeState state;
	bool valid = true;
	size_t moves = 0;
	double time = 20.0;

	Fixture(bool field)
	{
		state.transform.rotation = questcal::WireQuaternion(Q(Eigen::AngleAxisd(0.21, V::UnitY())));
		state.transform.translation = { { 0.3, 0.1, -0.4 } };
		state.transform.scale = 1.12;
		state.field.enabled = field;
		state.field.anchorCount = field ? 2 : 0;
		state.field.anchors[0].position[0] = -0.6;
		state.field.anchors[0].translationDelta[0] = 0.07;
		state.field.anchors[1].position[0] = 1.4;
		state.field.anchors[1].translationDelta[2] = -0.05;
		state.field.anchors[1].rotationDelta = questcal::WireQuaternion(Q(Eigen::AngleAxisd(0.08, V::UnitY())));
		for (uint32_t i = 0; i < raw.size(); ++i)
		{
			state.enabledMask |= uint64_t{1} << (9 + i);
			const V local(-0.5 + i * 0.3, 1.7 - i * 0.3, 0.2 + i * 0.2);
			raw[i] = Sample(9 + i, time, local, Q::Identity(), i < 3 ? V::Zero() : V(0.8, 0, -0.5));
			corrections.Bind(9 + i, "tracker-" + std::to_string(i));
		}
		original = raw;
		Observe();
		Publish();
		valid &= Error() < 1e-10;
	}

	void Observe()
	{
		for (auto &s : raw)
		{
			s.sampleTimeQpc = static_cast<int64_t>(std::llround(time / TickScale));
			watch.Note(s, TickScale, false);
		}
		auto changes = watch.TakeMoves();
		std::sort(changes.begin(), changes.end(), [](const auto &a, const auto &b) { return a.time < b.time; });
		moves += changes.size();
		for (const auto &move : changes)
			valid &= corrections.Follow(move);
	}

	void Publish()
	{
		std::copy(corrections.Snapshot().begin(), corrections.Snapshot().end(), state.frames);
		valid &= driver->TrySetRuntimeState(state);
		auto neutralFrames = state;
		for (auto &frame : neutralFrames.frames) frame = {};
		valid &= control->TrySetRuntimeState(neutralFrames);
	}

	double Error()
	{
		driver->SetPoseTimeForTest(time);
		control->SetPoseTimeForTest(time);
		double error = 0;
		for (size_t i = 0; i < raw.size(); ++i)
		{
			auto actual = DriverPose(raw[i]), expected = DriverPose(original[i]);
			driver->HandleDevicePoseUpdated(raw[i].deviceId, actual);
			control->HandleDevicePoseUpdated(raw[i].deviceId, expected);
			error = (std::max)(error, (World(actual) - World(expected)).norm());
			error = (std::max)(error, WorldRotation(actual).angularDistance(WorldRotation(expected)));
		}
		return error;
	}

	void MoveFrame(unsigned mask, const Q &rotation, const V &translation)
	{
		time += 0.01;
		for (size_t i = 0; i < raw.size(); ++i)
		{
			if (!(mask & (1u << i))) continue;
			auto &s = raw[i];
			const auto parts = UnpackRingSample(s);
			s.worldFromDriverRotation = questcal::WireQuaternion((rotation * parts.wfdRot).normalized());
			const V origin = rotation * parts.wfdTrans + translation;
			for (int axis = 0; axis < 3; ++axis) s.worldFromDriverTranslation[axis] = origin(axis);
		}
		Observe();
		Publish();
	}
};

void SplitFrames(Check check)
{
	const Q rotation = Eigen::AngleAxisd(9.79 * questcal::Pi / 180.0, V::UnitY()) *
		Eigen::AngleAxisd(74.08 * questcal::Pi / 180.0, V::UnitX());
	const V pivot(0, 1.7, 0);
	const V translation = pivot + V(0, 0, 0.538) - rotation * pivot;
	for (const unsigned mask : { 31u, 7u, 1u })
	for (const bool field : { false, true })
	{
		Fixture f(field);
		f.MoveFrame(mask, rotation, translation);
		double error = f.Error();
		const size_t expectedMoves = mask == 31 ? 5 : mask == 7 ? 3 : 1;
		bool pass = f.valid && f.moves == expectedMoves && error < 1e-9;
		// A new full snapshot (including a shared continuous correction) must
		// retain N, and field/base histories must follow the same trajectory.
		f.state.transform.translation.v[0] += 0.025;
		f.state.field.anchors[0].translationDelta[0] += 0.015;
		for (int step = 0; step < 100; ++step)
		{
			f.time += 0.01;
			f.Observe();
			f.Publish();
			error = (std::max)(error, f.Error());
		}
		pass &= f.moves == expectedMoves && error < 1e-9;
		// Reverse repeatedly, then move the other pair instead.
		for (int step = 0; step < 8; ++step)
		{
			const bool undo = step % 2 == 0;
			const Q r = undo ? rotation.conjugate() : rotation;
			const V t = undo ? V(-(rotation.conjugate() * translation)) : translation;
			f.MoveFrame(mask, r, t);
			error = (std::max)(error, f.Error());
		}
		f.MoveFrame(24, rotation, translation);
		error = (std::max)(error, f.Error());
		char detail[128];
		snprintf(detail, sizeof detail, "mask=%u, field=%d, maximum error %.3g", mask, field, error);
		check(field ? "frame correction: split/common/reversed frames with spatial field" :
			"frame correction: split/common/reversed frames survive shared updates",
			pass && f.valid && error < 1e-9, detail);
	}
}

void ReexpressionAndRecovery(Check check)
{
	Fixture f(false);
	const Q r(Eigen::AngleAxisd(0.7, V::UnitX()));
	const V t(0.1, 0.2, 0.3);
	f.MoveFrame(7, r, t);
	const auto before = f.corrections.Snapshot();
	const size_t moves = f.moves;
	auto &s = f.raw[0];
	const auto world = Composed(s);
	const Q newFrame(Eigen::AngleAxisd(-0.6, V::UnitY()));
	const V newOrigin(1, 0, 2);
	s.worldFromDriverRotation = questcal::WireQuaternion(newFrame);
	s.rotation = questcal::WireQuaternion(newFrame.conjugate() * world.rot);
	const V local = newFrame.conjugate() * (world.pos - newOrigin);
	for (int axis = 0; axis < 3; ++axis)
	{
		s.position[axis] = local(axis);
		s.worldFromDriverTranslation[axis] = newOrigin(axis);
	}
	f.time += 0.01;
	f.Observe();
	f.Publish();
	check("frame correction: re-expression preserves accumulated normalization",
		f.valid && f.moves == moves && before == f.corrections.Snapshot() && f.Error() < 1e-9, "");

	// A station proves a move while the mounted tracker is off. Only its
	// frame component is cancelled on return; an additional local jump stays.
	for (bool returned : { false, true })
	for (bool localJump : { false, true })
	{
		LighthouseFrameWatch watch;
		questcal::TrackerFrameCorrections corrections;
		auto tracker = Sample(9, 10, V(0, 1.7, 0));
		auto station = Sample(1, 10, V::Zero());
		watch.Note(tracker, TickScale, false);
		watch.Note(station, TickScale, true);
		if (returned)
		{
			tracker.poseIsValid = tracker.deviceIsConnected = false;
			tracker.sampleTimeQpc += 10000;
			watch.Note(tracker, TickScale, false);
		}
		const V offset = localJump ? V(0.2, 0, 0) : V::Zero();
		tracker = Sample(9, 10.02, V(0, 1.7, 0) + offset, r, t);
		watch.Note(tracker, TickScale, false);
		// The station's corroboration can arrive after the tracker's pose.
		station = Sample(1, 10.03, V::Zero(), r, t);
		watch.Note(station, TickScale, true);
		auto changes = watch.TakeMoves();
		bool pass = changes.size() == 1 && changes.front().returned == returned;
		for (const auto &move : changes) pass &= corrections.Follow(move);
		auto normalized = Composed(tracker);
		pass &= corrections.Normalize(9, normalized);
		watch.Note(tracker, TickScale, false);
		check(localJump ? "frame correction: a proven frame move preserves an independent local jump" :
			"frame correction: live/returning tracker follows a proven station move once",
			pass && (normalized.pos - V(0, 1.7, 0) - offset).norm() < 1e-9 && watch.TakeMoves().empty(), "");
	}
}

void SessionPublication(Check check)
{
	questcal::DriverSession session;
	protocol::SetRuntimeState received;
	unsigned snapshots = 0;
	session.SetTransport([&](const protocol::Request &request)
	{
		questcal::DriverTransportResult result;
		result.completed = true;
		result.connectionGeneration = 1;
		result.response.type = request.type == protocol::RequestHandshake ?
			protocol::ResponseHandshake : protocol::ResponseSuccess;
		if (request.type == protocol::RequestSetRuntimeState)
		{
			received = request.setRuntimeState;
			++snapshots;
		}
		return result;
	});
	session.SetErrorSink([](const std::string &) {}, []() {});
	session.SetDeviceEnumerator([](uint32_t id, const questcal::DriverSyncDesired &)
	{
		questcal::SyncDevice device;
		device.id = id;
		if (id == 0 || id == 9)
		{
			device.deviceClass = id == 0 ? questcal::SyncDeviceClass::Hmd : questcal::SyncDeviceClass::Other;
			device.trackingSystemKnown = true;
			device.trackingSystem = id == 0 ? "reference" : "lighthouse";
		}
		return device;
	});
	questcal::DriverApplyRequest request;
	request.enabled = true;
	request.desired.referenceTrackingSystem = "reference";
	request.desired.targetTrackingSystem = "lighthouse";
	const auto before = request;
	request.frames[9].translation.v[0] = 0.4;
	bool pass = !(before == request) && session.Apply(request, 1).synchronized;
	pass &= received.frames[9] == request.frames[9] && received.enabledMask == (uint64_t{1} << 9);
	request.desired.translationMeters.x() = 0.03;
	pass &= session.Apply(request, 2).synchronized && received.frames[9] == request.frames[9];
	check("frame correction: complete session publications retain per-device state",
		pass && snapshots == 2 && received.transform.translation.v[0] == 0.03, "");
}

void LifetimeAndValidation(Check check)
{
	questcal::TrackerFrameCorrections frames;
	frames.Bind(9, "first");
	LighthouseFrameWatch::Move move;
	move.id = 9;
	move.time = 5;
	move.rotation = Q(Eigen::AngleAxisd(0.4, V::UnitY()));
	move.translation = V(0.5, 0, -0.3);
	bool pass = frames.Follow(move);
	const auto snapshot = frames.Snapshot();
	questcal::PoseSample old;
	old.time = 4.99;
	pass &= !frames.Normalize(9, old);
	move.time = 4;
	pass &= !frames.Follow(move) && frames.Snapshot() == snapshot;
	check("frame correction: backlog and out-of-order inferred moves cannot mix frames", pass, "");
	pass = !frames.Bind(9, "") && !frames.Bind(9, "first") && frames.Snapshot() == snapshot;
	pass &= frames.Bind(9, "replacement") && frames.Snapshot()[9] == protocol::FrameCorrection{};
	pass &= frames.Snapshot()[10] == protocol::FrameCorrection{};
	move.time = 6;
	pass &= frames.Follow(move);
	frames.Reset();
	check("frame correction: reconnect retains state; new slot and session start neutral",
		pass && frames.Snapshot()[9] == protocol::FrameCorrection{}, "");

	protocol::SetRuntimeState input, output;
	input.enabledMask = uint64_t{1} << 9;
	input.frames[9] = snapshot[9];
	pass = questcal::driverinput::ValidateAndSanitize(input, output);
	const auto saved = output.frames[9];
	input.frames[63].translation.v[2] = std::numeric_limits<double>::quiet_NaN();
	pass &= !questcal::driverinput::ValidateAndSanitize(input, output) && output.frames[9] == saved;
	input.frames[63] = {};
	input.frames[9].rotation = {0, 0, 0, 0};
	pass &= !questcal::driverinput::ValidateAndSanitize(input, output) && output.frames[9] == saved;
	check("frame correction: malformed snapshots are rejected atomically", pass, "");
}

void RecalibrationSpace(Check check)
{
	Fixture f(false);
	f.MoveFrame(7, Q(Eigen::AngleAxisd(0.8, V::UnitX())), V(0.2, 0.6, -0.1));
	const auto atStart = f.corrections.Snapshot()[9];
	const Q oldRotation(f.state.transform.rotation.w, f.state.transform.rotation.x,
		f.state.transform.rotation.y, f.state.transform.rotation.z);
	const V oldTranslation(f.state.transform.translation.v);
	const Q n(atStart.rotation.w, atStart.rotation.x, atStart.rotation.y, atStart.rotation.z);
	// The raw solve's reference transform at the first accepted target pose.
	Q solved = oldRotation * n;
	V translated = oldTranslation + f.state.transform.scale * (oldRotation * V(atStart.translation.v));
	// Both the selected tracker and another group change frames mid-run.
	f.MoveFrame(7, Q(Eigen::AngleAxisd(0.2, V::UnitY())), V(-0.1, 0, 0.2));
	f.MoveFrame(24, Q(Eigen::AngleAxisd(-0.3, V::UnitZ())), V(0, 0.1, 0));
	const auto before = f.corrections.Snapshot();
	questcal::TrackerFrameCorrections::ExpressCalibration(atStart, solved, translated, f.state.transform.scale);
	f.state.transform.rotation = questcal::WireQuaternion(solved);
	f.state.transform.translation = questcal::WireVector(translated);
	++f.state.transform.generation;
	f.Publish();
	check("frame lifecycle: recalibration preserves split frames including moves during collection",
		f.valid && f.Error() < 1e-9 && solved.angularDistance(oldRotation) < 1e-9 &&
		(translated - oldTranslation).norm() < 1e-9 && before == f.corrections.Snapshot(), "");
	// An aborted measurement publishes no solve, while frame following continues.
	f.MoveFrame(8, Q(Eigen::AngleAxisd(0.1, V::UnitY())), V(0, 0, 0.3));
	check("frame lifecycle: cancelled measurement retains corrections observed while measuring",
		f.valid && f.Error() < 1e-9, "");
}

void RestartRecovery(Check check)
{
	Fixture f(false);
	f.driver->SetSessionForTest(41);
	f.control->SetSessionForTest(41);
	f.MoveFrame(7, Q(Eigen::AngleAxisd(0.6, V::UnitX())), V(0.2, 0.4, 0.1));
	f.state.frameProfileKey = 73;
	f.state.expectedSessionId = 41;
	for (uint32_t id = 9; id < 14; ++id)
		f.state.frameSerialKeys[id] = questcal::FrameIdentityKey("tracker-" + std::to_string(id - 9));
	f.Publish();
	// A temporary neutralization for runtime-pose collection must not erase
	// the checkpoint, even if the overlay exits during that measurement.
	f.valid &= f.driver->TrySetDeviceTransform(protocol::SetDeviceTransform(9, false));
	questcal::DriverSession session;
	session.SetErrorSink([](const std::string &) {}, [] {});
	session.SetDeviceEnumerator([](uint32_t id, const questcal::DriverSyncDesired &) {
		questcal::SyncDevice device;
		device.id = id;
		if (id == 0 || (id >= 9 && id < 14))
		{
			device.deviceClass = id == 0 ? questcal::SyncDeviceClass::Hmd : questcal::SyncDeviceClass::Other;
			device.trackingSystemKnown = true;
			device.trackingSystem = id == 0 ? "reference" : "lighthouse";
		}
		return device;
	});
	int reads = 0, writes = 0;
	session.SetTransport([&](const protocol::Request &request) {
		questcal::DriverTransportResult result;
		result.completed = true;
		result.connectionGeneration = 1;
		f.driver->GetRuntimeState(result.response);
		if (request.type == protocol::RequestHandshake)
			result.response.type = protocol::ResponseHandshake;
		else if (request.type == protocol::RequestGetRuntimeState)
		{
			++reads;
			result.response.type = protocol::ResponseRuntimeState;
		}
		else
		{
			++writes;
			result.response.type = f.driver->TrySetRuntimeState(request.setRuntimeState)
				? protocol::ResponseSuccess : protocol::ResponseInvalid;
		}
		return result;
	});
	questcal::DriverApplyRequest request;
	request.enabled = true;
	request.recoverFrames = true;
	request.frameProfileKey = 73;
	request.frameBoundMask = (uint64_t{31} << 9);
	request.desired.referenceTrackingSystem = "reference";
	request.desired.targetTrackingSystem = "lighthouse";
	request.desired.rotation = Q(f.state.transform.rotation.w, f.state.transform.rotation.x,
		f.state.transform.rotation.y, f.state.transform.rotation.z);
	request.desired.translationMeters = V(f.state.transform.translation.v);
	request.desired.scale = f.state.transform.scale;
	std::copy(std::begin(f.state.frameSerialKeys), std::end(f.state.frameSerialKeys), request.frameSerialKeys.begin());
	const auto applied = session.Apply(request, 30);
	check("frame lifecycle: overlay restart restores driver checkpoint before first publication",
		f.valid && applied.synchronized && applied.recoveryChecked && applied.framesRecovered &&
		reads == 1 && writes == 1 && f.Error() < 1e-9, "");
	protocol::Response snapshot(protocol::ResponseRuntimeState);
	f.driver->GetRuntimeState(snapshot);
	questcal::RecoveredFrames restored{};
	auto serials = request.frameSerialKeys;
	std::swap(serials[9], serials[14]);
	bool pass = questcal::RecoverTrackerFrames(snapshot, 41, 73, serials, restored);
	pass &= restored[9] == protocol::FrameCorrection{} && restored[14] == applied.frames[9];
	serials[14] = questcal::FrameIdentityKey("replacement");
	pass &= questcal::RecoverTrackerFrames(snapshot, 41, 73, serials, restored) &&
		restored[14] == protocol::FrameCorrection{};
	check("frame lifecycle: recovery follows serials and excludes replacement devices", pass, "");
	questcal::FrameSerialKeys sleeping{}, recoveredKeys{};
	questcal::RecoveredFrames sleepingFrames{};
	pass = questcal::RecoverTrackerFrames(snapshot, 41, 73, sleeping, sleepingFrames, &recoveredKeys);
	questcal::TrackerFrameCorrections waking;
	waking.Restore(sleepingFrames, recoveredKeys);
	waking.Bind(14, "tracker-0");
	pass &= waking.Snapshot()[14] == applied.frames[9] && waking.Snapshot()[9] == protocol::FrameCorrection{};
	waking.Bind(10, "replacement");
	pass &= waking.Snapshot()[10] == protocol::FrameCorrection{};
	check("frame lifecycle: sleeping tracker retains recovery until its serial appears", pass, "");
	// A different live device can occupy the sleeping tracker's old slot.
	sleeping[9] = questcal::FrameIdentityKey("replacement");
	pass = questcal::RecoverTrackerFrames(snapshot, 41, 73, sleeping, sleepingFrames, &recoveredKeys);
	waking.Reset();
	waking.Restore(sleepingFrames, recoveredKeys);
	waking.Bind(9, "replacement");
	waking.Bind(14, "tracker-0");
	pass &= waking.Snapshot()[9] == protocol::FrameCorrection{} && waking.Snapshot()[14] == applied.frames[9];
	check("frame lifecycle: a sleeping tracker survives reuse of its previous slot", pass, "");
	const auto activeRequest = request;
	request.frameSerialKeys = {};
	request.frameBoundMask = 0;
	const auto parked = session.Apply(request, 30.5);
	protocol::Response parkedSnapshot(protocol::ResponseRuntimeState);
	f.driver->GetRuntimeState(parkedSnapshot);
	pass = parked.synchronized && parked.framesRecovered;
	for (uint32_t id = 9; id < 14; ++id)
		pass &= !parked.targetDeviceMask[id] && parked.frames[id] == applied.frames[id];
	// A disabled publication retains the active checkpoint for later binding.
	pass &= parkedSnapshot.runtimeState.frames[9] == applied.frames[9];
	check("frame lifecycle: unidentified recovered corrections cannot drive runtime slots", pass, "");
	request = activeRequest;
	const auto before = restored;
	pass = !questcal::RecoverTrackerFrames(snapshot, 42, 73, serials, restored) && restored == before;
	pass &= !questcal::RecoverTrackerFrames(snapshot, 41, 74, serials, restored) && restored == before;
	request.driverSessionId = 41;
	f.driver->SetSessionForTest(42);
	pass &= !session.Apply(request, 31).synchronized && writes == 2;
	protocol::RejectReason reason = protocol::RejectReason::None;
	pass &= !f.driver->TrySetRuntimeState(f.state, &reason) &&
		reason == protocol::RejectReason::StaleSession;
	check("frame lifecycle: a SteamVR restart or profile change cannot import old corrections", pass, "");
	serials = request.frameSerialKeys;
	snapshot.runtimeState.frames[9].translation.v[0] = std::numeric_limits<double>::quiet_NaN();
	check("frame lifecycle: invalid recovered frame leaves all output unchanged",
		!questcal::RecoverTrackerFrames(snapshot, 41, 73, serials, restored) && restored == before, "");
}

void RecoveryProtocolGate(Check check)
{
	IPCServer server;
	int reads = 0, writes = 0;
	IPCServer::RequestSink sink;
	sink.setDeviceTransform = [&](const protocol::SetDeviceTransform &) { ++writes; return protocol::RejectReason::None; };
	sink.setRuntimeState = [&](const protocol::SetRuntimeState &) { ++writes; return protocol::RejectReason::None; };
	sink.poseHookMask = [] { return uint32_t{7}; };
	sink.getRuntimeState = [&](protocol::Response &response) {
		++reads;
		response.driverSessionId = 41;
		response.runtimeState.frameProfileKey = 73;
		response.runtimeState.frames[9].translation.v[0] = 0.3;
	};
	server.SetSinkForTest(std::move(sink));
	questcal::ipc::ConnectionState connection;
	protocol::Response response;
	protocol::Request read(protocol::RequestGetRuntimeState);
	server.DispatchForTest(read, response, connection);
	bool pass = response.type == protocol::ResponseInvalid && reads == 0;
	server.DispatchForTest(protocol::Request(protocol::RequestHandshake), response, connection);
	pass &= response.type == protocol::ResponseHandshake && response.driverSessionId == 41 &&
		response.runtimeState.frameProfileKey == 0 && reads == 1;
	server.DispatchForTest(read, response, connection);
	pass &= response.type == protocol::ResponseRuntimeState && response.driverSessionId == 41 &&
		response.runtimeState.frameProfileKey == 73 && response.runtimeState.frames[9].translation.v[0] == 0.3 && reads == 2;
	read.protocol.version = protocol::Version - 1;
	server.DispatchForTest(read, response, connection);
	pass &= response.type == protocol::ResponseInvalid && reads == 2 && writes == 0;
	check("frame lifecycle: checkpoint reads require the exact-version connection handshake", pass, "");
}

void ContinuousSpace(Check check)
{
	Fixture f(true);
	const Q r(Eigen::AngleAxisd(1.29, V::UnitX()));
	const V t(0, 1.2, -0.5);
	f.MoveFrame(7, r, t);
	questcal::ContinuousAlignment engine;
	questcal::MountExtrinsic mount;
	mount.valid = true;
	engine.SetExtrinsic(mount);
	bool pass = true;
	for (int i = 0; i < 4000; ++i)
	{
		const double time = 30.0 + i * 0.01;
		auto sample = f.raw[0];
		sample.sampleTimeQpc = static_cast<int64_t>(std::llround(time / TickScale));
		auto target = Composed(sample);
		pass &= f.corrections.Normalize(9, target);
		auto reference = Composed(f.original[0]);
		reference.time = time;
		engine.PushReference(reference);
		engine.PushTarget(target);
		engine.Update(questcal::RingTime(time), Q::Identity(), V::Zero(), 1.0, 0.0);
	}
	questcal::ContinuousAlignment::Correction correction;
	const auto diagnostics = engine.GetDiagnostics();
	check("frame correction: continuous calibration observes normalized target poses",
		pass && diagnostics.observations > 0 && !engine.PollReanchor(correction) &&
		(!engine.PollCorrection(correction) || correction.translation.norm() < 1e-9), "");
}

void PausedMonitorAndRecalibration(Check check)
{
	// SteamVR re-solves a station while the monitor is paused and a body
	// tracker is switched off. The watch keeps every device's frame through
	// the pause, so the station and the resting headset tracker prove the move
	// on their first samples after it, and the returning tracker gets the same.
	const Q r(Eigen::AngleAxisd(0.35, V::UnitY()) * Eigen::AngleAxisd(0.05, V::UnitZ()));
	const V t(0.6, 0.0, -0.4);
	const V headLocal(0.1, 1.6, 0.2), bodyLocal(0.3, 0.9, -0.4), otherLocal(-0.5, 0.4, 0.7);
	const Q otherFrame(Eigen::AngleAxisd(-0.4, V::UnitY()));
	const V otherOrigin(1.5, 0.0, 2.0);
	LighthouseFrameWatch watch;
	auto head = Sample(9, 10, headLocal);
	auto body = Sample(14, 10, bodyLocal);
	auto other = Sample(15, 10, otherLocal, otherFrame, otherOrigin);
	auto station = Sample(1, 10, V::Zero());
	watch.Note(head, TickScale, false);
	watch.Note(body, TickScale, false);
	watch.Note(other, TickScale, false);
	watch.Note(station, TickScale, true);
	const V headWorld = Composed(head).pos, bodyWorld = Composed(body).pos;
	body.poseIsValid = body.deviceIsConnected = false;
	body.sampleTimeQpc += 500000;
	watch.Note(body, TickScale, false);
	station = Sample(1, 300, V::Zero(), r, t);
	head = Sample(9, 300.01, headLocal, r, t);
	body = Sample(14, 300.02, bodyLocal, r, t);
	watch.Note(station, TickScale, true);
	watch.Note(head, TickScale, false);
	watch.Note(body, TickScale, false);
	auto moves = watch.TakeMoves();
	std::sort(moves.begin(), moves.end(), [](const auto &a, const auto &b) { return a.time < b.time; });
	questcal::TrackerFrameCorrections frames;
	frames.Bind(9, "head");
	frames.Bind(14, "body");
	frames.Bind(15, "other");
	bool pass = moves.size() == 2;
	for (const auto &move : moves)
		pass &= frames.Follow(move);
	auto headNow = Composed(head), bodyNow = Composed(body);
	pass &= frames.Normalize(9, headNow) && frames.Normalize(14, bodyNow);
	check("frame correction: a move made while the monitor is paused reaches a switched-off tracker",
		pass && (headNow.pos - headWorld).norm() < 1e-9 && (bodyNow.pos - bodyWorld).norm() < 1e-9, "");

	// Had the body tracker missed that move, a recalibration on the headset
	// tracker realigns it: the watch saw both last in the same station's frame.
	// The tracker in another station's frame keeps its own correction.
	questcal::TrackerFrameCorrections missed;
	missed.Bind(9, "head");
	missed.Bind(14, "body");
	missed.Bind(15, "other");
	LighthouseFrameWatch::Move split;
	split.id = 15;
	split.time = 200;
	split.rotation = Q(Eigen::AngleAxisd(0.2, V::UnitX()));
	split.translation = V(0.1, 0.2, 0.3);
	pass = missed.Follow(split);
	for (const auto &move : moves)
		if (move.id == 9)
			pass &= missed.Follow(move);
	const auto otherBefore = missed.Snapshot()[15];
	Q headRotation, frameRotation;
	V headTranslation, frameTranslation;
	pass &= watch.LastFrame(9, headRotation, headTranslation) && !watch.LastFrame(1, frameRotation, frameTranslation);
	int aligned = 0;
	for (const uint32_t id : { 14u, 15u })
		if (watch.LastFrame(id, frameRotation, frameTranslation) &&
			!questcal::WorldFromDriverChanged(frameRotation, frameTranslation, headRotation, headTranslation) &&
			missed.Align(id, 9))
			++aligned;
	auto bodyAligned = Composed(body);
	pass &= aligned == 1 && missed.Normalize(14, bodyAligned) && !missed.Align(14, 9);
	check("frame lifecycle: recalibration realigns trackers in the target's frame only",
		pass && (bodyAligned.pos - bodyWorld).norm() < 1e-9 && missed.Snapshot()[15] == otherBefore, "");

	// A recalibration whose target frame SteamVR moved mid-run: the run carries
	// the move back to its start frame, and expressing the result with the
	// start N is right only when the watch followed that same move.
	questcal::CalibrationRun::TargetFrame run;
	const ringpose::DriverLocalPoseSample first{ 10.0, Q::Identity(), headLocal, V::Zero(), V::Zero() };
	const ringpose::DriverLocalPoseSample second{ 10.011, Q::Identity(), headLocal, V::Zero(), V::Zero() };
	run.Accept(first, Q::Identity(), V::Zero());
	pass = run.Accept(second, r, t) == questcal::CalibrationRun::TargetFrame::Verdict::Moved;
	questcal::TrackerFrameCorrections start;
	start.Bind(9, "head");
	LighthouseFrameWatch::Move earlier = split;
	earlier.id = 9;
	earlier.time = 5.0;
	pass &= start.Follow(earlier);
	const auto nStart = start.Snapshot()[9];
	const Q startRotation(nStart.rotation.w, nStart.rotation.x, nStart.rotation.y, nStart.rotation.z);
	const V startTranslation(nStart.translation.v);
	LighthouseFrameWatch::Move during;
	during.id = 9;
	during.time = 10.011;
	during.rotation = r;
	during.translation = t;
	auto followed = start;
	pass &= followed.Follow(during);
	pass &= questcal::TrackerFrameCorrections::FollowedRun(startRotation, startTranslation,
		run.toStartRot, run.toStartTrans, followed.Snapshot()[9]);
	pass &= !questcal::TrackerFrameCorrections::FollowedRun(startRotation, startTranslation,
		run.toStartRot, run.toStartTrans, start.Snapshot()[9]);
	check("frame lifecycle: a recalibration requires the watch to have followed the run's frame moves", pass, "");
}

void DeferredFrameLog(Check check)
{
	// A pose callback only records a frame change; the IPC thread's next
	// request writes it, so a pose never waits on the log file.
	FILE *const saved = LogFile;
	const auto path = std::filesystem::temp_directory_path() / "questcal-frame-log-test.txt";
	FILE *file = nullptr;
	bool pass = fopen_s(&file, path.string().c_str(), "w+") == 0 && file != nullptr;
	std::string written;
	if (file)
	{
		LogFile = file;
		auto driver = std::make_unique<ServerTrackedDeviceProvider>();
		protocol::SetRuntimeState state;
		state.enabledMask = uint64_t{1} << 9;
		state.frames[9].translation.v[0] = 0.25;
		pass &= driver->TrySetRuntimeState(state);
		driver->SetPoseTimeForTest(20);
		for (int i = 0; i < 2; ++i)
		{
			auto pose = DriverPose(Sample(9, 20 + i * 0.01, V(0.1, 1.5, 0.2)));
			driver->HandleDevicePoseUpdated(9, pose);
		}
		std::fseek(file, 0, SEEK_END);
		pass &= std::ftell(file) == 0;
		pass &= driver->TrySetRuntimeState(state) && driver->TrySetRuntimeState(state);
		std::fflush(file);
		std::fseek(file, 0, SEEK_SET);
		char buffer[1024] = {};
		written.assign(buffer, std::fread(buffer, 1, sizeof buffer - 1, file));
		LogFile = saved;
		std::fclose(file);
		std::error_code ignored;
		std::filesystem::remove(path, ignored);
	}
	const size_t first = written.find("frame applied device 9 ");
	check("driver: frame-change log lines are written off the pose callback, once per change",
		pass && first != std::string::npos && written.find("frame applied", first + 1) == std::string::npos,
		written.c_str());
}

// Devices coming into a frame with no move to follow (TrackerFrameCorrections::
// Join), replayed against the truth. SteamVR's estimate of each station's pose
// carries an error, which a frame move multiplies by its delta; a device in a
// station's frame reports its true pose under that error. Every error starts
// as the identity, as at a calibration, so a device's correction is right when
// it undoes the error of the frame it is in, and Error() measures how far the
// corrected pose is from the truth.
struct JoinRig
{
	struct Pose
	{
		Q rotation = Q::Identity();
		V translation = V::Zero();
	};
	struct Device
	{
		int station = 0;      // whose frame it is in; -1 when it does not track
		V truth = V::Zero();
		bool connected = true;
	};
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	std::vector<Pose> stations;   // the true poses
	std::vector<Pose> estimates;  // SteamVR's
	std::map<uint32_t, Device> devices;
	uint32_t headset = vr::k_unTrackedDeviceIndexInvalid;
	uint32_t keep = vr::k_unTrackedDeviceIndexInvalid;
	bool join = true;
	double time = 10.0;
	std::vector<LighthouseFrameWatch::Switch> switches;
	std::vector<questcal::TrackerFrameCorrections::Joined> joined;
	std::vector<std::string> kept;
	size_t moves = 0;

	JoinRig(std::initializer_list<Pose> poses) : stations(poses), estimates(poses) {}

	// Where the device's true pose sits in its station's true frame.
	Pose Local(const Device &d) const
	{
		const Pose &b = stations[static_cast<size_t>(d.station)];
		return { b.rotation.conjugate(), b.rotation.conjugate() * (d.truth - b.translation) };
	}

	void Tick()
	{
		time += 0.05;
		for (size_t k = 0; k < estimates.size(); ++k)
			watch.Note(Sample(1 + static_cast<uint32_t>(k), time, V::Zero(), estimates[k].rotation,
				estimates[k].translation), TickScale, true);
		for (const auto &[id, d] : devices)
		{
			protocol::DevicePoseSample s;
			if (d.station < 0)
			{
				s = Sample(id, time, V::Zero());
				s.poseIsValid = false;
				s.trackingResult = vr::TrackingResult_Running_OutOfRange;
			}
			else
			{
				const Pose local = Local(d);
				const Pose &w = estimates[static_cast<size_t>(d.station)];
				s = Sample(id, time, local.translation, w.rotation, w.translation);
				s.rotation = questcal::WireQuaternion(local.rotation);
			}
			s.deviceIsConnected = d.connected;
			watch.Note(s, TickScale, false);
		}
		auto handed = watch.TakeMoves();
		std::stable_sort(handed.begin(), handed.end(), [](const auto &a, const auto &b) { return a.time < b.time; });
		for (const auto &move : handed)
			moves += frames.Follow(move) ? 1 : 0;
		for (const auto &change : watch.TakeSwitches())
		{
			if (!change.held)
				switches.push_back(change);
			if (!join)
				continue;
			const auto result = frames.Join(change, watch, headset, keep,
				[this](uint32_t id) { return devices.count(id) > 0; });
			joined.insert(joined.end(), result.joined.begin(), result.joined.end());
			// Refused for now, as JoinSwitches does: given back, said once.
			if (result.retry)
				watch.Hold(change);
			if (result.kept && !(result.retry && change.held))
				kept.emplace_back(result.kept);
		}
	}

	void RunUntil(double until)
	{
		while (time < until - 1e-9)
			Tick();
	}

	// SteamVR moves a station by `delta` (new world = delta * old world).
	void MoveStation(size_t k, const Pose &delta)
	{
		estimates[k].rotation = (delta.rotation * estimates[k].rotation).normalized();
		estimates[k].translation = delta.rotation * estimates[k].translation + delta.translation;
	}

	double Error(uint32_t id) const
	{
		const Device &d = devices.at(id);
		const Pose &e = estimates[static_cast<size_t>(d.station)];
		const Pose &b = stations[static_cast<size_t>(d.station)];
		// The raw pose under SteamVR's error, then the device's correction.
		const V raw = e.rotation * (b.rotation.conjugate() * (d.truth - b.translation)) + e.translation;
		return (questcal::TrackerFrameCorrections::Apply(frames.Snapshot()[id], raw) - d.truth).norm();
	}
};

// The world delta between two frames the session log printed for one move.
JoinRig::Pose LoggedMove(const Q &from, const V &fromT, const Q &to, const V &toT)
{
	const Q rotation = (to * from.conjugate()).normalized();
	return { rotation, toT - rotation * fromT };
}

// Two of the live log's stations (2026-10-03, rc.5): S-3 (9BC4FA08), the
// frame every device was in, and S-13 (3E3828F4).
JoinRig::Pose StationS3() { return { Q(0.224784, -0.008968, -0.958357, -0.175906).normalized(), V(-2.3537, -0.0951, -3.7054) }; }
JoinRig::Pose StationS13() { return { Q(-0.943541, 0.240049, 0.209179, 0.091385).normalized(), V(-2.6639, 0.0517, 0.1031) }; }

// 00:45:11, S-3: "Moving base 9BC4FA08 44mm and 0.4 deg ... Setting universe
// tilt from 9BC4FA08", 7.4 cm at every device in its frame.
JoinRig::Pose S3MoveAt0045()
{
	return LoggedMove(Q(0.225687, -0.004539, -0.958011, -0.176807), V(-2.3442, -0.1250, -3.6944),
		Q(-0.224575, 0.004334, 0.958272, 0.176813), V(-2.3582, -0.0580, -3.7213));
}

void FrameJoins(Check check)
{
	// 00:44:48 the chest tracker (hidden by Standable, so a tracking loss is a
	// device away) bootstrapped from S-13; 00:45:11 SteamVR moved S-3 under
	// every other device ("Moving base 9BC4FA08 44mm ... Setting universe
	// tilt", 7.4 cm at the tracker); 00:48:38 it bootstrapped back into S-3.
	const auto chest = [](bool join)
	{
		JoinRig rig{ StationS3(), StationS13() };
		rig.join = join;
		rig.headset = 9;
		rig.devices[9] = { 0, V(-1.530, -0.433, -1.262), true };
		rig.devices[16] = { 0, V(-1.483, -0.805, -1.293), false };
		rig.devices[17] = { 0, V(-1.305, -1.140, -1.453), false };
		rig.devices[18] = { 0, V(-1.387, -2.081, -1.639), false };
		rig.RunUntil(12.0);
		rig.devices[16].station = -1;
		rig.RunUntil(13.0);
		rig.devices[16].station = 1;
		rig.RunUntil(20.0);
		rig.MoveStation(0, S3MoveAt0045());
		rig.RunUntil(24.0);
		rig.devices[16].station = -1;
		rig.RunUntil(25.0);
		rig.devices[16].station = 0;
		rig.RunUntil(32.0);
		return rig;
	};
	{
		const JoinRig old = chest(false), fixed = chest(true);
		int returns = 0;
		for (const auto &s : fixed.switches)
			returns += s.id == 16 && s.cause == LighthouseFrameWatch::Switch::Cause::Returned ? 1 : 0;
		const bool took = fixed.joined.size() == 1 && fixed.joined[0].id == 16 && fixed.joined[0].source == 9 &&
			fixed.joined[0].fromHeadsetTracker;
		const bool aloneFirst = std::find(fixed.kept.begin(), fixed.kept.end(),
			std::string("no other calibrated device is in that frame")) != fixed.kept.end();
		double worst = 0;
		for (const uint32_t id : { 9u, 16u, 17u, 18u })
			worst = (std::max)(worst, fixed.Error(id));
		char detail[192];
		snprintf(detail, sizeof detail, "without the join %.1f cm off; with it %.3g m worst, %d returns, %zu joined, "
			"alone in S-13 %d", old.Error(16) * 100.0, worst, returns, fixed.joined.size(), aloneFirst);
		check("frame join: a body tracker back in the headset tracker's moved frame takes its correction (live 00:45)",
			old.Error(16) > 0.07 && old.Error(16) < 0.08 && worst < 1e-9 && returns == 2 && took && aloneFirst &&
			fixed.moves == old.moves, detail);
	}

	// 02:06:20 the left Knuckles (connected) jumped into S-13's frame with a
	// new solution; S-13 then moved under it (tilt only: SteamVR re-levelled the
	// universe from S-3) and S-3 moved 28.5 cm under the rest; by 02:08:16 it
	// was back in S-3's frame, its correction S-13's history.
	const auto knuckles = [](bool join)
	{
		JoinRig rig{ StationS3(), StationS13() };
		rig.join = join;
		rig.headset = 9;
		rig.devices[9] = { 0, V(-1.530, -0.433, -1.262), true };
		rig.devices[14] = { 0, V(-1.463, -1.426, -1.018), true };
		rig.devices[19] = { 0, V(-1.474, -2.080, -1.271), false };
		rig.RunUntil(12.0);
		rig.devices[14] = { 1, V(-1.433, -1.426, -1.018), true };
		rig.RunUntil(20.0);
		rig.MoveStation(1, LoggedMove(Q(-0.943073, 0.245764, 0.208891, 0.081107), V(-2.6881, 0.0553, 0.0754),
			Q(-0.939716, 0.258069, 0.209051, 0.081436), V(-2.6877, 0.0739, 0.0741)));
		rig.RunUntil(22.0);
		rig.MoveStation(0, LoggedMove(Q(-0.226086, 0.004613, 0.957916, 0.176806), V(-2.4654, 0.0453, -3.7428),
			Q(0.229020, -0.005155, -0.957219, -0.176791), V(-2.3216, -0.1581, -3.6161)));
		rig.RunUntil(26.0);
		rig.devices[14] = { 0, V(-1.433, -1.446, -1.018), true };
		rig.RunUntil(33.0);
		return rig;
	};
	{
		const JoinRig old = knuckles(false), fixed = knuckles(true);
		int jumps = 0;
		for (const auto &s : fixed.switches)
			jumps += s.id == 14 && s.cause == LighthouseFrameWatch::Switch::Cause::OwnJump ? 1 : 0;
		const bool took = fixed.joined.size() == 1 && fixed.joined[0].id == 14 && fixed.joined[0].source == 9;
		double worst = 0;
		for (const uint32_t id : { 9u, 14u, 19u })
			worst = (std::max)(worst, fixed.Error(id));
		char detail[160];
		snprintf(detail, sizeof detail, "without the join %.1f cm off; with it %.3g m worst, %d own-jump switches, %zu joined",
			old.Error(14) * 100.0, worst, jumps, fixed.joined.size());
		check("frame join: a controller back from another station's moving frame takes the headset tracker's correction (live 02:06)",
			old.Error(14) > 0.05 && worst < 1e-9 && jumps == 2 && took, detail);
	}

	// Without a headset tracker a device takes the correction its new frame's
	// devices share, keeps its own when they disagree, and alone keeps it too.
	// Two body trackers stay in S-3's frame through its move; a third, away in
	// S-13's meanwhile, comes back into it.
	const auto shared = [](bool disagree)
	{
		JoinRig rig{ StationS3(), StationS13() };
		rig.devices[15] = { 0, V(-1.587, -1.140, -1.448), false };
		rig.devices[16] = { 0, V(-1.483, -0.805, -1.293), false };
		rig.devices[17] = { 0, V(-1.305, -1.140, -1.453), false };
		rig.RunUntil(12.0);
		rig.devices[17].station = -1;
		rig.RunUntil(13.0);
		rig.devices[17].station = 1;
		rig.RunUntil(20.0);
		rig.MoveStation(0, S3MoveAt0045());
		rig.RunUntil(21.0);
		if (disagree)
		{
			LighthouseFrameWatch::Move stray;
			stray.id = 16;
			stray.time = rig.time;
			stray.rotation = Q(Eigen::AngleAxisd(0.01, V::UnitY()));
			rig.frames.Follow(stray);
		}
		rig.devices[17].station = -1;
		rig.RunUntil(24.0);
		rig.devices[17].station = 0;
		rig.RunUntil(31.0);
		return rig;
	};
	{
		const JoinRig agree = shared(false), apart = shared(true);
		auto keptFor = [](const JoinRig &rig, const char *why)
		{
			return std::find(rig.kept.begin(), rig.kept.end(), std::string(why)) != rig.kept.end();
		};
		const bool took = agree.joined.size() == 1 && agree.joined[0].id == 17 && !agree.joined[0].fromHeadsetTracker &&
			agree.joined[0].sharedBy == 2 && agree.Error(17) < 1e-9;
		const bool held = apart.joined.empty() && keptFor(apart, "the devices in that frame disagree") &&
			apart.Error(17) > 0.07;
		check("frame join: without a headset tracker a device takes the correction its frame's devices share, else keeps its own",
			took && held && keptFor(agree, "no other calibrated device is in that frame"), "");
	}

	// The headset tracker coming into a frame gives its correction to the
	// devices there and never takes one; a device being measured keeps its own.
	const auto headsetInto = [](uint32_t keep)
	{
		JoinRig rig{ StationS3(), StationS13() };
		rig.headset = 9;
		rig.keep = keep;
		rig.devices[9] = { 0, V(-1.530, -0.433, -1.262), true };
		rig.devices[14] = { 1, V(-1.463, -1.426, -1.018), true };
		rig.RunUntil(12.0);
		rig.MoveStation(0, S3MoveAt0045());
		rig.RunUntil(14.0);
		const auto before = rig.frames.Snapshot()[9];
		rig.devices[9] = { 1, V(-1.500, -0.433, -1.262), true };
		rig.RunUntil(21.0);
		return std::make_pair(rig, before);
	};
	{
		const auto [gives, headBefore] = headsetInto(vr::k_unTrackedDeviceIndexInvalid);
		const auto [measured, measuredHead] = headsetInto(14);
		const bool gave = gives.joined.size() == 1 && gives.joined[0].id == 14 && gives.joined[0].source == 9 &&
			gives.frames.Snapshot()[14] == gives.frames.Snapshot()[9] && gives.frames.Snapshot()[9] == headBefore;
		const bool keptMeasured = measured.joined.empty() &&
			measured.frames.Snapshot()[14] == protocol::FrameCorrection{} && measured.frames.Snapshot()[9] == measuredHead;
		check("frame join: the headset tracker gives its correction to the frame it comes into and never takes one",
			gave && keptMeasured, "");
	}

	// A device first seen in the headset tracker's frame after SteamVR moved it
	// takes the headset tracker's correction rather than none. A device whose
	// own pose jumps as its frame moves under it is still handed the move once,
	// with no switch: the headset tracker reported the move first.
	{
		const auto late = [](bool join)
		{
			JoinRig rig{ StationS3(), StationS13() };
			rig.join = join;
			rig.headset = 9;
			rig.devices[9] = { 0, V(-1.530, -0.433, -1.262), true };
			rig.devices[17] = { 0, V(-1.305, -1.140, -1.453), true };
			rig.RunUntil(12.0);
			rig.MoveStation(0, S3MoveAt0045());
			rig.devices[17].truth.x() += 0.04;
			rig.RunUntil(14.0);
			rig.devices[19] = { 0, V(-1.474, -2.080, -1.271), false };
			rig.RunUntil(15.0);
			return rig;
		};
		const JoinRig old = late(false), fixed = late(true);
		int jumpSwitches = 0;
		for (const auto &s : fixed.switches)
			jumpSwitches += s.id == 17 && s.cause != LighthouseFrameWatch::Switch::Cause::Appeared ? 1 : 0;
		const bool appeared = fixed.joined.size() == 1 && fixed.joined[0].id == 19 && fixed.Error(19) < 1e-9 &&
			old.Error(19) > 0.07;
		const bool followedOnce = fixed.Error(17) < 1e-9 && jumpSwitches == 0 && fixed.moves == 2 &&
			fixed.frames.Snapshot()[17] == fixed.frames.Snapshot()[9];
		char detail[128];
		snprintf(detail, sizeof detail, "first seen: %.3g m with the join, %.1f cm without; moves %zu, jump switches %d",
			fixed.Error(19), old.Error(19) * 100.0, fixed.moves, jumpSwitches);
		check("frame join: a device first seen in the headset tracker's moved frame takes its correction; a move that leads there is followed once",
			appeared && followedOnce, detail);
	}
}
// The overlay's drain (CalibrationMonitor.cpp), against a schedule of frames
// per device: moves of a device not yet known to be a target one are given
// back, moves before `setupUntil` are left alone as SteamVR's universe setup,
// and switches go through JoinSwitches. Every device is stationary unless its
// schedule says its own pose jumped; its truth is its local pose put in the
// frame the corrections started from (`reference`), so Error() is how far its
// corrected pose is from where it is.
struct ReplayRig
{
	using Membership = questcal::TrackerFrameCorrections::Membership;
	struct Frame
	{
		Q rotation = Q::Identity();
		V translation = V::Zero();
	};
	struct Device
	{
		V local = V::Zero();
		double appears = 0.0;
		double targetFrom = 0.0;       // Waiting before this
		std::map<double, int> frames;  // from this time, the index into Rig::frames
		std::map<double, V> jumps;     // from this time, its local position
		std::vector<double> lagged;    // a sample still in the frame before this change
		std::vector<std::pair<double, double>> calibrating;   // tracking result 101 between
	};
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	std::vector<Frame> placements;
	std::map<uint32_t, Device> devices;
	double time = 0.0, step = 0.01, setupUntil = -1e9;
	int reference = 0;   // the placement corrections start from
	size_t followed = 0, leftAlone = 0, heldMoves = 0;
	std::vector<questcal::TrackerFrameCorrections::SwitchOutcome> outcomes;

	static int At(const std::map<double, int> &schedule, double t)
	{
		int value = -1;
		for (const auto &[from, v] : schedule)
			if (from <= t + 1e-9)
				value = v;
		return value;
	}
	V Local(const Device &d, double t) const
	{
		V local = d.local;
		for (const auto &[from, v] : d.jumps)
			if (from <= t + 1e-9)
				local = v;
		return local;
	}
	Membership Classify(uint32_t id, double) const
	{
		const auto found = devices.find(id);
		if (found == devices.end())
			return Membership::Other;
		return time + 1e-9 >= found->second.targetFrom ? Membership::Target : Membership::Waiting;
	}

	void Tick()
	{
		time += step;
		for (const auto &[id, d] : devices)
		{
			if (time + 1e-9 < d.appears)
				continue;
			int index = At(d.frames, time);
			for (const double change : d.lagged)
				if (time + 1e-9 >= change && time - step < change - 1e-9)
					index = At(d.frames, change - step);
			const Frame &f = placements[static_cast<size_t>(index)];
			protocol::DevicePoseSample s = Sample(id, time, Local(d, time), f.rotation, f.translation);
			s.deviceIsConnected = true;
			for (const auto &[from, until] : d.calibrating)
				if (time >= from && time < until)
					s.trackingResult = vr::TrackingResult_Calibrating_OutOfRange;
			watch.Note(s, TickScale, false);
		}
		auto moves = watch.TakeMoves();
		std::sort(moves.begin(), moves.end(), [](const auto &a, const auto &b) { return a.time < b.time; });
		for (const auto &move : moves)
		{
			const Membership m = Classify(move.id, move.time);
			if (m == Membership::Waiting)
			{
				++heldMoves;
				watch.Hold(move);
			}
			else if (m == Membership::Target)
			{
				if (move.time < setupUntil)
					++leftAlone;
				else
					followed += frames.Follow(move) ? 1 : 0;
			}
		}
		const auto joined = frames.JoinSwitches(watch.TakeSwitches(), watch, vr::k_unTrackedDeviceIndexInvalid,
			vr::k_unTrackedDeviceIndexInvalid,
			[this](uint32_t id, double t) { return Classify(id, t); },
			[this](uint32_t id) { return Classify(id, time) == Membership::Target; });
		outcomes.insert(outcomes.end(), joined.begin(), joined.end());
	}

	void RunUntil(double until)
	{
		while (time < until - 1e-9)
			Tick();
	}

	double Error(uint32_t id) const
	{
		const Device &d = devices.at(id);
		const V local = Local(d, time);
		const Frame &now = placements[static_cast<size_t>(At(d.frames, time))];
		const Frame &start = placements[static_cast<size_t>(reference)];
		const V raw = now.rotation * local + now.translation;
		return (questcal::TrackerFrameCorrections::Apply(frames.Snapshot()[id], raw) -
			(start.rotation * local + start.translation)).norm();
	}

	// The devices whose correction differs from device `first`'s.
	std::string Disagreeing(uint32_t first) const
	{
		std::string out;
		for (const auto &[id, d] : devices)
			if (!questcal::TrackerFrameCorrections::Agree(frames.Snapshot()[id], frames.Snapshot()[first]))
				out += (out.empty() ? "" : " ") + std::to_string(id);
		return out.empty() ? "none" : out;
	}
};

ReplayRig::Frame LoggedFrame(double w, double x, double y, double z, double tx, double ty, double tz)
{
	return { Q(w, x, y, z).normalized(), V(tx, ty, tz) };
}

// 2026-10-15 21:29 to 21:31 (1.2.2-rc.2), seconds after 21:29:00: SteamVR
// kept re-placing base station D3D4E73B (S-5), the frame of every tracker,
// for two minutes after it chose its universe at 28.35, between two
// placements 5 to 10 deg apart, each time a device that had just come on
// measured it against another station ("RELATIONSHIP bases D3D4E73B <->
// f210fba6 ... Moving base D3D4E73B 466mm and 10.3 deg ... Moving the base
// for tracking too"). The placements are the frames the session log printed.
// Device 10 came on at 105, its samples a beat behind the others' at the move
// of 116 and its own pose jumping with the moves of 116, 120.19 and 143.49;
// its 116 move was lost and it ended 17 cm off the frame. Device 12 came on
// at 126, about a second before it was known to be a target device; its
// first-seen switch was dropped and it ended 8 deg off. Device 11 came on in
// 170EE067's frame and changed into D3D4E73B's at 148 with its own pose
// jumping; its join was refused because the frame disagreed.
void StartupReplacements(Check check)
{
	ReplayRig rig;
	rig.placements = {
		LoggedFrame(0.838526, -0.139433, -0.515252, -0.109309, -2.0833, 1.5315, -0.2854),   // 0: 28.0
		LoggedFrame(0.857096, -0.143400, -0.483732, -0.104051, -2.0782, 1.5325, -0.3831),   // 1: 29.0
		LoggedFrame(0.854736, -0.142070, -0.490777, -0.091546, -2.1102, 1.4769, -0.4243),   // 2: 29.5
		LoggedFrame(0.893663, -0.149395, -0.415694, -0.079029, -2.1086, 1.5182, -0.0488),   // 3: 30.9
		LoggedFrame(0.860401, -0.143128, -0.480775, -0.089883, -2.1084, 1.4747, -0.3685),   // 4: 33.06
		LoggedFrame(0.891414, -0.148968, -0.420495, -0.079832, -2.1035, 1.5265, -0.0879),   // 5: 86.8
		LoggedFrame(0.859423, -0.142945, -0.482523, -0.090173, -2.1106, 1.4741, -0.3798),   // 6: 104.86
		LoggedFrame(0.894962, -0.149642, -0.412891, -0.078560, -2.0970, 1.5280, -0.0432),   // 7: 105.06
		LoggedFrame(0.892011, -0.149367, -0.419456, -0.077867, -2.1225, 1.5400, 0.0997),    // 8: 116.0
		LoggedFrame(0.861378, -0.143311, -0.479024, -0.089591, -2.1025, 1.4684, -0.3612),   // 9: 118.3
		LoggedFrame(0.890555, -0.148805, -0.422312, -0.080135, -2.1229, 1.5303, 0.0789),    // 10: 120.19
		LoggedFrame(0.891574, -0.148998, -0.420157, -0.079775, -2.1126, 1.5461, -0.0614),   // 11: 120.39
		LoggedFrame(0.893242, -0.149315, -0.416599, -0.079180, -2.0831, 1.5320, 0.1073),    // 12: 129.55
		LoggedFrame(0.896883, -0.150008, -0.408702, -0.077860, -2.0705, 1.5079, -0.0197),   // 13: 129.95
		LoggedFrame(0.864049, -0.143811, -0.474188, -0.088787, -2.0785, 1.4741, -0.3287),   // 14: 130.15
		LoggedFrame(0.894993, -0.149481, -0.412543, -0.080322, -2.0837, 1.5326, -0.0351),   // 15: 143.49
		// 170EE067's frame, device 11's until 148.
		LoggedFrame(0.972073, 0.234025, 0.004101, -0.017033, 0, 0, 0),                      // 16: 124.0
		LoggedFrame(0.973956, 0.226018, -0.004082, 0.017592, 0, 0, 0),                      // 17: 129.5
		LoggedFrame(0.974391, 0.224024, 0.004345, -0.018899, 0, 0, 0),                      // 18: 130.0
		LoggedFrame(0.978553, 0.205491, 0.002965, -0.014120, 0, 0, 0),                      // 19: 130.2
	};
	const std::map<double, int> station = { { 0.0, 0 }, { 29.0, 1 }, { 29.5, 2 }, { 30.9, 3 }, { 33.06, 4 },
		{ 86.8, 5 }, { 104.86, 6 }, { 105.06, 7 }, { 116.0, 8 }, { 118.3, 9 }, { 120.19, 10 }, { 120.39, 11 },
		{ 129.55, 12 }, { 129.95, 13 }, { 130.15, 14 }, { 143.49, 15 } };
	// The universe was chosen at 28.35; the overlay leaves moves alone for ten
	// seconds after, so the corrections start from the placement of 33.06.
	rig.setupUntil = 38.35;
	rig.reference = 4;
	rig.time = 27.9;
	const V locals[] = { V(0.64, -0.67, -2.67), V(0.38, -0.27, -2.44), V(0.38, -0.72, -2.55), V(0.51, 0.03, -2.38) };
	for (uint32_t id = 1; id <= 4; ++id)
		rig.devices[id] = { locals[id - 1], 28.0, 0.0, station, {}, {}, {} };
	rig.devices[9] = { V(0.64, -0.25, -2.53), 103.0, 0.0, station, { { 105.06, V(0.66, -0.22, -2.48) } }, {}, {} };
	rig.devices[10] = { V(0.59, 0.35, -2.18), 105.1, 0.0, station,
		{ { 116.005, V(0.63, 0.33, -2.12) }, { 120.19, V(0.60, 0.38, -2.15) }, { 143.49, V(0.57, 0.36, -2.20) } },
		{ 116.0 }, { { 112.0, 115.9 }, { 140.0, 143.3 } } };
	rig.devices[12] = { V(0.76, 0.03, -2.54), 126.0, 127.0, station, { { 130.15, V(0.74, 0.05, -2.50) } }, {}, {} };
	rig.devices[11] = { V(0.43, 0.09, -2.10), 124.0, 0.0, { { 0.0, 16 }, { 129.5, 17 }, { 130.0, 18 }, { 130.2, 19 }, { 148.0, 15 } },
		{ { 148.0, V(0.45, 0.12, -2.05) } }, {}, {} };
	rig.RunUntil(160.0);

	double worst = 0.0;
	uint32_t worstId = 0;
	for (const auto &[id, d] : rig.devices)
		if (rig.Error(id) > worst)
		{
			worst = rig.Error(id);
			worstId = id;
		}
	size_t waited = 0, joined11 = 0, joined12 = 0;
	for (const auto &o : rig.outcomes)
	{
		waited += o.kind == questcal::TrackerFrameCorrections::SwitchOutcome::Kind::Held ? 1 : 0;
		const bool took = o.kind == questcal::TrackerFrameCorrections::SwitchOutcome::Kind::Joined;
		joined11 += took && o.change.id == 11 ? 1 : 0;
		joined12 += took && o.change.id == 12 ? 1 : 0;
	}
	char detail[320];
	std::snprintf(detail, sizeof detail,
		"disagreeing with device 1: %s; worst error %.2f mm (device %u); moves left alone %zu, followed %zu, "
		"held %zu; switches that waited %zu; device 12 joined %zu, device 11 joined %zu",
		rig.Disagreeing(1).c_str(), worst * 1000.0, worstId, rig.leftAlone, rig.followed, rig.heldMoves, waited,
		joined12, joined11);
	check("2026-10-15 startup: every device in D3D4E73B's frame ends with one correction, device 11 joins it",
		rig.Disagreeing(1) == "none" && joined11 == 1, detail);
	check("2026-10-15 startup: every corrected pose stays where it was when the setup window closed",
		worst < 0.001, detail);
	check("2026-10-15 startup: device 12's switch and moves wait until it is a target device, then it joins",
		joined12 == 1 && waited >= 1, detail);
}

// A device whose old-frame samples outlast the lag tolerance cannot have the
// move read off the others, and a later change of its frame comes while that
// one still waits: the two wait as one rather than the first being dropped,
// are settled as one switch when the first would have been, and the device
// takes the frame's correction.
void SupersededInference(Check check)
{
	ReplayRig rig;
	rig.placements = {
		LoggedFrame(0.894962, -0.149642, -0.412891, -0.078560, -2.0970, 1.5280, -0.0432),
		LoggedFrame(0.892011, -0.149367, -0.419456, -0.077867, -2.1225, 1.5400, 0.0997),
		LoggedFrame(0.861378, -0.143311, -0.479024, -0.089591, -2.1025, 1.4684, -0.3612),
	};
	rig.time = 0.9;
	rig.devices[1] = { V(0.64, -0.67, -2.67), 1.0, 0.0, { { 0.0, 0 }, { 5.0, 1 }, { 7.0, 2 } }, {}, {}, {} };
	// Still in the old frame 1.5 s after the move, then in the new one with its
	// own pose jumping, and jumping again with the next move.
	rig.devices[10] = { V(0.59, 0.35, -2.18), 1.0, 0.0, { { 0.0, 0 }, { 6.5, 1 }, { 7.0, 2 } },
		{ { 6.5, V(0.63, 0.33, -2.12) }, { 7.0, V(0.60, 0.38, -2.15) } }, {}, {} };
	// The two changes wait as one, from frame 0, and no move leads there from
	// it within the lag: settled as one switch when the first would have been,
	// at 11.5 s, and joined.
	rig.RunUntil(12.5);
	size_t jumped = 0;
	for (const auto &o : rig.outcomes)
		jumped += o.change.id == 10 && o.change.cause == LighthouseFrameWatch::Switch::Cause::OwnJump &&
			o.kind == questcal::TrackerFrameCorrections::SwitchOutcome::Kind::Joined ? 1 : 0;
	char detail[200];
	std::snprintf(detail, sizeof detail, "disagreeing with device 1: %s; joins after an own jump %zu; error %.2f mm",
		rig.Disagreeing(1).c_str(), jumped, rig.Error(10) * 1000.0);
	check("frame watch: a change still waiting when the device changes again waits with it and settles with it",
		rig.Disagreeing(1) == "none" && jumped == 1 && rig.Error(10) < 0.001, detail);
}

// Moves and switches of a device not yet known to be a target one wait for
// it, in order; a held switch of a device that turns out not to be ours is
// dropped and said so.
void HeldFrameChanges(Check check)
{
	using Membership = questcal::TrackerFrameCorrections::Membership;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	const Q a = Q::Identity(), b(Eigen::AngleAxisd(0.1, V::UnitY()));
	double time = 1.0;
	auto note = [&](uint32_t id, const Q &frame) { auto s = Sample(id, time, V(1, 0, 0), frame, V(0, 1, 0)); s.deviceIsConnected = true; watch.Note(s, TickScale, false); };
	note(3, a);
	note(4, a);
	time += 0.01;
	note(3, b);
	note(4, b);
	auto moves = watch.TakeMoves();
	for (const auto &m : moves)
		watch.Hold(m);
	const auto switches = watch.TakeSwitches();
	auto membership = [](uint32_t id, double) { return id == 3 ? Membership::Waiting : Membership::Other; };
	auto calibrated = [](uint32_t) { return false; };
	auto first = frames.JoinSwitches(switches, watch, vr::k_unTrackedDeviceIndexInvalid,
		vr::k_unTrackedDeviceIndexInvalid, membership, calibrated);
	const auto again = watch.TakeMoves();
	auto second = frames.JoinSwitches(watch.TakeSwitches(), watch, vr::k_unTrackedDeviceIndexInvalid,
		vr::k_unTrackedDeviceIndexInvalid, [](uint32_t, double) { return Membership::Other; }, calibrated);
	const bool heldThenDropped = first.size() == 1 && first[0].change.id == 3 &&
		first[0].kind == questcal::TrackerFrameCorrections::SwitchOutcome::Kind::Held &&
		second.size() == 1 && second[0].kind == questcal::TrackerFrameCorrections::SwitchOutcome::Kind::Dropped &&
		watch.TakeSwitches().empty();
	char detail[160];
	std::snprintf(detail, sizeof detail, "moves handed %zu, given back and handed again %zu (held flag %d); switches first %zu, then %zu",
		moves.size(), again.size(), again.empty() ? -1 : int(again.front().held), first.size(), second.size());
	check("frame watch: held moves come back in order; a held switch waits, then is dropped for a device not ours",
		moves.size() == 2 && again.size() == 2 && again.front().held && again[0].id == moves[0].id && heldThenDropped,
		detail);
}

} // namespace

// Where a calibration samples (CollectionSource.h): the raw channel when it
// holds a fresh trusted sample of both selected devices, each in one frame;
// otherwise runtime poses, read the way the fallback reads them.
void CollectionFallback(Check check)
{
	const double now = 20.0;
	auto tracking = [](uint32_t id, double time, const Q &frame = Q::Identity())
	{
		protocol::DevicePoseSample s = Sample(id, time, V(0.2, 1.4, -0.3), frame);
		s.deviceIsConnected = true;
		return s;
	};
	protocol::DevicePoseSample hidden = tracking(9, now - 0.1);
	hidden.deviceIsConnected = false;   // as the channel carries a tracker another driver hides
	protocol::DevicePoseSample hiddenReference = tracking(0, now - 0.1);
	hiddenReference.deviceIsConnected = false;
	protocol::DevicePoseSample lost = tracking(0, now - 0.1);
	lost.trackingResult = vr::TrackingResult_Running_OutOfRange;
	const Q moved(Eigen::AngleAxisd(0.05, V::UnitY()));
	struct Case
	{
		const char *name;
		std::vector<protocol::DevicePoseSample> batch;
		bool raw;
		const char *reason;
	};
	// Another device's frame moving is none of the preflight's business. The
	// target is in the target system, so it is tracking whatever its connection
	// flag says (RingSampleGate.h); the reference is not.
	const Case cases[] = {
		{ "pair", { tracking(0, now - 0.1), tracking(9, now - 0.1), tracking(4, now - 0.1, moved), tracking(4, now - 0.05) },
			true, "fresh trusted pair available" },
		{ "no target", { tracking(0, now - 0.1) }, false, "target had no fresh trusted samples" },
		{ "stale target", { tracking(0, now - 0.1), tracking(9, now - 0.6) }, false, "target had no fresh trusted samples" },
		{ "hidden target", { tracking(0, now - 0.1), hidden }, true, "fresh trusted pair available" },
		{ "hidden reference", { hiddenReference, tracking(9, now - 0.1) }, false,
			"reference had no fresh trusted samples" },
		{ "lost reference", { lost, tracking(9, now - 0.1) }, false, "reference had no fresh trusted samples" },
		{ "nothing", {}, false, "neither selected device had fresh trusted samples" },
		{ "target frame moved", { tracking(0, now - 0.1), tracking(9, now - 0.2), tracking(9, now - 0.1, moved) },
			false, "selected device changed world-from-driver during preflight" },
	};
	std::string wrong;
	for (const auto &c : cases)
	{
		questcal::CalibrationRun run;
		run.referenceId = 0;
		run.targetId = 9;
		const char *reason = "";
		const bool raw = questcal::PreflightPoseRing(run, c.batch, now, TickScale, reason);
		if (raw != c.raw || std::string(reason) != c.reason ||
			run.referenceUniverse.valid != raw || run.targetUniverse.valid != raw)
			wrong += std::string(wrong.empty() ? "" : ", ") + c.name + ": " + reason;
	}
	check("collection: the raw channel serves only a fresh trusted pair, else runtime poses do",
		wrong.empty(), wrong.c_str());

	vr::TrackedDevicePose_t pose{};
	const Q turn(Eigen::AngleAxisd(0.5, V(0.3, 0.9, 0.1).normalized()));
	const Eigen::Matrix3d r = turn.toRotationMatrix();
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			pose.mDeviceToAbsoluteTracking.m[i][j] = static_cast<float>(r(i, j));
		pose.mDeviceToAbsoluteTracking.m[i][3] = static_cast<float>(0.1 * (i + 1));
		pose.vVelocity.v[i] = static_cast<float>(0.2 * i);
		pose.vAngularVelocity.v[i] = static_cast<float>(-0.3 * i);
	}
	pose.bPoseIsValid = true;
	pose.bDeviceIsConnected = true;
	pose.eTrackingResult = vr::TrackingResult_Running_OK;
	questcal::PoseSample read;
	const bool accepted = questcal::RuntimePoseSample(pose, now, read);
	const bool faithful = accepted && read.time == now && read.rot.angularDistance(turn) < 1e-5 &&
		(read.pos - V(0.1, 0.2, 0.3)).norm() < 1e-6 && (read.vel - V(0.0, 0.2, 0.4)).norm() < 1e-6 &&
		(read.angVel - V(0.0, -0.3, -0.6)).norm() < 1e-6;
	auto refused = [&](void (*spoil)(vr::TrackedDevicePose_t &))
	{
		vr::TrackedDevicePose_t bad = pose;
		spoil(bad);
		questcal::PoseSample out;
		return !questcal::RuntimePoseSample(bad, now, out);
	};
	const bool gated =
		refused([](vr::TrackedDevicePose_t &p) { p.bPoseIsValid = false; }) &&
		refused([](vr::TrackedDevicePose_t &p) { p.eTrackingResult = vr::TrackingResult_Running_OutOfRange; }) &&
		refused([](vr::TrackedDevicePose_t &p) { p.mDeviceToAbsoluteTracking.m[1][3] = std::numeric_limits<float>::quiet_NaN(); }) &&
		refused([](vr::TrackedDevicePose_t &p) { p.vVelocity.v[0] = 1e9f; });
	char detail[96];
	snprintf(detail, sizeof detail, "accepted %d, faithful %d, gated %d", accepted, faithful, gated);
	check("collection: a runtime pose is read at the tick as the runtime gave it, and only when trusted",
		accepted && faithful && gated, detail);
}

// Switches past the bound are dropped, so the watch says so, as it does for
// moves: a device left with the correction of a frame it is no longer in
// must not be followed as if nothing was lost.
void SwitchOverflow(Check check)
{
	LighthouseFrameWatch watch;
	const Q a = Q::Identity(), b(Eigen::AngleAxisd(0.1, V::UnitY()));
	const V world(1, 0, 0);
	double time = 1.0;
	// One device re-expressed in the other frame at every sample, its pose in
	// the world unchanged: a switch each time.
	auto note = [&](const Q &frame)
	{
		auto s = Sample(3, time, frame.conjugate() * world, frame);
		s.rotation = questcal::WireQuaternion(frame.conjugate());
		s.deviceIsConnected = true;
		watch.Note(s, TickScale, false);
		time += 0.01;
	};
	for (int i = 0; i < 300; ++i)
		note(i % 2 ? b : a);
	const bool overflowed = watch.TakeSwitchOverflow();
	const size_t taken = watch.TakeSwitches().size();
	const bool cleared = !watch.TakeSwitchOverflow();
	note(a);
	note(b);
	const size_t after = watch.TakeSwitches().size();
	const bool stillClear = !watch.TakeSwitchOverflow();
	char detail[160];
	std::snprintf(detail, sizeof detail, "overflow %d, taken %zu, cleared %d; then %zu more, overflow %d",
		overflowed ? 1 : 0, taken, cleared ? 1 : 0, after, stillClear ? 0 : 1);
	check("frame watch: switches past the bound are dropped and said so",
		overflowed && taken == 256 && cleared && after == 2 && stillClear, detail);
}

// Sequences SteamVR makes rarely: a tracker off while its station is placed or
// re-solved, re-solves in a row or shown out of order, a device switched on or
// a solve behind as its frame moves, and the pose stream starting over while a
// followed move is still on its way to the driver.
namespace
{
// A stationary device's sample, at `local` in the given frame.
protocol::DevicePoseSample ReplaySample(uint32_t id, double time, const V &local, const Q &frameRotation,
	const V &frameTranslation)
{
	protocol::DevicePoseSample s = Sample(id, time, local, frameRotation, frameTranslation);
	s.deviceIsConnected = true;
	return s;
}

protocol::DevicePoseSample ReplayOff(uint32_t id, double time)
{
	protocol::DevicePoseSample s = Sample(id, time, V::Zero());
	s.deviceIsConnected = false;
	s.poseIsValid = false;
	s.trackingResult = vr::TrackingResult_Running_OutOfRange;
	return s;
}

const char *UniverseName(LighthouseVisibility::UniverseSetup u)
{
	return u == LighthouseVisibility::UniverseSetup::SettingUp ? "SettingUp"
		: u == LighthouseVisibility::UniverseSetup::Settled ? "Settled" : "Unknown";
}

double CorrectionShift(const protocol::FrameCorrection &a, const protocol::FrameCorrection &b, const V &at)
{
	return (questcal::TrackerFrameCorrections::Apply(a, at) - questcal::TrackerFrameCorrections::Apply(b, at)).norm();
}

// The correction that takes the frame (r, tr) back to the calibration's.
protocol::FrameCorrection BackToStart(const Q &r, const V &tr)
{
	protocol::FrameCorrection c;
	const Q n = r.conjugate().normalized();
	c.rotation = { n.w(), n.x(), n.y(), n.z() };
	const V nt = -(n * tr);
	for (int axis = 0; axis < 3; ++axis)
		c.translation.v[axis] = nt(axis);
	return c;
}

// CompensateTrackerFrameMoves and JoinTrackerFrames on the watch's output, as
// written (the stable sort, Follow, JoinSwitches): devices 9 and 10 are the
// calibrated target devices, `headset` the headset tracker.
struct MonitorRig
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t headset = vr::k_unTrackedDeviceIndexInvalid;
	std::string log;
	void Tick(double t)
	{
		using Membership = questcal::TrackerFrameCorrections::Membership;
		auto moves = watch.TakeMoves();
		std::stable_sort(moves.begin(), moves.end(), [](const auto &a, const auto &b) { return a.time < b.time; });
		for (const auto &m : moves)
		{
			char line[160];
			std::snprintf(line, sizeof line, "%sdevice %u move (frame moved %.2f, handed %.2f) %s", log.empty() ? "" : "; ",
				m.id, m.frameTime, t, frames.Follow(m) ? "followed" : "refused");
			log += line;
		}
		for (const auto &o : frames.JoinSwitches(watch.TakeSwitches(), watch, headset, vr::k_unTrackedDeviceIndexInvalid,
			[](uint32_t, double) { return Membership::Target; }, [](uint32_t id) { return id == 9 || id == 10; }))
		{
			char line[200];
			if (!o.result.joined.empty())
				for (const auto &j : o.result.joined)
				{
					std::snprintf(line, sizeof line, "%sdevice %u took device %u's at %.2f", log.empty() ? "" : "; ",
						j.id, j.source, t);
					log += line;
				}
			else if (o.kind != questcal::TrackerFrameCorrections::SwitchOutcome::Kind::Held || !o.change.held)
			{
				std::snprintf(line, sizeof line, "%sdevice %u switch (%s) at %.2f: %s", log.empty() ? "" : "; ",
					o.change.id, o.change.cause == LighthouseFrameWatch::Switch::Cause::OwnJump ? "own jump"
						: o.change.cause == LighthouseFrameWatch::Switch::Cause::Appeared ? "appeared"
						: o.change.cause == LighthouseFrameWatch::Switch::Cause::Returned ? "returned" : "re-expressed",
					t, o.kind == questcal::TrackerFrameCorrections::SwitchOutcome::Kind::Held ? "held"
						: (o.result.kept ? o.result.kept : "kept"));
				log += line;
			}
		}
	}
};
} // namespace

// A tracker switched off while SteamVR places a station it guessed (its
// universe setup), and back after the setup window, is handed that placement as
// a move read off the station's own: stamped with when the frame moved, it is
// judged against the setup by that time and left alone, as every other device
// in the frame left it. Judged by the tracker's return, it was followed.
void SetupPlacementOnReturn(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	LighthouseVisibility visibility;
	lighthouselog::Event start;
	start.kind = lighthouselog::Event::Kind::ServerStarted;
	visibility.Apply(start, 9.0);
	lighthouselog::Event chosen;
	chosen.kind = lighthouselog::Event::Kind::UniverseChosen;
	chosen.universeId = 1744988537;
	visibility.Apply(chosen, 11.0);   // SettingUp until 21.0 (universeSetupSeconds 10)
	// The guess and the placement (live 2026-09-27: 1205 mm and 42.7 deg).
	const Q guessR(Eigen::AngleAxisd(42.7 * questcal::Pi / 180.0, V::UnitY()));
	const V guessT(1.2, 0.0, 0.1);
	const Q placedR = Q::Identity();
	const V placedT = V::Zero();
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5);
	std::vector<LighthouseFrameWatch::Move> handed;
	std::string decisions;
	auto drain = [&](double now)
	{
		for (const auto &move : watch.TakeMoves())
		{
			// DuringUniverseSetup(ctx, move.frameTime), from the log's word.
			const auto u = visibility.Universe(move.frameTime);
			const bool setup = u == LighthouseVisibility::UniverseSetup::SettingUp;
			char line[160];
			std::snprintf(line, sizeof line, "%sdevice %u move at %.2f: universe %s, %s", decisions.empty() ? "" : "; ",
				move.id, move.time, UniverseName(u), setup ? "left alone" : (frames.Follow(move) ? "followed" : "refused"));
			decisions += line;
			handed.push_back(move);
		}
		(void)now;
		watch.TakeSwitches();
	};
	for (double t = 10.0; t < 31.0; t += 0.05)
	{
		const bool placed = t >= 12.0;
		const Q r = placed ? placedR : guessR;
		const V tr = placed ? placedT : guessT;
		watch.Note(ReplaySample(1, t, V::Zero(), r, tr), TickScale, true);   // the base station
		watch.Note(ReplaySample(10, t, at10, r, tr), TickScale, false);
		if (t < 11.45 || t >= 30.0)
			watch.Note(ReplaySample(9, t, at9, r, tr), TickScale, false);
		else
			watch.Note(ReplayOff(9, t), TickScale, false);
		drain(t);
	}
	const double shift9 = CorrectionShift(frames.Snapshot()[9], protocol::FrameCorrection{}, at9);
	const double shift10 = CorrectionShift(frames.Snapshot()[10], protocol::FrameCorrection{}, at10);
	char detail[640];
	std::snprintf(detail, sizeof detail,
		"%s; universe at the placement (12.00): %s; device 9's correction moved %.1f cm at it, device 10's %.1f cm",
		decisions.c_str(), UniverseName(visibility.Universe(12.0)), shift9 * 100.0, shift10 * 100.0);
	check("frame corrections: a station placed during the universe setup is left alone by a tracker back after it",
		shift9 < 1e-9 && shift10 < 1e-9, detail);
}

// A change of a device's frame its own pose jumped with is a move when any
// chain of the moves other devices showed leads to it: one through a frame
// between that the station skipped (a), and one the station moved on past
// before the device showed it (b). Walked as one path from the first move out
// of the old frame, neither led there, and the change, which the station's own
// samples show exactly, was settled as a switch after inferSeconds.
void LeadByAnyChain(Check check)
{
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const Q e2(Eigen::AngleAxisd(0.045, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01), t2(0.07, 0, 0.02);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5), jump(0.05, 0, 0);
	std::string out;
	bool pass = true;
	for (int variant = 0; variant < 2; ++variant)
	{
		LighthouseFrameWatch watch;
		std::vector<LighthouseFrameWatch::Move> moves;
		std::vector<LighthouseFrameWatch::Switch> switches;
		for (double t = 10.0; t < 17.0; t += 0.05)
		{
			// (a) Device 9 shows E1 at 10.5 and goes off before E2; the
			// station's next sample, at 10.75, is in E2; device 10 shows E2 at
			// 10.8 with its own pose jumping.
			// (b) The station shows E1 at 10.5 and E2 at 10.6; device 10, a
			// solve behind, shows E1 at 10.65 with its own pose jumping, then
			// E2 from 10.7.
			Q sr = e0, r9 = e0, r10 = e0;
			V st = t0, tr9 = t0, tr10 = t0;
			bool off9 = false, jumped = false;
			if (variant == 0)
			{
				if (t >= 10.5) { r9 = e1; tr9 = t1; }
				off9 = t >= 10.65;
				if (t >= 10.75) { sr = e2; st = t2; }
				if (t >= 10.8) { r10 = e2; tr10 = t2; jumped = true; }
			}
			else
			{
				if (t >= 10.5) { sr = e1; st = t1; r9 = e1; tr9 = t1; }
				if (t >= 10.6) { sr = e2; st = t2; r9 = e2; tr9 = t2; }
				if (t >= 10.65) { r10 = e1; tr10 = t1; jumped = true; }
				if (t >= 10.7) { r10 = e2; tr10 = t2; }
			}
			watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
			watch.Note(off9 ? ReplayOff(9, t) : ReplaySample(9, t, at9, r9, tr9), TickScale, false);
			watch.Note(ReplaySample(10, t, jumped ? V(at10 + jump) : at10, r10, tr10), TickScale, false);
			for (const auto &m : watch.TakeMoves()) moves.push_back(m);
			for (const auto &s : watch.TakeSwitches()) switches.push_back(s);
		}
		// What device 10 was handed for the change its own pose jumped with.
		const Q from = e0;
		const Q to = variant == 0 ? e2 : e1;
		bool movedThere = false, switchedThere = false;
		for (const auto &m : moves)
			movedThere |= m.id == 10 && m.fromRot.angularDistance(from) < 1e-9 && m.toRot.angularDistance(to) < 1e-9;
		double settledAt = -1;
		for (const auto &s : switches)
			if (s.id == 10 && s.cause == LighthouseFrameWatch::Switch::Cause::OwnJump &&
				s.fromRot.angularDistance(from) < 1e-9 && s.toRot.angularDistance(to) < 1e-9)
			{
				switchedThere = true;
				settledAt = s.time;
			}
		char line[200];
		std::snprintf(line, sizeof line, "%s(%c) device 10 %s: handed a move %d, settled as an own-jump switch %d (stamped %.2f)",
			out.empty() ? "" : "; ", variant == 0 ? 'a' : 'b', variant == 0 ? "E0 to E2" : "E0 to E1",
			movedThere ? 1 : 0, switchedThere ? 1 : 0, settledAt);
		out += line;
		pass &= movedThere && !switchedThere;
	}
	check("frame watch: a change any chain of the other devices' moves leads to is handed as a move", pass, out.c_str());
}

// A device that comes into a frame SteamVR moved, while the only other
// calibrated device whose frame it is (the headset tracker) is switched off,
// waits for it rather than keep its own correction: the headset tracker comes
// back into the frame by the move read off the station's samples and follows
// it, and the waiting device joins it then. Kept at once, the frame ended with
// two corrections.
void FrameJoinedOnReturn(Check check)
{
	JoinRig rig{ StationS3(), StationS13() };
	rig.headset = 9;
	rig.devices[9] = { 0, V(-1.530, -0.433, -1.262), true };
	rig.devices[10] = { 1, V(-1.463, -1.426, -1.018), true };
	rig.RunUntil(12.0);
	rig.devices[9].station = -1;           // switched off: its last frame is S-3's before the move
	rig.devices[9].connected = false;
	rig.RunUntil(13.0);
	rig.MoveStation(0, S3MoveAt0045());    // SteamVR moves S-3 while it is off
	rig.RunUntil(14.0);
	rig.devices[10].station = 0;           // device 10 starts a new solution from S-3
	rig.RunUntil(22.0);
	rig.devices[9].station = 0;            // the headset tracker comes back
	rig.devices[9].connected = true;
	rig.RunUntil(30.0);
	const bool one = questcal::TrackerFrameCorrections::Agree(rig.frames.Snapshot()[9], rig.frames.Snapshot()[10]);
	std::string kept;
	for (const auto &k : rig.kept)
		kept += (kept.empty() ? "" : ", ") + k;
	char detail[480];
	std::snprintf(detail, sizeof detail,
		"one correction in S-3's frame %d; error at device 9 %.2f mm, at device 10 %.1f cm; moves followed %zu; "
		"joins %zu; kept: %s", one ? 1 : 0, rig.Error(9) * 1000.0, rig.Error(10) * 100.0, rig.moves,
		rig.joined.size(), kept.c_str());
	check("frame corrections: a device alone in a moved frame joins the calibrated device that comes back into it", one, detail);
}

// The headset tracker's own pose jumps at two re-solves of its station in a
// row; the lead for the first comes a moment after the second, from a device a
// solve behind. The two changes wait as one, and the headset tracker follows
// both. Settled as a switch when the second came, the first had the headset
// tracker give a correction that never followed it to the device that did.
void ChangesInARow(Check check)
{
	using Membership = questcal::TrackerFrameCorrections::Membership;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const Q e2(Eigen::AngleAxisd(0.045, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01), t2(0.07, 0, 0.02);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5);
	std::string joins;
	size_t followed = 0;
	for (double t = 10.0; t < 17.0; t += 0.05)
	{
		// The station is a solve behind the headset tracker; device 10 is a
		// solve behind the station.
		Q sr = e0, r9 = e0, r10 = e0;
		V st = t0, tr9 = t0, tr10 = t0, local9 = at9;
		if (t >= 10.5 - 1e-9) { r9 = e1; tr9 = t1; local9 = at9 + V(0.05, 0, 0); }
		if (t >= 10.6 - 1e-9) { sr = e2; st = t2; r9 = e2; tr9 = t2; local9 = at9 + V(0.10, 0, 0); }
		if (t >= 10.65 - 1e-9) { r10 = e1; tr10 = t1; }
		if (t >= 10.7 - 1e-9) { r10 = e2; tr10 = t2; }
		watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
		watch.Note(ReplaySample(9, t, local9, r9, tr9), TickScale, false);
		watch.Note(ReplaySample(10, t, at10, r10, tr10), TickScale, false);
		auto moves = watch.TakeMoves();
		std::stable_sort(moves.begin(), moves.end(), [](const auto &a, const auto &b) { return a.time < b.time; });
		for (const auto &move : moves)
			followed += frames.Follow(move) ? 1 : 0;
		for (const auto &o : frames.JoinSwitches(watch.TakeSwitches(), watch, 9, vr::k_unTrackedDeviceIndexInvalid,
			[](uint32_t, double) { return Membership::Target; }, [](uint32_t id) { return id == 9 || id == 10; }))
			for (const auto &j : o.result.joined)
			{
				char line[96];
				std::snprintf(line, sizeof line, "%sdevice %u took device %u's at %.2f", joins.empty() ? "" : ", ",
					j.id, j.source, t);
				joins += line;
			}
	}
	// Both are in E2 and moved with their station from E0: the right correction
	// takes E2 back to E0.
	protocol::FrameCorrection right;
	const Q n = (e0 * e2.conjugate()).normalized();
	right.rotation = { n.w(), n.x(), n.y(), n.z() };
	const V nt = t0 - n * t2;
	for (int axis = 0; axis < 3; ++axis)
		right.translation.v[axis] = nt(axis);
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, at9);
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, at10);
	char detail[320];
	std::snprintf(detail, sizeof detail, "moves followed %zu; joins: %s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		followed, joins.empty() ? "none" : joins.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame watch: two changes in a row wait as one and follow the lead for both", off9 < 1e-6 && off10 < 1e-6, detail);
}

// A followed move whose synchronization waits behind one still in flight, when
// the overlay's pose stream starts over before that one returns:
// StartFrameSession resets the corrections and submits the recovery request,
// which replaces the waiting synchronization (DriverWorker keeps only the
// newest), so the driver's checkpoint never holds the move. The recovery,
// answered by the same driver session, gives back the corrections the overlay
// kept, the move in them; from the checkpoint it brought the correction back
// without it. The worker, the driver session, the driver and the corrections
// are the real ones, Keep and Recover among them; the overlay's glue
// (SynchronizeCalibrationDriver's request, StartFrameSession,
// PollCalibrationDriver's verdict filter and ApplyCompletion) is transcribed.
void MoveKeptAcrossStreamRestart(Check check)
{
	Fixture f(false);
	f.driver->SetSessionForTest(41);
	std::mutex gateMutex;
	std::condition_variable gateWake;
	bool gateArmed = false, gateOpen = false;
	std::atomic<bool> blocked{ false };
	std::mutex seenMutex;
	// Device 9's correction (its x translation) in each state the driver took.
	std::vector<double> reached9;
	questcal::DriverWorker worker;
	worker.Start([&](const protocol::Request &request)
	{
		if (request.type == protocol::RequestSetRuntimeState)
		{
			std::unique_lock<std::mutex> lock(gateMutex);
			if (gateArmed && !gateOpen)
			{
				blocked = true;
				gateWake.wait(lock, [&] { return gateOpen; });
			}
		}
		questcal::DriverTransportResult result;
		result.completed = true;
		result.connectionGeneration = 1;
		f.driver->GetRuntimeState(result.response);
		if (request.type == protocol::RequestHandshake)
			result.response.type = protocol::ResponseHandshake;
		else if (request.type == protocol::RequestGetRuntimeState)
			result.response.type = protocol::ResponseRuntimeState;
		else
		{
			result.response.type = f.driver->TrySetRuntimeState(request.setRuntimeState)
				? protocol::ResponseSuccess : protocol::ResponseInvalid;
			std::lock_guard<std::mutex> lock(seenMutex);
			reached9.push_back(request.setRuntimeState.frames[9].translation.v[0]);
		}
		return result;
	});

	// The overlay's state: ctx.frameDriverSession, ctx.frameRecoveryPending
	// (InitCalibrator) and ctx.framesBeforeSession.
	uint64_t session = 0;
	bool pending = true;
	questcal::TrackerFrameCorrections::Kept kept;
	const uint64_t profileKey = 73;
	auto job = [&]()
	{
		questcal::DriverStateJob j;
		auto &request = j.request;
		request.enabled = true;
		request.desired.referenceTrackingSystem = "reference";
		request.desired.targetTrackingSystem = "lighthouse";
		request.desired.rotation = Q(f.state.transform.rotation.w, f.state.transform.rotation.x,
			f.state.transform.rotation.y, f.state.transform.rotation.z);
		request.desired.translationMeters = V(f.state.transform.translation.v);
		request.desired.scale = f.state.transform.scale;
		request.frames = f.corrections.Snapshot();
		request.frameProfileKey = profileKey;
		request.driverSessionId = session;
		request.recoverFrames = pending;
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			request.frameSerialKeys[id] = f.corrections.IdentityKey(id);
			if (!f.corrections.Serial(id).empty())
				request.frameBoundMask |= uint64_t{1} << id;
			auto &device = j.devices[id];
			device.id = id;
			if (id == 0 || (id >= 9 && id < 14))
			{
				device.deviceClass = id == 0 ? questcal::SyncDeviceClass::Hmd : questcal::SyncDeviceClass::Other;
				device.trackingSystemKnown = true;
				device.trackingSystem = id == 0 ? "reference" : "lighthouse";
			}
		}
		j.time = f.time;
		return j;
	};
	questcal::DriverSyncTracker tracker;
	auto submit = [&]()
	{
		const auto s = worker.Submit(job());
		tracker.NoteSubmission(s.sequence, s.stateChanged);
		return s.sequence;
	};
	std::string answered;
	// PollCalibrationDriver until `sequence` is answered, with
	// ApplyCompletion's frame part.
	auto await = [&](uint64_t sequence)
	{
		questcal::DriverCompletion completion;
		for (int attempt = 0; attempt < 5000; ++attempt)
		{
			while (worker.Poll(completion))
			{
				const bool taken = tracker.NoteVerdict(completion.sequence, completion.result.synchronized);
				char line[64];
				std::snprintf(line, sizeof line, "%s%llu %s", answered.empty() ? "" : ", ",
					static_cast<unsigned long long>(completion.sequence), taken ? "taken" : "ignored");
				answered += line;
				const auto &result = completion.result;
				if (taken && result.synchronized)
				{
					session = result.driverSessionId;
					if (pending && result.recoveryChecked)
					{
						f.corrections.Recover(kept, result.driverSessionId, profileKey, result.framesRecovered,
							result.frameProfileKey, result.frames, result.frameSerialKeys);
						kept = {};
						pending = false;
					}
				}
				if (completion.sequence == sequence)
					return true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		return false;
	};

	// The overlay starts (StartFrameSession at its first tick): nothing to
	// recover yet.
	bool pass = await(submit()) && session == 41 && !pending;
	// A periodic synchronization that the driver is slow to answer.
	{
		std::lock_guard<std::mutex> lock(gateMutex);
		gateArmed = true;
	}
	const uint64_t slow = submit();
	for (int attempt = 0; attempt < 5000 && !blocked; ++attempt)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	// Device 9's station is re-solved: 3 degrees about the vertical and 4 cm.
	// The move is followed and synchronized (CompensateTrackerFrameMoves); that
	// request waits.
	const Q rotation(Eigen::AngleAxisd(3.0 * questcal::Pi / 180.0, V::UnitY()));
	const V translation(0.04, 0, 0.01);
	f.time += 0.01;
	{
		auto &s = f.raw[0];
		const auto parts = UnpackRingSample(s);
		s.worldFromDriverRotation = questcal::WireQuaternion((rotation * parts.wfdRot).normalized());
		const V origin = rotation * parts.wfdTrans + translation;
		for (int axis = 0; axis < 3; ++axis) s.worldFromDriverTranslation[axis] = origin(axis);
	}
	const size_t movesBefore = f.moves;
	f.Observe();
	const auto followed = f.corrections.Snapshot()[9];
	const uint64_t carrying = submit();
	// The pose stream starts over (a failed drain): StartFrameSession's steps,
	// then the binding of the serials SynchronizeCalibrationDriver reads.
	kept = f.corrections.Keep(session, profileKey);
	session = 0;
	pending = true;
	f.corrections.Reset();
	for (uint32_t i = 0; i < 5; ++i)
		f.corrections.Bind(9 + i, "tracker-" + std::to_string(i));
	const uint64_t recovery = submit();
	{
		std::lock_guard<std::mutex> lock(gateMutex);
		gateOpen = true;
	}
	gateWake.notify_all();
	pass &= await(recovery) && !pending;
	const auto restored = f.corrections.Snapshot()[9];
	// The next periodic synchronization publishes what the overlay now holds.
	pass &= await(submit());
	worker.Stop();
	const V at9(-0.5, 1.7, 0.2);
	const double shortBy = CorrectionShift(restored, followed, at9);
	const double error = f.Error();
	std::string driverSaw;
	for (double x : reached9)
	{
		char line[32];
		std::snprintf(line, sizeof line, "%s%.3f", driverSaw.empty() ? "" : " ", x);
		driverSaw += line;
	}
	char detail[512];
	std::snprintf(detail, sizeof detail,
		"move followed %zu; requests: start 1, slow %llu, the move's %llu, the recovery %llu; answered: %s; "
		"device 9's correction x in each state the driver took: %s; restored correction %.1f cm short of the "
		"followed one; driver output after the recovery off by %.1f cm",
		f.moves - movesBefore, static_cast<unsigned long long>(slow), static_cast<unsigned long long>(carrying),
		static_cast<unsigned long long>(recovery), answered.c_str(), driverSaw.c_str(), shortBy * 100.0, error * 100.0);
	check("frame corrections: a move followed just before the pose stream starts over is kept by the recovery",
		pass && shortBy < 1e-6 && error < 1e-6, detail);
}

// Device 9's own pose jumps as its station is re-solved (E0 to E1), and it
// shows the next re-solve (E1 to E2) as a move before the station, a solve
// behind, shows the first: E1 to E2 is recorded before E0 to E1. The headset
// tracker (device 10), two solves behind, then shows E2 with its own pose
// jumping, and the recorded steps lead it there. Taken in one pass in the order
// they were recorded, they led nowhere: the change was settled as a switch, and
// the headset tracker gave device 9 its correction, short of both re-solves.
void ChainRecordedOutOfOrder(Check check)
{
	MonitorRig rig;
	rig.headset = 10;
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const Q e2(Eigen::AngleAxisd(0.045, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01), t2(0.07, 0, 0.02);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5), jump(0.05, 0, 0);
	for (double t = 10.0; t < 17.0; t += 0.05)
	{
		Q sr = e0, r9 = e0, r10 = e0;
		V st = t0, tr9 = t0, tr10 = t0, local9 = at9, local10 = at10;
		if (t >= 10.5 - 1e-9) { r9 = e1; tr9 = t1; local9 = at9 + jump; }   // its own pose jumps with E1
		if (t >= 10.6 - 1e-9) { r9 = e2; tr9 = t2; }                        // E2 shown as a move
		if (t >= 10.65 - 1e-9) { sr = e1; st = t1; }                         // the station, a solve behind
		if (t >= 10.7 - 1e-9) { sr = e2; st = t2; }
		if (t >= 10.8 - 1e-9) { r10 = e2; tr10 = t2; local10 = at10 + jump; } // two solves behind, jumping
		rig.watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, local9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, local10, r10, tr10), TickScale, false);
		if (std::fmod(t + 1e-9, 0.25) < 0.05)
			rig.Tick(t);
	}
	const auto right = BackToStart(e2, t2);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9 + jump);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10 + jump);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame watch: a chain of moves recorded out of order leads the change", off9 < 1e-6 && off10 < 1e-6, detail);
}

// Once both devices have joined their first frame: device 9's own pose jumps
// with E0 to E1; it shows E1 to E2 as a move, which it follows; the station,
// behind, shows E0 to E3 in one step; device 9 then shows E3 with its own pose
// jumping. Merged from E0, its two changes were led by E0 to E3, which carried
// E1 to E2 a second time.
void MergeAcrossFollowedMove(Check check)
{
	MonitorRig rig;
	rig.headset = 10;
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const Q e2(Eigen::AngleAxisd(0.045, V::UnitY()));
	const Q e3(Eigen::AngleAxisd(0.070, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01), t2(0.07, 0, 0.02), t3(0.10, 0, 0.04);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5), jump(0.05, 0, 0);
	for (double t = 10.0; t < 19.0; t += 0.05)
	{
		Q sr = e0, r9 = e0, r10 = e0;
		V st = t0, tr9 = t0, tr10 = t0, local9 = at9;
		if (t >= 12.5 - 1e-9) { r9 = e1; tr9 = t1; local9 = at9 + jump; }        // own pose jumps with E1
		if (t >= 12.6 - 1e-9) { r9 = e2; tr9 = t2; }                             // E2 shown as a move
		if (t >= 12.7 - 1e-9) { sr = e3; st = t3; }                              // the station: E0 to E3
		if (t >= 12.75 - 1e-9) { r9 = e3; tr9 = t3; local9 = at9 + jump + jump; } // E3, its pose jumping
		if (t >= 12.8 - 1e-9) { r10 = e3; tr10 = t3; }                           // device 10 moves with it
		rig.watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, local9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, at10, r10, tr10), TickScale, false);
		if (std::fmod(t + 1e-9, 0.25) < 0.05 || std::abs(t - 12.6) < 1e-6)
			rig.Tick(t);
	}
	const auto right = BackToStart(e3, t3);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9 + jump + jump);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame watch: a change merged across a move the device followed follows that move once",
		off9 < 1e-6 && off10 < 1e-6, detail);
}

// The headset tracker's pose jumps as it comes into the other station's frame
// (G1), a change no recorded move can lead; device 9, in that station's frames,
// then shows G1 to G2 as a move, and the headset tracker shows G2 with its own
// pose jumping again. Merged into the first, its second change waited and was
// settled with it, though the move recorded leads it on its own: the headset
// tracker followed nothing and gave its correction, short of G1 to G2, to
// device 9. It follows G1 to G2 once the first change runs out of time.
void MergedIntoUnleadable(Check check)
{
	MonitorRig rig;
	rig.headset = 10;
	const Q e0 = Q::Identity();
	const Q g0(Eigen::AngleAxisd(0.30, V::UnitY()));
	const Q g1(Eigen::AngleAxisd(0.32, V::UnitY()));
	const Q g2(Eigen::AngleAxisd(0.345, V::UnitY()));
	const V t0(0, 0, 0), u0(0.5, 0, 0.2), u1(0.53, 0, 0.21), u2(0.57, 0, 0.22);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5), jump(0.05, 0, 0);
	for (double t = 10.0; t < 17.0; t += 0.05)
	{
		Q r9 = g0, r10 = e0;
		V tr9 = u0, tr10 = t0, local10 = at10;
		if (t >= 10.3 - 1e-9) { r9 = g1; tr9 = u1; }                                  // G0 to G1, a move
		if (t >= 10.5 - 1e-9) { r10 = g1; tr10 = u1; local10 = at10 + jump; }         // into G1, its pose jumping
		if (t >= 11.5 - 1e-9) { r9 = g2; tr9 = u2; }                                  // G1 to G2, a move
		if (t >= 11.6 - 1e-9) { r10 = g2; tr10 = u2; local10 = at10 + jump + jump; }  // G2, its pose jumping again
		rig.watch.Note(ReplaySample(1, t, V::Zero(), e0, t0), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, at9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, local10, r10, tr10), TickScale, false);
		if (std::fmod(t + 1e-9, 0.25) < 0.05)
			rig.Tick(t);
	}
	// The headset tracker's correction had it followed G1 to G2 from its own,
	// the identity.
	questcal::TrackerFrameCorrections expected;
	LighthouseFrameWatch::Move m;
	m.id = 10;
	m.time = 11.6;
	m.frameTime = 11.5;
	m.rotation = (g2 * g1.conjugate()).normalized();
	m.translation = u2 - m.rotation * u1;
	const bool followed = expected.Follow(m);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], expected.Snapshot()[10], at10 + jump + jump);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], expected.Snapshot()[10], at9);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; short of G1 to G2 (expected followed %d): device 10 %.1f mm, device 9 %.1f mm",
		rig.log.c_str(), followed ? 1 : 0, off10 * 1000.0, off9 * 1000.0);
	check("frame watch: a change merged into one no move leads still follows its own lead", followed && off10 < 1e-6,
		detail);
}

// Device 10 is switched on in a frame SteamVR just moved (E1) while the
// headset tracker (device 9) and the station, a solve behind, still show E0:
// no move into E1 is recorded yet, so its join finds no mate and no device to
// wait for. Kept at once, its correction stayed beside the one the headset
// tracker brought into E1 by the move it followed a moment later.
void JoinBeforeMoveShown(Check check)
{
	MonitorRig rig;
	rig.headset = 9;
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5);
	for (double t = 10.0; t < 17.0; t += 0.05)
	{
		Q sr = e0, r9 = e0;
		V st = t0, tr9 = t0;
		if (t >= 10.9 - 1e-9) { sr = e1; st = t1; }    // the station, a solve behind
		if (t >= 10.95 - 1e-9) { r9 = e1; tr9 = t1; }  // the headset tracker, a solve behind
		rig.watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, at9, r9, tr9), TickScale, false);
		if (t >= 10.5 - 1e-9)                          // switched on in the moved frame
			rig.watch.Note(ReplaySample(10, t, at10, e1, t1), TickScale, false);
		if (std::fmod(t + 1e-9, 0.25) < 0.05)
			rig.Tick(t);
	}
	const bool one = questcal::TrackerFrameCorrections::Agree(rig.frames.Snapshot()[9], rig.frames.Snapshot()[10]);
	const auto right = BackToStart(e1, t1);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; one correction in E1 %d; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), one ? 1 : 0, off9 * 1000.0, off10 * 1000.0);
	check("frame corrections: a device that comes into a moved frame before the move shows takes the frame's correction",
		one, detail);
}

// The headset tracker's pose jumps as it comes into the other station's frame
// (G1), a change no move leads; that station re-solves 4.8 s later and device 9
// shows the move to G2 at once, and the headset tracker, a solve behind, 0.5 s
// later. Its change ran out of time between the two: joined in G1, where no
// device was, it kept its own, then followed the move into G2 beside device
// 9's: two corrections in one frame. It joins in G2 once it shows the move.
void JoinInStaleOwnFrame(Check check)
{
	MonitorRig rig;
	rig.headset = 10;
	const Q e0 = Q::Identity();
	const Q g0(Eigen::AngleAxisd(0.30, V::UnitY()));
	const Q g1(Eigen::AngleAxisd(0.32, V::UnitY()));
	const Q g2(Eigen::AngleAxisd(0.345, V::UnitY()));
	const V t0(0, 0, 0), u0(0.5, 0, 0.2), u1(0.53, 0, 0.21), u2(0.57, 0, 0.22);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5), jump(0.05, 0, 0);
	for (double t = 10.0; t < 18.0; t += 0.05)
	{
		Q r9 = g0, r10 = e0;
		V tr9 = u0, tr10 = t0, local10 = at10;
		if (t >= 10.3 - 1e-9) { r9 = g1; tr9 = u1; }                           // G0 to G1, a move
		if (t >= 10.5 - 1e-9) { r10 = g1; tr10 = u1; local10 = at10 + jump; }  // into G1, its pose jumping
		if (t >= 15.3 - 1e-9) { r9 = g2; tr9 = u2; }                           // G1 to G2, a move
		if (t >= 15.8 - 1e-9) { r10 = g2; tr10 = u2; }                         // the same move, a solve behind
		rig.watch.Note(ReplaySample(1, t, V::Zero(), e0, t0), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, at9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, local10, r10, tr10), TickScale, false);
		if (std::fmod(t + 1e-9, 0.25) < 0.05)
			rig.Tick(t);
	}
	const bool one = questcal::TrackerFrameCorrections::Agree(rig.frames.Snapshot()[9], rig.frames.Snapshot()[10]);
	const double apart = CorrectionShift(rig.frames.Snapshot()[9], rig.frames.Snapshot()[10], at9);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; one correction in G2 %d; the two corrections %.1f mm apart at device 9",
		rig.log.c_str(), one ? 1 : 0, apart * 1000.0);
	check("frame corrections: a device a solve behind when its change settles joins in the frame it follows into",
		one, detail);
}

void RunTrackerFrameCorrectionScenarios(Check check)
{
	SplitFrames(check);
	ReexpressionAndRecovery(check);
	LifetimeAndValidation(check);
	SessionPublication(check);
	ContinuousSpace(check);
	RecalibrationSpace(check);
	RestartRecovery(check);
	RecoveryProtocolGate(check);
	PausedMonitorAndRecalibration(check);
	FrameJoins(check);
	StartupReplacements(check);
	SupersededInference(check);
	HeldFrameChanges(check);
	SwitchOverflow(check);
	DeferredFrameLog(check);
	CollectionFallback(check);
	SetupPlacementOnReturn(check);
	LeadByAnyChain(check);
	FrameJoinedOnReturn(check);
	ChangesInARow(check);
	MoveKeptAcrossStreamRestart(check);
	ChainRecordedOutOfOrder(check);
	MergeAcrossFollowedMove(check);
	MergedIntoUnleadable(check);
	JoinBeforeMoveShown(check);
	JoinInStaleOwnFrame(check);
}
