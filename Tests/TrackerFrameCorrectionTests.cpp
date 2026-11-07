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
	auto all = [](uint32_t) { return true; };
	auto never = [](double) { return false; };
	const auto aligned = missed.Anchor(9, watch, all, never);
	auto bodyAligned = Composed(body);
	pass &= aligned.size() == 1 && aligned[0].id == 14 && missed.Normalize(14, bodyAligned) &&
		missed.Anchor(9, watch, all, never).empty();
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
// Update), replayed against the truth. SteamVR's estimate of each station's pose
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
	uint32_t keep = vr::k_unTrackedDeviceIndexInvalid;
	// Off: the corrections as they were before any join, each device following
	// the moves it showed and keeping its own through a switch.
	bool join = true;
	double time = 10.0;
	std::vector<LighthouseFrameWatch::Switch> switches;
	// The devices a switch or their frame's entry gave a correction to.
	std::vector<questcal::TrackerFrameCorrections::Changed> joined;
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
		LighthouseFrameWatch::SortByTime(handed);
		const auto taken = watch.TakeSwitches();
		for (const auto &change : taken)
			if (!change.held)
				switches.push_back(change);
		if (!join)
		{
			for (const auto &move : handed)
				moves += frames.Follow(move) ? 1 : 0;
			return;
		}
		const auto result = frames.Update(watch, handed, taken, keep,
			[this](uint32_t id) { return devices.count(id) > 0; }, [](double) { return false; });
		moves += handed.size() - result.refused.size();
		for (const auto &c : result.changed)
			if (c.how != questcal::TrackerFrameCorrections::Changed::How::Moved)
				joined.push_back(c);
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
		// Alone in S-13's frame it has that frame's correction already; back in
		// S-3's it takes the moved frame's.
		const bool took = fixed.joined.size() == 1 && fixed.joined[0].id == 16 &&
			fixed.joined[0].how == questcal::TrackerFrameCorrections::Changed::How::Switched;
		double worst = 0;
		for (const uint32_t id : { 9u, 16u, 17u, 18u })
			worst = (std::max)(worst, fixed.Error(id));
		char detail[192];
		snprintf(detail, sizeof detail, "without the join %.1f cm off; with it %.3g m worst, %d returns, %zu joined",
			old.Error(16) * 100.0, worst, returns, fixed.joined.size());
		check("frame join: a body tracker back in its station's moved frame takes the frame's correction (live 00:45)",
			old.Error(16) > 0.07 && old.Error(16) < 0.08 && worst < 1e-9 && returns == 2 && took &&
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
		const bool took = fixed.joined.size() == 1 && fixed.joined[0].id == 14 &&
			fixed.joined[0].how == questcal::TrackerFrameCorrections::Changed::How::Switched;
		double worst = 0;
		for (const uint32_t id : { 9u, 14u, 19u })
			worst = (std::max)(worst, fixed.Error(id));
		char detail[160];
		snprintf(detail, sizeof detail, "without the join %.1f cm off; with it %.3g m worst, %d own-jump switches, %zu joined",
			old.Error(14) * 100.0, worst, jumps, fixed.joined.size());
		check("frame join: a controller back from another station's moving frame takes its own station's moved correction (live 02:06)",
			old.Error(14) > 0.05 && worst < 1e-9 && jumps == 2 && took, detail);
	}

	// A device takes the correction of the frame it comes into, and a device
	// whose correction strays from its frame's is given the frame's back: one
	// frame has one correction. Two body trackers stay in S-3's frame through
	// its move; a third, away in S-13's meanwhile, comes back into it.
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
		const bool took = agree.joined.size() == 1 && agree.joined[0].id == 17 &&
			agree.joined[0].how == questcal::TrackerFrameCorrections::Changed::How::Switched && agree.Error(17) < 1e-9;
		bool restored = false;
		for (const auto &c : apart.joined)
			restored |= c.id == 16 && c.how == questcal::TrackerFrameCorrections::Changed::How::Replaced;
		restored &= apart.frames.Snapshot()[16] == apart.frames.Snapshot()[15] && apart.Error(16) < 1e-9 &&
			apart.Error(17) < 1e-9;
		char detail[160];
		snprintf(detail, sizeof detail, "agree: %zu joined, device 17 %.3g m; apart: device 16 %.3g m, device 17 %.3g m",
			agree.joined.size(), agree.Error(17), apart.Error(16), apart.Error(17));
		check("frame join: a device takes the correction of the frame it comes into; one that strays from it is given it back",
			took && restored, detail);
	}

	// The headset tracker coming into a frame takes its correction like any
	// device: S-13 never moved, so its devices' correction is right, and giving
	// them the headset tracker's, which followed S-3's move, put them 7.4 cm
	// off. A device being measured keeps its own.
	const auto headsetInto = [](uint32_t keep)
	{
		JoinRig rig{ StationS3(), StationS13() };
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
		const auto [takes, headBefore] = headsetInto(vr::k_unTrackedDeviceIndexInvalid);
		const auto [measured, measuredHead] = headsetInto(14);
		const bool took = takes.joined.size() == 1 && takes.joined[0].id == 9 && !(takes.frames.Snapshot()[9] == headBefore) &&
			takes.frames.Snapshot()[14] == protocol::FrameCorrection{} && takes.Error(9) < 1e-9 && takes.Error(14) < 1e-9;
		const bool keptMeasured = measured.frames.Snapshot()[14] == protocol::FrameCorrection{} &&
			measured.Error(9) < 1e-9;
		char detail[160];
		snprintf(detail, sizeof detail, "headset tracker %.3g m, device 14 %.3g m; measured: headset tracker %.3g m",
			takes.Error(9), takes.Error(14), measured.Error(9));
		check("frame join: the headset tracker takes the correction of the frame it comes into, like any device",
			took && keptMeasured, detail);
	}

	// A device first seen in a frame after SteamVR moved it takes the moved
	// frame's correction rather than none. A device whose
	// own pose jumps as its frame moves under it is still handed the move once,
	// with no switch: the headset tracker reported the move first.
	{
		const auto late = [](bool join)
		{
			JoinRig rig{ StationS3(), StationS13() };
			rig.join = join;
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
		check("frame join: a device first seen in a moved frame takes its correction; a move that leads there is followed once",
			appeared && followedOnce, detail);
	}
}
// The overlay's drain (CalibrationMonitor.cpp), against a schedule of frames
// per device: moves of a device not yet known to be a target one are given
// back, moves before `setupUntil` are left alone as SteamVR's universe setup,
// and the rest go through Update. Every device is stationary unless its
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
	size_t followed = 0, leftAlone = 0, heldMoves = 0, waitedSwitches = 0;
	// Each target device's switch, and whether it took its new frame's correction.
	struct Outcome
	{
		LighthouseFrameWatch::Switch change;
		bool took = false;
	};
	std::vector<Outcome> outcomes;

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
		const auto admitted = questcal::TrackerFrameCorrections::Admit(watch, moves, watch.TakeSwitches(),
			[this](uint32_t id, double t) { return Classify(id, t); });
		heldMoves += admitted.waitingMoves.size();
		waitedSwitches += admitted.waitingSwitches.size();
		const auto result = frames.Update(watch, admitted.moves, admitted.switches, vr::k_unTrackedDeviceIndexInvalid,
			[this](uint32_t id) { return Classify(id, time) == Membership::Target; },
			[this](double t) { return t < setupUntil; });
		for (const auto &move : admitted.moves)
			++(move.frameTime < setupUntil ? leftAlone : followed);
		followed -= result.refused.size();
		for (const auto &change : admitted.switches)
		{
			Outcome o{ change, false };
			for (const auto &c : result.changed)
				o.took |= c.id == change.id && c.how == questcal::TrackerFrameCorrections::Changed::How::Switched;
			outcomes.push_back(o);
		}
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
	const size_t waited = rig.waitedSwitches;
	size_t joined11 = 0, joined12 = 0;
	for (const auto &o : rig.outcomes)
	{
		joined11 += o.took && o.change.id == 11 ? 1 : 0;
		joined12 += o.took && o.change.id == 12 ? 1 : 0;
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
		jumped += o.change.id == 10 && o.change.cause == LighthouseFrameWatch::Switch::Cause::OwnJump && o.took ? 1 : 0;
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
	const auto first = questcal::TrackerFrameCorrections::Admit(watch, {}, switches, membership);
	const auto again = watch.TakeMoves();
	const auto second = questcal::TrackerFrameCorrections::Admit(watch, {}, watch.TakeSwitches(),
		[](uint32_t, double) { return Membership::Other; });
	const bool heldThenDropped = first.waitingSwitches.size() == 1 && first.waitingSwitches[0].id == 3 &&
		first.switches.empty() && first.droppedSwitches.empty() && second.droppedSwitches.size() == 1 &&
		second.droppedSwitches[0].id == 3 && second.switches.empty() && watch.TakeSwitches().empty();
	char detail[160];
	std::snprintf(detail, sizeof detail, "moves handed %zu, given back and handed again %zu (held flag %d); switches first %zu, then %zu",
		moves.size(), again.size(), again.empty() ? -1 : int(again.front().held), first.waitingSwitches.size(),
		second.droppedSwitches.size());
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

// UpdateTrackerFrames on the watch's output, as written (the stable sort,
// Update): devices 9 and 10 are the calibrated target devices.
struct MonitorRig
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	std::string log;
	void Tick(double t)
	{
		auto moves = watch.TakeMoves();
		LighthouseFrameWatch::SortByTime(moves);
		const auto switches = watch.TakeSwitches();
		const auto result = frames.Update(watch, moves, switches, vr::k_unTrackedDeviceIndexInvalid,
			[](uint32_t id) { return id == 9 || id == 10; }, [](double) { return false; });
		auto took = [&result](uint32_t id, questcal::TrackerFrameCorrections::Changed::How how)
		{
			for (const auto &c : result.changed)
				if (c.id == id && c.how == how)
					return true;
			return false;
		};
		for (const auto &m : moves)
		{
			const bool refused = std::find(result.refused.begin(), result.refused.end(), m.id) != result.refused.end();
			char line[160];
			std::snprintf(line, sizeof line, "%sdevice %u move (frame moved %.2f, handed %.2f) %s", log.empty() ? "" : "; ",
				m.id, m.frameTime, t, refused ? "refused" : "followed");
			log += line;
		}
		for (const auto &s : switches)
		{
			char line[200];
			std::snprintf(line, sizeof line, "%sdevice %u switch (%s) at %.2f: %s", log.empty() ? "" : "; ",
				s.id, s.cause == LighthouseFrameWatch::Switch::Cause::OwnJump ? "own jump"
					: s.cause == LighthouseFrameWatch::Switch::Cause::Appeared ? "appeared"
					: s.cause == LighthouseFrameWatch::Switch::Cause::Returned ? "returned" : "re-expressed",
				t, took(s.id, questcal::TrackerFrameCorrections::Changed::How::Switched) ? "took its frame's" : "kept");
			log += line;
		}
		for (const auto &c : result.changed)
			if (c.how == questcal::TrackerFrameCorrections::Changed::How::Replaced)
			{
				char line[96];
				std::snprintf(line, sizeof line, "%sdevice %u took its frame's replaced correction at %.2f",
					log.empty() ? "" : "; ", c.id, t);
				log += line;
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
	// DuringUniverseSetup(ctx, time), from the log's word.
	auto setup = [&visibility](double time)
	{
		return visibility.Universe(time) == LighthouseVisibility::UniverseSetup::SettingUp;
	};
	auto drain = [&](double now)
	{
		const auto moves = watch.TakeMoves();
		frames.Update(watch, moves, watch.TakeSwitches(), vr::k_unTrackedDeviceIndexInvalid,
			[](uint32_t id) { return id == 9 || id == 10; }, setup);
		for (const auto &move : moves)
		{
			char line[160];
			std::snprintf(line, sizeof line, "%sdevice %u move at %.2f: universe %s, %s", decisions.empty() ? "" : "; ",
				move.id, move.time, UniverseName(visibility.Universe(move.frameTime)),
				setup(move.frameTime) ? "left alone" : "followed");
			decisions += line;
			handed.push_back(move);
		}
		(void)now;
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

// SteamVR re-placing a station it found knocked corrects that station's pose:
// the trackers in its frame were off from the knock until then (2026-11-06:
// S-16 re-placed 12.8 deg and 39 cm, 31 s after its reset line, and followed
// as a frame move it split them from the other frames). The re-placement is
// left alone, as during the universe setup; another station's re-solve at the
// same moment is followed. The predicate is CalibrationMonitor.cpp's
// ReplacesBumpedStation: the knocked station's own device in the frame.
void KnockedStationReplacement(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	LighthouseVisibility visibility;
	const uint32_t knocked = 0x04D47FB4;
	const Q aR = Q(Eigen::AngleAxisd(0.8, V::UnitY()));
	const V aT(1.1, 1.15, -0.58);
	const Q aR2 = Q(Eigen::AngleAxisd(12.8 * questcal::Pi / 180.0, V(0.3, 0.2, 0.93).normalized())) * aR;
	const V aT2 = aT + V(-0.014, -0.012, 0.005);
	const Q bR = Q(Eigen::AngleAxisd(-1.1, V::UnitY()));
	const V bT(-2.0, 1.5, -0.1);
	const Q bR2 = Q(Eigen::AngleAxisd(1.0 * questcal::Pi / 180.0, V::UnitY())) * bR;
	const V bT2 = bT + V(0.03, 0.0, 0.0);
	const V at9(-1.0, -0.2, 1.5), at10(-0.5, -0.9, 1.2), at11(1.5, -0.3, 0.8);
	auto leftAlone = [&](double time, const Q &fromR, const V &fromT, const Q &toR, const V &toT)
	{
		auto same = [](const Q &a, const V &aAt, const Q &b, const V &bAt)
		{
			return a.angularDistance(b) < 3.0 * questcal::Pi / 180.0 && (aAt - bAt).norm() < 0.10;
		};
		Q r;
		V tr;
		return visibility.ReplacingBumped(knocked, time) && watch.StationFrame(1, r, tr) &&
			(same(r, tr, fromR, fromT) || same(r, tr, toR, toT));
	};
	lighthouselog::Event bump;
	bump.kind = lighthouselog::Event::Kind::StationBumped;
	bump.stationId = knocked;
	lighthouselog::Event moved;
	moved.kind = lighthouselog::Event::Kind::StationMoved;
	moved.stationId = knocked;
	moved.moveMeters = 0.019;
	moved.moveDegrees = 0.1;
	const std::string bumpNote = visibility.Apply(bump, 11.0);
	const std::string burstNote = visibility.Apply(bump, 11.01);   // one knock, one note
	const bool pendingBefore = !visibility.ReplacingBumped(knocked, 9.0) && visibility.ReplacingBumped(knocked, 14.0);
	bool lineApplied = false;
	int leftAloneMoves = 0, followedMoves = 0;
	for (double t = 10.0; t < 25.0; t += 0.05)
	{
		const bool replaced = t >= 15.0;
		const Q &a = replaced ? aR2 : aR;
		const V &aAt = replaced ? aT2 : aT;
		const Q &b = replaced ? bR2 : bR;
		const V &bAt = replaced ? bT2 : bT;
		watch.Note(ReplaySample(1, t, V::Zero(), a, aAt), TickScale, true);   // the knocked station
		watch.Note(ReplaySample(9, t, at9, a, aAt), TickScale, false);
		watch.Note(ReplaySample(10, t, at10, a, aAt), TickScale, false);
		watch.Note(ReplaySample(2, t, V::Zero(), b, bAt), TickScale, true);   // another station
		watch.Note(ReplaySample(11, t, at11, b, bAt), TickScale, false);
		if (!lineApplied && t >= 15.2)   // SteamVR's line, read a little after the poses
		{
			visibility.Apply(moved, t);
			lineApplied = true;
		}
		const auto moves = watch.TakeMoves();
		frames.Update(watch, moves, watch.TakeSwitches(), vr::k_unTrackedDeviceIndexInvalid,
			[](uint32_t id) { return id == 9 || id == 10 || id == 11; }, leftAlone);
		for (const auto &move : moves)
			++(leftAlone(move.frameTime, move.fromRot, move.fromTrans, move.toRot, move.toTrans)
				? leftAloneMoves : followedMoves);
	}
	const bool closedAfter = !visibility.ReplacingBumped(knocked, 15.2 + 5.5);
	const double shift9 = CorrectionShift(frames.Snapshot()[9], protocol::FrameCorrection{}, aT + aR * at9);
	const double shift10 = CorrectionShift(frames.Snapshot()[10], protocol::FrameCorrection{}, aT + aR * at10);
	const double shift11 = CorrectionShift(frames.Snapshot()[11], protocol::FrameCorrection{}, bT + bR * at11);
	char detail[320];
	std::snprintf(detail, sizeof detail,
		"note \"%s\", burst note %d, pending %d, closed after %d; moves left alone %d, followed %d; "
		"corrections moved %.1f and %.1f cm in the knocked frame, %.1f cm in the other",
		bumpNote.substr(0, 48).c_str(), burstNote.empty() ? 0 : 1, pendingBefore, closedAfter,
		leftAloneMoves, followedMoves, shift9 * 100.0, shift10 * 100.0, shift11 * 100.0);
	check("frame corrections: SteamVR's re-placement of a knocked base station is a correction, not a frame move",
		!bumpNote.empty() && burstNote.empty() && pendingBefore && closedAfter &&
			leftAloneMoves == 2 && followedMoves == 1 &&
			shift9 < 1e-9 && shift10 < 1e-9 && shift11 > 0.01, detail);
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
	char detail[480];
	std::snprintf(detail, sizeof detail,
		"one correction in S-3's frame %d; error at device 9 %.2f mm, at device 10 %.1f cm; moves followed %zu; "
		"joins %zu", one ? 1 : 0, rig.Error(9) * 1000.0, rig.Error(10) * 100.0, rig.moves, rig.joined.size());
	check("frame corrections: a device alone in a moved frame joins the calibrated device that comes back into it",
		one && rig.Error(9) < 1e-9 && rig.Error(10) < 1e-9, detail);
}

// The headset tracker's own pose jumps at two re-solves of its station in a
// row; the lead for the first comes a moment after the second, from a device a
// solve behind. The two changes wait as one, and the headset tracker follows
// both. Settled as a switch when the second came, the first had the headset
// tracker give a correction that never followed it to the device that did.
void ChangesInARow(Check check)
{
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
		LighthouseFrameWatch::SortByTime(moves);
		const auto result = frames.Update(watch, moves, watch.TakeSwitches(), vr::k_unTrackedDeviceIndexInvalid,
			[](uint32_t id) { return id == 9 || id == 10; }, [](double) { return false; });
		followed += moves.size() - result.refused.size();
		for (const auto &c : result.changed)
			if (c.how != questcal::TrackerFrameCorrections::Changed::How::Moved)
			{
				char line[96];
				std::snprintf(line, sizeof line, "%sdevice %u took its frame's at %.2f", joins.empty() ? "" : ", ", c.id, t);
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

// Device 10's pose jumps as it comes into the other station's frame (G1), a
// change no recorded move can lead; device 9, in that station's frames since
// G0, then shows G1 to G2 as a move, and device 10 shows G2 with its own pose
// jumping again. Merged into the first, its second change waited and was
// settled with it, though the move recorded leads it on its own: device 10
// followed nothing. The watch hands it G1 to G2 once the first change runs out
// of time, and it ends with G2's correction, the one device 9 derived from G0.
void MergedIntoUnleadable(Check check)
{
	MonitorRig rig;
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
	// G2's correction: G0's, the identity, carried across G0 to G2.
	const auto right = BackToStart((g2 * g0.conjugate()).normalized(), u2 - (g2 * g0.conjugate()).normalized() * u0);
	const bool led = rig.log.find("device 10 move (frame moved 11.50") != std::string::npos;
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10 + jump + jump);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; G1 to G2 handed to device 10 %d; off G2's correction: device 10 %.1f mm, "
		"device 9 %.1f mm", rig.log.c_str(), led ? 1 : 0, off10 * 1000.0, off9 * 1000.0);
	check("frame watch: a change merged into one no move leads still follows its own lead", led && off10 < 1e-6 &&
		off9 < 1e-6, detail);
}

// Device 10 is switched on in a frame SteamVR just moved (E1) while the
// headset tracker (device 9) and the station, a solve behind, still show E0:
// no move into E1 is recorded yet, so its join finds no mate and no device to
// wait for. Kept at once, its correction stayed beside the one the headset
// tracker brought into E1 by the move it followed a moment later.
void JoinBeforeMoveShown(Check check)
{
	MonitorRig rig;
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

// Device 10 is switched on in a frame SteamVR just moved (E1); the headset
// tracker (device 9) and the station, a solve behind, show the move exactly
// lagSeconds later, the headset tracker with its pose jumping, and the
// monitor's tick runs between their two samples. The join's wait for the move
// ended at that tick with the headset tracker's change still waiting for its
// lead, so neither in the frame for the join nor still to come into it:
// device 10 kept its own, and the station's move then led the headset
// tracker into E1 beside it.
void JoinAtLagBound(Check check)
{
	MonitorRig rig;
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5), jump(0.05, 0, 0);
	for (int k = 0; k < 140; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const bool shown = k >= 30;   // 11.5 s: lagSeconds after device 10 came on
		const bool tick = k % 5 == 0;
		rig.watch.Note(ReplaySample(9, t, shown ? at9 + jump : at9, shown ? e1 : e0, shown ? t1 : t0), TickScale, false);
		if (k >= 10)
			rig.watch.Note(ReplaySample(10, t, at10, e1, t1), TickScale, false);   // switched on in the moved frame
		if (tick && k == 30)
			rig.Tick(t);   // between the headset tracker's sample and the station's
		rig.watch.Note(ReplaySample(1, t, V::Zero(), shown ? e1 : e0, shown ? t1 : t0), TickScale, true);
		if (tick && k != 30)
			rig.Tick(t);
	}
	const bool one = questcal::TrackerFrameCorrections::Agree(rig.frames.Snapshot()[9], rig.frames.Snapshot()[10]);
	const auto right = BackToStart(e1, t1);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9 + jump);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; one correction in E1 %d; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), one ? 1 : 0, off9 * 1000.0, off10 * 1000.0);
	check("frame corrections: a device in a moved frame waits through the lag bound for the device the move leads in",
		one, detail);
}

// Device 9 is switched off just after SteamVR moved its station's frame (E0
// to E1), before it showed the move, so its own join waits for it to show
// that move; device 10 is switched on in E1. With device 9's switch still to
// join, device 10's join counted it neither as a calibrated device in the
// frame nor as one still to come into it: alone, device 10 kept its own.
// Back 5.65 s later, device 9 followed the move, and its join, free at last,
// took device 10's correction.
void JoinWaitsForDeviceOffThroughMove(Check check)
{
	MonitorRig rig;   // no headset tracker
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5);
	for (int k = 0; k < 170; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const bool moved = k >= 6;   // 10.3 s
		rig.watch.Note(ReplaySample(1, t, V::Zero(), moved ? e1 : e0, moved ? t1 : t0), TickScale, true);
		if (k <= 6)
			rig.watch.Note(ReplaySample(9, t, at9, e0, t0), TickScale, false);     // still E0 at 10.3 s
		else if (k < 120)
			rig.watch.Note(ReplayOff(9, t), TickScale, false);                     // off from 10.35 s to 16 s
		else
			rig.watch.Note(ReplaySample(9, t, at9, e1, t1), TickScale, false);
		if (k >= 12)
			rig.watch.Note(ReplaySample(10, t, at10, e1, t1), TickScale, false);   // switched on in E1 at 10.6 s
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const auto right = BackToStart(e1, t1);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame corrections: a device in a moved frame waits for a calibrated device off through the move",
		off9 < 1e-6 && off10 < 1e-6, detail);
}

// As above, with device 9 first seen after device 10 came into E1: in E0, a
// solve behind the station's move, and off before it shows it. Back, it
// followed the move into E1, but its switch, still to join, came after
// device 10's: device 10's join, first, found it none of its mates and kept
// its own, and device 9's then took that correction.
void JoinAfterDeviceOffThroughMove(Check check)
{
	MonitorRig rig;   // no headset tracker
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5);
	for (int k = 0; k < 170; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const bool moved = k >= 6;   // 10.3 s
		rig.watch.Note(ReplaySample(1, t, V::Zero(), moved ? e1 : e0, moved ? t1 : t0), TickScale, true);
		if (k >= 14 && k <= 15)
			rig.watch.Note(ReplaySample(9, t, at9, e0, t0), TickScale, false);     // first seen at 10.7 s, in E0
		else if (k > 15 && k < 120)
			rig.watch.Note(ReplayOff(9, t), TickScale, false);                     // off from 10.8 s to 16 s
		else if (k >= 120)
			rig.watch.Note(ReplaySample(9, t, at9, e1, t1), TickScale, false);
		if (k >= 12)
			rig.watch.Note(ReplaySample(10, t, at10, e1, t1), TickScale, false);   // switched on in E1 at 10.6 s
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const auto right = BackToStart(e1, t1);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame corrections: a device back by a move joins before the devices that came into its frame meanwhile",
		off9 < 1e-6 && off10 < 1e-6, detail);
}

// Device 10 is switched on in E1 just after SteamVR moved the frame, and
// device 9 is first seen in E0 a moment later, a solve behind, and shows the
// move, which it follows. Joined first, device 9's switch still waits out
// the lag after it; device 10's wait was over a tick earlier, with device 9
// still to join and so no mate: device 10 kept its own, and device 9's join
// took it.
void JoinWaitsForDeviceThatFollowedIn(Check check)
{
	MonitorRig rig;   // no headset tracker
	const Q e0 = Q::Identity();
	const Q e1(Eigen::AngleAxisd(0.020, V::UnitY()));
	const V t0(0, 0, 0), t1(0.03, 0, 0.01);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5);
	for (int k = 0; k < 100; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const bool moved = k >= 6;   // 10.3 s
		rig.watch.Note(ReplaySample(1, t, V::Zero(), moved ? e1 : e0, moved ? t1 : t0), TickScale, true);
		if (k >= 9)
			rig.watch.Note(ReplaySample(10, t, at10, e1, t1), TickScale, false);   // switched on in E1 at 10.45 s
		if (k == 14)
			rig.watch.Note(ReplaySample(9, t, at9, e0, t0), TickScale, false);     // first seen at 10.7 s, in E0
		else if (k > 14)
			rig.watch.Note(ReplaySample(9, t, at9, e1, t1), TickScale, false);     // the move, at 10.75 s
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const auto right = BackToStart(e1, t1);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, at9);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, at10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame corrections: a device in a moved frame waits for a device that followed the move in to join first",
		off9 < 1e-6 && off10 < 1e-6, detail);
}

// With no monitor tick from 10 s to 15.5 s. Device 9 is seen in E0 and
// device 10 in the other station's frame (G0); SteamVR re-solves that
// station (G1), which device 10 shows as a move, and device 9 comes into G1
// with its own pose jumping, a change no move leads, settled as a switch
// when it runs out of time; the station is then re-solved again (G2), which
// both show as a move. The next tick had three switches, all of devices
// that had followed a move since: device 9's first (E0), device 10's (G0)
// and device 9's own jump (G1). Device 9's first, joined in G2 where
// device 10, still to join, was no mate, kept its own, and device 10's join
// then took it, losing the moves it had followed.
void NewestSwitchStandsForOlder(Check check)
{
	MonitorRig rig;   // no headset tracker
	const Q e0 = Q::Identity();
	const Q g0(Eigen::AngleAxisd(0.30, V::UnitY()));
	const Q g1(Eigen::AngleAxisd(0.32, V::UnitY()));
	const Q g2(Eigen::AngleAxisd(0.345, V::UnitY()));
	const V t0(0, 0, 0), u0(0.5, 0, 0.2), u1(0.53, 0, 0.21), u2(0.57, 0, 0.22);
	const V at9(0.4, 1.0, -0.6), at10(-0.3, 0.9, 0.5), jump(0.05, 0, 0);
	for (int k = 0; k < 140; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		Q r9 = e0, r10 = g0;
		V tr9 = t0, tr10 = u0, local9 = at9;
		if (k >= 4) { r10 = g1; tr10 = u1; }                       // 10.2 s: G0 to G1, a move
		if (k >= 6) { r9 = g1; tr9 = u1; local9 = at9 + jump; }   // 10.3 s: into G1, its pose jumping
		if (k >= 108) { r9 = r10 = g2; tr9 = tr10 = u2; }         // 15.4 s: G1 to G2, a move for both
		rig.watch.Note(ReplaySample(1, t, V::Zero(), e0, t0), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, local9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, at10, r10, tr10), TickScale, false);
		if (k >= 110 && k % 5 == 0)   // the monitor ticks again from 15.5 s
			rig.Tick(t);
	}
	// Device 10's correction had it followed G0 to G1 and G1 to G2 from its
	// own, the identity.
	questcal::TrackerFrameCorrections expected;
	auto follow = [&](const Q &fromR, const V &fromT, const Q &toR, const V &toT, double at)
	{
		LighthouseFrameWatch::Move m;
		m.id = 10;
		m.time = at;
		m.frameTime = at;
		m.rotation = (toR * fromR.conjugate()).normalized();
		m.translation = toT - m.rotation * fromT;
		return expected.Follow(m);
	};
	const bool followed = follow(g0, u0, g1, u1, 10.2) && follow(g1, u1, g2, u2, 15.4);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], expected.Snapshot()[10], at10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; device 10 off the correction it followed (expected followed %d): %.1f mm",
		rig.log.c_str(), followed ? 1 : 0, off10 * 1000.0);
	check("frame corrections: a device's older switch is not joined in the frame its newer one brought it to",
		followed && off10 < 1e-6, detail);
}

// The races the joins had, replayed on one correction per frame
// (TrackerFrameCorrections::Update). Each of these left a device off its
// frame's correction on the joins after 4f98658; the VirtualQuest model
// formal/frame-corrections/TrackerFramesAnchored.tla checks the same rules.
// E0 to E3 are one station's frames, moved in turn by SteamVR's re-solves.
namespace
{
const Q AnchorE0 = Q::Identity();
const Q AnchorE1(Eigen::AngleAxisd(0.020, V::UnitY()));
const Q AnchorE2(Eigen::AngleAxisd(0.045, V::UnitY()));
const Q AnchorE3(Eigen::AngleAxisd(0.070, V::UnitY()));
const V AnchorT0(0, 0, 0), AnchorT1(0.03, 0, 0.01), AnchorT2(0.07, 0, 0.02), AnchorT3(0.10, 0, 0.04);
const V AnchorAt9(0.4, 1.0, -0.6), AnchorAt10(-0.3, 0.9, 0.5), AnchorJump(0.05, 0, 0);

// Device 10 is switched on in E1 just after SteamVR moved the frame from E0
// (10.3 s); device 9 is first seen in E0, a solve behind, and SteamVR moves the
// frame again, E1 to E2, which both show as a move, device 9 from E0 in one
// step. On the joins, neither waited for the other, and device 9 took the
// correction device 10 kept alone: device 9 18.1 mm off, device 10 43.1 mm,
// with the second move before device 9's first tick or (`late`) after it.
void AnchoredBothMovedOn(Check check, bool late)
{
	MonitorRig rig;
	const int second = late ? 16 : 12;   // E1 to E2: 10.8 s, or 10.6 s
	for (int k = 0; k < 100; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const Q sr = k >= second ? AnchorE2 : k >= 6 ? AnchorE1 : AnchorE0;   // the station shows each move at once
		const V st = k >= second ? AnchorT2 : k >= 6 ? AnchorT1 : AnchorT0;
		rig.watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
		if (k >= 9)
			rig.watch.Note(ReplaySample(10, t, AnchorAt10, k > second ? AnchorE2 : AnchorE1,
				k > second ? AnchorT2 : AnchorT1), TickScale, false);
		if (k == 14)
			rig.watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k > 14)
			rig.watch.Note(ReplaySample(9, t, AnchorAt9, k > second + 1 ? AnchorE2 : AnchorE1,
				k > second + 1 ? AnchorT2 : AnchorT1), TickScale, false);
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const auto right = BackToStart(AnchorE2, AnchorT2);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, AnchorAt9);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, AnchorAt10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "second move %s: %s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		late ? "after 10.75 s" : "before 10.75 s", rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check(late ? "frame anchors: two devices that both moved on into a frame end right, the second move late"
		: "frame anchors: two devices that both moved on into a frame end right",
		rig.frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// SteamVR moves the frame from E0 to E1 (10.3 s) and again to E2 (11.0 s); the
// station, a solve behind, shows E0 to E2 in one step, so no move into or out
// of E1 is ever recorded. Device 10, switched on in E1, carries its own in
// there, short of E0 to E1, and takes E2's once it shows E1 to E2. On the
// joins, alone in E1 it kept its own and followed it into E2 beside device 9's
// (43.1 mm).
void AnchoredSkippedFrame(Check check)
{
	MonitorRig rig;
	for (int k = 0; k < 120; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		rig.watch.Note(ReplaySample(1, t, V::Zero(), k >= 21 ? AnchorE2 : AnchorE0, k >= 21 ? AnchorT2 : AnchorT0),
			TickScale, true);
		if (k >= 7)
			rig.watch.Note(ReplaySample(9, t, AnchorAt9, k >= 22 ? AnchorE2 : AnchorE0, k >= 22 ? AnchorT2 : AnchorT0),
				TickScale, false);
		if (k >= 8)
			rig.watch.Note(ReplaySample(10, t, AnchorAt10, k >= 32 ? AnchorE2 : AnchorE1, k >= 32 ? AnchorT2 : AnchorT1),
				TickScale, false);
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const auto right = BackToStart(AnchorE2, AnchorT2);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, AnchorAt9);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, AnchorAt10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: a device in a frame the station's record steps over takes the next frame's correction",
		rig.frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// Device 9's pose jumps as SteamVR moves the frame from E0 to E1 (12.5 s), a
// change that waits for its lead; it then shows E1 to E2 as a move (12.6 s).
// The station, a solve behind, shows E0 to E1 (12.8 s), which leads the waiting
// change after the newer move was handed. Follow refused that older move, and
// device 9 stayed short of E0 to E1 beside device 10 (18.0 mm).
void AnchoredLeadAfterNewerMove(Check check)
{
	MonitorRig rig;
	for (int k = 0; k < 160; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		Q sr = AnchorE0, r9 = AnchorE0, r10 = AnchorE0;
		V st = AnchorT0, tr9 = AnchorT0, tr10 = AnchorT0, local9 = AnchorAt9;
		if (k >= 50) { r9 = AnchorE1; tr9 = AnchorT1; local9 = AnchorAt9 + AnchorJump; }
		if (k >= 52) { r9 = AnchorE2; tr9 = AnchorT2; }
		if (k >= 56) { sr = AnchorE1; st = AnchorT1; }
		if (k >= 58) { sr = AnchorE2; st = AnchorT2; }
		if (k >= 57) { r10 = AnchorE1; tr10 = AnchorT1; }
		if (k >= 59) { r10 = AnchorE2; tr10 = AnchorT2; }
		rig.watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, local9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, AnchorAt10, r10, tr10), TickScale, false);
		if (k % 5 == 0 || k == 52)
			rig.Tick(t);
	}
	const auto right = BackToStart(AnchorE2, AnchorT2);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, AnchorAt9 + AnchorJump);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, AnchorAt10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: a change led only after the device showed a newer move ends at the frame's correction",
		rig.frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// SteamVR moves the frame from E0 to E1 (10.3 s), which only the station shows;
// device 10 is switched on in E1 at 11.0 s, alone. The station's frame is
// anchored at the start, so its move anchors E1. On the joins, no device had
// followed the move, and device 10 kept its own (43.1 mm).
void AnchoredSwitchedOnAfterMove(Check check)
{
	MonitorRig rig;
	for (int k = 0; k < 80; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		rig.watch.Note(ReplaySample(1, t, V::Zero(), k >= 6 ? AnchorE1 : AnchorE0, k >= 6 ? AnchorT1 : AnchorT0),
			TickScale, true);
		if (k >= 20)
			rig.watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE1, AnchorT1), TickScale, false);
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], BackToStart(AnchorE1, AnchorT1), AnchorAt10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 10 %.1f mm", rig.log.c_str(), off10 * 1000.0);
	check("frame anchors: a device switched on alone after a move only the base station showed takes the move",
		rig.frames.Unsettled() == 0 && off10 < 1e-6, detail);
}

// A station seen only through its trackers: SteamVR re-solves it (E0 to E1,
// 10.3 s) before any of them is on. Device 10 is switched on in E1 and carries
// its own in, short of the re-solve; device 9 is first seen in E0, a solve
// behind, and shows E0 to E1 as a move. The record orders E0 before E1, so the
// correction device 9 carries from E0 is the older one, and E1 takes it.
// Anchored as the start's, the first frame a device showed kept device 10's.
void AnchoredUnseenResolve(Check check)
{
	MonitorRig rig;
	for (int k = 0; k < 80; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		if (k >= 9)
			rig.watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE1, AnchorT1), TickScale, false);
		if (k == 14)
			rig.watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k > 14)
			rig.watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const auto right = BackToStart(AnchorE1, AnchorT1);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, AnchorAt9);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, AnchorAt10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: in a station seen only through its trackers, the frame the record orders first defines the next",
		rig.frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// Device 10, in another station's frames (H0, then H1 by a move it follows),
// comes into a third frame G with its own pose jumping, a change no move leads:
// it carries its correction in, made for H. Device 9, anchored in the base
// station's E0, is then re-expressed into G, its world pose kept, so E0's
// correction is right in G: G takes it, and device 10 with it.
void AnchoredReexpressedIntoCarried(Check check)
{
	MonitorRig rig;
	const Q h0(Eigen::AngleAxisd(0.30, V::UnitY())), h1(Eigen::AngleAxisd(0.32, V::UnitY()));
	const V u0(0.5, 0, 0.2), u1(0.53, 0, 0.21);
	const Q g(Eigen::AngleAxisd(-0.25, V::UnitY()));
	const V gt(-0.4, 0.0, 0.3);
	// Device 9's local pose in G that keeps its world pose (E0 is the identity).
	const V at9InG = g.conjugate() * (AnchorAt9 - gt);
	for (int k = 0; k < 200; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const bool into10 = k >= 20, into9 = k >= 150;   // 11.0 s and 17.5 s
		rig.watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		auto s9 = into9 ? ReplaySample(9, t, at9InG, g, gt) : ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0);
		if (into9)
			s9.rotation = questcal::WireQuaternion(g.conjugate());   // its world rotation kept too
		rig.watch.Note(s9, TickScale, false);
		if (into10)
			rig.watch.Note(ReplaySample(10, t, AnchorAt10 + AnchorJump, g, gt), TickScale, false);
		else
			rig.watch.Note(ReplaySample(10, t, AnchorAt10, k >= 10 ? h1 : h0, k >= 10 ? u1 : u0), TickScale, false);
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const bool one = rig.frames.Snapshot()[9] == rig.frames.Snapshot()[10];
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], protocol::FrameCorrection{}, AnchorAt9);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; one correction in G %d; device 9 off E0's %.1f mm", rig.log.c_str(),
		one ? 1 : 0, off9 * 1000.0);
	check("frame anchors: a device re-expressed from an anchored frame anchors the frame a device jumped into",
		rig.frames.Unsettled() == 0 && one && off9 < 1e-9, detail);
}

// Device 9's pose jumps with E0 to E1; it shows E1 to E2 as a move; the
// station, behind, shows E0 to E2 and E2 to E3; device 9 shows E3 with its pose
// jumping again, a change that does not continue the first, which is settled
// as a switch; device 10 then shows E0 to E1 and E1 to E3 as moves. On the
// joins, with device 9 the headset tracker, its settled switch gave device 10
// its correction, short of E0 to E1: device 9 17.7 mm off, device 10 42.9 mm.
void AnchoredSettledApart(Check check)
{
	MonitorRig rig;
	for (double t = 10.0; t < 19.0; t += 0.05)
	{
		Q sr = AnchorE0, r9 = AnchorE0, r10 = AnchorE0;
		V st = AnchorT0, tr9 = AnchorT0, tr10 = AnchorT0, local9 = AnchorAt9;
		if (t >= 12.5 - 1e-9) { r9 = AnchorE1; tr9 = AnchorT1; local9 = AnchorAt9 + AnchorJump; }
		if (t >= 12.6 - 1e-9) { r9 = AnchorE2; tr9 = AnchorT2; }
		if (t >= 12.65 - 1e-9) { sr = AnchorE2; st = AnchorT2; }
		if (t >= 12.7 - 1e-9) { sr = AnchorE3; st = AnchorT3; }
		if (t >= 12.75 - 1e-9) { r9 = AnchorE3; tr9 = AnchorT3; local9 = AnchorAt9 + AnchorJump + AnchorJump; }
		if (t >= 12.8 - 1e-9) { r10 = AnchorE1; tr10 = AnchorT1; }
		if (t >= 12.85 - 1e-9) { r10 = AnchorE3; tr10 = AnchorT3; }
		rig.watch.Note(ReplaySample(1, t, V::Zero(), sr, st), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, local9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, AnchorAt10, r10, tr10), TickScale, false);
		if (std::fmod(t + 1e-9, 0.25) < 0.05 || std::abs(t - 12.6) < 1e-6)
			rig.Tick(t);
	}
	const auto right = BackToStart(AnchorE3, AnchorT3);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, AnchorAt9 + AnchorJump + AnchorJump);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, AnchorAt10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off the right correction: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: a change settled apart leaves both devices at the frame's correction",
		rig.frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// With the monitor not ticking from 10 s to 15.5 s: device 9 is seen in E0,
// device 10 in another station's frame (G0); SteamVR re-solves that station
// (G1), which device 10 shows as a move, and device 9 comes into G1 with its
// own pose jumping. The next tick takes all three switches and the move at
// once. On the joins, device 9's older switch was joined in G1 and device 10
// took its correction, the move it followed lost (44.6 mm).
void AnchoredSwitchesInOneDrain(Check check)
{
	MonitorRig rig;
	const Q g0(Eigen::AngleAxisd(0.30, V::UnitY())), g1(Eigen::AngleAxisd(0.32, V::UnitY()));
	const V u0(0.5, 0, 0.2), u1(0.53, 0, 0.21);
	for (int k = 0; k < 140; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		Q r9 = AnchorE0, r10 = g0;
		V tr9 = AnchorT0, tr10 = u0, local9 = AnchorAt9;
		if (k >= 4) { r10 = g1; tr10 = u1; }
		if (k >= 6) { r9 = g1; tr9 = u1; local9 = AnchorAt9 + AnchorJump; }
		rig.watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		rig.watch.Note(ReplaySample(9, t, local9, r9, tr9), TickScale, false);
		rig.watch.Note(ReplaySample(10, t, AnchorAt10, r10, tr10), TickScale, false);
		if (k >= 110 && k % 5 == 0)
			rig.Tick(t);
	}
	const Q delta = (g1 * g0.conjugate()).normalized();
	const auto right = BackToStart(delta, u1 - delta * u0);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, AnchorAt10);
	const bool one = rig.frames.Snapshot()[9] == rig.frames.Snapshot()[10];
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; one correction in G1 %d; device 10 off the move it followed %.1f mm",
		rig.log.c_str(), one ? 1 : 0, off10 * 1000.0);
	check("frame anchors: switches and a move taken in one drain keep the move a device followed",
		rig.frames.Unsettled() == 0 && one && off10 < 1e-6, detail);
}

// A measurement keeps its target's correction as the run began with it but
// for the moves it shows, though its frame's correction is defined under it:
// device 10, measured from when it is switched on in E1 after an unseen
// re-solve, keeps its own, short of the re-solve, while device 9 brings E1 the
// older root's. Once the run ends it takes the frame's (Update's `held`).
void AnchoredMeasuredDeviceHeld(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = 10;
	protocol::FrameCorrection during;
	for (int k = 0; k < 120; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		if (k >= 9)
			watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE1, AnchorT1), TickScale, false);
		if (k == 14)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k > 14)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		if (k == 60)
		{
			during = frames.Snapshot()[10];
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run ends
		}
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held,
				[](uint32_t id) { return id == 9 || id == 10; }, [](double) { return false; });
	}
	const auto right = BackToStart(AnchorE1, AnchorT1);
	const bool keptDuring = during == protocol::FrameCorrection{};
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, AnchorAt9);
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, AnchorAt10);
	char detail[200];
	std::snprintf(detail, sizeof detail, "device 10 kept its own while measured %d; after: device 9 %.1f mm, device 10 %.1f mm",
		keptDuring ? 1 : 0, off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: a measured device keeps its correction until the run ends, then takes its frame's",
		frames.Unsettled() == 0 && keptDuring && off9 < 1e-6 && off10 < 1e-6, detail);
}

// Restored corrections that disagree in one frame: device 10's, first seen in
// E0, is that frame's, and device 9 takes it. Within one drain device 9 then
// comes into another station's frame G with its own pose jumping, a change no
// move leads: it carries E0's correction there, the one it had in the frame it
// left, not the one it was restored with.
void AnchoredJumpCarriesFrame(Check check)
{
	MonitorRig rig;
	questcal::TrackerFrameCorrections::Frames saved{};
	const Q n(Eigen::AngleAxisd(0.03, V::UnitY()));
	saved[10].rotation = { n.w(), n.x(), n.y(), n.z() };
	saved[10].translation = { { 0.02, 0.0, 0.01 } };
	rig.frames.Restore(saved, questcal::FrameSerialKeys{});
	const Q g(Eigen::AngleAxisd(0.30, V::UnitY()));
	const V gt(0.5, 0, 0.2);
	for (int k = 0; k < 120; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		rig.watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE0, AnchorT0), TickScale, false);
		rig.watch.Note(k >= 6 ? ReplaySample(9, t, AnchorAt9 + AnchorJump, g, gt)
			: ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		if (k >= 110 && k % 5 == 0)   // one drain from 15.5 s, the jump settled as a switch
			rig.Tick(t);
	}
	const bool carried = rig.frames.Snapshot()[9] == saved[10] && rig.frames.Snapshot()[10] == saved[10];
	char detail[600];
	std::snprintf(detail, sizeof detail, "%s; device 9 carried E0's correction into G %d", rig.log.c_str(), carried ? 1 : 0);
	check("frame anchors: a device that jumps out of a frame carries that frame's correction", rig.frames.Unsettled() == 0 && carried, detail);
}

// Device 9 jumps into another station's frame F (no base station of its own
// there), settled as a switch before the monitor's first tick, so F's root is
// that unclear change; device 10 comes on in F. SteamVR re-solves the station,
// F to G: device 9's own pose jumps with the move, a change waiting for its
// lead, and device 10, a solve behind, shows it as a move a moment later.
// Device 9 keeps its correction while the change waits: taken as G's then, it
// defined G with a correction that never followed F to G, and the derived
// one, rooted in the unclear change, gave way to it.
void AnchoredUnsettledWaits(Check check)
{
	MonitorRig rig;
	const Q f(Eigen::AngleAxisd(0.30, V::UnitY())), g(Eigen::AngleAxisd(0.32, V::UnitY()));
	const V ft(0.5, 0, 0.2), gt(0.53, 0, 0.21);
	for (int k = 0; k < 200; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		rig.watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		if (k < 6)
			rig.watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k < 140)   // 10.3 s: into F, its pose jumping
			rig.watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, f, ft), TickScale, false);
		else                // 17.0 s: F to G, its pose jumping again
			rig.watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump + AnchorJump, g, gt), TickScale, false);
		if (k >= 120)       // device 10 on in F at 16.0 s, a solve behind into G at 17.2 s
			rig.watch.Note(ReplaySample(10, t, AnchorAt10, k >= 144 ? g : f, k >= 144 ? gt : ft), TickScale, false);
		if (k >= 110 && k % 5 == 0)   // from 15.5 s
			rig.Tick(t);
	}
	// G's correction: F's, the identity device 9 carried in, across F to G.
	const Q delta = (g * f.conjugate()).normalized();
	const auto right = BackToStart(delta, gt - delta * ft);
	const double off9 = CorrectionShift(rig.frames.Snapshot()[9], right, AnchorAt9 + AnchorJump + AnchorJump);
	const double off10 = CorrectionShift(rig.frames.Snapshot()[10], right, AnchorAt10);
	char detail[900];
	std::snprintf(detail, sizeof detail, "%s; off F's correction across F to G: device 9 %.1f mm, device 10 %.1f mm",
		rig.log.c_str(), off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: a device whose frame change is still being read keeps its correction until it settles",
		rig.frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// Device 10, restored with its own correction, is in E0 from the start, but
// its first-seen switch was dropped (it was not known to be a target device in
// time). It still defines E0, as the first device there: device 9, restored
// with another and switched on later, takes device 10's.
void AnchoredDroppedSwitchDefinesFrame(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	questcal::TrackerFrameCorrections::Frames saved{};
	const Q n(Eigen::AngleAxisd(0.03, V::UnitY()));
	saved[10].rotation = { n.w(), n.x(), n.y(), n.z() };
	saved[10].translation = { { 0.02, 0.0, 0.01 } };
	frames.Restore(saved, questcal::FrameSerialKeys{});
	for (int k = 0; k < 80; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE0, AnchorT0), TickScale, false);
		if (k >= 40)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		if (k % 5 != 0)
			continue;
		auto switches = watch.TakeSwitches();
		switches.erase(std::remove_if(switches.begin(), switches.end(),
			[](const LighthouseFrameWatch::Switch &s) { return s.id == 10; }), switches.end());
		frames.Update(watch, watch.TakeMoves(), switches, vr::k_unTrackedDeviceIndexInvalid,
			[](uint32_t id) { return id == 9 || id == 10; }, [](double) { return false; });
	}
	const bool defined = frames.Snapshot()[10] == saved[10] && frames.Snapshot()[9] == saved[10];
	check("frame anchors: a device whose first switch was dropped still defines its frame for the next one", frames.Unsettled() == 0 && defined, "");
}

// A restored session: device 10's correction, restored, is not the identity,
// and the base station shows the frame it is in. The station's frame is not
// anchored with the identity, which is right only in a session that started
// fresh, and device 10 keeps the correction it was restored with.
void AnchoredRestoredStationNotAnchored(Check check)
{
	MonitorRig rig;
	questcal::TrackerFrameCorrections::Frames saved{};
	const Q n(Eigen::AngleAxisd(0.03, V::UnitY()));
	saved[10].rotation = { n.w(), n.x(), n.y(), n.z() };
	saved[10].translation = { { 0.02, 0.0, 0.01 } };
	rig.frames.Restore(saved, questcal::FrameSerialKeys{});
	for (int k = 0; k < 40; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		rig.watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		rig.watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE0, AnchorT0), TickScale, false);
		if (k % 5 == 0)
			rig.Tick(t);
	}
	check("frame anchors: a restored session does not anchor a base station's frame with the identity",
		rig.frames.Unsettled() == 0 && rig.frames.Snapshot()[10] == saved[10], rig.log.c_str());
}

// The correction a device in `from` carries across the move to `to`.
protocol::FrameCorrection AnchoredAcross(protocol::FrameCorrection c, const Q &fromR, const V &fromT, const Q &toR,
	const V &toT)
{
	const Q r = (toR * fromR.conjugate()).normalized();
	questcal::TrackerFrameCorrections::Compose(c, r, toT - r * fromT);
	return c;
}

bool AnchoredTargets(uint32_t id) { return id == 9 || id == 10; }
bool AnchoredNoSetup(double) { return false; }

// A fresh session, the base station in E0. SteamVR moves the frame E0 to E1
// (10.3 s) and E1 to E2 (11.0 s); the station, a solve behind, shows E0 to E2
// in one step. Device 10, switched on in E1 short of E0 to E1, is measured and
// recalibrated (Anchor) while it still shows E1, then shows E2: its
// recalibrated correction carried across E1 to E2 is E2's. The base station's
// start, offered again on the next update at the recalibration's generation,
// reached E2 first along E0 to E2 and refused it there (43.1 mm).
void AnchoredStationAtStartGeneration(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	protocol::FrameCorrection atRecalibration;
	for (int k = 0; k < 80; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), k >= 21 ? AnchorE2 : AnchorE0, k >= 21 ? AnchorT2 : AnchorT0), TickScale,
			true);
		if (k >= 9)
			watch.Note(ReplaySample(10, t, AnchorAt10, k >= 24 ? AnchorE2 : AnchorE1, k >= 24 ? AnchorT2 : AnchorT1),
				TickScale, false);
		if (k == 22)
		{
			atRecalibration = frames.Snapshot()[10];
			frames.Anchor(10, watch, AnchoredTargets, AnchoredNoSetup);
			held = vr::k_unTrackedDeviceIndexInvalid;
		}
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
		if (k == 10)
			held = 10;
	}
	const auto expected = AnchoredAcross(atRecalibration, AnchorE1, AnchorT1, AnchorE2, AnchorT2);
	const double off10 = CorrectionShift(frames.Snapshot()[10], expected, AnchorAt10);
	char detail[160];
	std::snprintf(detail, sizeof detail, "device 10 %.1f mm off its recalibration carried into E2; disagreements %zu",
		off10 * 1000.0, frames.Disagreements());
	check("frame anchors: a base station's start does not compete with a later recalibration",
		frames.Unsettled() == 0 && off10 < 1e-6 && frames.Disagreements() == 0, detail);
}

// Then device 9, in a station seen only through its trackers, is recalibrated
// too: the base station's start, offered at the second recalibration's
// generation, replaced the first one's derivation in E2, and device 10 lost its
// realignment (43.1 mm).
void AnchoredLaterRecalibrationKeepsEarlier(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3);
	protocol::FrameCorrection afterFirst;
	for (int k = 0; k < 80; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), k >= 24 ? AnchorE2 : AnchorE0, k >= 24 ? AnchorT2 : AnchorT0), TickScale,
			true);
		if (k >= 9)
			watch.Note(ReplaySample(10, t, AnchorAt10, k >= 21 ? AnchorE2 : AnchorE1, k >= 21 ? AnchorT2 : AnchorT1),
				TickScale, false);
		watch.Note(ReplaySample(9, t, AnchorAt9, g0, u0), TickScale, false);
		if (k == 18)
		{
			frames.Anchor(10, watch, AnchoredTargets, AnchoredNoSetup);
			held = vr::k_unTrackedDeviceIndexInvalid;
		}
		if (k == 40)
		{
			afterFirst = frames.Snapshot()[10];
			frames.Anchor(9, watch, AnchoredTargets, AnchoredNoSetup);
		}
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
		if (k == 10)
			held = 10;
	}
	const double moved = CorrectionShift(frames.Snapshot()[10], afterFirst, AnchorAt10);
	char detail[160];
	std::snprintf(detail, sizeof detail, "device 10 moved %.1f mm off its first recalibration's correction; disagreements %zu",
		moved * 1000.0, frames.Disagreements());
	check("frame anchors: a recalibration elsewhere leaves an earlier one's realignment",
		frames.Unsettled() == 0 && moved < 1e-6 && frames.Disagreements() == 0, detail);
}

// Device 9, measured, is re-expressed from E0 into G0 (a station seen only
// through its trackers), its world pose kept, so E0's correction is right
// there; SteamVR re-solves G0 to G1, which it follows. Device 10 is switched
// on in G1 with its own, short of the re-solve. The run fails: device 9 offers
// its correction as derived along what it showed, and G1 takes it. Taking G1's
// entry with nothing offered, device 9 ended 9.1 mm off, device 10 34.9 mm.
void AnchoredHeldDeviceOffersAtRelease(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	const V at9InG = g0.conjugate() * (AnchorAt9 - u0);
	for (int k = 0; k < 60; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		if (k < 4)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else
		{
			auto s9 = ReplaySample(9, t, at9InG, k >= 10 ? g1 : g0, k >= 10 ? u1 : u0);
			s9.rotation = questcal::WireQuaternion(g0.conjugate());   // its world rotation kept too
			watch.Note(s9, TickScale, false);
		}
		if (k >= 8)
			watch.Note(ReplaySample(10, t, AnchorAt10, g1, u1), TickScale, false);
		if (k == 20)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
		if (k == 2)
			held = 9;
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, g0, u0, g1, u1);
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, at9InG);
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, AnchorAt10);
	char detail[160];
	std::snprintf(detail, sizeof detail, "after the run: device 9 %.1f mm, device 10 %.1f mm", off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: a measured device whose run fails offers its correction as derived along what it showed",
		frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// Device 9 takes E1's anchored entry, then starts a new solution from a station
// seen only through its trackers (G0, never re-solved), its pose jumping: G0's
// entry is an unclear one with E1's correction. Device 10, switched on in G0
// with its own, right there, offers it as the older root and G0 takes it. It
// offered nothing into a frame with an entry, and took the unclear one
// (43.1 mm).
void AnchoredFirstSampleOverUnclear(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3);
	for (int k = 0; k < 200; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), k >= 6 ? AnchorE1 : AnchorE0, k >= 6 ? AnchorT1 : AnchorT0), TickScale,
			true);
		if (k >= 10 && k < 20)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		else if (k >= 20)
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, g0, u0), TickScale, false);
		if (k >= 140)
			watch.Note(ReplaySample(10, t, AnchorAt10, g0, u0), TickScale, false);
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), vr::k_unTrackedDeviceIndexInvalid,
				AnchoredTargets, AnchoredNoSetup);
	}
	const double off10 = CorrectionShift(frames.Snapshot()[10], protocol::FrameCorrection{}, AnchorAt10);
	const bool one = frames.Snapshot()[9] == frames.Snapshot()[10];
	char detail[160];
	std::snprintf(detail, sizeof detail, "device 10 %.1f mm off its own; one correction in G0 %d", off10 * 1000.0, one ? 1 : 0);
	check("frame anchors: a device first seen in a frame an unclear change defined offers its older root",
		frames.Unsettled() == 0 && off10 < 1e-6 && one, detail);
}

// Device 9, alone in a station seen only through its trackers, is measured
// from before the monitor's first tick, so its first-seen switch is skipped.
// SteamVR re-solves G0 to G1, which it shows, and the run ends before the tick
// that takes the move. Its release defines G0, and G1 is derived from it. With
// nothing defining G0, it carried its own into G1, short of the move (13.2 mm).
void AnchoredMoveFromEmptyFrame(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = 9;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	for (int k = 0; k < 60; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		watch.Note(ReplaySample(9, t, AnchorAt9, k >= 21 ? g1 : g0, k >= 21 ? u1 : u0), TickScale, false);
		if (k == 22)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run ends
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, g0, u0, g1, u1);
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, AnchorAt9);
	char detail[96];
	std::snprintf(detail, sizeof detail, "device 9 %.1f mm off its correction carried across G0 to G1", off9 * 1000.0);
	check("frame anchors: a measured device's move out of a frame nothing defined is carried once the run ends",
		frames.Unsettled() == 0 && off9 < 1e-6, detail);
}

// The same station, device 9 not measured, its first-seen switch dropped (not
// known to be a target device in time), and its move G0 to G1 in the first
// drain it is a target device: no entry derives G1, so the move carries its own
// correction there, rooted in G0. Assigned as it is, it carried its own into
// G1, short of the move.
void AnchoredMoveAfterDroppedSwitch(Check check)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	std::vector<LighthouseFrameWatch::Move> moves;
	for (int k = 0; k < 40; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(9, t, AnchorAt9, k >= 10 ? g1 : g0, k >= 10 ? u1 : u0), TickScale, false);
		for (const auto &m : watch.TakeMoves())
			moves.push_back(m);
		watch.TakeSwitches();   // dropped
		if (k == 20)
		{
			frames.Update(watch, moves, {}, vr::k_unTrackedDeviceIndexInvalid, AnchoredTargets, AnchoredNoSetup);
			moves.clear();
		}
		else if (k > 20 && k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), {}, vr::k_unTrackedDeviceIndexInvalid, AnchoredTargets, AnchoredNoSetup);
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, g0, u0, g1, u1);
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, AnchorAt9);
	char detail[96];
	std::snprintf(detail, sizeof detail, "device 9 %.1f mm off its correction carried across G0 to G1", off9 * 1000.0);
	check("frame anchors: a move out of a frame no switch defined carries the device's correction across it",
		frames.Unsettled() == 0 && off9 < 1e-6, detail);
}

// Device 10, in a station seen only through its trackers (H), carries its own
// in, and follows that station's re-solve to G: G's entry is derived from its
// guess. Device 9 is then re-expressed from the base station's anchored E0 into
// G, its world pose kept: SteamVR places G as it places E0, so E0's correction
// is right there, and the re-expression anchors G with it over the derived
// guess. Offered unanchored, as a first sample would be, it gave way to that
// guess, the older root by the record.
void AnchoredReexpressionAnchorsOverCarried(Check check)
{
	MonitorRig rig;
	const Q h(Eigen::AngleAxisd(0.30, V::UnitY())), g(Eigen::AngleAxisd(0.32, V::UnitY()));
	const V ht(0.5, 0, 0.2), gt(0.53, 0, 0.21);
	const V at9InG = g.conjugate() * (AnchorAt9 - gt);
	for (int k = 0; k < 60; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		rig.watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		rig.watch.Note(ReplaySample(10, t, AnchorAt10, k >= 10 ? g : h, k >= 10 ? gt : ht), TickScale, false);
		auto s9 = k >= 20 ? ReplaySample(9, t, at9InG, g, gt) : ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0);
		if (k >= 20)
			s9.rotation = questcal::WireQuaternion(g.conjugate());   // its world rotation kept too
		rig.watch.Note(s9, TickScale, false);
		if (k % 5 == 0)
			rig.Tick(t);
	}
	const bool anchored = rig.frames.Snapshot()[9] == protocol::FrameCorrection{} &&
		rig.frames.Snapshot()[10] == protocol::FrameCorrection{};
	check("frame anchors: a re-expression from an anchored frame anchors the frame over a derived guess",
		rig.frames.Unsettled() == 0 && anchored, rig.log.c_str());
}

// Device 9 takes the base station's anchored E0 and is measured from the next
// tick, still in E0; it is then re-expressed into G0 (a station seen only
// through its trackers) and follows G0 to G1; device 10 is switched on in G1
// short of the re-solve, and the run fails. Device 9's correction is derived
// from E0's anchored one, and G1 takes it. Rooted as carried into E0, which the
// record does not order against device 10's G1, it gave way. `oneDrain`: the
// re-expression and the move come in one drain, and the correction is for the
// frame after the move, not the one before it.
void AnchoredHeldDerivationFromStart(Check check, bool oneDrain)
{
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	const V at9InG = g0.conjugate() * (AnchorAt9 - u0);
	for (int k = 0; k < 60; ++k)
	{
		if (k == 1)
			held = 9;   // measured from the tick at 10.1 s
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		if (k < 4)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else
		{
			auto s9 = ReplaySample(9, t, at9InG, k >= 10 ? g1 : g0, k >= 10 ? u1 : u0);
			s9.rotation = questcal::WireQuaternion(g0.conjugate());
			watch.Note(s9, TickScale, false);
		}
		if (k >= 8)
			watch.Note(ReplaySample(10, t, AnchorAt10, g1, u1), TickScale, false);
		if (k == 20)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k == 0 || k == 2 || (k % 5 == 0 && (!oneDrain || k == 15 || k >= 20)))
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, g0, u0, g1, u1);
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, at9InG);
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, AnchorAt10);
	char detail[160];
	std::snprintf(detail, sizeof detail, "after the run: device 9 %.1f mm, device 10 %.1f mm", off9 * 1000.0, off10 * 1000.0);
	check(oneDrain ? "frame anchors: a measured device's switch and move in one drain are followed in the order they came"
		: "frame anchors: a measured device's correction keeps the root of the frame its run began in",
		frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail);
}

// Appends a formatted line to a scenario's detail, the lines joined by "; ".
template <class... Args>
void AnchoredNote(std::string &detail, const char *format, Args... args)
{
	char line[512];
	std::snprintf(line, sizeof line, format, args...);
	std::string text(line);
	while (!text.empty() && text.back() == '\n')
		text.pop_back();
	detail += (detail.empty() ? "" : "; ") + text;
}

// The base station in E0, anchored at the start. Device 10, in a station seen
// only through its trackers, follows its re-solve G0 to G1, then starts a new
// solution from the base station's E0, its pose jumping: a change no move
// leads, still being read when its run starts. Its record is unclear, its
// correction still G1's. Recorded as E0's anchored entry, the release offered
// that correction as E0's, and a disagreement was counted.
void AnchoredCaptureWhileUnsettledAnchored(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	for (int k = 0; k < 60; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		if (k < 10)
			watch.Note(ReplaySample(10, t, AnchorAt10, k >= 4 ? g1 : g0, k >= 4 ? u1 : u0), TickScale, false);
		else
			watch.Note(ReplaySample(10, t, AnchorAt10 + AnchorJump, AnchorE0, AnchorT0), TickScale, false);
		if (k == 20)
			held = 10;   // measured from here, its change still being read
		if (k == 40)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
	}
	AnchoredNote(detail, "run started while its target's change was read, into an anchored frame: disagreements counted %zu\n",
		frames.Disagreements());
	check("frame anchors: a run that starts while its target's change is being read derives nothing from the frame it shows",
		frames.Unsettled() == 0 && frames.Disagreements() == 0, detail.c_str());
}

// The same into a frame no entry is in (H0): device 9's run starts while its
// change into H0 is still being read, and fails. Device 10, first seen in H0
// with its own, right there, defines H0. Recorded as carried into H0, device 9's
// G1 correction defined H0 first, and device 10 took it (34.9 mm).
void AnchoredCaptureWhileUnsettledEmpty(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	const Q h0(Eigen::AngleAxisd(0.35, V::UnitY()));
	const V w0(0.6, 0.0, -0.2);
	for (int k = 0; k < 120; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		if (k < 10)
			watch.Note(ReplaySample(9, t, AnchorAt9, k >= 4 ? g1 : g0, k >= 4 ? u1 : u0), TickScale, false);
		else
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, h0, w0), TickScale, false);
		if (k >= 60)
			watch.Note(ReplaySample(10, t, AnchorAt10, h0, w0), TickScale, false);
		if (k == 20)
			held = 9;
		if (k == 40)
			held = vr::k_unTrackedDeviceIndexInvalid;
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
	}
	const double off10 = CorrectionShift(frames.Snapshot()[10], protocol::FrameCorrection{}, AnchorAt10);
	AnchoredNote(detail, "run started while its target's change was read, into a frame with no entry: device 10 %.1f mm off "
		"its own, right in H0\n", off10 * 1000.0);
	check("frame anchors: a run started during a change does not define the new frame for a later device",
		frames.Unsettled() == 0 && off10 < 1e-6, detail.c_str());
}

// Device 9, in the base station's anchored E0, is re-expressed into G0 (a
// station seen only through its trackers) and its run starts before the next
// tick; it follows G0 to G1, and device 10 is switched on in G1 short of the
// re-solve. The tick that first sees it held records the frame its first change
// of the drain left, E0, and replays the re-expression and the move on top: its
// correction, right, is offered from E0's anchored root when the run fails.
// Recorded as carried into G1, where it shows now, it was no older than device
// 10's, and device 9 took that (9.1 mm, device 10 34.9 mm).
void AnchoredCaptureBeforeTheDrainsChanges(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	const V at9InG = g0.conjugate() * (AnchorAt9 - u0);
	for (int k = 0; k < 40; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), AnchorE0, AnchorT0), TickScale, true);
		if (k < 1)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else
		{
			auto s9 = ReplaySample(9, t, at9InG, k >= 3 ? g1 : g0, k >= 3 ? u1 : u0);
			s9.rotation = questcal::WireQuaternion(g0.conjugate());   // its world rotation kept too
			watch.Note(s9, TickScale, false);
		}
		if (k >= 4)
			watch.Note(ReplaySample(10, t, AnchorAt10, g1, u1), TickScale, false);
		if (k == 2)
			held = 9;
		if (k == 10)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, g0, u0, g1, u1);
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, at9InG);
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, AnchorAt10);
	AnchoredNote(detail, "record taken after the drain's own changes: device 9 %.1f mm off, device 10 %.1f mm\n",
		off9 * 1000.0, off10 * 1000.0);
	check("frame anchors: a run's record starts from the frame its first change of the drain left",
		frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6, detail.c_str());
}

// Device 9 takes E1's anchored entry, then starts a new solution from a station
// seen only through its trackers (G0, never re-solved), its pose jumping: G0's
// entry is an unclear one with E1's correction. Device 10 is switched on in G0
// with its own, right there; `dropped`: its first-seen switch is dropped (it was
// not known to be a target device in time). Its first assignment offers its own
// as a first sample does, the older root. Taking G0's entry with nothing
// offered, it took the unclear guess (43.1 mm).
void AnchoredDroppedSwitchOverGuess(Check check, bool dropped)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3);
	for (int k = 0; k < 200; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), k >= 6 ? AnchorE1 : AnchorE0, k >= 6 ? AnchorT1 : AnchorT0), TickScale,
			true);
		if (k >= 10 && k < 20)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		else if (k >= 20)
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, g0, u0), TickScale, false);
		if (k >= 140)
			watch.Note(ReplaySample(10, t, AnchorAt10, g0, u0), TickScale, false);
		if (k % 5 != 0)
			continue;
		auto switches = watch.TakeSwitches();
		if (dropped)
			switches.erase(std::remove_if(switches.begin(), switches.end(),
				[](const LighthouseFrameWatch::Switch &s) { return s.id == 10; }), switches.end());
		frames.Update(watch, watch.TakeMoves(), switches, vr::k_unTrackedDeviceIndexInvalid, AnchoredTargets, AnchoredNoSetup);
	}
	const double off10 = CorrectionShift(frames.Snapshot()[10], protocol::FrameCorrection{}, AnchorAt10);
	AnchoredNote(detail, "first-seen switch %s, in a frame an unclear change defined: device 10 %.1f mm off its own, right in G0\n",
		dropped ? "dropped" : "kept", off10 * 1000.0);
	check(dropped ? "frame anchors: a device whose first switch was dropped offers its own over an unclear change's guess"
		: "frame anchors: a device whose first switch was kept offers its own over an unclear change's guess",
		frames.Unsettled() == 0 && off10 < 1e-6, detail.c_str());
}

// The same, device 10's switch dropped and the device known to be ours from the
// drain that takes device 9's own jump into G0 (43.1 mm before).
void AnchoredDroppedSwitchOverGuessLater(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3);
	bool ours10 = false;
	auto calibrated = [&ours10](uint32_t id) { return id == 9 || (id == 10 && ours10); };
	for (int k = 0; k < 200; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), k >= 6 ? AnchorE1 : AnchorE0, k >= 6 ? AnchorT1 : AnchorT0), TickScale,
			true);
		if (k >= 10 && k < 20)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		else if (k >= 20)
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, g0, u0), TickScale, false);
		if (k >= 90)
			watch.Note(ReplaySample(10, t, AnchorAt10, g0, u0), TickScale, false);
		if (k % 5 != 0)
			continue;
		auto switches = watch.TakeSwitches();
		switches.erase(std::remove_if(switches.begin(), switches.end(),
			[](const LighthouseFrameWatch::Switch &s) { return s.id == 10; }), switches.end());
		bool jump9 = false;
		for (const auto &s : switches)
			jump9 = jump9 || s.id == 9;
		if (jump9 && k >= 90)
			ours10 = true;
		frames.Update(watch, watch.TakeMoves(), switches, vr::k_unTrackedDeviceIndexInvalid, calibrated, AnchoredNoSetup);
	}
	const double off10 = CorrectionShift(frames.Snapshot()[10], protocol::FrameCorrection{}, AnchorAt10);
	AnchoredNote(detail, "first-seen switch dropped, known to be ours later: device 10 %.1f mm off its own, right in G0\n",
		off10 * 1000.0);
	check("frame anchors: a device known to be ours late offers its own over an unclear change's guess",
		frames.Unsettled() == 0 && off10 < 1e-6, detail.c_str());
}

// Device 9 takes the base station's anchored E0 and is measured. SteamVR
// re-solves E0 to E1 with its own pose jumping, a change still being read, then
// E1 to E2, which it shows as a move and follows; the base station is a solve
// behind. The run fails before the change settles: the release offers its
// correction, short of E0 to E1, as unclear. Offered as E0's anchored entry
// carried into E2, it anchored E2 wrong: device 10, switched on there, ended
// 43.1 mm off, and the base station's E0 to E2 counted a disagreement.
void AnchoredReleaseDuringChange(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	bool anchoredAtRelease = false;
	double offAtRelease = 0.0;
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, AnchorE0, AnchorT0, AnchorE2, AnchorT2);
	for (int k = 0; k < 120; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), k >= 60 ? AnchorE2 : AnchorE0, k >= 60 ? AnchorT2 : AnchorT0), TickScale,
			true);
		if (k < 10)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k < 14)
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, AnchorE1, AnchorT1), TickScale, false);
		else
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, AnchorE2, AnchorT2), TickScale, false);
		if (k >= 40)
			watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE2, AnchorT2), TickScale, false);
		if (k == 5)
			held = 9;
		if (k == 30)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k % 5 == 0)
		{
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
			if (k == 30)
			{
				for (const auto &e : frames.Table())
					if (questcal::TrackerFrameCorrections::Agree(e.correction, frames.Snapshot()[9]) && e.anchored)
						anchoredAtRelease = true;
				offAtRelease = CorrectionShift(frames.Snapshot()[9], right, AnchorAt9 + AnchorJump);
				AnchoredNote(detail, "at the release (11.5 s): device 9 settled %d, %.1f mm off E0's carried into E2; an anchored "
					"entry with its correction %d; table %zu entries\n", watch.Settled(9) ? 1 : 0, offAtRelease * 1000.0,
					anchoredAtRelease ? 1 : 0, frames.Table().size());
			}
		}
	}
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, AnchorAt10);
	AnchoredNote(detail, "at 16.0 s: device 10 %.1f mm off E0's correction carried into E2; disagreements %zu\n", off10 * 1000.0,
		frames.Disagreements());
	check("frame anchors: a run released during a change of its frame anchors nothing",
		frames.Unsettled() == 0 && off10 < 1e-6 && frames.Disagreements() == 0 && !anchoredAtRelease, detail.c_str());
}

// The same, the base station showing E0 to E1 in time for the drain that first
// sees the run ended: the release follows that drain's lead first, and offers
// the correction across both re-solves. Released before that drain's moves, it
// offered the correction short of E0 to E1 (18.0 mm, device 10 43.1 mm).
void AnchoredReleaseWithItsLead(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, AnchorE0, AnchorT0, AnchorE2, AnchorT2);
	for (int k = 0; k < 120; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const Q &e = k >= 60 ? AnchorE2 : k >= 28 ? AnchorE1 : AnchorE0;
		const V &u = k >= 60 ? AnchorT2 : k >= 28 ? AnchorT1 : AnchorT0;
		watch.Note(ReplaySample(1, t, V::Zero(), e, u), TickScale, true);
		if (k < 10)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k < 14)
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, AnchorE1, AnchorT1), TickScale, false);
		else
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, AnchorE2, AnchorT2), TickScale, false);
		if (k >= 40)
			watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE2, AnchorT2), TickScale, false);
		if (k == 5)
			held = 9;
		if (k == 30)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k % 5 == 0)
		{
			const auto moves = watch.TakeMoves();
			size_t led = 0;
			for (const auto &m : moves)
				led += m.id == 9;
			frames.Update(watch, moves, watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
			if (k == 30)
			{
				bool anchored = false;
				for (const auto &x : frames.Table())
					anchored = anchored || (x.anchored && questcal::TrackerFrameCorrections::Agree(x.correction, frames.Snapshot()[9]));
				AnchoredNote(detail, "at the release (11.5 s): device 9 settled %d, moves of it in that drain %zu, %.1f mm off "
					"E0's carried into E2; an anchored entry with its correction %d\n", watch.Settled(9) ? 1 : 0, led,
					CorrectionShift(frames.Snapshot()[9], right, AnchorAt9 + AnchorJump) * 1000.0, anchored ? 1 : 0);
			}
		}
	}
	const double off9 = CorrectionShift(frames.Snapshot()[9], right, AnchorAt9 + AnchorJump);
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, AnchorAt10);
	AnchoredNote(detail, "at 16.0 s: device 9 %.1f mm, device 10 %.1f mm off E0's correction carried into E2; disagreements %zu\n",
		off9 * 1000.0, off10 * 1000.0, frames.Disagreements());
	check("frame anchors: a run released in the drain that brings its change's lead follows that lead first",
		frames.Unsettled() == 0 && off9 < 1e-6 && off10 < 1e-6 && frames.Disagreements() == 0, detail.c_str());
}

// Device 9 takes E1's anchored entry, then starts a new solution from a station
// seen only through its trackers (G0), its pose jumping, a change still being
// read. Device 10 is switched on in G0 with its own, right there, and SteamVR
// re-solves G0 to G1, which both show before the next tick. Device 9's move out
// of G0, which has no entry yet, offers its correction as unclear: it is E1's,
// not G0's. Offered as G0's carried across the move, it defined G1 first, and
// device 10's right derivation, with the same root, was refused (43.2 mm).
void AnchoredMoveDuringChange(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	const Q g0(Eigen::AngleAxisd(-0.25, V::UnitY())), g1(Eigen::AngleAxisd(-0.23, V::UnitY()));
	const V u0(-0.4, 0.0, 0.3), u1(-0.37, 0.0, 0.31);
	size_t moves9 = 0;
	for (int k = 0; k < 140; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		watch.Note(ReplaySample(1, t, V::Zero(), k >= 6 ? AnchorE1 : AnchorE0, k >= 6 ? AnchorT1 : AnchorT0), TickScale,
			true);
		if (k >= 10 && k < 20)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		else if (k >= 20)
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, k >= 42 ? g1 : g0, k >= 42 ? u1 : u0), TickScale, false);
		if (k >= 41)
			watch.Note(ReplaySample(10, t, AnchorAt10, k >= 43 ? g1 : g0, k >= 43 ? u1 : u0), TickScale, false);
		if (k % 5 == 0)
		{
			auto moves = watch.TakeMoves();
			for (const auto &m : moves)
				moves9 += m.id == 9;
			if (k == 45)
				AnchoredNote(detail, "tick at 12.25 s: device 9 settled %d, its moves handed out %zu\n", watch.Settled(9) ? 1 : 0,
					moves9);
			frames.Update(watch, moves, watch.TakeSwitches(), vr::k_unTrackedDeviceIndexInvalid, AnchoredTargets, AnchoredNoSetup);
		}
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, g0, u0, g1, u1);
	const double off10 = CorrectionShift(frames.Snapshot()[10], right, AnchorAt10);
	AnchoredNote(detail, "move of a device whose change is still being read, from a frame with no entry: device 10 %.1f mm off "
		"its own carried into G1\n", off10 * 1000.0);
	check("frame anchors: a move of a device whose change is still being read derives nothing",
		frames.Unsettled() == 0 && off10 < 1e-6, detail.c_str());
}

// The base station in E0, anchored at the start. SteamVR re-solves E0 to E1,
// which device 9 shows first, and E1 to E2, which it shows with its own pose
// jumping, a change still being read at the next tick: device 9 takes no entry
// there and keeps E0's correction. The base station leads the change; device 9
// is measured from before the next tick, which records the frame its first
// move of the drain (the lead's) left, E1, and the run fails. E1's anchored
// entry is not the one its correction holds, so the record's root is unclear.
// Taken as E1's anchored entry, the release offered E0's correction carried
// across E1 to E2 only (18.0 mm), and a disagreement was counted.
void AnchoredCaptureOfAnEntryNotTaken(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	protocol::FrameCorrection atRelease;
	for (int k = 0; k < 60; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const Q &e = k >= 12 ? AnchorE2 : k >= 7 ? AnchorE1 : AnchorE0;
		const V &u = k >= 12 ? AnchorT2 : k >= 7 ? AnchorT1 : AnchorT0;
		watch.Note(ReplaySample(1, t, V::Zero(), e, u), TickScale, true);
		if (k < 6)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k < 8)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		else
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, AnchorE2, AnchorT2), TickScale, false);
		if (k == 13)
			held = 9;
		if (k == 18)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k % 5 == 0)
		{
			if (k == 20)
				atRelease = frames.Snapshot()[9];
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
			if (k == 10)
				AnchoredNote(detail, "tick at 10.5 s: device 9 settled %d, %.1f mm off E1's entry\n", watch.Settled(9) ? 1 : 0,
					CorrectionShift(frames.Snapshot()[9], AnchoredAcross(protocol::FrameCorrection{}, AnchorE0, AnchorT0,
						AnchorE1, AnchorT1), AnchorAt9) * 1000.0);
		}
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, AnchorE0, AnchorT0, AnchorE2, AnchorT2);
	AnchoredNote(detail, "record of an entry the device had not taken: at the release device 9 %.1f mm off E2's, "
		"disagreements %zu; at 13.0 s device 9 %.1f mm off\n", CorrectionShift(atRelease, right, AnchorAt9 + AnchorJump) * 1000.0,
		frames.Disagreements(), CorrectionShift(frames.Snapshot()[9], right, AnchorAt9 + AnchorJump) * 1000.0);
	check("frame anchors: a run's record takes no root from an entry its device has not taken",
		frames.Unsettled() == 0 && frames.Disagreements() == 0 &&
		CorrectionShift(frames.Snapshot()[9], right, AnchorAt9 + AnchorJump) < 1e-6, detail.c_str());
}

// The same, SteamVR re-solving E2 to E3, which device 9 shows before the tick
// that releases it: the release follows that move and offers its correction to
// E3. Rooted as E1's anchored entry, it anchored E3 before the record's
// derivation reached it, and device 10, switched on there, took it: device 9
// 17.8 mm off, device 10 42.9 mm, a disagreement counted.
void AnchoredCaptureOfAnEntryNotTakenAnchors(Check check)
{
	std::string detail;
	LighthouseFrameWatch watch;
	questcal::TrackerFrameCorrections frames;
	uint32_t held = vr::k_unTrackedDeviceIndexInvalid;
	for (int k = 0; k < 80; ++k)
	{
		const double t = 10.0 + 0.05 * k;
		const Q &e = k >= 19 ? AnchorE3 : k >= 12 ? AnchorE2 : k >= 7 ? AnchorE1 : AnchorE0;
		const V &u = k >= 19 ? AnchorT3 : k >= 12 ? AnchorT2 : k >= 7 ? AnchorT1 : AnchorT0;
		watch.Note(ReplaySample(1, t, V::Zero(), e, u), TickScale, true);
		if (k < 6)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE0, AnchorT0), TickScale, false);
		else if (k < 8)
			watch.Note(ReplaySample(9, t, AnchorAt9, AnchorE1, AnchorT1), TickScale, false);
		else if (k < 16)
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, AnchorE2, AnchorT2), TickScale, false);
		else
			watch.Note(ReplaySample(9, t, AnchorAt9 + AnchorJump, AnchorE3, AnchorT3), TickScale, false);
		if (k >= 30)
			watch.Note(ReplaySample(10, t, AnchorAt10, AnchorE3, AnchorT3), TickScale, false);
		if (k == 13)
			held = 9;
		if (k == 18)
			held = vr::k_unTrackedDeviceIndexInvalid;   // the run fails
		if (k % 5 == 0)
			frames.Update(watch, watch.TakeMoves(), watch.TakeSwitches(), held, AnchoredTargets, AnchoredNoSetup);
	}
	const auto right = AnchoredAcross(protocol::FrameCorrection{}, AnchorE0, AnchorT0, AnchorE3, AnchorT3);
	bool anchoredWrong = false;
	for (const auto &x : frames.Table())
		if (!questcal::WorldFromDriverChanged(x.rot, x.trans, AnchorE3, AnchorT3))
			anchoredWrong = anchoredWrong || (x.anchored && CorrectionShift(x.correction, right, AnchorAt10) > 1e-3);
	AnchoredNote(detail, "the release reaching a frame before the record: at 14.0 s device 9 %.1f mm, device 10 %.1f mm off "
		"E0's carried into E3; E3 anchored wrong %d; disagreements %zu\n",
		CorrectionShift(frames.Snapshot()[9], right, AnchorAt9 + AnchorJump) * 1000.0,
		CorrectionShift(frames.Snapshot()[10], right, AnchorAt10) * 1000.0, anchoredWrong ? 1 : 0, frames.Disagreements());
	check("frame anchors: a release from such a record anchors nothing ahead of the record's derivation",
		frames.Unsettled() == 0 && frames.Disagreements() == 0 && !anchoredWrong &&
		CorrectionShift(frames.Snapshot()[9], right, AnchorAt9 + AnchorJump) < 1e-6 &&
		CorrectionShift(frames.Snapshot()[10], right, AnchorAt10) < 1e-6, detail.c_str());
}
} // namespace

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
	KnockedStationReplacement(check);
	LeadByAnyChain(check);
	FrameJoinedOnReturn(check);
	ChangesInARow(check);
	MoveKeptAcrossStreamRestart(check);
	ChainRecordedOutOfOrder(check);
	MergeAcrossFollowedMove(check);
	MergedIntoUnleadable(check);
	JoinBeforeMoveShown(check);
	JoinInStaleOwnFrame(check);
	JoinAtLagBound(check);
	JoinWaitsForDeviceOffThroughMove(check);
	JoinAfterDeviceOffThroughMove(check);
	JoinWaitsForDeviceThatFollowedIn(check);
	NewestSwitchStandsForOlder(check);
	AnchoredBothMovedOn(check, false);
	AnchoredBothMovedOn(check, true);
	AnchoredSkippedFrame(check);
	AnchoredLeadAfterNewerMove(check);
	AnchoredSwitchedOnAfterMove(check);
	AnchoredUnseenResolve(check);
	AnchoredReexpressedIntoCarried(check);
	AnchoredSettledApart(check);
	AnchoredSwitchesInOneDrain(check);
	AnchoredMeasuredDeviceHeld(check);
	AnchoredJumpCarriesFrame(check);
	AnchoredUnsettledWaits(check);
	AnchoredDroppedSwitchDefinesFrame(check);
	AnchoredRestoredStationNotAnchored(check);
	AnchoredStationAtStartGeneration(check);
	AnchoredLaterRecalibrationKeepsEarlier(check);
	AnchoredHeldDeviceOffersAtRelease(check);
	AnchoredFirstSampleOverUnclear(check);
	AnchoredMoveFromEmptyFrame(check);
	AnchoredMoveAfterDroppedSwitch(check);
	AnchoredReexpressionAnchorsOverCarried(check);
	AnchoredHeldDerivationFromStart(check, false);
	AnchoredHeldDerivationFromStart(check, true);
	AnchoredCaptureWhileUnsettledAnchored(check);
	AnchoredCaptureWhileUnsettledEmpty(check);
	AnchoredCaptureBeforeTheDrainsChanges(check);
	AnchoredDroppedSwitchOverGuess(check, true);
	AnchoredDroppedSwitchOverGuess(check, false);
	AnchoredDroppedSwitchOverGuessLater(check);
	AnchoredReleaseDuringChange(check);
	AnchoredReleaseWithItsLead(check);
	AnchoredMoveDuringChange(check);
	AnchoredCaptureOfAnEntryNotTaken(check);
	AnchoredCaptureOfAnEntryNotTakenAnchors(check);
}
