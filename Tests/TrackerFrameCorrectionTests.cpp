#include "../Driver/ServerTrackedDeviceProvider.h"
#include "../Driver/ProtocolValidation.h"
#include "../Driver/PoseTransform.h"
#include "../Driver/Logging.h"
#include "../Overlay/TrackerFrameCorrections.h"
#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/DriverSession.h"
#include "../Overlay/CalibrationRun.h"
#include "../Overlay/CollectionSource.h"
#include "../common/MathConstants.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <string>
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
		std::sort(handed.begin(), handed.end(), [](const auto &a, const auto &b) { return a.time < b.time; });
		for (const auto &move : handed)
			moves += frames.Follow(move) ? 1 : 0;
		for (const auto &change : watch.TakeSwitches())
		{
			switches.push_back(change);
			if (!join)
				continue;
			const auto result = frames.Join(change, watch, headset, keep,
				[this](uint32_t id) { return devices.count(id) > 0; });
			joined.insert(joined.end(), result.joined.begin(), result.joined.end());
			if (result.kept)
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
	DeferredFrameLog(check);
	CollectionFallback(check);
}
