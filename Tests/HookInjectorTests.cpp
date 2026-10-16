#include "../Driver/HookChain.h"
#include "../Driver/HookLifecyclePolicy.h"
#include "../Driver/InterfaceHookInjector.h"
#include "../Driver/Logging.h"
#include "../Driver/OpenVRHookLayout.h"
#include "../Driver/ServerTrackedDeviceProvider.h"
#include "../Overlay/HookCoverage.h"
#include "../common/Protocol.h"

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <initializer_list>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// The real detours, installed with MinHook on stand-ins for vrserver's driver
// context and host. Teardown disables the hooks without removing them, so each
// scenario's Init enables the same hooks again, as a second Init in vrserver
// would.
namespace
{
using Check = void (*)(const char *, bool, const char *);

std::atomic<int> ContextCalls{ 0 };
std::atomic<int> HostPoseCalls{ 0 };
std::atomic<double> HostReceivedOriginX{ 0.0 };
// Set, the host's next pose update calls the hooked slot again for the same
// device from inside itself, as a host version forwarding to another would;
// what that inner call receives lands in InnerOriginX.
std::atomic<bool> ReenterOnce{ false };
std::atomic<double> InnerOriginX{ 0.0 };
thread_local int HostDepth = 0;
// The driver-space position x of the last pose an outer host call received,
// and how many calls each low device id received.
std::atomic<double> HostReceivedPositionX{ 0.0 };
std::atomic<int> HostCallsByDevice[8] = {};
// Run once from inside the host's next pose update, as another thread's pose
// can arrive while the host handles one.
std::atomic<void (*)()> InsideHostOnce{ nullptr };

using PoseUpdateFn = void (*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t);

// Distinct bodies, so identical-code folding cannot merge a hooked target with
// an unrelated function.
struct FakeHost
{
	virtual __declspec(noinline) void Unused(uint32_t device)
	{
		HostPoseCalls.fetch_sub(static_cast<int>(device) + 7);
	}
	virtual __declspec(noinline) void TrackedDevicePoseUpdated(uint32_t device,
		const vr::DriverPose_t &pose, uint32_t size)
	{
		HostPoseCalls.fetch_add(static_cast<int>(device + size) | 1);
		if (device < 8)
			HostCallsByDevice[device].fetch_add(1);
		if (HostDepth == 0)
			HostReceivedPositionX.store(pose.vecPosition[0]);
		(HostDepth > 0 ? InnerOriginX : HostReceivedOriginX).store(pose.vecWorldFromDriverTranslation[0]);
		if (ReenterOnce.exchange(false))
		{
			++HostDepth;
			void **vtable = *reinterpret_cast<void ***>(this);
			reinterpret_cast<PoseUpdateFn>(vtable[openvr_hook::PoseUpdateSlot])(this, device, pose, size);
			--HostDepth;
		}
		if (void (*during)() = InsideHostOnce.exchange(nullptr))
			during();
	}
};

struct FakeContext
{
	virtual __declspec(noinline) void *GetGenericInterface(const char *version,
		vr::EVRInitError *error);
};

FakeHost Host;
FakeContext Context;

void *FakeContext::GetGenericInterface(const char *version, vr::EVRInitError *error)
{
	ContextCalls.fetch_add(1);
	if (error)
		*error = vr::VRInitError_None;
	return std::strcmp(version, "IVRServerDriverHost_006") == 0 ? &Host : nullptr;
}

// Through the vtable slot, as InitServerDriverContext and other drivers do; a
// direct call could be devirtualised past the patched function.
void RequestHostInterface()
{
	using Fn = void *(*)(void *, const char *, vr::EVRInitError *);
	void **vtable = *reinterpret_cast<void ***>(&Context);
	auto fn = reinterpret_cast<Fn>(vtable[openvr_hook::GetGenericInterfaceSlot]);
	vr::EVRInitError error = vr::VRInitError_None;
	fn(&Context, "IVRServerDriverHost_006", &error);
}

// A tracking pose through the host's vtable slot, as a device driver sends it;
// returns the world-from-driver origin the host received. `size` is the
// DriverPose_t size the caller claims; `worldRotation`, when given, replaces
// the identity world-from-driver rotation.
double SendPoseThroughHost(uint32_t device = 0, uint32_t size = sizeof(vr::DriverPose_t),
	double positionX = 0.0, const vr::HmdQuaternion_t *worldRotation = nullptr)
{
	vr::DriverPose_t pose{};
	pose.vecPosition[0] = positionX;
	pose.poseIsValid = pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.qRotation.w = pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = 1;
	if (worldRotation)
		pose.qWorldFromDriverRotation = *worldRotation;
	void **vtable = *reinterpret_cast<void ***>(&Host);
	auto fn = reinterpret_cast<PoseUpdateFn>(vtable[openvr_hook::PoseUpdateSlot]);
	HostReceivedOriginX.store(0.0);
	fn(&Host, device, pose, size);
	return HostReceivedOriginX.load();
}

// Moves device 0's world origin, so a pose that went through the detour
// reaches the host with a non-zero origin.
bool CalibrateDeviceZero(ServerTrackedDeviceProvider &provider)
{
	protocol::SetRuntimeState state;
	state.enabledMask = 1;
	state.transform.translation.v[0] = 0.2;
	state.field.enabled = 1;
	state.field.anchorCount = 1;
	state.field.anchors[0].position[0] = 0.2;
	state.field.anchors[0].translationDelta[0] = -0.1;
	state.transform.generation = state.field.generation = 1;
	return provider.TrySetRuntimeState(state);
}

// At most five seconds, so a seam that is never reached fails the scenario
// instead of hanging the harness.
bool AwaitFlag(const std::atomic<bool> &flag)
{
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
	while (!flag.load())
	{
		if (std::chrono::steady_clock::now() >= deadline)
			return false;
		std::this_thread::yield();
	}
	return true;
}

std::atomic<bool> ParkArmed{ false };
std::atomic<bool> Parked{ false };

// Holds a TryInstallPoseHook call past its accept check, with the setup mutex
// held, until teardown has cleared the ready flag or half a second passes.
void ParkUntilReadyCleared()
{
	if (!ParkArmed.exchange(false))
		return;
	Parked.store(true);
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
	while (PoseUpdateHookMask() != 0 && std::chrono::steady_clock::now() < deadline)
		std::this_thread::yield();
}

// A detour that has passed its accept check when teardown begins must not
// leave a ready flag behind: the next Init in this process would then skip
// installing the pose hook and still report it installed.
void HookReadyFlagTeardownRaceScenario(Check check)
{
	const char *name = "hooks: teardown leaves no stale ready flag";
	if (!LogFile)
		LogFile = stderr;   // the LOG macro writes unconditionally
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	const bool calibrated = CalibrateDeviceZero(*provider);
	if (!InjectHooks(provider.get(), context))
	{
		check(name, false, "InjectHooks failed");
		return;
	}
	RequestHostInterface();
	const uint32_t installed = PoseUpdateHookMask();
	const double firstOrigin = SendPoseThroughHost();

	Parked.store(false);
	ParkArmed.store(true);
	TryInstallAfterAcceptCheckForTest = &ParkUntilReadyCleared;
	std::thread request(&RequestHostInterface);
	const bool parked = AwaitFlag(Parked);
	const bool released = DisableHooks();
	request.join();
	TryInstallAfterAcceptCheckForTest = nullptr;
	const uint32_t after = PoseUpdateHookMask();

	char detail[160];
	std::snprintf(detail, sizeof detail,
		"mask after install %u (host origin x %.3f), parked %d, after teardown %u, released %d",
		installed, firstOrigin, parked ? 1 : 0, after, released ? 1 : 0);
	check(name, calibrated && installed == protocol::PoseHook006 && firstOrigin != 0.0 &&
		parked && after == 0 && released, detail);

	// The next Init in the same process installs the hook afresh: a pose sent
	// through the host reaches it calibrated.
	const bool reinjected = InjectHooks(provider.get(), context);
	if (reinjected)
		RequestHostInterface();
	const uint32_t again = PoseUpdateHookMask();
	const double origin = reinjected ? SendPoseThroughHost() : 0.0;
	const bool cleaned = !reinjected || DisableHooks();
	std::snprintf(detail, sizeof detail,
		"reinjected %d, mask %u, host origin x %.3f, torn down %d",
		reinjected ? 1 : 0, again, origin, cleaned ? 1 : 0);
	check("hooks: a second Init installs the pose hook again",
		reinjected && again == protocol::PoseHook006 &&
		origin != 0.0 && cleaned && PoseUpdateHookMask() == 0, detail);
}

std::atomic<bool> InsideDriver{ false };
std::atomic<bool> LeaveDriver{ false };
std::atomic<bool> TeardownWaiting{ false };

// Holds a pose callback inside the driver until the scenario lets it go, or
// AwaitFlag's deadline passes.
void HoldInsideDriver()
{
	InsideDriver.store(true);
	AwaitFlag(LeaveDriver);
}

void NoteTeardownWaiting()
{
	TeardownWaiting.store(true);
}

// Teardown waits for a pose callback that has found the driver, since it may
// still publish into the ring; the callback then forwards its pose through the
// disabled hook's trampoline, and the next pose goes straight to the host.
void TeardownWaitsForDriverScenario(Check check)
{
	const char *name = "hooks: teardown waits for a pose callback inside the driver";
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	const bool calibrated = CalibrateDeviceZero(*provider);
	if (!InjectHooks(provider.get(), context))
	{
		check(name, false, "InjectHooks failed");
		return;
	}
	RequestHostInterface();
	const uint32_t installed = PoseUpdateHookMask();

	InsideDriver.store(false);
	LeaveDriver.store(false);
	TeardownWaiting.store(false);
	InsideDriverCallbackForTest = &HoldInsideDriver;
	BeforeDriverWaitForTest = &NoteTeardownWaiting;
	std::atomic<double> heldOrigin{ 0.0 };
	std::thread pose([&] { heldOrigin.store(SendPoseThroughHost()); });
	const bool held = AwaitFlag(InsideDriver);

	std::atomic<bool> released{ false };
	std::atomic<bool> returned{ false };
	std::thread teardown([&] {
		released.store(DisableHooks());
		returned.store(true);
	});
	const bool waiting = AwaitFlag(TeardownWaiting);
	// Ample time for a teardown that skipped the wait to return.
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	const bool returnedWhileHeld = returned.load();
	LeaveDriver.store(true);
	teardown.join();
	pose.join();
	InsideDriverCallbackForTest = nullptr;
	BeforeDriverWaitForTest = nullptr;
	const double after = SendPoseThroughHost();

	char detail[160];
	std::snprintf(detail, sizeof detail,
		"mask %u, held %d, waiting %d, returned while held %d, released %d, "
		"held pose origin x %.3f, next %.3f",
		installed, held ? 1 : 0, waiting ? 1 : 0, returnedWhileHeld ? 1 : 0,
		released.load() ? 1 : 0, heldOrigin.load(), after);
	check(name, calibrated && installed == protocol::PoseHook006 && held && waiting &&
		!returnedWhileHeld && released.load() && heldOrigin.load() != 0.0 && after == 0.0 &&
		PoseUpdateHookMask() == 0, detail);
}

// A callback that never leaves ends the wait at its deadline instead of
// holding SteamVR's shutdown.
void CallbackWaitDeadlineScenario(Check check)
{
	questcal::hooks::CallbackActivity activity;
	const bool idleAtOnce = activity.WaitUntilIdle(std::chrono::milliseconds(0));
	bool gaveUp = false;
	{
		questcal::hooks::CallbackActivity::Guard inside(activity);
		gaveUp = !activity.WaitUntilIdle(std::chrono::milliseconds(5));
	}
	const bool idleAfter = activity.WaitUntilIdle(std::chrono::milliseconds(0));
	char detail[96];
	std::snprintf(detail, sizeof detail, "idle at once %d, gave up %d, idle after %d",
		idleAtOnce ? 1 : 0, gaveUp ? 1 : 0, idleAfter ? 1 : 0);
	check("hooks: teardown stops waiting for a callback that stays inside",
		idleAtOnce && gaveUp && idleAfter, detail);
}

// The hook reports the devices whose poses it has seen, and forwards untouched
// a pose whose DriverPose_t size is not this build's, and one re-entering the
// hook for the same device from inside the host call it forwards to: that pose
// is already transformed, and its raw form already published.
void HookStatusScenario(Check check)
{
	const char *name = "hooks: the pose hook reports what it has seen";
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	const bool calibrated = CalibrateDeviceZero(*provider);
	if (!InjectHooks(provider.get(), context))
	{
		check(name, false, "InjectHooks failed");
		return;
	}
	RequestHostInterface();
	const protocol::HookStatus fresh = PoseHookStatus();

	const double mismatched = SendPoseThroughHost(5, sizeof(vr::DriverPose_t) - 8);
	InnerOriginX.store(0.0);
	ReenterOnce.store(true);
	const double outer = SendPoseThroughHost(0);
	const double inner = InnerOriginX.load();
	const protocol::HookStatus seen = PoseHookStatus();
	const bool released = DisableHooks();

	char detail[220];
	std::snprintf(detail, sizeof detail,
		"fresh %llx/%u/%u; devices %llx, mismatched %u (origin %.3f), re-entrant %u "
		"(outer %.3f, inner %.3f); released %d",
		static_cast<unsigned long long>(fresh.hookedDevices), fresh.mismatchedPoseUpdates,
		fresh.reentrantPoseUpdates, static_cast<unsigned long long>(seen.hookedDevices),
		seen.mismatchedPoseUpdates, mismatched, seen.reentrantPoseUpdates, outer, inner,
		released ? 1 : 0);
	check(name, calibrated && fresh.hookedDevices == 0 && fresh.mismatchedPoseUpdates == 0 &&
		fresh.reentrantPoseUpdates == 0 && seen.hookedDevices == 1 &&
		seen.mismatchedPoseUpdates == 1 && mismatched == 0.0 &&
		seen.reentrantPoseUpdates == 1 && outer != 0.0 && inner == outer && released, detail);
}

constexpr uint32_t QuietStation = 3;

// The station's own next pose, from its driver's thread, arriving while the
// host handles a re-send.
void StationPoseFromAnotherThread()
{
	std::thread pose([] { SendPoseThroughHost(QuietStation, sizeof(vr::DriverPose_t), 2.5); });
	pose.join();
}

// A base station's driver sends a pose minutes to hours apart, so RunFrame
// sends a quiet station's last pose again, through the hook's original, once
// the calibration has moved: marked as handled, so a host forwarding it to
// another hooked version passes it on untouched. A tracker, and a station
// heard from within half a second, wait for their own next pose; the
// station's own pose arriving during a re-send has the re-send repeated from
// that pose.
void QuietBaseStationScenario(Check check)
{
	const char *name = "hooks: a quiet base station's pose follows the calibration";
	const uint32_t tracker = QuietStation + 1;
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	provider->SetDeviceClassForTest(QuietStation, vr::TrackedDeviceClass_TrackingReference);
	provider->SetDeviceClassForTest(tracker, vr::TrackedDeviceClass_GenericTracker);
	auto calibrate = [&](double x, uint32_t generation)
	{
		protocol::SetRuntimeState state;
		state.enabledMask = (uint64_t{ 1 } << QuietStation) | (uint64_t{ 1 } << tracker);
		state.transform.translation.v[0] = x;
		state.transform.generation = generation;
		return provider->TrySetRuntimeState(state);
	};
	if (!calibrate(0.2, 1) || !InjectHooks(provider.get(), context))
	{
		check(name, false, "setup failed");
		return;
	}
	RequestHostInterface();
	for (auto &calls : HostCallsByDevice)
		calls.store(0);
	provider->SetPoseTimeForTest(10.0);
	const double first = SendPoseThroughHost(QuietStation, sizeof(vr::DriverPose_t), 1.5);
	SendPoseThroughHost(tracker);

	// One re-send pass at `time`: the station's host calls in it, and the
	// origin and position the last outer one carried.
	struct Pass
	{
		int calls;
		double origin;
		double position;
	};
	auto pass = [&](double time)
	{
		const int before = HostCallsByDevice[QuietStation].load();
		HostReceivedOriginX.store(0.0);
		HostReceivedPositionX.store(0.0);
		provider->SetPoseTimeForTest(time);
		provider->ResendQuietPosesForTest();
		return Pass{ HostCallsByDevice[QuietStation].load() - before,
			HostReceivedOriginX.load(), HostReceivedPositionX.load() };
	};
	const bool moved = calibrate(0.5, 2);
	const Pass early = pass(10.2);
	const Pass quiet = pass(10.6);
	const Pass settled = pass(10.8);

	const uint32_t reentrantBefore = PoseHookStatus().reentrantPoseUpdates;
	const bool movedAgain = calibrate(0.7, 3);
	InnerOriginX.store(0.0);
	ReenterOnce.store(true);
	const Pass forwarded = pass(11.0);
	const double inner = InnerOriginX.load();
	const uint32_t reentrant = PoseHookStatus().reentrantPoseUpdates - reentrantBefore;

	const bool movedThird = calibrate(0.9, 4);
	InsideHostOnce.store(&StationPoseFromAnotherThread);
	const Pass raced = pass(11.2);
	const Pass repeated = pass(11.35);
	const Pass after = pass(11.8);
	const int trackerCalls = HostCallsByDevice[tracker].load();
	const bool released = DisableHooks();
	provider->SetPoseTimeForTest(-1.0);

	char detail[320];
	std::snprintf(detail, sizeof detail,
		"first origin %.2f; early %d, quiet %d (origin %.2f, position %.2f), settled %d; "
		"forwarded %d (origin %.2f, inner %.2f, re-entrant %u); raced %d, repeated %d "
		"(origin %.2f, position %.2f), after %d; tracker calls %d; released %d",
		first, early.calls, quiet.calls, quiet.origin, quiet.position, settled.calls,
		forwarded.calls, forwarded.origin, inner, reentrant, raced.calls, repeated.calls,
		repeated.origin, repeated.position, after.calls, trackerCalls, released ? 1 : 0);
	auto equal = [](double a, double b) { return std::abs(a - b) < 1e-9; };
	check(name, moved && movedAgain && movedThird && equal(first, 0.2) &&
		early.calls == 0 &&
		quiet.calls == 1 && equal(quiet.origin, 0.5) && equal(quiet.position, 1.5) &&
		settled.calls == 0 &&
		forwarded.calls == 2 && equal(forwarded.origin, 0.7) && equal(inner, 0.7) && reentrant == 1 &&
		raced.calls == 2 &&
		repeated.calls == 1 && equal(repeated.origin, 0.9) && equal(repeated.position, 2.5) &&
		after.calls == 0 && trackerCalls == 1 && released, detail);
}

// A station whose rotation is unit only to float precision, as the lighthouse
// driver sends them: |q|^2 1.4e-8 short, as LHB-170EE067's was on 2026-10-15.
// Once its re-send carries the moved calibration, later passes have nothing
// to send; measuring the turn as 2 acos(q . q) re-sent it every pass.
void ShortRotationStationScenario(Check check)
{
	const char *name = "hooks: a station rotation short of unit length is not re-sent again";
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	provider->SetDeviceClassForTest(QuietStation, vr::TrackedDeviceClass_TrackingReference);
	auto calibrate = [&](double x, uint32_t generation)
	{
		protocol::SetRuntimeState state;
		state.enabledMask = uint64_t{ 1 } << QuietStation;
		state.transform.translation.v[0] = x;
		state.transform.generation = generation;
		return provider->TrySetRuntimeState(state);
	};
	if (!calibrate(0.2, 1) || !InjectHooks(provider.get(), context))
	{
		check(name, false, "setup failed");
		return;
	}
	RequestHostInterface();
	const vr::HmdQuaternion_t rotation{ 0.9729632735, 0.2299866825, 0.004872145131, -0.02061170898 };
	const double shortfall = 1.0 - (rotation.w * rotation.w + rotation.x * rotation.x +
		rotation.y * rotation.y + rotation.z * rotation.z);
	provider->SetPoseTimeForTest(20.0);
	SendPoseThroughHost(QuietStation, sizeof(vr::DriverPose_t), 1.5, &rotation);
	auto pass = [&](double time)
	{
		const int before = HostCallsByDevice[QuietStation].load();
		provider->SetPoseTimeForTest(time);
		provider->ResendQuietPosesForTest();
		return HostCallsByDevice[QuietStation].load() - before;
	};
	const bool moved = calibrate(0.5, 2);
	const int quiet = pass(20.6);
	int later = 0;
	for (int i = 1; i <= 10; ++i)
		later += pass(20.6 + 0.15 * i);
	const bool released = DisableHooks();
	provider->SetPoseTimeForTest(-1.0);

	char detail[128];
	std::snprintf(detail, sizeof detail, "|q|^2 short by %.2e; quiet %d, later passes %d; released %d",
		shortfall, quiet, later, released ? 1 : 0);
	check(name, moved && shortfall > 1e-9 && quiet == 1 && later == 0 && released, detail);
}

// A target SteamVR tracks but the hook has not seen is reported once three
// synchronizations in a row have missed it; one the hook has seen, an
// untracked one and one that is not a target never are.
void HookCoverageScenario(Check check)
{
	questcal::HookCoverage coverage;
	const uint64_t targets = 0b1110;
	const uint64_t tracked = 0b0111;
	const uint64_t first = coverage.Note(0b0010, tracked, targets);
	const uint64_t second = coverage.Note(0b0010, tracked, targets);
	const uint64_t third = coverage.Note(0b0010, tracked, targets);
	const uint64_t fourth = coverage.Note(0b0010, tracked, targets);
	const uint64_t reached = coverage.Note(0b0110, tracked, targets);

	char detail[96];
	std::snprintf(detail, sizeof detail, "reported %llx %llx %llx %llx, then %llx",
		static_cast<unsigned long long>(first), static_cast<unsigned long long>(second),
		static_cast<unsigned long long>(third), static_cast<unsigned long long>(fourth),
		static_cast<unsigned long long>(reached));
	check("hooks: a target that bypasses the pose hook is reported after three syncs",
		first == 0 && second == 0 && third == 0b0100 && fourth == 0b0100 && reached == 0, detail);
}
// Code at made-up addresses for the jump walk: reads inside it succeed, any
// other read fails, as ReadProcessMemory does on unmapped memory.
struct FakeCode
{
	static constexpr uintptr_t Base = 0x10000;
	std::vector<uint8_t> bytes = std::vector<uint8_t>(0x400, 0xCC);

	bool operator()(uintptr_t at, void *out, size_t size) const
	{
		if (at < Base || at + size > Base + bytes.size())
			return false;
		std::memcpy(out, bytes.data() + (at - Base), size);
		return true;
	}
	void Put(uintptr_t at, std::initializer_list<uint8_t> code)
	{
		std::copy(code.begin(), code.end(), bytes.begin() + (at - Base));
	}
	void Put64(uintptr_t at, uint64_t value)
	{
		std::memcpy(bytes.data() + (at - Base), &value, sizeof value);
	}
	// jmp rel32 from `at` to `to`.
	void Jump(uintptr_t at, uintptr_t to)
	{
		const int32_t rel = static_cast<int32_t>(static_cast<intptr_t>(to) - static_cast<intptr_t>(at + 5));
		Put(at, { 0xE9 });
		std::memcpy(bytes.data() + (at - Base) + 1, &rel, sizeof rel);
	}
	// MinHook's x64 relay: jmp [rip+0], then the absolute target.
	void Relay(uintptr_t at, uint64_t to)
	{
		Put(at, { 0xFF, 0x25, 0, 0, 0, 0 });
		Put64(at + 6, to);
	}
};

// The pose hook's own chain, as MinHook leaves it and with the forms other
// hooking libraries write in front: each walk ends at the detour, and a thunk
// behind the detour is not followed.
void HookChainOwnScenario(Check check)
{
	using questcal::hookchain::Follow;
	constexpr uintptr_t B = FakeCode::Base;
	constexpr uintptr_t Detour = 0x7FF000001000;   // never read: compared first
	FakeCode code;
	code.Jump(B + 0x00, B + 0x100);   // entry -> relay -> detour
	code.Relay(B + 0x100, Detour);
	code.Jump(B + 0x20, B + 0x00);    // a hot patch: jmp short back to a jmp rel32
	code.Put(B + 0x30, { 0xEB, 0xEE });   // 0x30 + 2 - 0x12 = 0x20
	code.Put(B + 0x40, { 0x48, 0xB8 });   // mov rax, imm64; jmp rax
	code.Put64(B + 0x42, B + 0x30);
	code.Put(B + 0x4A, { 0xFF, 0xE0 });
	code.Put(B + 0x60, { 0x49, 0xBB });   // mov r11, imm64; jmp r11
	code.Put64(B + 0x62, B + 0x40);
	code.Put(B + 0x6A, { 0x41, 0xFF, 0xE3 });
	code.Jump(B + 0x200, B + 0x300);  // a detour that is itself an incremental-link thunk
	code.Jump(B + 0x300, B + 0x380);

	const auto minhook = Follow(B + 0x00, Detour, code);
	const auto hotPatch = Follow(B + 0x30, Detour, code);
	const auto absolute = Follow(B + 0x60, Detour, code);
	const auto atDetour = Follow(B + 0x00, B + 0x00, code);
	const auto thunk = Follow(B + 0x200, B + 0x300, code);

	char detail[200];
	std::snprintf(detail, sizeof detail, "minhook %d/%u, hot patch %d/%u, absolute %d/%u, at detour %d/%u, "
		"thunk %d/%u end %llx", minhook.reachedDetour, minhook.hops, hotPatch.reachedDetour, hotPatch.hops,
		absolute.reachedDetour, absolute.hops, atDetour.reachedDetour, atDetour.hops, thunk.reachedDetour,
		thunk.hops, static_cast<unsigned long long>(thunk.end));
	check("hooks: the jump walk follows the pose hook's own chain to its detour",
		minhook.reachedDetour && minhook.hops == 2 && minhook.end == Detour &&
		hotPatch.reachedDetour && hotPatch.hops == 4 &&
		absolute.reachedDetour && absolute.hops == 6 && !absolute.unreadable &&
		atDetour.reachedDetour && atDetour.hops == 0 &&
		thunk.reachedDetour && thunk.hops == 1 && thunk.end == B + 0x300, detail);
}

// A second MinHook user patched the entry over the pose hook's patch: the walk
// ends in its detour. Code that is not a jump, a jump loop, and a jump to
// memory that cannot be read never count as reaching the detour.
void HookChainForeignScenario(Check check)
{
	using questcal::hookchain::Follow;
	using questcal::hookchain::MaxHops;
	constexpr uintptr_t B = FakeCode::Base;
	constexpr uintptr_t Detour = 0x7FF000001000;
	FakeCode code;
	code.Jump(B + 0x00, B + 0x180);   // the second user's entry patch -> its relay
	code.Relay(B + 0x180, B + 0x280); // -> its detour, ordinary code
	code.Put(B + 0x280, { 0x48, 0x89, 0x5C, 0x24, 0x08 });
	code.Put(B + 0x40, { 0x40, 0x53 });   // an unpatched function: push rbx
	code.Put(B + 0x50, { 0x48, 0xB8 });   // mov rax, imm64 not followed by jmp rax
	code.Put64(B + 0x52, Detour);
	code.Put(B + 0x5A, { 0x90, 0x90 });
	code.Put(B + 0x60, { 0xE9, 0xFB, 0xFF, 0xFF, 0xFF });   // jmp to itself
	code.Jump(B + 0x70, 0x500000);    // to memory that is not there
	code.Put(B + 0x80, { 0xFF, 0x25, 0x00, 0x00, 0x10, 0x00 });   // pointer not there
	code.Put(B + 0x3FF, { 0xE9 });    // a jump cut off by the end of memory

	const auto ahead = Follow(B + 0x00, Detour, code);
	const auto plain = Follow(B + 0x40, Detour, code);
	const auto movOnly = Follow(B + 0x50, Detour, code);
	const auto loop = Follow(B + 0x60, Detour, code);
	const auto gone = Follow(B + 0x70, Detour, code);
	const auto pointer = Follow(B + 0x80, Detour, code);
	const auto cut = Follow(B + 0x3FF, Detour, code);

	char detail[260];
	std::snprintf(detail, sizeof detail, "ahead %d end %llx hops %u; plain %d/%u; mov %d/%u; loop %d/%u; "
		"gone %d unreadable %d end %llx; pointer %d unreadable %d; cut %d unreadable %d",
		ahead.reachedDetour, static_cast<unsigned long long>(ahead.end), ahead.hops, plain.reachedDetour,
		plain.hops, movOnly.reachedDetour, movOnly.hops, loop.reachedDetour, loop.hops, gone.reachedDetour,
		gone.unreadable, static_cast<unsigned long long>(gone.end), pointer.reachedDetour, pointer.unreadable,
		cut.reachedDetour, cut.unreadable);
	check("hooks: the jump walk ends in code that runs ahead of the detour",
		!ahead.reachedDetour && ahead.end == B + 0x280 && ahead.hops == 2 && !ahead.unreadable &&
		!plain.reachedDetour && plain.end == B + 0x40 && plain.hops == 0 &&
		!movOnly.reachedDetour && movOnly.end == B + 0x50 &&
		!loop.reachedDetour && loop.hops == MaxHops &&
		!gone.reachedDetour && gone.unreadable && gone.end == 0x500000 &&
		!pointer.reachedDetour && pointer.unreadable &&
		!cut.reachedDetour && cut.end == B + 0x3FF, detail);
}

// The module name the driver reports is taken up to the first NUL inside its
// array only, without control characters; a driver's file is named as the
// player knows the driver.
void ForeignModuleNameScenario(Check check)
{
	char unterminated[8];
	std::memcpy(unterminated, "ABCDEFGH", sizeof unterminated);
	const char odd[] = "dri\x01ver_X\x7F.dll\0hidden";
	const std::string full = questcal::ForeignModuleFile(unterminated, sizeof unterminated);
	const std::string cleaned = questcal::ForeignModuleFile(odd, sizeof odd);
	const std::string smooth = questcal::ForeignDriverName("driver_SmoothTracking.dll");
	const std::string upper = questcal::ForeignDriverName("DRIVER_Lighthouse.DLL");
	const std::string bare = questcal::ForeignDriverName("driver_.dll");
	const std::string other = questcal::ForeignDriverName("OVRInjector.dll");
	const std::string exe = questcal::ForeignDriverName("driver_tool.exe");

	char detail[200];
	std::snprintf(detail, sizeof detail, "'%s' '%s' '%s' '%s' '%s' '%s' '%s'", full.c_str(), cleaned.c_str(),
		smooth.c_str(), upper.c_str(), bare.c_str(), other.c_str(), exe.c_str());
	check("hooks: the module ahead is read safely and named as its driver",
		full == "ABCDEFGH" && cleaned == "driver_X.dll" && smooth == "SmoothTracking" &&
		upper == "Lighthouse" && bare.empty() && other.empty() && exe.empty(), detail);
}

// What OpenVR-SmoothTracking's detour does to a pose: moves it (smoothing) and
// zeroes its velocity, then calls the code its patch displaced.
PoseUpdateFn ForeignDisplaced = nullptr;
std::atomic<int> ForeignCalls{ 0 };
void ForeignSmoothingDetour(void *host, uint32_t device, const vr::DriverPose_t &pose, uint32_t size)
{
	ForeignCalls.fetch_add(1);
	vr::DriverPose_t smoothed = pose;
	smoothed.vecPosition[0] += 0.5;
	smoothed.vecVelocity[0] = smoothed.vecVelocity[1] = smoothed.vecVelocity[2] = 0.0;
	ForeignDisplaced(host, device, smoothed, size);
}

// The second hooker's relay, in this image so the entry's jmp rel32 reaches it.
alignas(16) uint8_t ForeignRelay[16] = {};

// Drivers that hook by taking the host's vtable slot, and call what it held.
// Distinct functions, each with its own next, as two drivers would be.
PoseUpdateFn FirstTakerNext = nullptr;
PoseUpdateFn SecondTakerNext = nullptr;
__declspec(noinline) void FirstSlotTaker(void *host, uint32_t device, const vr::DriverPose_t &pose,
	uint32_t size)
{
	ForeignCalls.fetch_add(10);
	FirstTakerNext(host, device, pose, size);
}
__declspec(noinline) void SecondSlotTaker(void *host, uint32_t device, const vr::DriverPose_t &pose,
	uint32_t size)
{
	ForeignCalls.fetch_add(100);
	SecondTakerNext(host, device, pose, size);
}

// `executable` keeps the page executable afterwards, for code written into
// this image's data.
bool WriteCode(void *at, const void *bytes, size_t size, bool executable = false)
{
	DWORD previous = 0;
	if (!VirtualProtect(at, size, PAGE_EXECUTE_READWRITE, &previous))
		return false;
	std::memcpy(at, bytes, size);
	DWORD ignored = 0;
	if (!executable)
		VirtualProtect(at, size, previous, &ignored);
	return FlushInstructionCache(GetCurrentProcess(), at, size) != FALSE;
}

void **HostPoseSlot()
{
	return *reinterpret_cast<void ***>(&Host) + openvr_hook::PoseUpdateSlot;
}

bool SetHostPoseSlot(void *value)
{
	return WriteCode(HostPoseSlot(), &value, sizeof value);
}

// This test executable's file name, as the driver reports a module.
std::string OwnModuleFile()
{
	wchar_t path[MAX_PATH] = {};
	GetModuleFileNameW(nullptr, path, MAX_PATH);
	const wchar_t *name = path;
	for (const wchar_t *c = path; *c; ++c)
		if (*c == L'\\' || *c == L'/')
			name = c + 1;
	char utf8[MAX_PATH * 3] = {};
	WideCharToMultiByte(CP_UTF8, 0, name, -1, utf8, sizeof utf8, nullptr, nullptr);
	return utf8;
}

// The 2026-10-14 case: OpenVR-SmoothTracking loads after this driver and
// MinHooks TrackedDevicePoseUpdated over the pose hook's patch, so the pose
// ring received its smoothed, zero-velocity poses. The order check finds its
// code ahead, names its module, and puts the observation hook in the host's
// vtable: the ring then gets each pose raw, once, while the smoothing and the
// calibration still reach the host as before. Teardown gives the slot back.
void ForeignHookObservedScenario(Check check)
{
	const char *name = "hooks: a driver hooking poses ahead of the pose hook is found and raw poses are recorded ahead of it";
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	const bool calibrated = CalibrateDeviceZero(*provider);
	if (!InjectHooks(provider.get(), context))
	{
		check(name, false, "InjectHooks failed");
		return;
	}
	RequestHostInterface();
	void *const target = *HostPoseSlot();
	RefreshPoseHookOrder();
	const protocol::HookStatus clean = PoseHookStatus();

	// The second MinHook user's patch: the entry's jump now leads to its relay,
	// and its detour calls where the jump it replaced led.
	uint8_t ownPatch[5];
	std::memcpy(ownPatch, target, sizeof ownPatch);
	int32_t ownRel;
	std::memcpy(&ownRel, ownPatch + 1, sizeof ownRel);
	ForeignDisplaced = reinterpret_cast<PoseUpdateFn>(static_cast<uint8_t *>(target) + 5 + ownRel);
	const uint64_t foreignDetour = reinterpret_cast<uint64_t>(&ForeignSmoothingDetour);
	uint8_t relay[14] = { 0xFF, 0x25, 0, 0, 0, 0 };
	std::memcpy(relay + 6, &foreignDetour, sizeof foreignDetour);
	const intptr_t toRelay = reinterpret_cast<intptr_t>(ForeignRelay) - (reinterpret_cast<intptr_t>(target) + 5);
	const int32_t foreignRel = static_cast<int32_t>(toRelay);
	uint8_t foreignPatch[5] = { 0xE9 };
	std::memcpy(foreignPatch + 1, &foreignRel, sizeof foreignRel);
	const bool patched = ownPatch[0] == 0xE9 && toRelay == foreignRel &&
		WriteCode(ForeignRelay, relay, sizeof relay, true) && WriteCode(target, foreignPatch, sizeof foreignPatch);

	// Before the check: the ring gets what the second hooker made of the pose.
	const uint32_t publishedBefore = provider->PublishedForTest();
	ForeignCalls.store(0);
	SendPoseThroughHost(0, sizeof(vr::DriverPose_t), 1.0);
	const protocol::DevicePoseSample unobserved = provider->LastPublishedForTest();
	const bool smoothedBefore = provider->PublishedForTest() == publishedBefore + 1 &&
		unobserved.position[0] == 1.5 && ForeignCalls.load() == 1;

	RefreshPoseHookOrder();
	const protocol::HookStatus found = PoseHookStatus();
	const bool observerInSlot = *HostPoseSlot() != target;

	vr::DriverPose_t pose{};
	pose.vecPosition[0] = 2.0;
	pose.vecVelocity[0] = 0.25;
	pose.poseIsValid = pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.qRotation.w = pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = 1;
	const uint32_t publishedAhead = provider->PublishedForTest();
	HostReceivedOriginX.store(0.0);
	reinterpret_cast<PoseUpdateFn>(*HostPoseSlot())(&Host, 0, pose, sizeof pose);
	const protocol::DevicePoseSample raw = provider->LastPublishedForTest();
	const bool rawOnce = provider->PublishedForTest() == publishedAhead + 1 &&
		raw.position[0] == 2.0 && raw.velocity[0] == 0.25;
	const bool hostGotBoth = ForeignCalls.load() == 2 && HostReceivedPositionX.load() == 2.5 &&
		HostReceivedOriginX.load() != 0.0;
	const protocol::HookStatus observed = PoseHookStatus();

	const bool unpatched = WriteCode(target, ownPatch, sizeof ownPatch);
	RefreshPoseHookOrder();
	const protocol::HookStatus gone = PoseHookStatus();
	const bool released = DisableHooks();
	const bool slotBack = *HostPoseSlot() == target;

	char detail[400];
	std::snprintf(detail, sizeof detail,
		"clean checks %u ahead %x observer %x; patched %d smoothed before %d; found ahead %x seen %x "
		"observer %x failed %x module '%s' in slot %d; raw once %d (%.2f, %.2f) host %d (%.2f); observed %u "
		"around %u; gone ahead %x seen %x observer %x; unpatched %d released %d slot back %d",
		clean.orderChecks, clean.foreignAheadMask, clean.observerMask, patched, smoothedBefore,
		found.foreignAheadMask, found.foreignAheadSeenMask, found.observerMask, found.observerFailedMask,
		found.foreignModule, observerInSlot, rawOnce, raw.position[0], raw.velocity[0], hostGotBoth,
		HostReceivedPositionX.load(), observed.observedPoseUpdates, observed.unobservedPoseUpdates,
		gone.foreignAheadMask, gone.foreignAheadSeenMask, gone.observerMask, unpatched, released, slotBack);
	check(name, calibrated && clean.orderChecks == 1 && clean.foreignAheadMask == 0 &&
		clean.foreignAheadSeenMask == 0 && clean.observerMask == 0 && clean.foreignModule[0] == '\0' &&
		patched && smoothedBefore &&
		found.foreignAheadMask == protocol::PoseHook006 && found.foreignAheadSeenMask == protocol::PoseHook006 &&
		found.observerMask == protocol::PoseHook006 && found.observerFailedMask == 0 &&
		OwnModuleFile() == found.foreignModule && observerInSlot &&
		rawOnce && hostGotBoth && observed.observedPoseUpdates == 1 && observed.unobservedPoseUpdates == 0 &&
		unpatched && gone.foreignAheadMask == 0 && gone.foreignAheadSeenMask == protocol::PoseHook006 &&
		gone.observerMask == protocol::PoseHook006 && released && slotBack, detail);
}

// Another driver takes the host's vtable slot after the observer and calls
// it: the observer is reported overtaken, and never placed on top of that
// code, which would make the two call each other. Poses still flow.
void ObserverOvertakenScenario(Check check)
{
	const char *name = "hooks: an observation hook overtaken in the host vtable is reported and not placed again";
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	if (!InjectHooks(provider.get(), context))
	{
		check(name, false, "InjectHooks failed");
		return;
	}
	RequestHostInterface();
	void *const target = *HostPoseSlot();

	// A vtable hook ahead of the pose hook, as a driver hooking by slot does.
	void *const first = reinterpret_cast<void *>(&FirstSlotTaker);
	void *const second = reinterpret_cast<void *>(&SecondSlotTaker);
	FirstTakerNext = reinterpret_cast<PoseUpdateFn>(target);
	const bool firstTaken = SetHostPoseSlot(first);
	RefreshPoseHookOrder();
	const protocol::HookStatus placed = PoseHookStatus();
	void *const observer = *HostPoseSlot();

	// A second one takes the slot from the observer.
	SecondTakerNext = reinterpret_cast<PoseUpdateFn>(observer);
	const bool secondTaken = SetHostPoseSlot(second);
	RefreshPoseHookOrder();
	const protocol::HookStatus overtaken = PoseHookStatus();
	const bool notReplaced = *HostPoseSlot() == second;
	const uint32_t publishedBefore = provider->PublishedForTest();
	HostReceivedPositionX.store(0.0);
	ForeignCalls.store(0);
	SendPoseThroughHost(0, sizeof(vr::DriverPose_t), 3.0);
	const bool flows = HostReceivedPositionX.load() == 3.0 && ForeignCalls.load() == 110 &&
		provider->PublishedForTest() == publishedBefore + 1 &&
		provider->LastPublishedForTest().position[0] == 3.0;

	// Unwound in order: the second taker leaves, then teardown takes the
	// observer out, then the first taker leaves.
	const bool secondLeft = SetHostPoseSlot(observer);
	const bool released = DisableHooks();
	const bool observerLeft = *HostPoseSlot() == first;
	const bool firstLeft = SetHostPoseSlot(target);

	char detail[260];
	std::snprintf(detail, sizeof detail, "taken %d/%d; placed observer %x failed %x; overtaken observer %x "
		"failed %x ahead %x; not replaced %d flows %d; unwound %d %d %d %d",
		firstTaken, secondTaken, placed.observerMask, placed.observerFailedMask, overtaken.observerMask,
		overtaken.observerFailedMask, overtaken.foreignAheadMask, notReplaced, flows, secondLeft, released,
		observerLeft, firstLeft);
	check(name, firstTaken && secondTaken && placed.observerMask == protocol::PoseHook006 &&
		placed.observerFailedMask == 0 && observer != first && observer != target &&
		overtaken.observerFailedMask == protocol::PoseHook006 &&
		overtaken.foreignAheadMask == protocol::PoseHook006 && notReplaced && flows &&
		secondLeft && released && observerLeft && firstLeft, detail);
}
} // namespace

void RunHookInjectorScenarios(Check check)
{
	HookReadyFlagTeardownRaceScenario(check);
	TeardownWaitsForDriverScenario(check);
	CallbackWaitDeadlineScenario(check);
	HookStatusScenario(check);
	QuietBaseStationScenario(check);
	ShortRotationStationScenario(check);
	HookCoverageScenario(check);
	HookChainOwnScenario(check);
	HookChainForeignScenario(check);
	ForeignModuleNameScenario(check);
	ForeignHookObservedScenario(check);
	ObserverOvertakenScenario(check);
}
