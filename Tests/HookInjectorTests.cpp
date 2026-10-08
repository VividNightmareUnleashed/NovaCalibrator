#include "../Driver/HookLifecyclePolicy.h"
#include "../Driver/InterfaceHookInjector.h"
#include "../Driver/Logging.h"
#include "../Driver/OpenVRHookLayout.h"
#include "../Driver/ServerTrackedDeviceProvider.h"
#include "../Overlay/HookCoverage.h"
#include "../common/Protocol.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

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
// DriverPose_t size the caller claims.
double SendPoseThroughHost(uint32_t device = 0, uint32_t size = sizeof(vr::DriverPose_t),
	double positionX = 0.0)
{
	vr::DriverPose_t pose{};
	pose.vecPosition[0] = positionX;
	pose.poseIsValid = pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.qRotation.w = pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = 1;
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
} // namespace

void RunHookInjectorScenarios(Check check)
{
	HookReadyFlagTeardownRaceScenario(check);
	TeardownWaitsForDriverScenario(check);
	CallbackWaitDeadlineScenario(check);
	HookStatusScenario(check);
	QuietBaseStationScenario(check);
	HookCoverageScenario(check);
}
