#include "Logging.h"
#include "Hooking.h"
#include "HookChain.h"
#include "InterfaceHookInjector.h"
#include "OpenVRHookLayout.h"
#include "HookLifecyclePolicy.h"
#include "ServerTrackedDeviceProvider.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>

#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
void (*TryInstallAfterAcceptCheckForTest)() = nullptr;
void (*InsideDriverCallbackForTest)() = nullptr;
void (*BeforeDriverWaitForTest)() = nullptr;
#endif

namespace
{

std::atomic<ServerTrackedDeviceProvider *> Driver{ nullptr };
std::atomic<bool> AcceptingHookRequests{ false };
std::atomic<bool> PoseHook005Ready{ false };
std::atomic<bool> PoseHook006Ready{ false };
// Pose callbacks inside the driver, which teardown waits to leave.
questcal::hooks::CallbackActivity ActiveCallbacks;
// What the pose hook has seen since InjectHooks (protocol::HookStatus).
std::atomic<uint64_t> HookedDevices{ 0 };
std::atomic<uint32_t> MismatchedPoseUpdates{ 0 };
std::atomic<uint32_t> ReentrantPoseUpdates{ 0 };
std::atomic<uint32_t> ObservedPoseUpdates{ 0 };
std::atomic<uint32_t> UnobservedPoseUpdates{ 0 };
// What RefreshPoseHookOrder found, protected by StatusMutex.
std::mutex StatusMutex;
uint32_t OrderChecks = 0;
uint32_t ForeignAheadMask = 0;
uint32_t ForeignAheadSeenMask = 0;
uint32_t ObserverFailedMask = 0;
char ForeignModule[sizeof(protocol::HookStatus::foreignModule)] = {};
// The device whose pose this thread is forwarding through a pose detour. A
// host whose TrackedDevicePoseUpdated calls another hooked version (005
// forwarding to 006, say) would otherwise transform the pose twice and publish
// the transformed one as raw.
thread_local uint32_t ForwardingDevice = vr::k_unTrackedDeviceIndexInvalid;
// The device whose pose this thread is forwarding through an observation hook,
// which has already recorded it raw.
thread_local uint32_t ObservingDevice = vr::k_unTrackedDeviceIndexInvalid;
std::mutex HookSetupMutex;
bool MinHookInitialized = false;   // protected by HookSetupMutex
bool HooksInstalled = false;       // protected by HookSetupMutex
using CallbackGuard = questcal::hooks::CallbackActivity::Guard;

// HandleDevicePoseUpdated never blocks, so this only covers a pose thread that
// was preempted inside it.
constexpr std::chrono::milliseconds DriverReleaseWait{ 1000 };

// Once a hook has been enabled, a thread may be inside its detour or its
// trampoline at any later moment, and only stopping every thread in vrserver
// could prove otherwise. So neither ever goes away: this module stays loaded,
// and MinHook initialized with its trampolines, until the process exits.
// Teardown only disables the hooks, after which a detour still running only
// forwards.
bool PinThisModule()
{
	HMODULE module = nullptr;
	return GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_PIN |
		GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
		reinterpret_cast<LPCSTR>(&PinThisModule), &module) != FALSE;
}

Hook<void*(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *)>
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

using PoseUpdateHook =
	Hook<void(*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t)>;
using PoseUpdateDetour =
	void (*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t);

PoseUpdateHook TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");
PoseUpdateHook TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

// A driver loaded after this one that hooks TrackedDevicePoseUpdated the same
// way (OpenVR-SmoothTracking does, with MinHook) patches the function's entry
// over this hook's patch, so its code runs first and the pose hook receives
// what it made of each pose: smoothed, with its velocities zeroed. The
// calibration is still applied there, after it, as it always was, so what the
// player sees with that driver does not change; but the solver, drift and jump
// detection need the pose as the device's own driver sent it. So once
// RefreshPoseHookOrder finds foreign code ahead, an observation hook takes the
// host's vtable slot, which device drivers call through, and records each pose
// raw before forwarding it to whatever the slot held.
struct ObserverHook
{
	// The host vtable slot the pose hook was installed from. Set once, under
	// HookSetupMutex; host objects live as long as vrserver.
	std::atomic<void **> slot{ nullptr };
	// What the slot held when the observer took it, which the observer calls.
	// Set before the observer can be reached, and kept: a thread may still be
	// forwarding through it after teardown.
	std::atomic<PoseUpdateDetour> next{ nullptr };
	// While set, the observer records raw poses, and the pose hook does not
	// record those that came through it.
	std::atomic<bool> active{ false };
	// The observer is in the slot (HookSetupMutex).
	bool inSlot = false;
	// The observer left the slot other than through teardown, or teardown could
	// not take it out (something took the slot after it). Something may still
	// call it, and it only forwards; placing it again on top of other code could
	// make the two call each other (HookSetupMutex).
	bool stranded = false;
};

ObserverHook Observer005;
ObserverHook Observer006;

// The forwarding body every interface version shares. The detours stay
// distinct functions because each is the address MinHook patches in.
void ForwardPoseUpdate(PoseUpdateHook &hook, const ObserverHook &observer, void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	// Set before the hook was first enabled and never cleared, since hooks are
	// disabled but not removed. A call through null would take vrserver down,
	// so it is checked all the same.
	auto original = hook.originalFunc.load(std::memory_order_acquire);

	// The outer detour already handled this device's pose.
	if (ForwardingDevice == unWhichDevice)
	{
		ReentrantPoseUpdates.fetch_add(1, std::memory_order_relaxed);
		if (original)
			original(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	struct Forwarding
	{
		uint32_t outer;
		~Forwarding() { ForwardingDevice = outer; }
	} forwarding{ ForwardingDevice };
	ForwardingDevice = unWhichDevice;

	// Our copy is sized by the vendored DriverPose_t, so a caller passing a
	// different layout gets its own object forwarded untouched: no ring publish,
	// no transform. A layout change then degrades to "calibration not applied",
	// never to a corrupt device. No logging: stdio must not run on a pose thread.
	if (unPoseStructSize != sizeof(vr::DriverPose_t))
	{
		MismatchedPoseUpdates.fetch_add(1, std::memory_order_relaxed);
		if (original)
			original(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	if (unWhichDevice < vr::k_unMaxTrackedDeviceCount)
	{
		// A load first, so after a device's first pose this is all it costs.
		const uint64_t bit = uint64_t{ 1 } << unWhichDevice;
		if ((HookedDevices.load(std::memory_order_relaxed) & bit) == 0)
			HookedDevices.fetch_or(bit, std::memory_order_relaxed);
	}

	// An observation hook, ahead of the code ahead of this hook, recorded this
	// pose raw on its way here.
	bool publish = true;
	if (observer.active.load(std::memory_order_acquire))
	{
		if (ObservingDevice == unWhichDevice)
			publish = false;
		else
			UnobservedPoseUpdates.fetch_add(1, std::memory_order_relaxed);
	}

	auto pose = newPose;
	{
		// Counted from before Driver is read until the driver is done. The count
		// and this load are sequentially consistent with DisableHooks' store, so
		// a callback is either counted when teardown looks or finds no driver.
		CallbackGuard callback(ActiveCallbacks);
		if (ServerTrackedDeviceProvider *driver = Driver.load(std::memory_order_seq_cst))
		{
#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
			if (InsideDriverCallbackForTest)
				InsideDriverCallbackForTest();
#endif
			driver->HandleDevicePoseUpdated(unWhichDevice, pose, PoseHostCall{ _this, original },
				publish);
		}
	}
	if (original)
		original(_this, unWhichDevice, pose, unPoseStructSize);
}

void DetourTrackedDevicePoseUpdated005(void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	ForwardPoseUpdate(TrackedDevicePoseUpdatedHook005, Observer005, _this, unWhichDevice,
		newPose, unPoseStructSize);
}

void DetourTrackedDevicePoseUpdated006(void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	ForwardPoseUpdate(TrackedDevicePoseUpdatedHook006, Observer006, _this, unWhichDevice,
		newPose, unPoseStructSize);
}

// Records the pose raw, then forwards it untouched to what the slot held.
void ObservePoseUpdate(ObserverHook &observer, void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	const PoseUpdateDetour next = observer.next.load(std::memory_order_acquire);
	// Not a pose this thread is already forwarding: one the host passes on to
	// another version's vtable, or one re-sent after it was handled.
	if (observer.active.load(std::memory_order_acquire) && ObservingDevice != unWhichDevice &&
		ForwardingDevice != unWhichDevice && unPoseStructSize == sizeof(vr::DriverPose_t) &&
		unWhichDevice < vr::k_unMaxTrackedDeviceCount)
	{
		CallbackGuard callback(ActiveCallbacks);
		if (ServerTrackedDeviceProvider *driver = Driver.load(std::memory_order_seq_cst))
		{
			driver->PublishRawPose(unWhichDevice, newPose);
			ObservedPoseUpdates.fetch_add(1, std::memory_order_relaxed);
		}
	}
	struct Observing
	{
		uint32_t outer;
		~Observing() { ObservingDevice = outer; }
	} observing{ ObservingDevice };
	ObservingDevice = unWhichDevice;
	if (next)
		next(_this, unWhichDevice, newPose, unPoseStructSize);
}

void ObserveTrackedDevicePoseUpdated005(void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	ObservePoseUpdate(Observer005, _this, unWhichDevice, newPose, unPoseStructSize);
}

void ObserveTrackedDevicePoseUpdated006(void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	ObservePoseUpdate(Observer006, _this, unWhichDevice, newPose, unPoseStructSize);
}

// Supporting another interface version is one row here.
struct PoseHookBinding
{
	const char *interfaceVersion;
	PoseUpdateHook *hook;
	PoseUpdateDetour detour;
	std::atomic<bool> *ready;
	uint32_t bit;   // protocol::PoseHook005...
	ObserverHook *observer;
	PoseUpdateDetour observe;
};

const PoseHookBinding PoseHookBindings[] = {
	{ "IVRServerDriverHost_005", &TrackedDevicePoseUpdatedHook005,
		&DetourTrackedDevicePoseUpdated005, &PoseHook005Ready, protocol::PoseHook005,
		&Observer005, &ObserveTrackedDevicePoseUpdated005 },
	{ "IVRServerDriverHost_006", &TrackedDevicePoseUpdatedHook006,
		&DetourTrackedDevicePoseUpdated006, &PoseHook006Ready, protocol::PoseHook006,
		&Observer006, &ObserveTrackedDevicePoseUpdated006 },
};

// The driver context's GetGenericInterface slot, for the order check.
std::atomic<void **> ContextSlot{ nullptr };

// Reads without faulting: a decoded jump can point anywhere.
bool ReadMemory(uintptr_t address, void *out, size_t size)
{
	SIZE_T read = 0;
	return address != 0 && ReadProcessMemory(GetCurrentProcess(),
		reinterpret_cast<LPCVOID>(address), out, size, &read) != FALSE && read == size;
}

questcal::hookchain::Walk WalkFrom(uintptr_t start, uintptr_t detour)
{
	return questcal::hookchain::Follow(start, detour, ReadMemory);
}

// The protection that lets `protect`'s page be written and still run what it
// ran; 0 when it cannot be written that way.
DWORD WritableProtection(DWORD protect)
{
	if (protect & (PAGE_GUARD | PAGE_NOACCESS))
		return 0;
	switch (protect & 0xFF)
	{
	case PAGE_READONLY:
	case PAGE_READWRITE:
	case PAGE_WRITECOPY:
		return PAGE_READWRITE;
	case PAGE_EXECUTE_READ:
	case PAGE_EXECUTE_READWRITE:
	case PAGE_EXECUTE_WRITECOPY:
		return PAGE_EXECUTE_READWRITE;
	default:
		return 0;
	}
}

// Swaps `expected` in the vtable slot for `desired`: an aligned pointer store,
// so a caller reading the slot meanwhile gets one or the other.
bool ExchangeSlot(void **slot, void *expected, void *desired)
{
	MEMORY_BASIC_INFORMATION info{};
	if (!slot || (reinterpret_cast<uintptr_t>(slot) % alignof(void *)) != 0 ||
		VirtualQuery(slot, &info, sizeof info) != sizeof info || info.State != MEM_COMMIT)
		return false;
	const DWORD writable = WritableProtection(info.Protect);
	if (!writable)
		return false;
	DWORD previous = 0;
	if (!VirtualProtect(slot, sizeof(void *), writable, &previous))
		return false;
	const void *seen = InterlockedCompareExchangePointer(slot, desired, expected);
	DWORD ignored = 0;
	VirtualProtect(slot, sizeof(void *), previous, &ignored);
	return seen == expected;
}

bool DisableEveryHook()
{
	// The context hook goes first: while it is enabled it can still install a new
	// pose hook behind the ones being disabled here.
	bool success = GetGenericInterfaceHook.Disable();
	for (const PoseHookBinding &binding : PoseHookBindings)
		success = binding.hook->Disable() && success;
	return success;
}

void TryInstallPoseHook(const char *interfaceVersion, void *originalInterface)
{
	if (!AcceptingHookRequests.load(std::memory_order_acquire))
		return;

	const PoseHookBinding *binding = nullptr;
	for (const PoseHookBinding &candidate : PoseHookBindings)
	{
		if (std::strcmp(interfaceVersion, candidate.interfaceVersion) == 0)
		{
			binding = &candidate;
			break;
		}
	}
	if (!binding)
		return;

	if (!originalInterface)
	{
		LOG("Cannot hook %s: GetGenericInterface returned null", interfaceVersion);
		return;
	}

	// Under the mutex, accepting implies MinHook is initialized.
	std::lock_guard<std::mutex> lock(HookSetupMutex);
	if (!AcceptingHookRequests.load(std::memory_order_acquire))
		return;
#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
	if (TryInstallAfterAcceptCheckForTest)
		TryInstallAfterAcceptCheckForTest();
#endif

	if (binding->ready->load(std::memory_order_relaxed))
		return;
	if (binding->hook->CreateHookInObjectVTable(
		originalInterface, openvr_hook::PoseUpdateSlot, binding->detour))
	{
		if (!binding->observer->slot.load(std::memory_order_relaxed))
			binding->observer->slot.store(*static_cast<void ***>(originalInterface) +
				openvr_hook::PoseUpdateSlot, std::memory_order_release);
		binding->ready->store(true, std::memory_order_release);
	}
}

void *DetourGetGenericInterface(vr::IVRDriverContext *_this,
	const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	auto original = GetGenericInterfaceHook.originalFunc.load(std::memory_order_acquire);
	if (!original)
		return nullptr;

	if (!pchInterfaceVersion)
	{
		LOG("IVRDriverContext::GetGenericInterface called with a null version");
		if (peError)
			*peError = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}

	TRACE("ServerTrackedDeviceProvider::DetourGetGenericInterface(%s)", pchInterfaceVersion);
	void *originalInterface = original(_this, pchInterfaceVersion, peError);
	TryInstallPoseHook(pchInterfaceVersion, originalInterface);
	return originalInterface;
}

} // namespace

void ResendPose(const PoseHostCall &call, uint32_t device, const vr::DriverPose_t &pose)
{
	if (!call.host || !call.original)
		return;
	struct Forwarding
	{
		uint32_t outer;
		~Forwarding() { ForwardingDevice = outer; }
	} forwarding{ ForwardingDevice };
	ForwardingDevice = device;
	call.original(call.host, device, pose, sizeof(vr::DriverPose_t));
}

bool InjectHooks(ServerTrackedDeviceProvider *driver, vr::IVRDriverContext *pDriverContext)
{
	if (!pDriverContext)
	{
		LOG("InjectHooks: invalid driver context");
		return false;
	}

	std::lock_guard<std::mutex> lock(HookSetupMutex);
	if (HooksInstalled)
	{
		LOG("InjectHooks: hooks are already installed");
		return false;
	}
	HookedDevices.store(0, std::memory_order_relaxed);
	MismatchedPoseUpdates.store(0, std::memory_order_relaxed);
	ReentrantPoseUpdates.store(0, std::memory_order_relaxed);
	ObservedPoseUpdates.store(0, std::memory_order_relaxed);
	UnobservedPoseUpdates.store(0, std::memory_order_relaxed);
	{
		std::lock_guard<std::mutex> status(StatusMutex);
		OrderChecks = ForeignAheadMask = ForeignAheadSeenMask = ObserverFailedMask = 0;
		ForeignModule[0] = '\0';
	}
	// Once per process; a later Init enables the hooks the first one created.
	if (!MinHookInitialized)
	{
		if (!PinThisModule())
		{
			LOG("InjectHooks: could not pin the driver module (error %u)", GetLastError());
			return false;
		}
		MH_STATUS err = MH_Initialize();
		if (err != MH_OK)
		{
			LOG("MH_Initialize error: %s", MH_StatusToString(err));
			return false;
		}
		MinHookInitialized = true;
	}
	Driver.store(driver, std::memory_order_release);

	if (!GetGenericInterfaceHook.CreateHookInObjectVTable(
		pDriverContext, openvr_hook::GetGenericInterfaceSlot, &DetourGetGenericInterface))
	{
		Driver.store(nullptr, std::memory_order_release);
		return false;
	}

	if (!ContextSlot.load(std::memory_order_relaxed))
		ContextSlot.store(*reinterpret_cast<void ***>(pDriverContext) +
			openvr_hook::GetGenericInterfaceSlot, std::memory_order_release);
	HooksInstalled = true;
	AcceptingHookRequests.store(true, std::memory_order_release);
	return true;
}

bool IsPoseUpdateHookInstalled()
{
	return PoseUpdateHookMask() != 0;
}

uint32_t PoseUpdateHookMask()
{
	uint32_t mask = 0;
	if (PoseHook005Ready.load(std::memory_order_acquire))
		mask |= protocol::PoseHook005;
	if (PoseHook006Ready.load(std::memory_order_acquire))
		mask |= protocol::PoseHook006;
	return mask;
}

protocol::HookStatus PoseHookStatus()
{
	protocol::HookStatus status;
	status.hookedDevices = HookedDevices.load(std::memory_order_relaxed);
	status.mismatchedPoseUpdates = MismatchedPoseUpdates.load(std::memory_order_relaxed);
	status.reentrantPoseUpdates = ReentrantPoseUpdates.load(std::memory_order_relaxed);
	status.observedPoseUpdates = ObservedPoseUpdates.load(std::memory_order_relaxed);
	status.unobservedPoseUpdates = UnobservedPoseUpdates.load(std::memory_order_relaxed);
	for (const PoseHookBinding &binding : PoseHookBindings)
		if (binding.observer->active.load(std::memory_order_acquire))
			status.observerMask |= binding.bit;
	std::lock_guard<std::mutex> lock(StatusMutex);
	status.orderChecks = OrderChecks;
	status.foreignAheadMask = ForeignAheadMask;
	status.foreignAheadSeenMask = ForeignAheadSeenMask;
	status.observerFailedMask = ObserverFailedMask;
	std::memcpy(status.foreignModule, ForeignModule, sizeof status.foreignModule);
	return status;
}

namespace
{
// The file name of the module `address` lies in, UTF-8, cut to fit `out`
// with its NUL; empty when it lies in none.
void ModuleFileName(uintptr_t address, char (&out)[sizeof ForeignModule])
{
	out[0] = '\0';
	HMODULE module = nullptr;
	if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
		GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
		reinterpret_cast<LPCWSTR>(address), &module) || !module)
		return;
	wchar_t path[MAX_PATH] = {};
	const DWORD length = GetModuleFileNameW(module, path, MAX_PATH);
	if (length == 0 || length >= MAX_PATH)
		return;
	const wchar_t *name = path;
	for (const wchar_t *c = path; *c; ++c)
		if (*c == L'\\' || *c == L'/')
			name = c + 1;
	char utf8[MAX_PATH * 3] = {};
	if (WideCharToMultiByte(CP_UTF8, 0, name, -1, utf8, sizeof utf8, nullptr, nullptr) <= 0)
		return;
	// Cut on a character boundary, never inside a UTF-8 sequence.
	size_t keep = std::strlen(utf8);
	if (keep >= sizeof out)
	{
		keep = sizeof out - 1;
		while (keep > 0 && (static_cast<unsigned char>(utf8[keep]) & 0xC0) == 0x80)
			--keep;
	}
	std::memcpy(out, utf8, keep);
	out[keep] = '\0';
}

// Places the observer in front of `current`, the code its walk found ahead.
// Called with HookSetupMutex held.
bool PlaceObserver(const PoseHookBinding &binding, void **slot, void *current)
{
	ObserverHook &observer = *binding.observer;
	observer.next.store(reinterpret_cast<PoseUpdateDetour>(current), std::memory_order_release);
	if (!ExchangeSlot(slot, current, reinterpret_cast<void *>(binding.observe)))
		return false;
	observer.inSlot = true;
	// Only now: until the observer is reached, the pose hook records every pose
	// itself. A pose the observer saw before this store is recorded by neither,
	// never by both.
	observer.active.store(true, std::memory_order_release);
	return true;
}
} // namespace

void RefreshPoseHookOrder()
{
	uint32_t ahead = 0;
	uint32_t failed = 0;
	uint32_t placed = 0;
	uintptr_t firstAhead = 0;
	{
		std::lock_guard<std::mutex> lock(HookSetupMutex);
		if (!HooksInstalled)
			return;
		auto noteAhead = [&](uint32_t bit, uintptr_t end) {
			ahead |= bit;
			// The pose hooks' code ahead is the one worth naming.
			if (!firstAhead || bit != protocol::ContextHook)
				firstAhead = end;
		};

		void **contextSlot = ContextSlot.load(std::memory_order_acquire);
		uint64_t contextTarget = 0;
		if (contextSlot && ReadMemory(reinterpret_cast<uintptr_t>(contextSlot),
			&contextTarget, sizeof contextTarget))
		{
			const auto walk = WalkFrom(static_cast<uintptr_t>(contextTarget),
				reinterpret_cast<uintptr_t>(&DetourGetGenericInterface));
			if (!walk.reachedDetour)
				noteAhead(protocol::ContextHook, walk.end);
		}

		for (const PoseHookBinding &binding : PoseHookBindings)
		{
			ObserverHook &observer = *binding.observer;
			void **slot = observer.slot.load(std::memory_order_acquire);
			if (!binding.ready->load(std::memory_order_acquire) || !slot)
				continue;
			uint64_t current = 0;
			if (!ReadMemory(reinterpret_cast<uintptr_t>(slot), &current, sizeof current))
				continue;
			const uintptr_t detour = reinterpret_cast<uintptr_t>(binding.detour);
			const uintptr_t observe = reinterpret_cast<uintptr_t>(binding.observe);
			if (current == observe)
			{
				// The observer is first; the code behind it may since have gone.
				const auto behind = WalkFrom(reinterpret_cast<uintptr_t>(
					observer.next.load(std::memory_order_acquire)), detour);
				if (!behind.reachedDetour)
					noteAhead(binding.bit, behind.end);
				if (!observer.active.load(std::memory_order_relaxed))
					observer.active.store(true, std::memory_order_release);
				continue;
			}

			const auto walk = WalkFrom(static_cast<uintptr_t>(current), detour);
			if (walk.reachedDetour)
			{
				if (observer.inSlot)
				{
					// What replaced the observer reaches the pose hook only by
					// jumps (a hook removed by restoring what it found): nothing
					// runs ahead now, and the pose hook records again.
					observer.active.store(false, std::memory_order_release);
					observer.inSlot = false;
					observer.stranded = true;
				}
				continue;
			}
			noteAhead(binding.bit, walk.end);
			if (observer.inSlot || observer.stranded)
			{
				// Code took the slot after the observer. It may call the
				// observer, so the observer is never placed on top of it.
				failed |= binding.bit;
				continue;
			}
			if (PlaceObserver(binding, slot, reinterpret_cast<void *>(current)))
				placed |= binding.bit;
			else
				failed |= binding.bit;
		}
	}

	char module[sizeof ForeignModule] = {};
	if (firstAhead)
		ModuleFileName(firstAhead, module);
	std::lock_guard<std::mutex> status(StatusMutex);
	++OrderChecks;
	const uint32_t newlyAhead = ahead & ~ForeignAheadSeenMask;
	const uint32_t newlyFailed = failed & ~ObserverFailedMask;
	ForeignAheadMask = ahead;
	ForeignAheadSeenMask |= ahead;
	// Kept once it happened: what the pose hook recorded meanwhile was not raw.
	ObserverFailedMask |= failed;
	if (firstAhead)
		std::memcpy(ForeignModule, module, sizeof ForeignModule);
	if (newlyAhead)
		LOG("Code in %s runs ahead of hook mask 0x%x (pose hooks 1 and 2, driver context 4)",
			module[0] ? module : "no module", newlyAhead);
	if (placed)
		LOG("Recording raw poses from the host vtable, ahead of that code, for pose hook mask 0x%x",
			placed);
	if (newlyFailed)
		LOG("Could not record raw poses ahead of that code for pose hook mask 0x%x", newlyFailed);
}

bool DisableHooks()
{
	// Stop detours from starting new hook creation before waiting on the setup
	// mutex. A detour already waiting for the mutex rechecks this flag.
	AcceptingHookRequests.store(false, std::memory_order_release);
	// Sequentially consistent, like the callbacks' count and their load of
	// Driver: once the wait below finds no callback counted, every later
	// callback finds no driver.
	Driver.store(nullptr, std::memory_order_seq_cst);

	{
		std::lock_guard<std::mutex> lock(HookSetupMutex);
		// Under the mutex, not before it: a detour that passed its accept
		// check before the store above sets its ready flag while holding
		// the mutex. Cleared earlier, that flag outlives the hook it names,
		// and the next Init in this process skips installing the hook while
		// IsPoseUpdateHookInstalled still reports it.
		for (const PoseHookBinding &binding : PoseHookBindings)
			binding.ready->store(false, std::memory_order_release);
		HooksInstalled = false;
		// The observers stop recording, and leave their slots when nothing has
		// taken them since. A thread still in one only forwards, to code that
		// stays where it is.
		for (const PoseHookBinding &binding : PoseHookBindings)
		{
			ObserverHook &observer = *binding.observer;
			observer.active.store(false, std::memory_order_release);
			if (!observer.inSlot)
				continue;
			observer.inSlot = false;
			if (!ExchangeSlot(observer.slot.load(std::memory_order_acquire),
				reinterpret_cast<void *>(binding.observe),
				reinterpret_cast<void *>(observer.next.load(std::memory_order_acquire))))
			{
				observer.stranded = true;
				LOG("A pose observation hook could not leave the host vtable; it stays and only forwards");
			}
		}
		if (MinHookInitialized && !DisableEveryHook())
		{
			LOG("One or more hooks could not be disabled individually; disabling all MinHook targets");
			MH_STATUS error = MH_DisableHook(MH_ALL_HOOKS);
			if (error != MH_OK && error != MH_ERROR_DISABLED)
				LOG("Failed to disable all MinHook targets: %s; their detours stay in place and only forward",
					MH_StatusToString(error));
			else
				DisableEveryHook();   // synchronize each hook's lifecycle state
		}
	}

	// A thread still inside a detour finishes safely (see PinThisModule); only
	// the driver it may have reached needs waiting for.
#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
	if (BeforeDriverWaitForTest)
		BeforeDriverWaitForTest();
#endif
	if (ActiveCallbacks.WaitUntilIdle(DriverReleaseWait))
		return true;
	LOG("A pose callback was still inside the driver after %lld ms",
		static_cast<long long>(DriverReleaseWait.count()));
	return false;
}
