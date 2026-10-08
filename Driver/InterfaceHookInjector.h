#pragma once

#include "../common/Protocol.h"

#include <openvr_driver.h>

class ServerTrackedDeviceProvider;

// How a device's own driver reached the pose hook: the host object it called,
// which SteamVR attributes the pose to, and the original behind the detour it
// came through. Kept so a pose can later be re-sent as that driver would.
struct PoseHostCall
{
	void *host = nullptr;
	void (*original)(void *, uint32_t, const vr::DriverPose_t &, uint32_t) = nullptr;
};

// Re-sends `pose` for `device` through `call`, marked as already handled for
// this thread: a host that forwards one interface version to another hooked one
// passes it on untouched instead of handing it back as a new raw pose.
void ResendPose(const PoseHostCall &call, uint32_t device, const vr::DriverPose_t &pose);

// Installs the driver-context hook. ServerTrackedDeviceProvider::Init invokes
// OpenVR context initialization through it, which must in turn install at least
// one supported pose hook before initialization is considered successful.
bool InjectHooks(ServerTrackedDeviceProvider *driver, vr::IVRDriverContext *pDriverContext);
bool IsPoseUpdateHookInstalled();
uint32_t PoseUpdateHookMask();
// What the pose hook has seen since InjectHooks.
protocol::HookStatus PoseHookStatus();
// Disables every hook and returns true once no pose callback can still reach
// the driver. The detours themselves stay in place, pinned with this module,
// and only forward. False means a callback was still inside the driver when
// the wait ran out: what it reads (the pose ring above all) must not be
// released by the caller.
bool DisableHooks();

#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
// Called inside TryInstallPoseHook with the setup mutex held, after the accept
// recheck and before the ready flag is read.
extern void (*TryInstallAfterAcceptCheckForTest)();
// Called inside a pose detour once it has found the driver, before handing it
// the pose.
extern void (*InsideDriverCallbackForTest)();
// Called in DisableHooks once the hooks are disabled, before it waits for the
// pose callbacks inside the driver.
extern void (*BeforeDriverWaitForTest)();
#endif
