#include "Logging.h"
#include "../common/Version.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdio>

// vrserver is not the only process that loads the driver: SteamVR's watchdog
// loads it in another process, for hours with no session (2026-10-22: loaded
// at 10:12, unloaded at 23:38, no SteamVR that day). Both write to one log, so
// the load lines say whose they are. Kernel32 only, as DllMain allows.
static void HostProcess(char *text, size_t size)
{
	char path[MAX_PATH];
	const DWORD length = GetModuleFileNameA(nullptr, path, MAX_PATH);
	const char *name = "unknown process";
	if (length > 0 && length < MAX_PATH)
	{
		name = path;
		for (DWORD i = 0; i < length; ++i)
			if (path[i] == '\\' || path[i] == '/')
				name = path + i + 1;
	}
	snprintf(text, size, "%s, pid %lu", name, static_cast<unsigned long>(GetCurrentProcessId()));
}

BOOL APIENTRY DllMain(HMODULE /*hModule*/, DWORD ul_reason_for_call, LPVOID /*lpReserved*/)
{
	char host[MAX_PATH + 32];
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		OpenLogFile();
		HostProcess(host, sizeof host);
		LOG("Nova Calibrator driver " QUESTCAL_VERSION_STRING " loaded in %s", host);
		break;
	case DLL_PROCESS_DETACH:
		HostProcess(host, sizeof host);
		LOG("Nova Calibrator driver unloaded from %s", host);
		break;
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
		break;
	}
	return TRUE;
}
