#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace questcal
{

// The module the driver found running ahead of its hooks
// (protocol::HookStatus::foreignModule): the bytes up to the first NUL inside
// the array, without control characters, so a peer's unterminated or odd bytes
// can neither run past it nor break a log line.
inline std::string ForeignModuleFile(const char *bytes, size_t size)
{
	std::string file;
	for (size_t i = 0; i < size && bytes[i] != '\0'; ++i)
	{
		const unsigned char c = static_cast<unsigned char>(bytes[i]);
		if (c >= 0x20 && c != 0x7F)
			file += static_cast<char>(c);
	}
	return file;
}

// The SteamVR driver a module file is, as the player knows it:
// driver_SmoothTracking.dll is "SmoothTracking". Empty when the file is not a
// driver's (driver_<name>.dll, any case).
inline std::string ForeignDriverName(const std::string &file)
{
	auto lower = [](std::string text) {
		for (char &c : text)
			if (c >= 'A' && c <= 'Z')
				c = static_cast<char>(c - 'A' + 'a');
		return text;
	};
	const std::string prefix = "driver_";
	const std::string suffix = ".dll";
	if (file.size() <= prefix.size() + suffix.size())
		return std::string();
	const std::string folded = lower(file);
	if (folded.compare(0, prefix.size(), prefix) != 0 ||
		folded.compare(folded.size() - suffix.size(), suffix.size(), suffix) != 0)
		return std::string();
	return file.substr(prefix.size(), file.size() - prefix.size() - suffix.size());
}

// Target devices SteamVR tracks whose poses have not passed the driver's pose
// hook, so no calibration reaches them: another driver resolved its host
// interface first, or SteamVR uses a host version the driver does not hook.
// The driver reports the devices it has seen at each handshake, about once a
// second, so a device that started tracking just after one handshake would
// look missing until the next; only one still missing after BypassSyncs
// synchronizations in a row is reported.
struct HookCoverage
{
	// One bit per OpenVR id: vr::k_unMaxTrackedDeviceCount.
	static constexpr uint32_t Devices = 64;
	static constexpr uint8_t BypassSyncs = 3;
	std::array<uint8_t, Devices> missedSyncs{};

	// Returns the devices to report.
	uint64_t Note(uint64_t hookedDevices, uint64_t trackedDevices, uint64_t targetDevices)
	{
		uint64_t bypassing = 0;
		for (uint32_t id = 0; id < Devices; ++id)
		{
			const uint64_t bit = uint64_t{ 1 } << id;
			const bool missing = (targetDevices & bit) != 0 && (trackedDevices & bit) != 0 &&
				(hookedDevices & bit) == 0;
			missedSyncs[id] = missing
				? static_cast<uint8_t>((std::min)(missedSyncs[id] + 1, int{ BypassSyncs }))
				: uint8_t{ 0 };
			if (missedSyncs[id] >= BypassSyncs)
				bypassing |= bit;
		}
		return bypassing;
	}
};

} // namespace questcal
