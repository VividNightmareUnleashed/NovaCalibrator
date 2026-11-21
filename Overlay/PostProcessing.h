#pragma once

#include "../common/Protocol.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

// The Post Processing page's settings, and what they ask of the driver
// (Driver/PoseFilter.h runs the filters). Plain values, so the page, the
// settings record, the driver request and the test harness share one account
// of which device is filtered and how.
//
// Three filters:
//  - "One Euro": a low-pass filter whose cutoff rises with the speed the
//    device's driver reports. Three strengths.
//  - "Soften jumps": a correction the device's driver makes, and a jump
//    that the velocities it reports do not account for, is let out over a
//    tenth of a second or less.
//  - "Classic": a part of a device's reported velocities is taken off, as the
//    add-on OpenVR-SmoothTracking does, so the runtime predicts less.
// The first two act on the devices the player picked, never the headset or
// the headset tracker. "Classic" has one value for each device, the headset
// included.
namespace questcal
{
namespace postprocess
{

enum class Strength { Off = 0, Light, Medium, Strong };
constexpr int StrengthCount = 4;

// The page's switch is off until the player sets it: the filters change what
// every game sees, so they start only when asked for.
constexpr bool EnabledByDefault = false;
// The most device choices a record keeps, as for the device names.
constexpr size_t MaxDeviceChoices = 64;
constexpr size_t MaxSerialBytes = 128;
// "Classic" moves in steps of this, from 0 (nothing) to 100 (no prediction).
constexpr int ClassicStepPercent = 5;
// How long "Soften jumps" takes to let a correction out.
constexpr double ReleaseSeconds = 0.1;

struct Settings
{
	bool enabled = EnabledByDefault;
	Strength oneEuro = Strength::Medium;
	bool softenJumps = true;
	// Whether "One Euro" and "Soften jumps" act on a device, by serial. Only
	// the player's own choices: a device with no entry follows DefaultFiltered.
	std::map<std::string, bool> filtered;
	// "Classic": the percent taken off a device's velocities, by serial. Only
	// values above 0.
	std::map<std::string, int> classic;

	bool operator==(const Settings &other) const
	{
		return enabled == other.enabled && oneEuro == other.oneEuro && softenJumps == other.softenJumps &&
			filtered == other.filtered && classic == other.classic;
	}
	bool operator!=(const Settings &other) const { return !(*this == other); }
};

// What a device is to the filters.
enum class DeviceKind
{
	Other,           // base stations, virtual trackers, anything else: never touched
	Headset,
	HeadsetTracker,  // continuous calibration's tracker: never touched
	Tracker,
	Controller,
};

struct Device
{
	uint32_t id = 0;
	DeviceKind kind = DeviceKind::Other;
	std::string serial;
};

// What SteamVR's class makes a device. A body estimator's virtual trackers
// repeat real ones, which are filtered themselves; `headsetTracker` is the
// tracker continuous calibration reads.
inline DeviceKind KindOf(vr::ETrackedDeviceClass deviceClass, bool virtualSystem, bool headsetTracker)
{
	if (virtualSystem)
		return DeviceKind::Other;
	switch (deviceClass)
	{
	case vr::TrackedDeviceClass_HMD: return DeviceKind::Headset;
	case vr::TrackedDeviceClass_Controller: return DeviceKind::Controller;
	case vr::TrackedDeviceClass_GenericTracker:
		return headsetTracker ? DeviceKind::HeadsetTracker : DeviceKind::Tracker;
	default: return DeviceKind::Other;
	}
}

// "One Euro" and "Soften jumps" are for what the player wears and holds.
inline bool TakesFilters(DeviceKind kind)
{
	return kind == DeviceKind::Tracker || kind == DeviceKind::Controller;
}

// "Classic" takes the headset too, as the add-on it follows does.
inline bool TakesClassic(DeviceKind kind)
{
	return kind == DeviceKind::Headset || TakesFilters(kind);
}

// Body trackers are filtered until the player says otherwise; a controller
// is not, because a hand notices any change first.
inline bool DefaultFiltered(DeviceKind kind)
{
	return kind == DeviceKind::Tracker;
}

inline bool ValidSerial(const std::string &serial)
{
	return !serial.empty() && serial.size() <= MaxSerialBytes;
}

inline int ClampClassicPercent(int percent)
{
	const int clamped = (std::max)(0, (std::min)(100, percent));
	return (clamped + ClassicStepPercent / 2) / ClassicStepPercent * ClassicStepPercent;
}

// Whether the two pose filters act on this device when they are on.
inline bool Filtered(const Settings &settings, const Device &device)
{
	if (!TakesFilters(device.kind) || !ValidSerial(device.serial))
		return false;
	const auto choice = settings.filtered.find(device.serial);
	return choice != settings.filtered.end() ? choice->second : DefaultFiltered(device.kind);
}

inline int ClassicPercent(const Settings &settings, const Device &device)
{
	if (!TakesClassic(device.kind) || !ValidSerial(device.serial))
		return 0;
	const auto value = settings.classic.find(device.serial);
	return value != settings.classic.end() ? ClampClassicPercent(value->second) : 0;
}

// "Classic" is on when it takes anything off a device that is here.
inline bool ClassicOn(const Settings &settings, const std::vector<Device> &devices)
{
	for (const Device &device : devices)
		if (ClassicPercent(settings, device) > 0)
			return true;
	return false;
}

// Records the player's choice for a device; false when the record is full
// or the serial cannot be a key.
inline bool SetFiltered(Settings &settings, const std::string &serial, bool filtered)
{
	if (!ValidSerial(serial))
		return false;
	const auto existing = settings.filtered.find(serial);
	if (existing == settings.filtered.end() && settings.filtered.size() >= MaxDeviceChoices)
		return false;
	settings.filtered[serial] = filtered;
	return true;
}

inline bool SetClassicPercent(Settings &settings, const std::string &serial, int percent)
{
	if (!ValidSerial(serial))
		return false;
	const int value = ClampClassicPercent(percent);
	if (value == 0)
	{
		settings.classic.erase(serial);
		return true;
	}
	const auto existing = settings.classic.find(serial);
	if (existing == settings.classic.end() && settings.classic.size() >= MaxDeviceChoices)
		return false;
	settings.classic[serial] = value;
	return true;
}

// The filter each strength is. Measured on a recorded lighthouse session
// (four devices): Light leaves 53% to 60% of the shake at rest for 0.2 to
// 0.4 ms of delay in movement, Medium 34% to 42% for 1.1 to 1.6 ms, Strong
// 19% to 26% for 3.3 to 4.6 ms. Off keeps Medium's numbers with the filter
// not running, so every value sent is one the driver accepts.
inline protocol::PoseFilter FilterFor(Strength strength, bool softenJumps)
{
	protocol::PoseFilter filter;
	switch (strength)
	{
	case Strength::Light:
		filter.minCutoffHz = 2.0;
		filter.speedGain = 320.0;
		filter.turnMinCutoffHz = 2.0;
		filter.turnGain = 200.0;
		break;
	case Strength::Strong:
		filter.minCutoffHz = 0.5;
		filter.speedGain = 80.0;
		filter.turnMinCutoffHz = 0.5;
		filter.turnGain = 50.0;
		break;
	case Strength::Off:
	case Strength::Medium:
		filter.minCutoffHz = 1.0;
		filter.speedGain = 160.0;
		filter.turnMinCutoffHz = 1.0;
		filter.turnGain = 100.0;
		break;
	}
	filter.smooth = strength != Strength::Off ? 1u : 0u;
	filter.releaseCorrections = softenJumps ? 1u : 0u;
	filter.releaseSeconds = ReleaseSeconds;
	return filter;
}

// What the driver is asked for (protocol::SetRuntimeState's filter fields).
struct DriverPlan
{
	protocol::PoseFilter filter;
	uint64_t filterMask = 0;
	std::array<double, vr::k_unMaxTrackedDeviceCount> predictionCut{};

	bool operator==(const DriverPlan &other) const
	{
		return filter == other.filter && filterMask == other.filterMask && predictionCut == other.predictionCut;
	}
};

// `active` is the module installed and the page's switch on. `classicPaused`
// is another add-on already taking the prediction down (ClassicWaits below):
// "Classic" then takes nothing more off.
inline DriverPlan PlanFor(const Settings &settings, const std::vector<Device> &devices, bool active,
	bool classicPaused)
{
	DriverPlan plan;
	if (!active || !settings.enabled)
		return plan;
	plan.filter = FilterFor(settings.oneEuro, settings.softenJumps);
	const bool filters = plan.filter.smooth != 0 || plan.filter.releaseCorrections != 0;
	for (const Device &device : devices)
	{
		if (device.id >= vr::k_unMaxTrackedDeviceCount)
			continue;
		if (filters && Filtered(settings, device))
			plan.filterMask |= uint64_t{ 1 } << device.id;
		if (!classicPaused)
			plan.predictionCut[device.id] = ClassicPercent(settings, device) / 100.0;
	}
	return plan;
}

// The add-on whose velocity scaling "Classic" repeats, by its SteamVR driver
// name (driver_SmoothTracking.dll is "SmoothTracking"), any case.
inline bool IsSmoothTrackingDriver(const std::string &driverName)
{
	static const char known[] = "smoothtracking";
	if (driverName.size() != sizeof known - 1)
		return false;
	for (size_t i = 0; i < driverName.size(); ++i)
	{
		const char c = driverName[i];
		if ((c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c) != known[i])
			return false;
	}
	return true;
}

// What other add-ons' code ahead of the driver's pose hook means for the
// filters, from the driver's hook status (protocol::HookStatus).
struct OtherAddOns
{
	// Their post-processing has no effect on a device this app filters: none
	// is ahead, or the driver holds each pose as the device's own driver sent
	// it and gives a filtered device that motion back.
	bool noEffect = true;
	// OpenVR-SmoothTracking's is the code ahead.
	bool smoothTracking = false;

	bool operator==(const OtherAddOns &other) const
	{
		return noEffect == other.noEffect && smoothTracking == other.smoothTracking;
	}
};

// The driver holds a pose as its own driver sent it where an observation hook
// is ahead of the code ahead of that pose hook and was never overtaken there.
// `foreignDriverName` is the SteamVR driver the first code ahead belongs to.
inline OtherAddOns ReadOtherAddOns(uint32_t foreignAheadMask, uint32_t observerMask, uint32_t observerFailedMask,
	const std::string &foreignDriverName)
{
	const uint32_t ahead = foreignAheadMask & (protocol::PoseHook005 | protocol::PoseHook006);
	OtherAddOns found;
	found.noEffect = (ahead & ~observerMask) == 0 && (ahead & observerFailedMask) == 0;
	found.smoothTracking = ahead != 0 && IsSmoothTrackingDriver(foreignDriverName);
	return found;
}

// OpenVR-SmoothTracking still takes the prediction down on the poses "Classic"
// would act on, so "Classic" takes nothing more off.
inline bool ClassicWaits(const OtherAddOns &others)
{
	return others.smoothTracking && !others.noEffect;
}

} // namespace postprocess
} // namespace questcal
