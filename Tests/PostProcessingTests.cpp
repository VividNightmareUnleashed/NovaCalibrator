#include "../Overlay/HookCoverage.h"
#include "../Overlay/SettingsRecordJson.h"
#include "../Driver/ProtocolValidation.h"

#include <cstdint>
#include <cstdio>
#include <exception>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

// The Post Processing page's settings (Overlay/PostProcessing.h): which
// device each filter acts on, what the driver is asked for, and how the
// settings are saved.
namespace
{
using Check = void (*)(const char *, bool, const char *);
using namespace questcal::postprocess;

// A headset, the headset tracker, two body trackers, two controllers, a base
// station and a device past the last OpenVR id.
std::vector<Device> Room()
{
	return {
		{ 0, DeviceKind::Headset, "HMD-1" },
		{ 1, DeviceKind::HeadsetTracker, "LHR-MOUNT" },
		{ 2, DeviceKind::Tracker, "LHR-HIP" },
		{ 3, DeviceKind::Tracker, "LHR-FOOT" },
		{ 4, DeviceKind::Controller, "LHR-LEFT" },
		{ 5, DeviceKind::Controller, "LHR-RIGHT" },
		{ 6, DeviceKind::Other, "LHB-BASE" },
		{ 64, DeviceKind::Tracker, "LHR-BEYOND" },
	};
}

uint64_t Bits(std::initializer_list<uint32_t> ids)
{
	uint64_t mask = 0;
	for (uint32_t id : ids)
		mask |= uint64_t{ 1 } << id;
	return mask;
}

// What a new install has, and which device a filter may act on.
void DeviceScenarios(Check check)
{
	char detail[240];
	const std::vector<Device> room = Room();
	Settings settings;
	int filtered = 0, classic = 0;
	for (const Device &device : room)
	{
		filtered += Filtered(settings, device);
		classic += TakesClassic(device.kind);
	}
	snprintf(detail, sizeof detail, "switch %d, strength %d, soften %d; %d of 8 devices filtered, %d take \"Classic\"",
		settings.enabled, static_cast<int>(settings.oneEuro), settings.softenJumps, filtered, classic);
	check("post processing: a new install has the switch off, and the body trackers picked for the day it goes on",
		!settings.enabled && settings.oneEuro == Strength::Medium && settings.softenJumps && filtered == 3 &&
			Filtered(settings, room[2]) && Filtered(settings, room[3]) && !Filtered(settings, room[4]) &&
			!Filtered(settings, room[0]) && !Filtered(settings, room[1]) && !Filtered(settings, room[6]) &&
			classic == 6 && !ClassicOn(settings, room), detail);

	// The player's choices win over the defaults, for the devices that can
	// take them, and the headset tracker takes none whatever is stored.
	const bool set = SetFiltered(settings, "LHR-LEFT", true) && SetFiltered(settings, "LHR-HIP", false) &&
		SetFiltered(settings, "LHR-MOUNT", true) && SetFiltered(settings, "HMD-1", true) &&
		SetClassicPercent(settings, "HMD-1", 20) && SetClassicPercent(settings, "LHR-FOOT", 47) &&
		SetClassicPercent(settings, "LHR-MOUNT", 80) && SetClassicPercent(settings, "LHR-LEFT", 3) &&
		SetClassicPercent(settings, "LHR-RIGHT", 250);
	const bool refused = !SetFiltered(settings, "", true) && !SetClassicPercent(settings, std::string(MaxSerialBytes + 1, 'x'), 50);
	snprintf(detail, sizeof detail, "hip %d, left %d, mount %d, headset %d; classic headset %d, foot %d, mount %d, left %d, "
		"right %d; %zu values kept", Filtered(settings, room[2]), Filtered(settings, room[4]), Filtered(settings, room[1]),
		Filtered(settings, room[0]), ClassicPercent(settings, room[0]), ClassicPercent(settings, room[3]),
		ClassicPercent(settings, room[1]), ClassicPercent(settings, room[4]), ClassicPercent(settings, room[5]),
		settings.classic.size());
	check("post processing: a choice wins over the default, a value keeps to its steps, and the headset tracker takes neither",
		set && refused && !Filtered(settings, room[2]) && Filtered(settings, room[4]) && !Filtered(settings, room[1]) &&
			!Filtered(settings, room[0]) && ClassicPercent(settings, room[0]) == 20 &&
			ClassicPercent(settings, room[3]) == 45 && ClassicPercent(settings, room[1]) == 0 &&
			ClassicPercent(settings, room[4]) == 5 && ClassicPercent(settings, room[5]) == 100 &&
			ClassicOn(settings, room) && SetClassicPercent(settings, "LHR-LEFT", 0) && settings.classic.count("LHR-LEFT") == 0,
		detail);

	// A full record takes no new device, and still changes one it has.
	Settings full;
	for (size_t i = 0; i < MaxDeviceChoices; ++i)
	{
		SetFiltered(full, "T" + std::to_string(i), true);
		SetClassicPercent(full, "T" + std::to_string(i), 50);
	}
	const bool bounded = !SetFiltered(full, "ONE-MORE", true) && !SetClassicPercent(full, "ONE-MORE", 50) &&
		SetFiltered(full, "T0", false) && SetClassicPercent(full, "T0", 25) && SetClassicPercent(full, "ONE-MORE", 0);
	snprintf(detail, sizeof detail, "%zu choices and %zu values of at most %zu", full.filtered.size(), full.classic.size(),
		MaxDeviceChoices);
	check("post processing: the record keeps a bounded number of devices",
		bounded && full.filtered.size() == MaxDeviceChoices && full.classic.size() == MaxDeviceChoices && !full.filtered.at("T0") &&
			full.classic.at("T0") == 25, detail);
}

// What the driver is asked for.
void PlanScenarios(Check check)
{
	char detail[320];
	const std::vector<Device> room = Room();
	Settings settings;
	SetClassicPercent(settings, "HMD-1", 20);
	SetClassicPercent(settings, "LHR-FOOT", 50);
	SetClassicPercent(settings, "LHR-MOUNT", 80);
	SetClassicPercent(settings, "LHR-BEYOND", 80);

	// Nothing while the switch is off or the module is not installed.
	const DriverPlan off = PlanFor(settings, room, true, false);
	settings.enabled = true;
	const DriverPlan noModule = PlanFor(settings, room, false, false);
	const DriverPlan on = PlanFor(settings, room, true, false);
	double cuts = 0.0;
	for (double cut : on.predictionCut)
		cuts += cut;
	snprintf(detail, sizeof detail, "switch off: mask 0x%llx; no module: mask 0x%llx; on: mask 0x%llx, smooth %u, release %u, "
		"cuts headset %.2f foot %.2f mount %.2f, %.2f in all", static_cast<unsigned long long>(off.filterMask),
		static_cast<unsigned long long>(noModule.filterMask), static_cast<unsigned long long>(on.filterMask), on.filter.smooth,
		on.filter.releaseCorrections, on.predictionCut[0], on.predictionCut[3], on.predictionCut[1], cuts);
	check("post processing: the driver is asked for nothing until the module is installed and the switch is on",
		off == DriverPlan() && noModule == DriverPlan() && on.filterMask == Bits({ 2, 3 }) && on.filter.smooth == 1 &&
			on.filter.releaseCorrections == 1 && on.filter.releaseSeconds == ReleaseSeconds && on.predictionCut[0] == 0.2 &&
			on.predictionCut[3] == 0.5 && on.predictionCut[1] == 0.0 && cuts == 0.7, detail);

	// Each strength is the filter measured for it, inside the limits the
	// driver holds a state to.
	const double expected[StrengthCount][4] = {
		{ 1.0, 160.0, 1.0, 100.0 }, { 2.0, 320.0, 2.0, 200.0 }, { 1.0, 160.0, 1.0, 100.0 }, { 0.5, 80.0, 0.5, 50.0 } };
	int matched = 0, accepted = 0;
	for (int level = 0; level < StrengthCount; ++level)
		for (int soften = 0; soften < 2; ++soften)
		{
			settings.oneEuro = static_cast<Strength>(level);
			settings.softenJumps = soften != 0;
			const DriverPlan plan = PlanFor(settings, room, true, false);
			matched += plan.filter.minCutoffHz == expected[level][0] && plan.filter.speedGain == expected[level][1] &&
				plan.filter.turnMinCutoffHz == expected[level][2] && plan.filter.turnGain == expected[level][3] &&
				plan.filter.smooth == (level != 0 ? 1u : 0u) && plan.filter.releaseCorrections == (soften ? 1u : 0u) &&
				// With neither filter running no device is marked.
				plan.filterMask == (level != 0 || soften ? Bits({ 2, 3 }) : 0);
			protocol::SetRuntimeState state;
			state.filter = plan.filter;
			state.filterMask = plan.filterMask;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
				state.predictionCut[id] = plan.predictionCut[id];
			questcal::driverinput::PoseFilterState validated;
			accepted += questcal::driverinput::ValidatePoseFilters(state, validated) && validated.filterMask == plan.filterMask;
		}
	snprintf(detail, sizeof detail, "%d of 8 plans are the measured filter, and the driver accepts %d", matched, accepted);
	check("post processing: each strength asks for its measured filter, and the driver accepts every plan",
		matched == 8 && accepted == 8, detail);

	// While OpenVR-SmoothTracking runs ahead, "Classic" takes nothing more off
	// and the other two filters stay.
	settings.oneEuro = Strength::Medium;
	settings.softenJumps = true;
	const DriverPlan paused = PlanFor(settings, room, true, true);
	cuts = 0.0;
	for (double cut : paused.predictionCut)
		cuts += cut;
	const std::string named = questcal::ForeignDriverName("driver_SmoothTracking.dll");
	snprintf(detail, sizeof detail, "paused: mask 0x%llx, cuts %.2f in all; \"%s\" known %d", static_cast<unsigned long long>(
		paused.filterMask), cuts, named.c_str(), IsSmoothTrackingDriver(named));
	check("post processing: \"Classic\" stands back for OpenVR-SmoothTracking, and only for it",
		paused.filterMask == Bits({ 2, 3 }) && paused.filter == on.filter && cuts == 0.0 && IsSmoothTrackingDriver(named) &&
			IsSmoothTrackingDriver("SMOOTHTRACKING") && !IsSmoothTrackingDriver("SmoothTracking2") &&
			!IsSmoothTrackingDriver("") && !IsSmoothTrackingDriver(questcal::ForeignDriverName("driver_lighthouse.dll")),
		detail);

	// What the driver's hook status means for the page. The masks: pose hooks 1
	// and 2, the driver context 4. With an observation hook ahead of the code
	// ahead, the driver gives a filtered device its own motion back.
	const auto read = [](uint32_t ahead, uint32_t observer, uint32_t failed, const char *file)
	{
		return ReadOtherAddOns(ahead, observer, failed, questcal::ForeignDriverName(file));
	};
	const auto is = [](const OtherAddOns &others, bool noEffect, bool smoothTracking, bool waits)
	{
		return others.noEffect == noEffect && others.smoothTracking == smoothTracking && ClassicWaits(others) == waits;
	};
	const char *const smooth = "driver_SmoothTracking.dll";
	const bool nothingAhead = is(read(0, 0, 0, ""), true, false, false) && is(OtherAddOns(), true, false, false);
	const bool observed = is(read(2, 2, 0, smooth), true, true, false);
	const bool notObserved = is(read(2, 0, 0, smooth), false, true, true);
	const bool overtaken = is(read(2, 2, 2, smooth), false, true, true);
	const bool oneOfTwo = is(read(3, 2, 0, smooth), false, true, true);
	const bool otherAddOn = is(read(2, 2, 0, "driver_other.dll"), true, false, false) &&
		is(read(2, 0, 2, "OVRInjector.dll"), false, false, false);
	// Code ahead of the driver context's hook changes no pose, and a failure
	// with nothing ahead now has nothing to act on.
	const bool noPoseHook = is(read(4, 0, 0, smooth), true, false, false) && is(read(0, 0, 2, smooth), true, false, false);
	snprintf(detail, sizeof detail, "nothing ahead %d, observed %d, not observed %d, overtaken %d, one hook of two %d, "
		"another add-on %d, no pose hook %d", nothingAhead, observed, notObserved, overtaken, oneOfTwo, otherAddOn,
		noPoseHook);
	check("post processing: other add-ons have no effect while the driver holds each pose as its device sent it, and "
		"\"Classic\" waits only when it does not",
		nothingAhead && observed && notObserved && overtaken && oneOfTwo && otherAddOn && noPoseHook, detail);
}

std::string Written(const questcal::SettingsRecord &record)
{
	std::ostringstream out;
	questcal::WriteSettings(record, 3, out);
	return out.str();
}

// The settings in the Settings record.
void RecordScenarios(Check check)
{
	char detail[320];
	questcal::SettingsRecord record;
	const std::string plain = Written(record);
	record.postProcessing.enabled = true;
	record.postProcessing.oneEuro = Strength::Strong;
	record.postProcessing.softenJumps = false;
	SetFiltered(record.postProcessing, "LHR-LEFT", true);
	SetFiltered(record.postProcessing, "LHR-HIP", false);
	SetClassicPercent(record.postProcessing, "HMD-1", 35);
	const std::string text = Written(record);
	questcal::SettingsRecord reread;
	std::istringstream in(text);
	questcal::ParseSettings(reread, in);
	// A record from before the page reads as the defaults, over what was live.
	questcal::SettingsRecord older = record;
	std::istringstream before(plain);
	questcal::ParseSettings(older, before);
	snprintf(detail, sizeof detail, "defaults write the key: %d; reread equal: %d; an older record resets: %d",
		plain.find("post_processing") != std::string::npos, reread.postProcessing == record.postProcessing,
		older.postProcessing == Settings());
	check("post processing: the settings are saved and read back, and a record without them is the defaults",
		plain.find("post_processing") == std::string::npos && text.find("post_processing") != std::string::npos &&
			reread.postProcessing == record.postProcessing && Written(reread) == text && older.postProcessing == Settings(),
		detail);

	// A strength this build does not know is dropped and a device entry it
	// cannot use is skipped; the rest of the record loads.
	std::string many = "{\"ui_advanced\":true,\"post_processing\":{\"enabled\":true,\"one_euro\":9,"
		"\"filtered\":{\"\":true,\"LHR-A\":1,\"LHR-B\":false";
	// The fillers sort after the named devices, which a record reads first.
	for (int i = 0; i < 100; ++i)
		many += ",\"Z" + std::to_string(i) + "\":true";
	many += "},\"classic\":{\"LHR-A\":250,\"LHR-B\":-5,\"LHR-C\":\"50\",\"LHR-D\":12,\"LHR-E\":1e300";
	for (int i = 0; i < 100; ++i)
		many += ",\"Z" + std::to_string(i) + "\":50";
	many += "}}}";
	questcal::SettingsRecord odd;
	std::istringstream oddIn(many);
	bool threw = false;
	try
	{
		questcal::ParseSettings(odd, oddIn);
	}
	catch (const std::exception &)
	{
		threw = true;
	}
	const Settings &kept = odd.postProcessing;
	// A field of the wrong type refuses the record, which stays as it was.
	int refused = 0;
	bool untouched = true;
	for (const char *wrong : { "{\"ui_advanced\":false,\"post_processing\":7}",
		"{\"ui_advanced\":false,\"post_processing\":{\"soften_jumps\":\"yes\"}}",
		"{\"ui_advanced\":false,\"post_processing\":{\"classic\":[]}}" })
	{
		questcal::SettingsRecord live = record;
		live.uiAdvanced = true;
		std::istringstream wrongIn(wrong);
		try
		{
			questcal::ParseSettings(live, wrongIn);
		}
		catch (const std::exception &)
		{
			++refused;
		}
		untouched = untouched && live.uiAdvanced && live.postProcessing == record.postProcessing;
	}
	snprintf(detail, sizeof detail, "threw %d; enabled %d, strength %d, soften %d, %zu choices, %zu values, LHR-D %d; "
		"%d of 3 records with a wrong type refused, live record untouched %d", threw, kept.enabled,
		static_cast<int>(kept.oneEuro), kept.softenJumps, kept.filtered.size(), kept.classic.size(),
		kept.classic.count("LHR-D") ? kept.classic.at("LHR-D") : -1, refused, untouched);
	check("post processing: an unknown strength and an unusable device entry are dropped, and a wrong type refuses the record",
		!threw && odd.uiAdvanced && kept.enabled && kept.oneEuro == Strength::Medium && kept.softenJumps &&
			kept.filtered.size() == MaxDeviceChoices && kept.filtered.count("") == 0 && kept.filtered.count("LHR-A") == 0 &&
			kept.filtered.count("LHR-B") == 1 && kept.classic.size() == MaxDeviceChoices && kept.classic.count("LHR-A") == 0 &&
			kept.classic.count("LHR-B") == 0 && kept.classic.count("LHR-C") == 0 && kept.classic.count("LHR-D") == 1 &&
			kept.classic.at("LHR-D") == 10 && refused == 3 && untouched,
		detail);
}

} // namespace

void RunPostProcessingScenarios(Check check)
{
	DeviceScenarios(check);
	PlanScenarios(check);
	RecordScenarios(check);
}
