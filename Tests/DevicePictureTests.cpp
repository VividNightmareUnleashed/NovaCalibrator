#include "../Overlay/DevicePictures.h"

#include <Windows.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>

// Which picture each device the overlay can meet gets, and that every one
// of them is built in.
namespace
{
using Check = void (*)(const char *, bool, const char *);

struct Case
{
	bool headset;
	bool leftHand;
	const char *model;
	const char *renderModel;
	const char *expected; // null for no picture
	bool mirrored;
};

// The names drivers report: SteamVR's render models, and model numbers
// where a driver names its own.
const Case kCases[] = {
	{ true, false, "Meta Quest Pro", "", "DEVICE_HEADSET_QUEST", false },
	{ true, false, "Oculus Quest2", "", "DEVICE_HEADSET_QUEST", false },
	{ true, false, "Miramar", "", "DEVICE_HEADSET_QUEST", false },
	{ true, false, "Meta Quest 3S", "", "DEVICE_HEADSET_QUEST", false },
	{ true, false, "Steam Frame", "", "DEVICE_HEADSET_STEAM_FRAME", false },
	{ true, false, "PICO 4 Ultra", "", "DEVICE_HEADSET_PICO_4_ULTRA", false },
	{ true, false, "Galaxy XR", "", "DEVICE_HEADSET_GALAXY_XR", false },
	{ false, false, "Meta Quest Pro (Right Controller)", "oculus_quest_pro_controller_right", "DEVICE_TOUCH_PRO", false },
	{ false, true, "Touch Pro Left", "", "DEVICE_TOUCH_PRO_LEFT", false },
	{ false, false, "Meta Quest 3 (Left Controller)", "oculus_quest_plus_controller_left", "DEVICE_TOUCH_PLUS", true },
	{ false, false, "Meta Quest 3S (Right Controller)", "", "DEVICE_TOUCH_PLUS", false },
	{ false, false, "Oculus Quest2 (Right Controller)", "oculus_quest2_controller_right", "DEVICE_TOUCH_QUEST_2", false },
	{ false, false, "Oculus Quest (Right Controller)", "oculus_quest_controller_right", "DEVICE_TOUCH_QUEST_1", false },
	{ false, true, "Knuckles Left", "{indexcontroller}valve_controller_knu_1_0_left", "DEVICE_INDEX_CONTROLLER_LEFT", false },
	{ false, false, "Knuckles Right", "{indexcontroller}valve_controller_knu_1_0_right", "DEVICE_INDEX_CONTROLLER", false },
	{ false, false, "Vive. Controller MV", "vr_controller_vive_1_5", "DEVICE_VIVE_WAND", false },
	{ false, false, "", "{frame_controller}frame_controller_right", "DEVICE_STEAM_FRAME_CONTROLLER", false },
	{ false, false, "", "{vrlink}pico_4_ultra_controller_right", "DEVICE_PICO_4_ULTRA_CONTROLLER", false },
	{ false, false, "", "{vrlink}pico_4_controller_left", "DEVICE_PICO_4_CONTROLLER", true },
	{ false, false, "", "{vrlink}vive_focus3_controller_right", "DEVICE_FOCUS_3_CONTROLLER", false },
	{ false, false, "VIVE Tracker 3.0 MV", "{htc}vr_tracker_vive_3_0", "DEVICE_VIVE_TRACKER_3", false },
	{ false, false, "VIVE Tracker Pro MV", "{htc}vr_tracker_vive_1_0", "DEVICE_VIVE_TRACKER_2018", false },
	// A tracker has no hand, whatever role it was given.
	{ false, true, "Tundra Tracker", "{tundra_labs}tundra_tracker", "DEVICE_TUNDRA_TRACKER", false },
	{ false, false, "SlimeVR Tracker", "{slimevr}slimevr_tracker", nullptr, false },
	{ true, false, "Index", "generic_hmd", nullptr, false },
	{ false, false, "", "", nullptr, false },
};

void RunMatchingScenario(Check check)
{
	std::string misses;
	for (const Case &c : kCases)
	{
		const questcal::DevicePicture picture =
			questcal::PictureOfDevice(c.headset, c.leftHand, c.model, c.renderModel);
		const bool same = (picture.resource == nullptr) == (c.expected == nullptr) &&
			(!c.expected || strcmp(picture.resource, c.expected) == 0) && picture.mirrored == c.mirrored;
		if (!same)
			misses += std::string(misses.empty() ? "" : "; ") + (c.model[0] ? c.model : c.renderModel) + " -> " +
				(picture.resource ? picture.resource : "none") + (picture.mirrored ? " mirrored" : "");
	}
	check("device pictures: each device gets its own picture", misses.empty(), misses.c_str());
}

// The Overlay sources, found from the harness binary (x64/<config>/).
std::filesystem::path OverlayFolder()
{
	wchar_t exe[MAX_PATH] = {};
	GetModuleFileNameW(nullptr, exe, MAX_PATH);
	for (std::filesystem::path dir = std::filesystem::path(exe).parent_path(); !dir.empty() && dir != dir.parent_path();
		dir = dir.parent_path())
		if (std::filesystem::exists(dir / "Overlay" / "Art.rc"))
			return dir / "Overlay";
	return {};
}

void RunEmbeddedScenario(Check check)
{
	const std::filesystem::path overlay = OverlayFolder();
	std::ifstream rc(overlay / "Art.rc", std::ios::binary);
	std::stringstream text;
	text << rc.rdbuf();
	const std::string source = text.str();
	// NAME RCDATA "assets\\devices\\file.png"
	const std::regex entry(R"re((DEVICE_\w+)\s+RCDATA\s+"assets\\\\devices\\\\([\w.-]+)")re");
	std::set<std::string> embedded;
	std::string problems;
	for (std::sregex_iterator it(source.begin(), source.end(), entry), end; it != end; ++it)
	{
		embedded.insert((*it)[1].str());
		if (!std::filesystem::exists(overlay / "assets" / "devices" / (*it)[2].str()))
			problems += "missing file " + (*it)[2].str() + "; ";
	}
	const std::set<std::string> named(std::begin(questcal::kDevicePictures), std::end(questcal::kDevicePictures));
	for (const std::string &name : named)
		if (!embedded.count(name))
			problems += "not embedded " + name + "; ";
	for (const std::string &name : embedded)
		if (!named.count(name))
			problems += "embedded but never shown " + name + "; ";
	for (const Case &c : kCases)
		if (c.expected && !named.count(c.expected))
			problems += std::string("unlisted ") + c.expected + "; ";
	char detail[64];
	snprintf(detail, sizeof detail, "%zu embedded", embedded.size());
	check("device pictures: every picture is built in", !overlay.empty() && problems.empty(),
		problems.empty() ? detail : problems.c_str());
}
}

void RunDevicePictureScenarios(Check check)
{
	RunMatchingScenario(check);
	RunEmbeddedScenario(check);
}
