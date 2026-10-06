#pragma once

// Which built-in picture shows a device. The pictures are PNGs in Art.rc:
// the design's renders of SteamVR's own models for controllers and trackers
// (renders in its studio for the devices it does not show) and product photos
// for headsets. SteamVR's render model name says what a device is when its
// driver names one; the model number is read too, for drivers that name their
// own models.

#include <cctype>
#include <string>

namespace questcal
{

struct DevicePicture
{
	const char *resource = nullptr; // the embedded PNG, or null when none is built in
	bool mirrored = false;          // a right-hand picture to show mirrored, for the left hand
};

// Every picture PictureOfDevice can name, each embedded by Art.rc.
inline constexpr const char *kDevicePictures[] = {
	"DEVICE_HEADSET_QUEST", "DEVICE_HEADSET_STEAM_FRAME", "DEVICE_HEADSET_PICO_4_ULTRA", "DEVICE_HEADSET_GALAXY_XR",
	"DEVICE_TOUCH_PRO", "DEVICE_TOUCH_PRO_LEFT", "DEVICE_TOUCH_PLUS", "DEVICE_TOUCH_QUEST_2", "DEVICE_TOUCH_QUEST_1",
	"DEVICE_INDEX_CONTROLLER", "DEVICE_INDEX_CONTROLLER_LEFT", "DEVICE_VIVE_WAND", "DEVICE_STEAM_FRAME_CONTROLLER",
	"DEVICE_PICO_4_CONTROLLER", "DEVICE_PICO_4_ULTRA_CONTROLLER", "DEVICE_FOCUS_3_CONTROLLER",
	"DEVICE_VIVE_TRACKER_3", "DEVICE_VIVE_TRACKER_2018", "DEVICE_TUNDRA_TRACKER",
};

// headset: the device is the HMD. leftHand: SteamVR gives it the left hand's
// role. A device none of the pictures shows gets none.
inline DevicePicture PictureOfDevice(bool headset, bool leftHand, const std::string &model,
	const std::string &renderModel)
{
	const auto lowered = [](std::string text) {
		for (char &c : text)
			c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
		return text;
	};
	const std::string m = lowered(model), r = lowered(renderModel);
	const auto has = [&](const char *part) {
		return m.find(part) != std::string::npos || r.find(part) != std::string::npos;
	};

	DevicePicture picture;
	if (headset)
	{
		// One picture for every Quest, as the setup's Meta Quest choice shows.
		picture.resource =
			has("quest") || has("miramar") || has("eureka") || has("panther") || has("seacliff") ? "DEVICE_HEADSET_QUEST" :
			has("frame") ? "DEVICE_HEADSET_STEAM_FRAME" :
			has("pico") ? "DEVICE_HEADSET_PICO_4_ULTRA" :
			has("galaxy") || has("samsung") ? "DEVICE_HEADSET_GALAXY_XR" : nullptr;
		return picture;
	}

	// Most specific first: a Quest 3's controller names the headset too.
	picture.resource =
		has("quest_pro_controller") || has("touch pro") || has("quest pro") ? "DEVICE_TOUCH_PRO" :
		has("quest_plus_controller") || has("touch plus") || has("quest 3") || has("quest3") ? "DEVICE_TOUCH_PLUS" :
		has("quest2_controller") || has("quest 2") || has("quest2") ? "DEVICE_TOUCH_QUEST_2" :
		has("oculus_quest_controller") || has("rifts_controller") || has("rift s") ? "DEVICE_TOUCH_QUEST_1" :
		has("valve_controller_knu") || has("knuckles") || has("index controller") ? "DEVICE_INDEX_CONTROLLER" :
		has("frame_controller") || has("steam frame") ? "DEVICE_STEAM_FRAME_CONTROLLER" :
		has("pico_4_ultra") || has("pico 4 ultra") ? "DEVICE_PICO_4_ULTRA_CONTROLLER" :
		has("pico_4_controller") || has("pico 4") ? "DEVICE_PICO_4_CONTROLLER" :
		has("focus3") || has("focus 3") || has("xr elite") ? "DEVICE_FOCUS_3_CONTROLLER" :
		has("vr_controller_vive") || has("vive controller") || has("vive. controller") ? "DEVICE_VIVE_WAND" :
		has("vr_tracker_vive_3_0") || has("tracker 3.0") ? "DEVICE_VIVE_TRACKER_3" :
		has("vr_tracker_vive_1_0") || has("vive tracker") ? "DEVICE_VIVE_TRACKER_2018" :
		has("tundra") ? "DEVICE_TUNDRA_TRACKER" : nullptr;
	// The trackers have no hand; a controller with no role says its hand in
	// its names. The Touch Pro and the Index have left-hand pictures of their
	// own, the rest show the right hand's mirrored.
	const std::string resource = picture.resource ? picture.resource : "";
	const bool namedLeft = (r.size() >= 5 && r.compare(r.size() - 5, 5, "_left") == 0) ||
		m.find("left") != std::string::npos;
	if (resource.empty() || resource.find("TRACKER") != std::string::npos || !(leftHand || namedLeft))
		return picture;
	if (resource == "DEVICE_TOUCH_PRO")
		picture.resource = "DEVICE_TOUCH_PRO_LEFT";
	else if (resource == "DEVICE_INDEX_CONTROLLER")
		picture.resource = "DEVICE_INDEX_CONTROLLER_LEFT";
	else
		picture.mirrored = true;
	return picture;
}

} // namespace questcal
