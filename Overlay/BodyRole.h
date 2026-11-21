#pragma once

#include <openvr.h>
#include <cstdint>
#include <string>
#include <utility>

namespace questcal::bodyrole
{
// Which part of the body a device is on, as an index into text::DeviceNames.
enum Part
{
	Head, LeftHand, RightHand, Waist, Chest, LeftFoot, RightFoot, LeftKnee, RightKnee, LeftElbow, RightElbow,
	LeftShoulder, RightShoulder, OtherTracker, OtherController, NotABody
};

inline Part PartOf(uint32_t deviceClass, int32_t controllerRole, const std::string &controllerType)
{
	if (deviceClass == static_cast<uint32_t>(vr::TrackedDeviceClass_HMD))
		return Head;
	if (deviceClass == static_cast<uint32_t>(vr::TrackedDeviceClass_Controller))
		return controllerRole == vr::TrackedControllerRole_LeftHand ? LeftHand
			: controllerRole == vr::TrackedControllerRole_RightHand ? RightHand : OtherController;
	if (deviceClass != static_cast<uint32_t>(vr::TrackedDeviceClass_GenericTracker))
		return NotABody;
	// SteamVR's role for a tracker, such as "vive_tracker_left_foot".
	static const std::pair<const char *, Part> roles[] = {
		{ "waist", Waist }, { "chest", Chest }, { "left_foot", LeftFoot }, { "right_foot", RightFoot },
		{ "left_knee", LeftKnee }, { "right_knee", RightKnee }, { "left_elbow", LeftElbow },
		{ "right_elbow", RightElbow }, { "left_shoulder", LeftShoulder }, { "right_shoulder", RightShoulder },
	};
	for (const auto &role : roles)
		if (controllerType.find(role.first) != std::string::npos)
			return role.second;
	return OtherTracker;
}
}
