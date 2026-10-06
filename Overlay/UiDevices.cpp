// The pair sheet ("Choose the pair"), device names, and the live VR state
// query.
#include "stdafx.h"
#include "UiInternal.h"

void DeviceIcon(ImDrawList *dl, const VRDevice &dev, ImVec2 c, float s, ImU32 col)
{
	switch (dev.deviceClass)
	{
	case vr::TrackedDeviceClass_HMD:
		IconHMD(dl, c, s, col);
		break;
	case vr::TrackedDeviceClass_Controller:
		IconController(dl, c, s, col, dev.controllerRole == vr::TrackedControllerRole_LeftHand);
		break;
	default:
		IconTracker(dl, c, s, col);
		break;
	}
}

// Player-given names. A tile shows the name in place of the model; everywhere
// else a device is named, the same lookup applies so "Hip" is "Hip" in the
// tracker pick and the guide too.
const std::string *FindDeviceName(const std::string &serial)
{
	auto it = CalCtx.deviceNames.find(serial);
	return it != CalCtx.deviceNames.end() && !it->second.empty() ? &it->second : nullptr;
}

std::string DeviceDisplayName(const VRDevice &dev)
{
	const std::string *name = FindDeviceName(dev.serial);
	return name ? *name : dev.model;
}

// Inline rename: one device at a time.
static int s_renameDeviceId = -1;
static bool s_renameFocus = false;
static char s_renameBuf[CalibrationContext::DeviceNameMaxBytes + 1] = {};

void CommitDeviceName(const VRDevice &dev, const char *text)
{
	std::string name(text);
	while (!name.empty() && isspace(static_cast<unsigned char>(name.back())))
		name.pop_back();
	size_t start = 0;
	while (start < name.size() && isspace(static_cast<unsigned char>(name[start])))
		++start;
	name.erase(0, start);

	auto previous = CalCtx.deviceNames;
	if (name.empty() || name == dev.model)
		CalCtx.deviceNames.erase(dev.serial);
	else if (CalCtx.deviceNames.count(dev.serial) ||
		CalCtx.deviceNames.size() < CalibrationContext::DeviceNameMaxCount)
		CalCtx.deviceNames[dev.serial] = name;
	if (CalCtx.deviceNames != previous)
		SaveSettingOrRestore(CalCtx.deviceNames, previous);
}

// Drops a selection that is no longer in the system's device list, then
// defaults to the left-hand controller, else the system's first device.
void EnsureDeviceSelection(const VRState &state, uint32_t &selected, const std::string &system)
{
	const uint32_t none = vr::k_unTrackedDeviceIndexInvalid;

	if (selected != none)
	{
		bool matched = false;
		for (auto &device : state.devices)
			if (device.trackingSystem == system && selected == (uint32_t)device.id)
			{
				matched = true;
				break;
			}
		if (!matched)
			selected = none;
	}

	if (selected == none)
	{
		for (auto &device : state.devices)
			if (device.trackingSystem == system && device.controllerRole == vr::TrackedControllerRole_LeftHand)
			{
				selected = (uint32_t)device.id;
				break;
			}
	}

	if (selected == none)
	{
		for (auto &device : state.devices)
			if (device.trackingSystem == system)
			{
				selected = (uint32_t)device.id;
				break;
			}
	}
}

// Users know their hardware, not driver names; show friendly labels for the
// systems we recognize and fall back to the raw name for anything else.
std::string FriendlySystemName(const std::string &raw)
{
	if (raw == "oculus")     return "Meta Quest";
	if (raw == "lighthouse") return "SteamVR Lighthouse";
	if (raw == "holographic") return "Windows Mixed Reality";
	return raw;
}

// The picks are candidates for the next base calibration; the active
// profile's system names are committed only after a successful solve. The
// reference is the headset's system (LoadVRState lists it first) unless the
// pick is still there; the target is any other system, and one system can
// never be both.
void SettlePairSystems(const VRState &state)
{
	if (state.trackingSystems.empty())
		return;
	auto present = [&](const std::string &system) {
		return std::find(state.trackingSystems.begin(), state.trackingSystems.end(), system) != state.trackingSystems.end();
	};
	if (!present(CalCtx.pendingReferenceTrackingSystem))
		CalCtx.pendingReferenceTrackingSystem = state.trackingSystems.front();
	if (CalCtx.pendingTargetTrackingSystem == CalCtx.pendingReferenceTrackingSystem ||
		!present(CalCtx.pendingTargetTrackingSystem))
	{
		CalCtx.pendingTargetTrackingSystem.clear();
		for (const auto &system : state.trackingSystems)
			if (system != CalCtx.pendingReferenceTrackingSystem)
			{
				CalCtx.pendingTargetTrackingSystem = system;
				break;
			}
	}
	EnsureDeviceSelection(state, CalCtx.referenceID, CalCtx.pendingReferenceTrackingSystem);
	// Nothing to pick on the other side means nothing may stay picked: this id
	// is what the identify pulse buzzes and what StartCalibration freezes.
	if (CalCtx.pendingTargetTrackingSystem.empty())
		CalCtx.targetID = vr::k_unTrackedDeviceIndexInvalid;
	else
		EnsureDeviceSelection(state, CalCtx.targetID, CalCtx.pendingTargetTrackingSystem);
}

// ---------------------------------------------------------------------------
// The pair sheet
// ---------------------------------------------------------------------------

static void BatteryGlyph(ImDrawList *dl, ImVec2 topLeft, float level, ImU32 colour)
{
	const float w = 20.0f, h = 10.0f;
	dl->AddRect(topLeft, ImVec2(topLeft.x + w, topLeft.y + h), ui::Rgba(236, 238, 244, 0.55f), 2.5f, 0, 1.2f);
	dl->AddRectFilled(ImVec2(topLeft.x + w + 1.0f, topLeft.y + 3.0f), ImVec2(topLeft.x + w + 3.0f, topLeft.y + h - 3.0f),
		ui::Rgba(236, 238, 244, 0.55f), 1.0f);
	const float fill = (w - 4.0f) * std::clamp(level, 0.0f, 1.0f);
	if (fill > 0.5f)
		dl->AddRectFilled(ImVec2(topLeft.x + 2.0f, topLeft.y + 2.0f), ImVec2(topLeft.x + 2.0f + fill, topLeft.y + h - 2.0f), colour, 1.0f);
}

// One device as a tile: picture, name, and battery or state. Hovering shows
// rename and identify; the rename field takes over the name line.
static bool DeviceTile(const VRDevice &dev, const FlexRect &r, bool selected)
{
	ImGui::PushID(dev.id);
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImGui::SetCursorScreenPos(r.min);
	// The rename and identify tools sit on top of the tile and must win the
	// hover.
	ImGui::SetNextItemAllowOverlap();
	const bool pressed = ImGui::InvisibleButton("tile", r.Size(), ImGuiButtonFlags_EnableNav);
	const bool hov = ImGui::IsItemHovered();
	const bool focused = ImGui::IsItemFocused();
	if (selected)
	{
		ui::FillRounded(dl, r.min, r.max, ui::Rgba(35, 112, 232, 0.18f), 20.0f);
		ui::InnerEdge(dl, r.min, r.max, ui::Rgba(47, 124, 242), 20.0f, 2.0f);
	}
	else
		ui::FillRounded(dl, r.min, r.max, hov ? ui::Rgba(255, 255, 255, 0.09f) : ui::col::Card, 20.0f);

	FlexRect art;
	art.min = ImVec2(r.Center().x - 44.0f, r.min.y + 6.0f);
	art.max = ImVec2(art.min.x + 88.0f, art.min.y + 88.0f);
	DrawDeviceArt(dl, dev, art, dev.connected ? ui::col::Muted : ui::col::Faint);

	const ui::TextStyle nameStyle{ ui::Weight::SemiBold, 16.0f, 20.0f };
	const float textW = r.W() - 16.0f;
	const float nameY = r.min.y + 6.0f + 88.0f + 6.0f;
	if (s_renameDeviceId == dev.id)
	{
		ImGui::SetCursorScreenPos(ImVec2(r.min.x + 6.0f, nameY - 3.0f));
		ImGui::PushItemWidth(r.W() - 12.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 2));
		ImGui::PushFont(ui::FontOf(ui::Weight::SemiBold), ui::GlyphSize(15.0f));
		if (s_renameFocus)
		{
			ImGui::SetKeyboardFocusHere();
			s_renameFocus = false;
		}
		const bool entered = ImGui::InputTextWithHint("##rename", Tr("Hip, Left foot, Chest..."), s_renameBuf, sizeof s_renameBuf,
			ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
		// The VR keyboard may finish without an Enter, so leaving the field
		// commits too; Escape reverts the buffer first, which commits no
		// change.
		const bool finished = entered || ImGui::IsItemDeactivated();
		ImGui::PopFont();
		ImGui::PopStyleVar();
		ImGui::PopItemWidth();
		if (finished)
		{
			CommitDeviceName(dev, s_renameBuf);
			s_renameDeviceId = -1;
		}
	}
	else
	{
		const std::string name = ui::Ellipsize(nameStyle, DeviceDisplayName(dev), textW);
		const float w = ui::MeasureLine(nameStyle, name.c_str()).x;
		ui::DrawLine(dl, nameStyle, ImVec2(r.Center().x - w * 0.5f, nameY), dev.connected ? ui::col::Text : ui::col::Faint, name.c_str());
	}

	// What else there is to know: the headset is the headset; otherwise the
	// battery, or that the device is off.
	const float subY = nameY + 20.0f;
	const ui::TextStyle subStyle{ ui::Weight::Regular, 14.0f, 18.0f };
	const bool low = dev.connected && dev.battery >= 0.0f && dev.battery < kLowBattery && !dev.charging;
	if (dev.deviceClass == vr::TrackedDeviceClass_HMD)
	{
		const char *text = Tr("Headset");
		const float w = ui::MeasureLine(subStyle, text).x;
		ui::DrawLine(dl, subStyle, ImVec2(r.Center().x - w * 0.5f, subY), ui::Rgba(236, 238, 244, 0.64f), text);
	}
	else if (!dev.connected)
	{
		const char *text = Tr("Off");
		const float w = ui::MeasureLine(subStyle, text).x;
		ui::DrawLine(dl, subStyle, ImVec2(r.Center().x - w * 0.5f, subY), ui::col::Faint, text);
	}
	else if (dev.battery >= 0.0f)
	{
		const std::string text = FormatString("%d%%", static_cast<int>(std::lround(std::min(dev.battery, 1.0f) * 100.0f)));
		const float w = ui::MeasureLine(subStyle, text.c_str()).x + 26.0f;
		const float x = r.Center().x - w * 0.5f;
		BatteryGlyph(dl, ImVec2(x, subY + 4.0f), dev.battery, low ? ui::col::Alert : ui::col::Good);
		ui::DrawLine(dl, subStyle, ImVec2(x + 26.0f, subY), low ? ui::col::Alert : ui::Rgba(236, 238, 244, 0.64f), text.c_str());
	}

	if (selected)
	{
		const ImVec2 c(r.max.x - 10.0f - 12.0f, r.min.y + 10.0f + 12.0f);
		dl->AddCircleFilled(c, 12.0f, ui::col::Accent, 24);
		ui::DrawIcon(dl, ui::Icon::Check, c, 14.0f, ui::col::White, 3.0f);
	}

	// Rename, and buzz this one device: with six identical trackers these are
	// what tell the tiles apart. Shown while the tile is hovered or focused.
	const bool showTools = hov || focused || ImGui::IsMouseHoveringRect(r.min, r.max);
	if (showTools && s_renameDeviceId != dev.id)
	{
		const ImU32 fill = ui::Rgba(20, 22, 26, 0.72f), ink = ui::Rgba(236, 238, 244, 0.92f);
		ImVec2 at(r.min.x + 10.0f + 14.0f, r.min.y + 10.0f + 14.0f);
		if (ui::RoundButton("rename", at, 28.0f, ui::Icon::Pencil, 15.0f, fill, ink))
		{
			s_renameDeviceId = dev.id;
			const std::string *current = FindDeviceName(dev.serial);
			strncpy_s(s_renameBuf, current ? current->c_str() : "", _TRUNCATE);
			s_renameFocus = true;
		}
		if (ImGui::IsItemHovered())
			ShowTip("Rename this device");
		// A headset can't buzz or blink on command, so it gets rename only.
		if (dev.deviceClass != vr::TrackedDeviceClass_HMD)
		{
			at.x += 34.0f;
			if (ui::RoundButton("buzz", at, 28.0f, ui::Icon::Identify, 15.0f, fill, ink))
				StartIdentifyPulse(static_cast<uint32_t>(dev.id), vr::k_unTrackedDeviceIndexInvalid);
			if (ImGui::IsItemHovered())
				ShowTip("Vibrate or blink this device");
		}
	}
	ui::FocusRing(dl, r.min, r.max, 20.0f);
	ImGui::PopID();
	return pressed;
}

void BuildPairSheet(const VRState &state)
{
	SettlePairSystems(state);
	const FlexRect c = SheetPanel(960.0f, 670.0f);
	if (SheetHeader(c, Tr("Choose the pair")))
		CloseSheet();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const ui::TextStyle leadStyle{ ui::Weight::Regular, 18.0f, 26.0f };
	const float leadH = ui::DrawText(dl, leadStyle, ImVec2(c.min.x, c.min.y + 44.0f + 6.0f), c.W(), ui::Rgba(236, 238, 244, 0.70f),
		Tr("Pick a Quest device and a SteamVR device to hold together while you calibrate."));

	// The groups scroll on their own when more devices than fit are on.
	const float footerH = 52.0f;
	const ImVec2 listMin(c.min.x, c.min.y + 44.0f + 6.0f + leadH + 16.0f);
	const ImVec2 listSize(c.W(), c.max.y - footerH - 18.0f - listMin.y);
	ImGui::SetCursorScreenPos(listMin);
	ImGui::BeginChild("##pairgroups", listSize, ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
	ImDrawList *gdl = ImGui::GetWindowDrawList();
	const float width = ImGui::GetContentRegionAvail().x;
	const ImVec2 origin = ImGui::GetCursorScreenPos();
	float y = 0.0f;
	std::vector<std::string> systems;
	if (!CalCtx.pendingReferenceTrackingSystem.empty())
		systems.push_back(CalCtx.pendingReferenceTrackingSystem);
	for (const auto &system : state.trackingSystems)
		if (system != CalCtx.pendingReferenceTrackingSystem)
			systems.push_back(system);
	if (systems.empty())
	{
		const char *text = Tr("No tracked devices found");
		ui::DrawText(gdl, ui::type::Body, ImVec2(origin.x, origin.y + 60.0f), width, ui::col::Muted, text, ui::Align::Center);
		y = 140.0f;
	}
	const int columns = 6;
	const float gap = 12.0f;
	const float tileW = std::floor((width - gap * static_cast<float>(columns - 1)) / static_cast<float>(columns));
	const float tileH = 156.0f;
	for (size_t s = 0; s < systems.size(); ++s)
	{
		const std::string &system = systems[s];
		const bool reference = s == 0;
		if (s > 0)
			y += 20.0f;
		ui::DrawLine(gdl, ui::type::Strong, ImVec2(origin.x, origin.y + y), ui::Rgba(236, 238, 244, 0.82f),
			FriendlySystemName(system).c_str());
		y += 22.0f + 10.0f;
		int index = 0;
		for (const auto &device : state.devices)
		{
			if (device.trackingSystem != system)
				continue;
			const int column = index % columns;
			const int row = index / columns;
			FlexRect r;
			r.min = ImVec2(origin.x + (tileW + gap) * static_cast<float>(column), origin.y + y + (tileH + gap) * static_cast<float>(row));
			r.max = ImVec2(r.min.x + tileW, r.min.y + tileH);
			const uint32_t id = static_cast<uint32_t>(device.id);
			const bool selected = reference ? CalCtx.referenceID == id
				: CalCtx.targetID == id && CalCtx.pendingTargetTrackingSystem == system;
			if (DeviceTile(device, r, selected))
			{
				if (reference)
					CalCtx.referenceID = id;
				else
				{
					CalCtx.targetID = id;
					CalCtx.pendingTargetTrackingSystem = system;
				}
			}
			++index;
		}
		if (index == 0)
		{
			ui::DrawLine(gdl, ui::type::Body, ImVec2(origin.x, origin.y + y), ui::col::Muted, Tr("No devices connected"));
			y += 24.0f;
		}
		else
			y += (tileH + gap) * static_cast<float>((index + columns - 1) / columns) - gap;
	}
	if (systems.size() == 1)
	{
		y += 20.0f;
		ui::DrawText(gdl, ui::type::Body, ImVec2(origin.x, origin.y + y), width, ui::col::Muted,
			Tr("No trackers found. Turn on a tracker and make sure SteamVR sees it."));
		y += 24.0f;
	}
	ImGui::SetCursorScreenPos(origin);
	ImGui::Dummy(ImVec2(width, y + 4.0f));
	// More tiles below the fold: the list fades into the panel instead of
	// ending on a clean cut that reads as the last row. Drawn in the list's
	// own draw list, which ImGui renders over the sheet's.
	const float hiddenBelow = ImGui::GetScrollMaxY() - ImGui::GetScrollY();
	if (hiddenBelow > 1.0f)
	{
		const float panelTop = c.min.y - 28.0f, panelBottom = c.max.y + 32.0f;
		const float listBottom = listMin.y + listSize.y;
		const float fadeH = std::min(56.0f, hiddenBelow + 16.0f);
		const ImU32 panel = ui::Mix(ui::col::SheetTop, ui::col::SheetBottom, (listBottom - panelTop) / (panelBottom - panelTop));
		gdl->AddRectFilledMultiColor(ImVec2(origin.x, listBottom - fadeH), ImVec2(origin.x + width, listBottom),
			ui::Fade(panel, 0.0f), ui::Fade(panel, 0.0f), panel, panel);
	}
	ImGui::EndChild();

	FlexRect done, identify;
	const float doneW = std::max(140.0f, ui::PillWidth("Done", 18.0f));
	const float identifyW = ui::PillWidth("Identify", 18.0f, ui::Icon::Identify);
	done.min = ImVec2(c.max.x - doneW, c.max.y - footerH);
	done.max = c.max;
	identify.min = ImVec2(done.min.x - 12.0f - identifyW, done.min.y);
	identify.max = ImVec2(done.min.x - 12.0f, done.max.y);
	if (ui::PillButton("##identifypair", "Identify", ui::Btn::Secondary, identify, ui::Icon::Identify))
		StartIdentifyPulse(CalCtx.targetID, CalCtx.referenceID);
	if (ImGui::IsItemHovered())
		ShowTip("Vibrates or blinks the two selected devices so you can tell which is which.");
	if (ui::PillButton("##donepair", "Done", ui::Btn::Primary, done))
		CloseSheet();
}

// ---------------------------------------------------------------------------
// The live VR state
// ---------------------------------------------------------------------------

VRState LoadVRState()
{
	VRState state;
	state.trackingSystems.reserve(vr::k_unMaxTrackedDeviceCount);
	state.devices.reserve(vr::k_unMaxTrackedDeviceCount);

	if (g_uiPreviewMode)
		return PreviewVRState();

	if (!vr::VRSystem())
		return state;

	// One pose query per refresh for the readiness chips in the guide: raw
	// universe, no prediction, so it costs the same as the property reads.
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
		vr::TrackingUniverseRawAndUncalibrated, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);

	auto &trackingSystems = state.trackingSystems;
	auto readStringProperty = [](uint32_t id, vr::ETrackedDeviceProperty property)
	{
		char value[vr::k_unMaxPropertyStringSize] = {};
		vr::ETrackedPropertyError error = vr::TrackedProp_Success;
		uint32_t size = vr::VRSystem()->GetStringTrackedDeviceProperty(id,
			property, value, static_cast<uint32_t>(sizeof value), &error);
		if (error != vr::TrackedProp_Success || size <= 1 || size > sizeof value ||
			value[size - 1] != '\0')
			return std::string();
		return std::string(value, size - 1);
	};

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		if (deviceClass != vr::TrackedDeviceClass_TrackingReference)
		{
			std::string system = readStringProperty(id, vr::Prop_TrackingSystemName_String);
			if (!system.empty())
			{
				auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
				if (existing != trackingSystems.end())
				{
					if (deviceClass == vr::TrackedDeviceClass_HMD)
					{
						trackingSystems.erase(existing);
						trackingSystems.insert(trackingSystems.begin(), system);
					}
				}
				else
				{
					trackingSystems.push_back(system);
				}

				VRDevice device;
				device.id = id;
				device.deviceClass = deviceClass;
				device.trackingSystem = system;

				device.model = readStringProperty(id, vr::Prop_ModelNumber_String);
				device.serial = readStringProperty(id, vr::Prop_SerialNumber_String);
				device.renderModel = readStringProperty(id, vr::Prop_RenderModelName_String);

				vr::ETrackedPropertyError roleError = vr::TrackedProp_Success;
				int32_t role = vr::VRSystem()->GetInt32TrackedDeviceProperty(
					id, vr::Prop_ControllerRoleHint_Int32, &roleError);
				if (roleError == vr::TrackedProp_Success)
					device.controllerRole = static_cast<vr::ETrackedControllerRole>(role);

				device.connected = vr::VRSystem()->IsTrackedDeviceConnected(id);
				device.tracking = device.connected && poses[id].bPoseIsValid &&
					poses[id].eTrackingResult == vr::TrackingResult_Running_OK;

				vr::ETrackedPropertyError perr = vr::TrackedProp_Success;
				if (vr::VRSystem()->GetBoolTrackedDeviceProperty(id, vr::Prop_DeviceProvidesBatteryStatus_Bool, &perr) && perr == vr::TrackedProp_Success)
				{
					float pct = vr::VRSystem()->GetFloatTrackedDeviceProperty(id, vr::Prop_DeviceBatteryPercentage_Float, &perr);
					if (perr == vr::TrackedProp_Success)
						device.battery = pct;
					device.charging = vr::VRSystem()->GetBoolTrackedDeviceProperty(id, vr::Prop_DeviceIsCharging_Bool, &perr);
				}

				// SteamVR's own device icon, matched to state: green "ready"
				// art while connected, red low-battery art below kLowBattery,
				// gray "off" art when the device drops out. Falls back down
				// the chain when a driver doesn't ship a variant.
				// "{driver}/icons/x.png" resolves through IVRResources.
				auto resolveIcon = [&](vr::ETrackedDeviceProperty prop) -> std::string {
					std::string resource = readStringProperty(id, prop);
					if (resource.empty() || !vr::VRResources())
						return std::string();
					char fullPath[MAX_PATH] = {};
					uint32_t len = vr::VRResources()->GetResourceFullPath(
						resource.c_str(), "", fullPath, MAX_PATH);
					if (len == 0 || len >= MAX_PATH || !FileExists(fullPath))
						return std::string();
					return Prefer2x(fullPath);
				};

				if (!device.connected)
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceOff_String);
				else if (device.battery >= 0.0f && device.battery < kLowBattery && !device.charging)
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceAlertLow_String);
				if (device.iconPath.empty())
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceReady_String);
				if (device.iconPath.empty())
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceOff_String);

				state.devices.push_back(device);
			}
			else
			{
				// This loader runs once a second, so each device is reported
				// once: the latch records what has been said, not device state.
				static bool reportedMissingSystem[vr::k_unMaxTrackedDeviceCount] = {};
				if (!reportedMissingSystem[id])
				{
					reportedMissingSystem[id] = true;
					AppendSessionLog(FormatString(
						"Device %u has no tracking system name and is not shown in the device panes", id));
				}
			}
		}
	}

	return state;
}
