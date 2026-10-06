// -uipreview: fake devices and fake calibration state, no SteamVR.
#include "stdafx.h"
#include "UiInternal.h"
#include "QualityBands.h"

// ---------------------------------------------------------------------------
// VR state
// ---------------------------------------------------------------------------

// Preview only: point fake devices at real SteamVR icon files when the local
// install has them (vector fallbacks otherwise).
std::string PreviewIconPath(const char *driverRelative)
{
	static std::string base;
	if (base.empty())
	{
		char buf[MAX_PATH] = {};
		DWORD len = sizeof buf;
		if (RegGetValueA(HKEY_CURRENT_USER, "Software\\Valve\\Steam", "SteamPath", RRF_RT_REG_SZ, nullptr, buf, &len) == ERROR_SUCCESS)
			base = std::string(buf) + "\\steamapps\\common\\SteamVR\\drivers\\";
		else
			base = "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\drivers\\";
	}
	std::string path = base + driverRelative;
	return FileExists(path) ? path : std::string();
}

// The fake devices behind -uipreview / -uipreview-many.
VRState PreviewVRState()
{
	VRState state;
	state.trackingSystems = { "oculus", "lighthouse" };

	VRDevice hmd;
	hmd.id = 0;
	hmd.deviceClass = vr::TrackedDeviceClass_HMD;
	hmd.model = "Meta Quest Pro";
	hmd.renderModel = "oculusHmdRenderModel";
	hmd.serial = "1PASH5D1P17365";
	hmd.trackingSystem = "oculus";
	hmd.iconPath = PreviewIconPath("oculus\\resources\\icons\\quest_headset_ready_2x.png");
	hmd.tracking = true;
	state.devices.push_back(hmd);

	VRDevice touchPro;
	touchPro.id = 3 + kPreviewManyTrackerCount;
	touchPro.deviceClass = vr::TrackedDeviceClass_Controller;
	touchPro.model = "Touch Pro Right";
	touchPro.renderModel = "oculus_quest_pro_controller_right";
	touchPro.serial = "PREVIEW-TOUCH-PRO-RIGHT";
	touchPro.trackingSystem = "oculus";
	touchPro.controllerRole = vr::TrackedControllerRole_RightHand;
	// The Oculus driver ships no Quest controller art; its Touch icons are
	// the Rift S set.
	touchPro.iconPath = PreviewIconPath("oculus\\resources\\icons\\rifts_right_controller_ready_2x.png");
	touchPro.battery = 0.75f;
	touchPro.tracking = true;
	state.devices.push_back(touchPro);

	VRDevice right;
	right.id = 1;
	right.deviceClass = vr::TrackedDeviceClass_Controller;
	right.model = "Knuckles Right";
	right.renderModel = "{indexcontroller}valve_controller_knu_1_0_right";
	right.serial = "LHR-A3C36EA5";
	right.trackingSystem = "lighthouse";
	right.controllerRole = vr::TrackedControllerRole_RightHand;
	right.battery = 0.80f;
	right.iconPath = PreviewIconPath("indexcontroller\\resources\\icons\\right_controller_status_ready_2x.png");
	right.tracking = true;
	state.devices.push_back(right);

	VRDevice left;
	left.id = 2;
	left.deviceClass = vr::TrackedDeviceClass_Controller;
	left.model = "Knuckles Left";
	left.renderModel = "{indexcontroller}valve_controller_knu_1_0_left";
	left.serial = "LHR-841C98C3";
	left.trackingSystem = "lighthouse";
	left.controllerRole = vr::TrackedControllerRole_LeftHand;
	left.battery = 0.12f;
	left.iconPath = PreviewIconPath("indexcontroller\\resources\\icons\\left_controller_status_ready_low_2x.png");
	left.tracking = true;
	state.devices.push_back(left);

	int trackerCount = g_uiPreviewMany ? kPreviewManyTrackerCount : 1;
	for (int i = 0; i < trackerCount; ++i)
	{
		VRDevice tracker;
		tracker.id = 3 + i;
		tracker.deviceClass = vr::TrackedDeviceClass_GenericTracker;
		// -uipreview-many has the design's Tundra Tracker on the chest.
		const bool tundra = g_uiPreviewMany && i == 3;
		tracker.model = tundra ? "Tundra Tracker" : "VIVE Tracker 3.0";
		tracker.renderModel = tundra ? "{tundra_labs}tundra_tracker" : "{htc}vr_tracker_vive_3_0";
		char serial[32];
		snprintf(serial, sizeof serial, "LHR-77E5A2%02X", 0x11 + i);
		tracker.serial = serial;
		tracker.trackingSystem = "lighthouse";
		// Demo the row states: the lone tracker sits disconnected; with
		// -uipreview-many they fan out across battery levels instead, the
		// last one staying disconnected to keep that row state on screen.
		tracker.connected = g_uiPreviewMany ? (i != kPreviewManyTrackerCount - 1) : false;
		tracker.tracking = tracker.connected;
		if (tracker.connected)
			tracker.battery = 0.95f - 0.12f * (float)i;
		const char *art =
			!tracker.connected ? "htc\\resources\\icons\\tracker_status_off_2x.png" :
			tracker.battery < kLowBattery ? "htc\\resources\\icons\\tracker_status_ready_low_2x.png" :
			"htc\\resources\\icons\\tracker_status_ready.png"; // no 2x ready ships
		tracker.iconPath = PreviewIconPath(art);
		state.devices.push_back(tracker);
	}

	return state;
}

// What the updater reports, with a downloaded release faked for the home
// screen's notices preview (the updater never runs in preview).
questcal::update::Snapshot CurrentUpdate()
{
	if (g_uiPreviewMode && g_uiPreviewScenario == PreviewScenario::Notices)
	{
		questcal::update::Snapshot ready;
		ready.state = questcal::update::State::Ready;
		ready.version = "1.2.1";
		return ready;
	}
	return questcal::update::AppUpdater.GetSnapshot();
}

// An activity entry from secondsAgo, as the monitors would have written it.
static void PreviewActivity(const char *text, CalibrationContext::Tone tone, double secondsAgo,
	CalibrationContext::Event event)
{
	CalCtx.PushActivity(text, tone, event);
	CalCtx.activity.back().unixTime = static_cast<double>(std::time(nullptr)) - secondsAgo;
}

// The guide opened on the fake pair: Touch Pro Right against the first
// VIVE tracker.
static void PreviewGuide()
{
	CalCtx.referenceID = 3 + kPreviewManyTrackerCount;
	CalCtx.targetID = 3;
	CalCtx.pendingReferenceTrackingSystem = "oculus";
	CalCtx.pendingTargetTrackingSystem = "lighthouse";
	OpenGuide(false, false);
}

// UI preview (-uipreview): plausible fake state so every part of the interface
// renders without SteamVR. Fake devices come from LoadVRState; profile saves
// are disabled while previewing.
void SetupPreviewState()
{
	CalCtx.validProfile = true;
	CalCtx.enabled = true;
	// Past the first-launch setup, unless a scenario shows it.
	CalCtx.onboarded = true;
	// The Lighthouse module installed, as the design shows the sidebar.
	CalCtx.modules.lighthouse = questcal::ModuleStatus::Installed;
	CalCtx.referenceTrackingSystem = "oculus";
	CalCtx.targetTrackingSystem = "lighthouse";

	// Three of the fake trackers carry player-given names so the named and
	// unnamed row treatments sit side by side.
	if (g_uiPreviewMany)
	{
		CalCtx.deviceNames["LHR-77E5A212"] = "Hip";
		CalCtx.deviceNames["LHR-77E5A213"] = "Left foot";
		CalCtx.deviceNames["LHR-77E5A214"] = "Chest";
	}

	CalCtx.lastResult.valid = true;
	CalCtx.lastResult.rotationRmsDeg = 2.53;
	CalCtx.lastResult.translationRmsMeters = 0.010;
	CalCtx.lastResult.timeOffset = 0.0038;
	CalCtx.lastResult.scale = 1.002;
	CalCtx.transform.scale = 1.002;

	CalCtx.calibrationUnixTime = static_cast<double>(std::time(nullptr)) - 180.0;
	CalCtx.alignment = CalibrationContext::AlignmentHealth::Stale;
	CalCtx.driftScore = 0.70;
	CalCtx.driftSlideEvents = 21;
	CalCtx.driftMaxSlideM = 0.08;
	CalCtx.discontinuousLossEvents = 0;

	CalCtx.appliedTimeOffset = -0.0038;
	CalCtx.applyTimeOffset = true;
	CalCtx.fieldEnabled = true;
	CalCtx.chaperone.valid = true;
	CalCtx.chaperone.geometry.resize(26);
	CalCtx.chaperone.playSpaceSize.v[0] = 2.1f;
	CalCtx.chaperone.playSpaceSize.v[1] = 2.4f;
	CalCtx.chaperone.copyUnixTime = static_cast<double>(std::time(nullptr)) - 840.0;

	CalibrationContext::FieldAnchor anchor;
	anchor.position = Eigen::Vector3d(1.2, 1.1, -0.8);
	anchor.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.004, Eigen::Vector3d::UnitY()));
	anchor.translationMeters = Eigen::Vector3d(0.02, 0.0, -0.01);
	CalCtx.fieldAnchors.push_back(anchor);

	// Continuous calibration in its healthy maintaining state (the tracker
	// serial matches the first fake VIVE tracker in -uipreview-many).
	CalCtx.continuousEnabled = true;
	CalCtx.continuousTrackerSerial = "LHR-77E5A211";
	CalCtx.hideMountedTracker = true;
	CalCtx.mountExtrinsic.valid = true;
	CalCtx.mountExtrinsic.rot = Eigen::Quaterniond(
		Eigen::AngleAxisd(2.7, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
	CalCtx.mountExtrinsic.pos = Eigen::Vector3d(0.05, -0.08, 0.03);
	CalCtx.mountExtrinsic.rotRmsDeg = 0.21;
	CalCtx.mountExtrinsic.posRmsM = 0.004;
	CalCtx.continuousState = questcal::ContinuousAlignment::State::Tracking;
	CalCtx.continuousDeviation.valid = true;
	CalCtx.continuousDeviation.yawDeg = 0.08;
	CalCtx.continuousDeviation.tiltDeg = 0.11;
	CalCtx.continuousDeviation.posM = 0.004;
	CalCtx.continuousScatterRotDeg = 0.19;
	CalCtx.continuousScatterPosM = 0.006;
	CalCtx.autoCorrectionsApplied = 14;
	CalCtx.lastAutoCorrectionUnixTime = static_cast<double>(std::time(nullptr)) - 42.0;

	// Base station visibility as the lighthouse log would have reported it:
	// every device learned four stations, the left controller is down to
	// one (the red figure), one tracker lost one, and two stations carry
	// most of the drops so the line under the panes has an order.
	{
		using K = lighthouselog::Event::Kind;
		static const std::map<int, uint32_t> ids = {
			{ 5, 0xD3D4E73Bu }, { 8, 0x170EE067u }, { 9, 0xF210FBA6u }, { 16, 0x04D47FB4u } };
		auto line = [](K kind, const std::string &serial, int channel, std::vector<int> visible)
		{
			lighthouselog::Event e;
			e.kind = kind;
			e.serial = serial;
			e.channel = channel;
			e.stationId = ids.at(channel);
			e.visibleKnown = true;
			e.visibleChannels = std::move(visible);
			for (int c : e.visibleChannels)
				e.visibleIds.push_back(ids.at(c));
			e.historical = true;
			return e;
		};
		auto &vis = CalCtx.lighthouse;
		std::vector<std::string> serials = { "LHR-A3C36EA5", "LHR-841C98C3" };
		for (int i = 0; i < kPreviewManyTrackerCount; ++i)
			serials.push_back(FormatString("LHR-77E5A2%02X", 0x11 + i));
		for (const auto &s : serials)
		{
			vis.Apply(line(K::StationAdded, s, 5, { 5 }), 0.0);
			vis.Apply(line(K::StationAdded, s, 8, { 5, 8 }), 0.0);
			vis.Apply(line(K::StationAdded, s, 9, { 5, 8, 9 }), 0.0);
			vis.Apply(line(K::StationAdded, s, 16, { 5, 8, 9, 16 }), 0.0);
		}
		for (int n = 0; n < 7; ++n)
		{
			vis.Apply(line(K::StationDropped, "LHR-77E5A211", 16, { 5, 8, 9 }), 0.0);
			vis.Apply(line(K::StationAdded, "LHR-77E5A211", 16, { 5, 8, 9, 16 }), 0.0);
		}
		for (int n = 0; n < 3; ++n)
		{
			vis.Apply(line(K::StationDropped, "LHR-A3C36EA5", 5, { 8, 9, 16 }), 0.0);
			vis.Apply(line(K::StationAdded, "LHR-A3C36EA5", 5, { 5, 8, 9, 16 }), 0.0);
		}
		vis.Apply(line(K::StationDropped, "LHR-77E5A212", 16, { 5, 8, 9 }), 0.0);
		vis.Apply(line(K::StationDropped, "LHR-841C98C3", 16, { 5, 8, 9 }), 0.0);
		vis.Apply(line(K::StationDropped, "LHR-841C98C3", 8, { 5, 9 }), 0.0);
		vis.Apply(line(K::StationDropped, "LHR-841C98C3", 5, { 9 }), 0.0);
		CalCtx.lighthouseLogAvailable = true;
		CalCtx.lighthouseLogPath = lighthouselog::DefaultLogPath();
	}

	// A recent history for the home screen and the activity sheet, oldest
	// first, as the monitors and runs write it: the design's. The activity
	// sheet's runs on to the tracker switching off.
	using Event = CalibrationContext::Event;
	using Tone = CalibrationContext::Tone;
	const bool sheet = g_uiPreviewScenario == PreviewScenario::Activity;
	const double minute = 60.0, last = sheet ? 24.0 * minute : 42.0;
	// Yesterday at 21:47, whatever the time now.
	const std::time_t now = std::time(nullptr);
	std::tm local{};
	const double sinceMidnight = localtime_s(&local, &now) == 0
		? local.tm_hour * 3600.0 + local.tm_min * 60.0 + local.tm_sec : 12.0 * 3600.0;
	PreviewActivity("Calibration complete.", Tone::Good, sinceMidnight + (2.0 * 60.0 + 13.0) * minute,
		Event::Calibrated);
	PreviewActivity("Calibrated with Touch Pro Right and VIVE Tracker 3.0.", Tone::Good, last + 43.0 * minute,
		Event::Calibrated);
	PreviewActivity("Chaperone restored.", Tone::Good, last + 38.0 * minute, Event::Chaperone);
	if (sheet)
		PreviewActivity("Anchor added. This spot now has its own correction.", Tone::Good, last + 21.0 * minute,
			Event::Anchor);
	PreviewActivity("Re-aligned your trackers after your headset's tracking shifted.", Tone::Good, last,
		Event::Realigned);
	if (sheet)
	{
		PreviewActivity("Continuous calibration paused: tracking drifted too far to correct safely.", Tone::Warn,
			13.0 * minute, Event::Paused);
		PreviewActivity("SteamVR switched the headset tracker off after it sat still for 5 minutes.", Tone::Warn,
			30.0, Event::TrackerOff);
	}

	switch (g_uiPreviewScenario)
	{
	case PreviewScenario::Guide:
	case PreviewScenario::GuideWait:
	case PreviewScenario::Move:
	case PreviewScenario::Result:
	case PreviewScenario::Failed:
		PreviewGuide();
		if (g_uiPreviewScenario == PreviewScenario::GuideWait)
		{
			// Started the moment the tracker woke: the run waits for its new
			// solution to settle before it measures.
			CalCtx.state = CalibrationState::Begin;
			CalCtx.run.waitInstruction = "Waiting for VIVE Tracker 3.0 to settle.";
			CalCtx.run.waitNote =
				"Calibration starts on its own in a few seconds. Keep the tracker in view of its base stations.";
			s_guide.stage = GuideStage::Running;
		}
		if (g_uiPreviewScenario == PreviewScenario::Move)
		{
			// Three seconds into a ten-second run.
			BeginGuidedRun();
			s_guide.stage = GuideStage::Running;
			s_guide.countdownStart = -(kCountdownSeconds + 3.0);
		}
		if (g_uiPreviewScenario == PreviewScenario::Result)
		{
			CalCtx.lastRunPassed = true;
			CalCtx.Outcome("Calibration complete", "Check in VR that your trackers line up with your body.",
				"", "Rotation RMS 2.53 degrees; position RMS 1.0 cm", CalibrationContext::Tone::Good,
				CalibrationContext::Event::Calibrated, "Calibrated with Touch Pro Right and VIVE Tracker 3.0.");
			s_guide.stage = GuideStage::Done;
		}
		if (g_uiPreviewScenario == PreviewScenario::Failed)
		{
			// The refused solve a preview run under this flag ends in too.
			CalCtx.lastRunPassed = false;
			CalCtx.Outcome("Calibration failed", "The devices didn't turn far enough.",
				"Make bigger turns, and keep them pressed together.",
				"Rotation coverage 0.21 of 1.00 (need 0.60)", CalibrationContext::Tone::Warn);
			s_guide.stage = GuideStage::Done;
			s_modalDetails = true;
		}
		break;
	case PreviewScenario::Pair:
		OpenSheet(Sheet::Pair);
		break;
	case PreviewScenario::Chaperone:
		OpenSheet(Sheet::Chaperone);
		break;
	case PreviewScenario::Anchors:
		OpenSheet(Sheet::Anchors);
		break;
	case PreviewScenario::Activity:
		OpenSheet(Sheet::Activity);
		break;
	case PreviewScenario::ClearCalibration:
		OpenDialog(Dialog::ClearCalibration);
		break;
	case PreviewScenario::ChaperoneWarning:
		CalCtx.chaperoneWarningAck = false;
		OpenSheet(Sheet::Chaperone);
		OpenDialog(Dialog::ChaperoneWarning);
		break;
	case PreviewScenario::Notices:
		// Calibration switched off for a missing headset, a downloaded update
		// waiting (CurrentUpdate), and an error in the banner.
		CalCtx.enabled = false;
		CalCtx.disableReason = CalibrationContext::DisableReason::HmdMismatch;
		CalCtx.ReportError("Couldn't read your chaperone from SteamVR, so your protected walls weren't changed.\n",
			CalibrationContext::ErrorSource::ChaperoneMonitor);
		break;
	case PreviewScenario::Frozen:
		// The loop measured a deviation too large to correct and stopped:
		// the band shows its two actions and the activity card the event.
		CalCtx.continuousState = questcal::ContinuousAlignment::State::Frozen;
		CalCtx.continuousFreezeFromRestart = true;
		CalCtx.continuousDeviation.yawDeg = 2.6;
		CalCtx.continuousDeviation.tiltDeg = 0.9;
		CalCtx.continuousDeviation.posM = 0.11;
		CalCtx.Tell("Continuous calibration paused: tracking drifted too far to correct safely.",
			CalibrationContext::Tone::Warn, CalibrationContext::Event::Paused);
		break;
	case PreviewScenario::TrackerOff:
		// The headset came off for a while and SteamVR switched its tracker
		// off for sitting still: the loop waits, the feed says why and how to
		// keep it from happening.
		CalCtx.continuousTrackerConnected = false;
		CalCtx.continuousState = questcal::ContinuousAlignment::State::Inactive;
		CalCtx.continuousDeviation.valid = false;
		// Unmaintained for long enough that the drift evidence calls it bad.
		CalCtx.driftScore = questcal::DriftVeryPoorScore;
		CalCtx.Tell("SteamVR switched the headset tracker off after it sat still for 5 minutes.",
			CalibrationContext::Tone::Warn, CalibrationContext::Event::TrackerOff);
		CalCtx.Tell("To keep this from happening, set \xE2\x80\x9CTurn off controllers after\xE2\x80\x9D to Never in "
			"SteamVR's Startup / Shutdown settings.", CalibrationContext::Tone::Neutral, CalibrationContext::Event::TrackerOff);
		break;
	case PreviewScenario::SetupWelcome:
	case PreviewScenario::SetupHeadset:
	case PreviewScenario::SetupTrackers:
	case PreviewScenario::SetupUpdates:
	case PreviewScenario::SetupCalibrate:
		// A first launch still in its setup.
		CalCtx.onboarded = false;
		g_setupStep = static_cast<SetupStep>(static_cast<int>(g_uiPreviewScenario) - static_cast<int>(PreviewScenario::SetupWelcome));
		[[fallthrough]];
	case PreviewScenario::Empty:
		// First launch: no profile, no chaperone, nothing measured.
		CalCtx.validProfile = false;
		CalCtx.enabled = false;
		CalCtx.lastResult.valid = false;
		CalCtx.fieldAnchors.clear();
		CalCtx.chaperone.valid = false;
		CalCtx.chaperone.geometry.clear();
		CalCtx.continuousEnabled = false;
		CalCtx.continuousTrackerSerial.clear();
		CalCtx.mountExtrinsic.valid = false;
		CalCtx.continuousState = questcal::ContinuousAlignment::State::Inactive;
		CalCtx.continuousDeviation.valid = false;
		CalCtx.autoCorrectionsApplied = 0;
		CalCtx.activity.clear();
		break;
	case PreviewScenario::Lighthouse:
		g_page = Page::Lighthouse;
		break;
	case PreviewScenario::Settings:
	case PreviewScenario::SettingsMore:
		g_page = Page::Settings;
		break;
	case PreviewScenario::Editor:
		// A transform with something in every field, as a real one has.
		g_page = Page::Settings;
		CalCtx.transform.rotation = CalibrationContext::RebuildRotationFromEuler(Eigen::Vector3d(-0.0482, -1.8422, 0.3175));
		CalCtx.transform.translationMeters = Eigen::Vector3d(0.124063, -0.031127, 0.41889);
		SeedTransformEditorDraft();
		CalCtx.state = CalibrationState::Editing;
		break;
	case PreviewScenario::Credits:
		g_page = Page::Settings;
		OpenSheet(Sheet::Credits);
		break;
	case PreviewScenario::Healthy:
		break;
	}
}
