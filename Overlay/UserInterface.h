#pragma once

// UI preview mode (-uipreview): run the window with fake devices and no
// SteamVR, for iterating on the interface. Profile saves are disabled.
// -uipreview-many additionally fakes a batch of trackers to stress the device
// list past its scroll threshold (count: kPreviewManyTrackerCount).
extern bool g_uiPreviewMode;
extern bool g_uiPreviewMany;

// Preview scenarios cover frozen alignment, a headset tracker SteamVR switched
// off, a failed solve, an empty profile, the calibration sheet's stages, the
// pair, chaperone, anchors and activity sheets, the confirmation dialogs, the
// home screen's notices, the Lighthouse page, Settings (scrolled to its end
// too), the calibration editor and the credits. Each implies -uipreview-many.
enum class PreviewScenario
{
	Healthy, Frozen, TrackerOff, Failed, Empty, Guide, GuideWait, Move, Result, Lighthouse, Settings,
	Pair, Chaperone, Anchors, Activity, ClearCalibration, ChaperoneWarning, Notices, SettingsMore, Editor, Credits
};
extern PreviewScenario g_uiPreviewScenario;

void ApplyTheme();
void SetupPreviewState();
void BuildMainWindow(bool runningInOverlay);
void RequestApplicationExit();
