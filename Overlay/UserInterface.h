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
// too), the calibration editor, the credits and each page of the first-launch
// setup. Each implies -uipreview-many.
enum class PreviewScenario
{
	Healthy, Frozen, TrackerOff, Failed, Empty, Guide, GuideWait, Move, Result, Lighthouse, Settings,
	Pair, Chaperone, Anchors, Activity, ClearCalibration, ChaperoneWarning, Notices, SettingsMore, Editor, Credits,
	SetupWelcome, SetupHeadset, SetupTrackers, SetupUpdates, SetupCalibrate
};
extern PreviewScenario g_uiPreviewScenario;

void ApplyTheme();
void SetupPreviewState();
void BuildMainWindow(bool runningInOverlay);

// The desktop window's own title bar, above the screen the overlay shows. The
// window procedure (QuestCalibrator.cpp) hit-tests it and runs its buttons, the
// UI draws it; both read this layout. The buttons sit at the right edge,
// minimize then close.
constexpr int TitleBarHeight = 32;
constexpr int TitleBarButtonWidth = 46;
enum class TitleBarButton { None, Minimize, Close };
struct TitleBarState
{
	TitleBarButton hovered = TitleBarButton::None;
	TitleBarButton pressed = TitleBarButton::None;
	bool focused = true;
};
// Drawn at negative y, outside the overlay's texture and -shot pictures.
void BuildTitleBar(const TitleBarState &state);
void RequestApplicationExit();
