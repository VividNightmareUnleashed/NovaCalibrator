#pragma once

// Shared UI internals. The public interface is declared in UserInterface.h.

#include "stdafx.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "CalibrationGuide.h"
#include "Configuration.h"
#include "ProfileValidation.h"
#include "Updater.h"
#include "UiLayout.h"
#include "UiKit.h"
#include "Localization.h"
#include "../common/Protocol.h"
#include "../common/Version.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GL/gl3w.h>
#include <wincodec.h>
#include <shellapi.h>
#pragma comment(lib, "windowscodecs.lib")
#pragma comment(lib, "shell32.lib")

// Every widget below translates the text it is handed, so a call site passes
// English and only direct ImGui and draw-list text needs Tr of its own.
using questcal::i18n::Tr;

struct VRDevice
{
	int id = -1;
	vr::TrackedDeviceClass deviceClass;
	std::string model;
	std::string serial;
	std::string trackingSystem;
	std::string iconPath;   // absolute path to the SteamVR device icon, matched to state (ready/low/off)
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
	bool connected = true;
	bool tracking = false;       // pose valid and Running_OK at the last 1 Hz refresh
	float battery = -1.0f;       // 0..1, or -1 when the device reports none
	bool charging = false;
};

struct VRState
{
	std::vector<std::string> trackingSystems;
	std::vector<VRDevice> devices;
};

// Below this level the device icon swaps to its red low-battery art and the
// battery bar fill goes red.
static const float kLowBattery = 0.15f;

// Rolls a setting back when it cannot be saved. Profile-backed toggles use
// SaveProfileFieldEdit instead, which persists a candidate record before
// touching live state, so they need no rollback.
template<typename T>
static void SaveSettingOrRestore(T &value, const T &previous)
{
	if (!SaveSettingsWithResult(CalCtx).settingsSaved)
		value = previous;
}

struct IdentifyPulseState
{
	bool active = false;
	uint32_t targetId = vr::k_unTrackedDeviceIndexInvalid;
	uint32_t referenceId = vr::k_unTrackedDeviceIndexInvalid;
	unsigned pulsesRemaining = 0;
	double nextPulseTime = 0.0;
};

struct DeviceIconTex
{
	GLuint tex = 0;
	int w = 0, h = 0;
	bool failed = false;
};

// Unknown is not a severity, it is the absence of one: nothing has been
// measured to rate. Sorting it below Good keeps every "bad enough to say
// something" test (>= Rating_Poor) reading false for it with no special case.
enum CalRating { Rating_Unknown = -1, Rating_Good = 0, Rating_Decent, Rating_Poor, Rating_VeryPoor };

// What the continuous loop is doing, derived from the same terms ContinuousTick
// gates its own shouldRun on. Every status text, colour, rating and nudge reads
// this rather than re-deriving a slice of it.
enum class ContinuousStatus
{
	Off,         // the feature is switched off
	NoTracker,   // enabled, but no mounted tracker picked
	NeedsMount,  // picked, but no mount offset learned for it yet
	NotRunning,  // armed, yet the loop's runtime preconditions do not hold
	TrackerOff,  // running, but the headset tracker is not connected (SteamVR switched it off)
	Gathering,   // running, not enough observations yet (State::Inactive)
	Tracking,
	Coasting,    // also: running, but the tracker has delivered no tracked pose lately
	HeadsetUnseen, // running, but the headset's own stream stopped (a streamed headset pausing)
	Frozen,
	Holding,
};

enum class GuideStage { Idle, GetSet, Countdown, Running, Done };

enum class GuideDemo { Wrist, Mounted, HeadsetContact };

struct GuideState
{
	GuideDemo demo = GuideDemo::Wrist;
	GuideStage stage = GuideStage::Idle;
	bool openRequested = false;
	bool animate = true;
	double animationTime = 0.0;
	bool anchor = false;       // field-anchor run
	bool mountRun = false;     // head-referenced run for the headset tracker
	double countdownStart = 0.0;
	double lastMetricsTime = -1.0;
	questcal::GuideMetrics metrics;
};

static const float kCountdownSeconds = 3.0f;

// Exceed the four-row scroll threshold and exercise several battery states.
static const int kPreviewManyTrackerCount = 6;

// The pages the sidebar switches between. Lighthouse and Smoothing are
// optional modules (CalCtx.modules); a page whose module is not installed
// stays greyed out in the sidebar.
enum class Page { Calibration = 0, Lighthouse, Settings };

// What covers the page: one sheet at a time, and a dialog that may sit over
// it. Both live at the window's root (UiSheets.cpp), so anything can open
// them.
enum class Sheet { None, Pair, Calibrate, Chaperone, Anchors, Activity, Editor, Credits };
enum class Dialog { None, ClearCalibration, ClearAnchors, ChaperoneWarning };

// The sidebar's width; pages lay out in what is left of the window.
static const float kSidebarW = 248.0f;

// Shared state, each owned by one file.
extern IdentifyPulseState g_identifyPulse;
extern Page g_page;
extern double g_chapWarnOpenedAt;
extern GuideState s_guide;
extern bool s_modalDetails;

// Functions shared across the Ui*.cpp files.
void IconHMD(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconController(ImDrawList *dl, ImVec2 c, float s, ImU32 col, bool leftHand);
void IconTracker(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
bool LoadTextureFromFile(const char *path, GLuint *outTex, int *outW, int *outH);
bool LoadGuideTexture(GuideDemo demo, GLuint *outTex);
const std::string &GuideModelCredits();
const DeviceIconTex *GetDeviceIconTex(const std::string &path);
bool FileExists(const std::string &path);
std::string Prefer2x(const std::string &path);
bool EscapePressed();
void ShowTip(const char *text, bool leftOfCursor = false);
void BuildLighthouseScreen(const VRState &state);
std::string FormatString(const char *fmt, ...);
bool PoseChannelDown();
ContinuousStatus ContinuousStatusNow();
const char *ContinuousStatusLine(ContinuousStatus status);
CalRating ComputeCalibrationRating(ContinuousStatus continuous);
const char *RatingLabel(CalRating r);
const char *RecalibrationNudge(CalRating rating);
std::optional<std::string> FormatUnixAge(double unixTime);
bool ProtectChaperone();
void DeviceIcon(ImDrawList *dl, const VRDevice &dev, ImVec2 c, float s, ImU32 col);
const std::string *FindDeviceName(const std::string &serial);
std::string DeviceDisplayName(const VRDevice &dev);
void CommitDeviceName(const VRDevice &dev, const char *text);
void EnsureDeviceSelection(const VRState &state, uint32_t &selected, const std::string &system);
std::string FriendlySystemName(const std::string &raw);
// The tracking systems the pair is picked from: the reference (the headset's)
// and every other one; settles the pending picks onto systems that exist.
void SettlePairSystems(const VRState &state);
void StartIdentifyPulse(uint32_t targetId, uint32_t referenceId);
void OpenGuide(bool anchor, bool mountRun);
void StartMountSetup(const VRState &state);
bool BeginGuidedRun();
// The motion demonstration in box, with its caption along the bottom unless
// caption is false.
void DrawGuideAnimation(ImDrawList *dl, const FlexRect &box, double t, GuideDemo demo, bool caption = true);
void ReleaseGuideTexture();
void SeedTransformEditorDraft();
void SaveProfileEditorDraft();
std::string PreviewIconPath(const char *driverRelative);
// What the updater reports (UiPreview.cpp fakes a ready update for a preview).
questcal::update::Snapshot CurrentUpdate();
void UpdateIdentifyPulse(double now);
VRState LoadVRState();
VRState PreviewVRState();

// The window's pages, each laid out from origin across width; each returns
// the height it took so the page can scroll.
float BuildHomePage(const VRState &state, ImVec2 origin, float width);
float BuildLighthousePage(const VRState &state, ImVec2 origin, float width);
float BuildSettingsPage(const VRState &state, ImVec2 origin, float width);

// Sheets and dialogs (UiSheets.cpp). Opening one from anywhere takes effect
// at the window's root on the next frame.
void OpenSheet(Sheet sheet);
void CloseSheet();
Sheet CurrentSheet();
void OpenDialog(Dialog dialog);
void CloseDialog();
void BuildOverlays(const VRState &state);
bool DialogOpen();
// A sheet's or dialog's panel, centred and drawn; the rectangle inside its
// padding is returned for the content.
FlexRect SheetPanel(float width, float height);
FlexRect DialogPanel(float width, float height);
// The title row every sheet starts with: its title (translated by the
// caller) and the close button. Returns true when the close button was
// pressed, or Escape with no dialog over the sheet.
bool SheetHeader(const FlexRect &content, const char *title);
// Each sheet sizes and draws its own panel.
void BuildPairSheet(const VRState &state);
void BuildCalibrateSheet(const VRState &state);
void BuildChaperoneSheet();
void BuildAnchorsSheet();
void BuildActivitySheet();
void BuildEditorSheet();
void BuildCreditsSheet();
void BuildClearCalibrationDialog();
void BuildClearAnchorsDialog();
void BuildChaperoneWarningDialog();
// Picks the guide's motion demo for the devices about to be calibrated.
void ChooseGuideDemo(const VRState &state);
// The calibration sheet's own size, which its stages share.
static const float kCalibrateSheetW = 880.0f;
static const float kCalibrateSheetH = 626.0f;

// The banner for CalCtx.uiError across the top of a page; returns its height
// (zero when there is nothing to say).
float BuildErrorBanner(ImVec2 origin, float width);
// An activity entry's local time, as "HH:MM".
std::string ActivityClock(double unixTime);
// CalCtx.activity's indices in the order the feed is read: newest first.
std::vector<size_t> ActivityNewestFirst();

// Pictures (UiWidgets.cpp): a built-in PNG resource as a texture, loaded once.
const DeviceIconTex *ArtTexture(const char *resource);
// A device's picture fitted into box: SteamVR's own art for it, or its vector
// glyph when there is none.
void DrawDeviceArt(ImDrawList *dl, const VRDevice &dev, const FlexRect &box, ImU32 fallbackInk);
