// Window entry point: the backdrop, the sidebar, the current page, and the
// sheets and dialogs over them.
#include "stdafx.h"
#include "UiInternal.h"


bool g_uiPreviewMode = false;
bool g_uiPreviewMany = false;
PreviewScenario g_uiPreviewScenario = PreviewScenario::Healthy;
ImFont *g_fontBody = nullptr;
ImFont *g_fontSmall = nullptr;
ImFont *g_fontTitle = nullptr;

IdentifyPulseState g_identifyPulse;
Page g_page = Page::Calibration;

void UpdateIdentifyPulse(double now)
{
	if (!g_identifyPulse.active)
		return;
	auto system = vr::VRSystem();
	if (!system || g_identifyPulse.pulsesRemaining == 0)
	{
		g_identifyPulse = IdentifyPulseState();
		return;
	}

	// Driven by the main loop, so shutdown is safe and pulse trains never
	// overlap; it asks to be woken every 5 ms while pulsing.
	CalCtx.wantedUpdateInterval = std::min(CalCtx.wantedUpdateInterval, 0.005);
	if (now < g_identifyPulse.nextPulseTime)
		return;
	if (g_identifyPulse.targetId < vr::k_unMaxTrackedDeviceCount)
		system->TriggerHapticPulse(g_identifyPulse.targetId, 0, 2000);
	if (g_identifyPulse.referenceId < vr::k_unMaxTrackedDeviceCount)
		system->TriggerHapticPulse(g_identifyPulse.referenceId, 0, 2000);
	--g_identifyPulse.pulsesRemaining;
	g_identifyPulse.nextPulseTime = now + 0.005;
}

void StartIdentifyPulse(uint32_t targetId, uint32_t referenceId)
{
	g_identifyPulse = { true, targetId, referenceId, 100, ImGui::GetTime() };
}

// ---------------------------------------------------------------------------
// Sidebar
// ---------------------------------------------------------------------------

static FlexRect NavSlot(ImVec2 windowPos, float top)
{
	FlexRect r;
	r.min = ImVec2(windowPos.x + 12.0f, top);
	r.max = ImVec2(windowPos.x + kSidebarW - 12.0f, top + 48.0f);
	return r;
}

static void BuildSidebar(bool runningInOverlay)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const ImVec2 o = ImGui::GetWindowPos();
	const float h = ImGui::GetWindowHeight();
	dl->AddRectFilled(o, ImVec2(o.x + kSidebarW, o.y + h), ui::col::Sidebar);

	// The app's mark and name.
	if (const DeviceIconTex *icon = ArtTexture("ART_ICON_SMALL"))
		dl->AddImage(static_cast<ImTextureID>(icon->tex), ImVec2(o.x + 20.0f, o.y + 24.0f), ImVec2(o.x + 56.0f, o.y + 60.0f));
	ui::DrawLine(dl, ui::TextStyle{ ui::Weight::SemiBold, 17.0f, 36.0f }, ImVec2(o.x + 68.0f, o.y + 24.0f), ui::col::Text,
		"Nova Calibrator");

	// The pages. A module the installer left out greys its page out.
	const bool lighthouse = questcal::Modules::On(CalCtx.modules.lighthouse);
	float y = o.y + 92.0f;
	if (ui::NavRow("##navcalibration", NavSlot(o, y), ui::Icon::Reticle, "Calibration", g_page == Page::Calibration, false))
		g_page = Page::Calibration;
	y += 52.0f;
	if (ui::NavRow("##navlighthouse", NavSlot(o, y), ui::Icon::Lighthouse, "Lighthouse", g_page == Page::Lighthouse, !lighthouse,
		nullptr, "Lighthouse module is currently not installed. Select it during installation."))
		g_page = Page::Lighthouse;
	y += 52.0f;
	ui::NavRow("##navsmoothing", NavSlot(o, y), ui::Icon::Smoothing, "Smoothing", false, true, "Soon", "Work in progress");

	// Settings and the version at the foot, with a line on how to drive the
	// window when it would otherwise be a mystery.
	const float versionY = o.y + h - 24.0f - 20.0f;
	const float settingsY = versionY - 10.0f - 48.0f;
	static double s_lastMouseMove = 0.0;
	const ImGuiIO &io = ImGui::GetIO();
	if (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)
		s_lastMouseMove = ImGui::GetTime();
	// The keyboard works (arrows and Enter), and nothing else says so: a
	// quiet line once the pointer has rested over the window.
	const bool keyboardHint = ImGui::IsMousePosValid() && ImGui::GetTime() - s_lastMouseMove > 6.0;
	const char *hint = runningInOverlay ? Tr("Close the SteamVR dashboard to use the mouse")
		: keyboardHint ? Tr("Arrow keys to move \xC2\xB7 Enter to select") : nullptr;
	if (hint)
	{
		const ui::TextLines lines = ui::WrapText(ui::type::Caption, hint, kSidebarW - 28.0f);
		ui::DrawLines(dl, ui::type::Caption, lines, ImVec2(o.x + 14.0f, settingsY - 12.0f - lines.height), kSidebarW - 28.0f,
			ui::col::Faint);
	}
	if (ui::NavRow("##navsettings", NavSlot(o, settingsY), ui::Icon::Gear, "Settings", g_page == Page::Settings, false))
		g_page = Page::Settings;
	ui::DrawLine(dl, ui::type::Footnote, ImVec2(o.x + 26.0f, versionY), ui::Rgba(236, 238, 244, 0.50f),
		Tr(FormatString("Version %s", QUESTCAL_VERSION_STRING)).c_str());
}

// ---------------------------------------------------------------------------
// Pages
// ---------------------------------------------------------------------------

// A page that has not moved to the design kit yet: its old flowing layout,
// inset by the page margins.
static float LegacyPage(void (*build)(const VRState &), const VRState &state, ImVec2 origin, float width)
{
	ImGui::SetCursorScreenPos(ImVec2(origin.x + 40.0f, origin.y));
	ImGui::BeginChild("##legacy", ImVec2(width - 80.0f, 0.0f), ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoBackground);
	build(state);
	ImGui::Dummy(ImVec2(0.0f, 8.0f));
	ImGui::EndChild();
	return ImGui::GetItemRectSize().y;
}

static void BuildPage(const VRState &state)
{
	const float width = ImGui::GetContentRegionAvail().x;
	const ImVec2 origin = ImGui::GetCursorScreenPos();
	// The error banner sits across the top of whatever page is showing.
	const float banner = BuildErrorBanner(ImVec2(origin.x + 40.0f, origin.y + 24.0f), width - 80.0f);
	const float top = banner > 0.0f ? 24.0f + banner + 22.0f : 30.0f;
	const ImVec2 pageOrigin(origin.x, origin.y + top);
	float height = 0.0f;
	switch (g_page)
	{
	case Page::Calibration:
		height = BuildHomePage(state, pageOrigin, width);
		break;
	case Page::Lighthouse:
		height = LegacyPage(BuildLighthouseScreen, state, pageOrigin, width);
		break;
	case Page::Settings:
		height = LegacyPage(BuildSettingsScreen, state, pageOrigin, width);
		break;
	}
	ImGui::SetCursorScreenPos(origin);
	ImGui::Dummy(ImVec2(width, top + height + 30.0f));
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

// The main window never scrolls itself: the page does.
static const ImGuiWindowFlags bareWindowFlags =
	ImGuiWindowFlags_NoTitleBar |
	ImGuiWindowFlags_NoResize |
	ImGuiWindowFlags_NoMove |
	ImGuiWindowFlags_NoScrollbar |
	ImGuiWindowFlags_NoScrollWithMouse |
	ImGuiWindowFlags_NoBackground;

void BuildMainWindow(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	const bool visible = ImGui::Begin("MainWindow", nullptr, bareWindowFlags);
	ImGui::PopStyleVar();
	if (!visible)
	{
		ImGui::End();
		return;
	}

	// Refreshed at 1 Hz: re-querying vrserver's properties and the icon files
	// at the ~90 Hz dashboard frame rate would be a cross-process call storm.
	static VRState state;
	static double lastStateRefresh = -1e9;
	double now = ImGui::GetTime();
	UpdateIdentifyPulse(now);
	if (now - lastStateRefresh >= 1.0)
	{
		state = LoadVRState();
		lastStateRefresh = now;
	}
	// The picks stay on devices that exist, whatever is showing.
	SettlePairSystems(state);

	ui::PageBackdrop(ImGui::GetWindowDrawList(), ImVec2(0.0f, 0.0f), io.DisplaySize);
	BuildSidebar(runningInOverlay);

	// A page whose module went away falls back to calibration.
	if (g_page == Page::Lighthouse && !questcal::Modules::On(CalCtx.modules.lighthouse))
		g_page = Page::Calibration;
	ImGui::SetCursorScreenPos(ImVec2(kSidebarW, 0.0f));
	// NavFlattened: keyboard focus walks straight from the sidebar into the
	// page's controls instead of stopping on the child as one item.
	ImGui::BeginChild("##page", ImVec2(io.DisplaySize.x - kSidebarW, io.DisplaySize.y), ImGuiChildFlags_NavFlattened,
		ImGuiWindowFlags_NoBackground);
	BuildPage(state);
	ImGui::EndChild();

	BuildOverlays(state);
	ImGui::End();
}
