// Sheets and dialogs: the panels that cover the page. They are hosted at the
// main window's root, so any page, sheet or button can open one (an ImGui
// popup must be opened in the scope it is drawn in).
#include "stdafx.h"
#include "UiInternal.h"
#include "../common/MathConstants.h"

static Sheet s_sheet = Sheet::None;
static Sheet s_sheetRequest = Sheet::None;
static bool s_sheetClose = false;
static Dialog s_dialog = Dialog::None;
static Dialog s_dialogRequest = Dialog::None;
static bool s_dialogClose = false;

// When the one-time drift warning was opened; gates its accept button.
double g_chapWarnOpenedAt = 0.0;

void OpenSheet(Sheet sheet)
{
	s_sheetRequest = sheet;
}

void CloseSheet()
{
	s_sheetClose = true;
}

Sheet CurrentSheet()
{
	return s_sheet;
}

void OpenDialog(Dialog dialog)
{
	s_dialogRequest = dialog;
	if (dialog == Dialog::ChaperoneWarning)
		g_chapWarnOpenedAt = ImGui::GetTime();
}

void CloseDialog()
{
	s_dialogClose = true;
}

bool DialogOpen()
{
	return s_dialog != Dialog::None || s_dialogRequest != Dialog::None;
}

// A full-window, see-through popup: the panel is drawn on it, and ImGui's
// modal dim (the theme's scrim) covers what is behind.
static const ImGuiWindowFlags kOverlayFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
	ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar |
	ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoSavedSettings;

static bool BeginOverlay(const char *id)
{
	const ImVec2 display = ImGui::GetIO().DisplaySize;
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(display, ImGuiCond_Always);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	const bool open = ImGui::BeginPopupModal(id, nullptr, kOverlayFlags);
	ImGui::PopStyleVar(2);
	return open;
}

// A dialog sits in the scope of whatever is under it: the sheet's popup when
// a sheet is open, the window otherwise.
static void BuildDialogLayer()
{
	if (s_dialogRequest != Dialog::None)
	{
		s_dialog = s_dialogRequest;
		s_dialogRequest = Dialog::None;
		ImGui::OpenPopup("##dialog");
	}
	if (s_dialog == Dialog::None)
	{
		s_dialogClose = false;
		return;
	}
	if (!BeginOverlay("##dialog"))
	{
		s_dialog = Dialog::None;
		return;
	}
	switch (s_dialog)
	{
	case Dialog::ClearCalibration: BuildClearCalibrationDialog(); break;
	case Dialog::ClearAnchors: BuildClearAnchorsDialog(); break;
	case Dialog::ChaperoneWarning: BuildChaperoneWarningDialog(); break;
	case Dialog::None: break;
	}
	if (s_dialogClose)
	{
		s_dialogClose = false;
		s_dialog = Dialog::None;
		ImGui::CloseCurrentPopup();
	}
	ImGui::EndPopup();
}

void BuildOverlays(const VRState &state)
{
	// The guide asks for its sheet through OpenGuide, the editor through the
	// calibration state; both may come from code that knows nothing of sheets.
	if (s_guide.openRequested)
	{
		ChooseGuideDemo(state);
		s_sheetRequest = Sheet::Calibrate;
		s_guide.openRequested = false;
	}
	if (CalCtx.state == CalibrationState::Editing && s_sheet != Sheet::Editor && s_sheetRequest == Sheet::None)
		s_sheetRequest = Sheet::Editor;

	if (s_sheetRequest != Sheet::None)
	{
		s_sheet = s_sheetRequest;
		s_sheetRequest = Sheet::None;
		s_sheetClose = false;
		ImGui::OpenPopup("##sheet");
	}
	if (s_sheet == Sheet::None)
	{
		s_sheetClose = false;
		BuildDialogLayer();
		return;
	}
	if (!BeginOverlay("##sheet"))
	{
		s_sheet = Sheet::None;
		return;
	}
	ImGui::PushID(static_cast<int>(s_sheet));
	switch (s_sheet)
	{
	case Sheet::Pair: BuildPairSheet(state); break;
	case Sheet::Calibrate: BuildCalibrateSheet(state); break;
	case Sheet::Chaperone: BuildChaperoneSheet(); break;
	case Sheet::Anchors: BuildAnchorsSheet(); break;
	case Sheet::Activity: BuildActivitySheet(); break;
	case Sheet::Editor: BuildEditorSheet(); break;
	case Sheet::Credits: BuildCreditsSheet(); break;
	case Sheet::None: break;
	}
	ImGui::PopID();
	BuildDialogLayer();
	if (s_sheetClose)
	{
		s_sheetClose = false;
		s_sheet = Sheet::None;
		ImGui::CloseCurrentPopup();
	}
	ImGui::EndPopup();
}

static FlexRect Centred(float width, float height)
{
	const ImVec2 display = ImGui::GetIO().DisplaySize;
	FlexRect r;
	r.min = ImVec2(std::floor((display.x - width) * 0.5f), std::floor((display.y - height) * 0.5f));
	r.max = ImVec2(r.min.x + width, r.min.y + height);
	return r;
}

FlexRect SheetPanel(float width, float height)
{
	const FlexRect panel = Centred(width, height);
	ui::PanelSurface(ImGui::GetWindowDrawList(), panel.min, panel.max, 30.0f);
	FlexRect content;
	content.min = ImVec2(panel.min.x + 32.0f, panel.min.y + 28.0f);
	content.max = ImVec2(panel.max.x - 32.0f, panel.max.y - 32.0f);
	return content;
}

FlexRect DialogPanel(float width, float height)
{
	const FlexRect panel = Centred(width, height);
	ui::PanelSurface(ImGui::GetWindowDrawList(), panel.min, panel.max, 28.0f);
	FlexRect content;
	content.min = ImVec2(panel.min.x + 30.0f, panel.min.y + 30.0f);
	content.max = ImVec2(panel.max.x - 30.0f, panel.max.y - 26.0f);
	return content;
}

bool SheetHeader(const FlexRect &content, const char *title)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ui::DrawLine(dl, ui::type::SheetHeading, ImVec2(content.min.x, content.min.y + 6.0f), ui::col::Text, title);
	const bool closed = ui::CloseButton("##close", ImVec2(content.max.x - 44.0f, content.min.y));
	return closed || (EscapePressed() && !DialogOpen());
}

// ---------------------------------------------------------------------------
// The error banner and activity times, shared by pages and sheets
// ---------------------------------------------------------------------------

float BuildErrorBanner(ImVec2 origin, float width)
{
	if (CalCtx.uiError.empty())
		return 0.0f;
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const char *message = Tr(CalCtx.uiError.c_str());
	const float dismissW = ui::PillWidth("Dismiss", 15.0f, ui::Icon::None, 16.0f);
	const float textX = origin.x + 16.0f + 22.0f + 12.0f;
	const float textW = width - (textX - origin.x) - 12.0f - dismissW - 8.0f;
	const ui::TextLines lines = ui::WrapText(ui::type::Callout, message, textW);
	const float height = std::max(52.0f, lines.height + 22.0f);
	const ImVec2 b(origin.x + width, origin.y + height);
	ui::FillRounded(dl, origin, b, ui::Rgba(255, 107, 97, 0.12f), 16.0f);
	ui::InnerEdge(dl, origin, b, ui::Rgba(255, 107, 97, 0.28f), 16.0f);
	ui::DrawIcon(dl, ui::Icon::Warn, ImVec2(origin.x + 16.0f + 11.0f, origin.y + height * 0.5f), 22.0f, ui::col::DangerInk);
	ui::DrawLines(dl, ui::type::Callout, lines, ImVec2(textX, origin.y + (height - lines.height) * 0.5f), textW, ui::col::Text);
	FlexRect dismiss;
	dismiss.min = ImVec2(b.x - 8.0f - dismissW, origin.y + (height - 36.0f) * 0.5f);
	dismiss.max = ImVec2(dismiss.min.x + dismissW, dismiss.min.y + 36.0f);
	if (ui::PillButton("##dismisserror", "Dismiss", ui::Btn::Secondary, dismiss, ui::Icon::None, 15.0f))
	{
		CalCtx.uiError.clear();
		CalCtx.uiErrorSource = CalibrationContext::ErrorSource::None;
	}
	return height;
}

std::vector<size_t> ActivityNewestFirst()
{
	// Newest first, but lines written in the same second keep the order they
	// were written in: a follow-up ("To keep SteamVR from doing this...")
	// stays under what it follows.
	std::vector<size_t> order;
	const auto &activity = CalCtx.activity;
	size_t end = activity.size();
	while (end > 0)
	{
		size_t start = end - 1;
		while (start > 0 && activity[start - 1].unixTime == activity[end - 1].unixTime)
			--start;
		for (size_t i = start; i < end; ++i)
			order.push_back(i);
		end = start;
	}
	return order;
}

std::string ActivityClock(double unixTime)
{
	char stamp[16] = "";
	const std::time_t t = static_cast<std::time_t>(unixTime);
	std::tm tm;
	if (localtime_s(&tm, &t) == 0)
		std::strftime(stamp, sizeof stamp, "%H:%M", &tm);
	return stamp;
}

// ---------------------------------------------------------------------------
// Activity
// ---------------------------------------------------------------------------

static void ToneIcon(CalibrationContext::Tone tone, ui::Icon &icon, ImU32 &ink, ImU32 &fill)
{
	using Tone = CalibrationContext::Tone;
	switch (tone)
	{
	case Tone::Good: icon = ui::Icon::Check; ink = ui::col::Good; fill = ui::col::GoodTint; break;
	case Tone::Warn: icon = ui::Icon::Warn; ink = ui::col::Caution; fill = ui::col::CautionTint; break;
	case Tone::Bad: icon = ui::Icon::Warn; ink = ui::col::Alert; fill = ui::col::AlertTint; break;
	default: icon = ui::Icon::Info; ink = ui::col::Link; fill = ui::col::LinkTint; break;
	}
}

// An entry's icon: what it is about, or its tone for the rest.
static void EntryIcon(const CalibrationContext::ActivityEntry &entry, ui::Icon &icon, ImU32 &ink, ImU32 &fill)
{
	using Event = CalibrationContext::Event;
	ToneIcon(entry.tone, icon, ink, fill);
	switch (entry.event)
	{
	case Event::TrackerOff: icon = ui::Icon::Power; ink = ui::Rgba(236, 238, 244, 0.75f); fill = ui::Rgba(255, 255, 255, 0.08f); break;
	case Event::Paused: icon = ui::Icon::Pause; ink = ui::col::Caution; fill = ui::col::CautionTint; break;
	case Event::Realigned: icon = ui::Icon::Refresh; ink = ui::col::Link; fill = ui::col::LinkTint; break;
	case Event::Anchor: icon = ui::Icon::Anchor; ink = ui::col::Link; fill = ui::col::LinkTint; break;
	case Event::Chaperone: icon = ui::Icon::Chaperone; ink = ui::col::Link; fill = ui::col::LinkTint; break;
	case Event::Calibrated: icon = ui::Icon::Check; break;
	default: break;
	}
}

// "Today", "Yesterday" or the date, for the day an entry belongs to.
static std::string DayLabel(double unixTime)
{
	const std::time_t now = std::time(nullptr), t = static_cast<std::time_t>(unixTime);
	std::tm today, then;
	if (localtime_s(&today, &now) != 0 || localtime_s(&then, &t) != 0)
		return std::string();
	if (then.tm_year == today.tm_year && then.tm_yday == today.tm_yday)
		return Tr("Today");
	std::time_t yesterdayTime = now - 24 * 60 * 60;
	std::tm yesterday;
	if (localtime_s(&yesterday, &yesterdayTime) == 0 && then.tm_year == yesterday.tm_year && then.tm_yday == yesterday.tm_yday)
		return Tr("Yesterday");
	char date[32] = "";
	std::strftime(date, sizeof date, "%Y-%m-%d", &then);
	return date;
}

void BuildActivitySheet()
{
	const FlexRect content = SheetPanel(880.0f, 626.0f);
	if (SheetHeader(content, Tr("Activity")))
		CloseSheet();
	const ImVec2 listMin(content.min.x, content.min.y + 44.0f + 14.0f);
	const ImVec2 listSize(content.W(), content.max.y - listMin.y);
	ImGui::SetCursorScreenPos(listMin);
	ImGui::BeginChild("##activitylist", listSize, ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const float width = ImGui::GetContentRegionAvail().x;
	const ImVec2 origin = ImGui::GetCursorScreenPos();

	struct Row { YGNodeRef row, icon, time, text; const CalibrationContext::ActivityEntry *entry; };
	struct Day { YGNodeRef label, card; std::vector<Row> rows; std::string name; };
	std::vector<Day> days;
	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, width);
	for (size_t index : ActivityNewestFirst())
	{
		const auto *it = &CalCtx.activity[index];
		const std::string name = DayLabel(it->unixTime);
		if (days.empty() || days.back().name != name)
		{
			Day day;
			day.name = name;
			day.label = ui::TextNode(fl, root, ui::type::FootnoteStrong, name);
			YGNodeStyleSetMargin(day.label, YGEdgeTop, days.empty() ? 0.0f : 18.0f);
			day.card = fl.Column(root);
			YGNodeStyleSetMargin(day.card, YGEdgeTop, 8.0f);
			days.push_back(std::move(day));
		}
		Row row;
		row.entry = it;
		row.row = fl.Row(days.back().card);
		YGNodeStyleSetMinHeight(row.row, 56.0f);
		YGNodeStyleSetPadding(row.row, YGEdgeVertical, 8.0f);
		YGNodeStyleSetPadding(row.row, YGEdgeHorizontal, 18.0f);
		YGNodeStyleSetAlignItems(row.row, YGAlignCenter);
		YGNodeStyleSetGap(row.row, YGGutterColumn, 14.0f);
		row.icon = fl.Box(row.row, 32.0f, 32.0f);
		row.time = fl.Box(row.row, 48.0f, 20.0f);
		row.text = ui::TextNode(fl, row.row, ui::type::Body, Tr(it->text.c_str()));
		YGNodeStyleSetFlexGrow(row.text, 1.0f);
		YGNodeStyleSetFlexShrink(row.text, 1.0f);
		days.back().rows.push_back(row);
	}
	if (days.empty())
	{
		YGNodeRef empty = fl.Box(root, width, 96.0f);
		fl.Compute(origin, width, YGUndefined);
		const FlexRect r = fl.Rect(empty);
		ui::Card(dl, r.min, r.max);
		const char *text = Tr("Nothing yet. Calibrations and corrections will appear here.");
		ui::DrawText(dl, ui::type::Body, ImVec2(r.min.x, r.min.y + 36.0f), r.W(), ui::col::Muted, text, ui::Align::Center);
		ImGui::Dummy(ImVec2(width, 96.0f));
		ImGui::EndChild();
		return;
	}
	fl.Compute(origin, width, YGUndefined);
	for (const Day &day : days)
	{
		const FlexRect label = fl.Rect(day.label);
		ui::DrawLine(dl, ui::type::FootnoteStrong, label.min, ui::col::Muted, day.name.c_str());
		const FlexRect card = fl.Rect(day.card);
		ui::Card(dl, card.min, card.max);
		for (size_t i = 0; i < day.rows.size(); ++i)
		{
			const Row &row = day.rows[i];
			const FlexRect r = fl.Rect(row.row);
			if (i > 0)
				ui::Hairline(dl, r.min.x + 64.0f, r.max.x, r.min.y);
			ui::Icon icon;
			ImU32 ink, fill;
			EntryIcon(*row.entry, icon, ink, fill);
			ui::RoundIcon(dl, fl.Rect(row.icon).Center(), 32.0f, icon, 17.0f, ink, fill);
			const FlexRect time = fl.Rect(row.time);
			ui::DrawLine(dl, ui::TextStyle{ ui::Weight::Regular, 15.0f, time.H() }, time.min, ui::col::Quiet,
				ActivityClock(row.entry->unixTime).c_str());
			const FlexRect text = fl.Rect(row.text);
			ui::DrawText(dl, ui::type::Body, text.min, text.W(), ui::col::Text, Tr(row.entry->text.c_str()));
		}
	}
	ImGui::SetCursorScreenPos(origin);
	ImGui::Dummy(ImVec2(width, YGNodeLayoutGetHeight(root)));
	ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Chaperone
// ---------------------------------------------------------------------------

// What the protect action does: the first press ever shows the drift warning
// first; that dialog protects once it is accepted.
static void ProtectOrWarn()
{
	if (CalCtx.chaperoneWarningAck)
		ProtectChaperone();
	else
		OpenDialog(Dialog::ChaperoneWarning);
}

void BuildChaperoneSheet()
{
	const bool armed = CalCtx.chaperone.valid;
	const float width = 720.0f;
	const float contentW = width - 64.0f;
	const bool chaperoneError = !CalCtx.uiError.empty() &&
		(CalCtx.uiErrorSource == CalibrationContext::ErrorSource::Chaperone ||
			CalCtx.uiErrorSource == CalibrationContext::ErrorSource::ChaperoneMonitor);

	std::string status, detail;
	if (armed)
	{
		auto copied = FormatUnixAge(CalCtx.chaperone.copyUnixTime);
		status = copied ? Tr(FormatString("Protected %s", copied->c_str())) : std::string(Tr("Protected"));
		if (CalCtx.chaperone.geometry.empty())
			detail = Tr("Only the play area center was saved (no walls), so there's nothing to restore automatically.");
		else
		{
			const size_t walls = CalCtx.chaperone.geometry.size();
			detail = Tr(FormatString("%zu wall%s \xC2\xB7 %.1f \xC3\x97 %.1f m", walls, walls == 1 ? "" : "s",
				CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]));
		}
	}
	else
	{
		status = Tr("Not protected yet");
		detail = Tr("Protect your current walls to keep them.");
	}

	// Laid out once to know the panel's height, then drawn.
	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, contentW);
	fl.Box(root, contentW, 44.0f);
	YGNodeRef lead = ui::TextNode(fl, root, ui::type::Body, Tr(
		"Nova Calibrator saves your SteamVR walls and puts them back if SteamVR or the headset ever loses them. "
		"If you redraw your walls, protect them again."));
	YGNodeStyleSetMargin(lead, YGEdgeTop, 6.0f);
	YGNodeRef card = fl.Row(root);
	YGNodeStyleSetMargin(card, YGEdgeTop, 20.0f);
	YGNodeStyleSetPadding(card, YGEdgeVertical, 18.0f);
	YGNodeStyleSetPadding(card, YGEdgeHorizontal, 20.0f);
	YGNodeStyleSetAlignItems(card, YGAlignCenter);
	YGNodeStyleSetGap(card, YGGutterColumn, 16.0f);
	YGNodeRef cardIcon = fl.Box(card, 56.0f, 56.0f);
	YGNodeRef cardText = fl.Column(card);
	YGNodeStyleSetFlexGrow(cardText, 1.0f);
	YGNodeStyleSetFlexShrink(cardText, 1.0f);
	YGNodeRef cardTitle = ui::TextNode(fl, cardText, ui::type::Item, status);
	YGNodeRef cardDetail = ui::TextNode(fl, cardText, ui::type::Callout, detail);
	YGNodeStyleSetMargin(cardDetail, YGEdgeTop, 2.0f);
	YGNodeRef autoRow = nullptr, autoText = nullptr, autoTitle = nullptr, autoDesc = nullptr, autoSwitch = nullptr;
	if (armed)
	{
		autoRow = fl.Row(root);
		YGNodeStyleSetMargin(autoRow, YGEdgeTop, 14.0f);
		YGNodeStyleSetMinHeight(autoRow, 80.0f);
		YGNodeStyleSetPadding(autoRow, YGEdgeVertical, 12.0f);
		YGNodeStyleSetPadding(autoRow, YGEdgeLeft, 20.0f);
		YGNodeStyleSetPadding(autoRow, YGEdgeRight, 16.0f);
		YGNodeStyleSetAlignItems(autoRow, YGAlignCenter);
		YGNodeStyleSetGap(autoRow, YGGutterColumn, 16.0f);
		autoText = fl.Column(autoRow);
		YGNodeStyleSetFlexGrow(autoText, 1.0f);
		YGNodeStyleSetFlexShrink(autoText, 1.0f);
		autoTitle = ui::TextNode(fl, autoText, ui::type::RowTitle, Tr("Restore automatically"));
		autoDesc = ui::TextNode(fl, autoText, ui::type::Footnote, Tr("Turn off to use your Quest's boundary each session instead."));
		YGNodeStyleSetMargin(autoDesc, YGEdgeTop, 2.0f);
		autoSwitch = fl.Box(autoRow, ui::kSwitchW, ui::kSwitchH);
	}
	YGNodeRef warning = fl.Row(root);
	YGNodeStyleSetMargin(warning, YGEdgeTop, 14.0f);
	YGNodeStyleSetGap(warning, YGGutterColumn, 10.0f);
	YGNodeRef warningIcon = fl.Box(warning, 20.0f, 20.0f);
	YGNodeRef warningText = ui::TextNode(fl, warning, ui::type::Footnote,
		Tr("Keep the Quest boundary turned on too. A protected chaperone doesn't guarantee that your play area is clear."));
	YGNodeStyleSetFlexShrink(warningText, 1.0f);
	YGNodeRef error = nullptr;
	if (chaperoneError)
	{
		error = ui::TextNode(fl, root, ui::type::Footnote, Tr(CalCtx.uiError.c_str()));
		YGNodeStyleSetMargin(error, YGEdgeTop, 12.0f);
	}
	YGNodeRef footer = fl.Row(root);
	YGNodeStyleSetMargin(footer, YGEdgeTop, 28.0f);
	YGNodeStyleSetJustifyContent(footer, YGJustifyFlexEnd);
	YGNodeStyleSetGap(footer, YGGutterColumn, 12.0f);
	YGNodeRef restore = armed ? fl.Box(footer, ui::PillWidth("Restore now", 18.0f), 52.0f) : nullptr;
	YGNodeRef protect = fl.Box(footer, ui::PillWidth(armed ? "Protect current walls" : "Protect chaperone", 18.0f), 52.0f);
	fl.Compute(ImVec2(0.0f, 0.0f), contentW, YGUndefined);
	const float height = YGNodeLayoutGetHeight(root) + 28.0f + 32.0f;

	const FlexRect content = SheetPanel(width, height);
	if (SheetHeader(content, Tr("Chaperone")))
		CloseSheet();
	fl.Compute(content.min, contentW, YGUndefined);
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const FlexRect leadR = fl.Rect(lead);
	ui::DrawText(dl, ui::type::Body, leadR.min, leadR.W(), ui::col::Prose, Tr(
		"Nova Calibrator saves your SteamVR walls and puts them back if SteamVR or the headset ever loses them. "
		"If you redraw your walls, protect them again."));
	const FlexRect cardR = fl.Rect(card);
	ui::Card(dl, cardR.min, cardR.max);
	const FlexRect iconR = fl.Rect(cardIcon);
	ui::FillRounded(dl, iconR.min, iconR.max, ui::col::LinkTint, 16.0f);
	ui::DrawIcon(dl, ui::Icon::Chaperone, iconR.Center(), 28.0f, ui::col::Link);
	ui::DrawText(dl, ui::type::Item, fl.Rect(cardTitle).min, fl.Rect(cardTitle).W(), ui::col::Text, status.c_str());
	ui::DrawText(dl, ui::type::Callout, fl.Rect(cardDetail).min, fl.Rect(cardDetail).W(), ui::col::Muted, detail.c_str());
	if (armed)
	{
		const FlexRect row = fl.Rect(autoRow);
		ui::Card(dl, row.min, row.max);
		ui::DrawText(dl, ui::type::RowTitle, fl.Rect(autoTitle).min, fl.Rect(autoTitle).W(), ui::col::Text, Tr("Restore automatically"));
		ui::DrawText(dl, ui::type::Footnote, fl.Rect(autoDesc).min, fl.Rect(autoDesc).W(), ui::col::Muted,
			Tr("Turn off to use your Quest's boundary each session instead."));
		if (ui::ToggleSwitch("##autorestore", CalCtx.chaperone.autoApply, fl.Rect(autoSwitch).min))
		{
			// Disarming is fail-closed: a failed write must never roll the
			// in-memory state back to armed, and a failed enable is reverted
			// until Settings can be persisted truthfully.
			if (!SaveSettings(CalCtx))
			{
				CalCtx.chaperone.autoApply = false;
				CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
			}
		}
	}
	ui::DrawIcon(dl, ui::Icon::Warn, fl.Rect(warningIcon).Center(), 20.0f, ui::col::Caution);
	const FlexRect warningR = fl.Rect(warningText);
	ui::DrawText(dl, ui::type::Footnote, warningR.min, warningR.W(), ui::col::Caution,
		Tr("Keep the Quest boundary turned on too. A protected chaperone doesn't guarantee that your play area is clear."));
	if (error)
		ui::DrawText(dl, ui::type::Footnote, fl.Rect(error).min, fl.Rect(error).W(), ui::col::Alert, Tr(CalCtx.uiError.c_str()));
	if (restore && ui::PillButton("##restore", "Restore now", ui::Btn::Secondary, fl.Rect(restore)))
		ApplyChaperoneBounds();
	if (ui::PillButton("##protect", armed ? "Protect current walls" : "Protect chaperone", ui::Btn::Primary, fl.Rect(protect)))
		ProtectOrWarn();
}

// ---------------------------------------------------------------------------
// Field anchors
// ---------------------------------------------------------------------------

void BuildAnchorsSheet()
{
	const float width = 720.0f;
	const float contentW = width - 64.0f;
	const auto &anchors = CalCtx.fieldAnchors;
	const char *leadText = Tr("If your trackers are a little off in one part of the room, stand there and add an anchor. "
		"Nova Calibrator fixes that spot and blends the correction in as you walk toward it.");

	struct AnchorRow { YGNodeRef row, icon, title, detail; std::string titleText, detailText; };
	std::vector<AnchorRow> rows;
	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, contentW);
	fl.Box(root, contentW, 44.0f);
	YGNodeRef lead = ui::TextNode(fl, root, ui::type::Body, leadText);
	YGNodeStyleSetMargin(lead, YGEdgeTop, 6.0f);
	YGNodeRef list = fl.Column(root);
	YGNodeStyleSetMargin(list, YGEdgeTop, 20.0f);
	YGNodeRef empty = nullptr;
	if (anchors.empty())
	{
		empty = fl.Box(list, contentW, 64.0f);
	}
	for (size_t i = 0; i < anchors.size(); ++i)
	{
		const auto &a = anchors[i];
		// The correction at the anchor's own spot, against the main calibration.
		const Eigen::Vector3d targetPt = a.rotation.conjugate() * (a.position - a.translationMeters);
		const Eigen::Vector3d basePos = CalCtx.transform.rotation * targetPt + CalCtx.transform.translationMeters;
		const double posDeltaCm = (a.position - basePos).norm() * 100.0;
		const double rotDeltaDeg = a.rotation.angularDistance(CalCtx.transform.rotation) * 180.0 / questcal::Pi;
		const double fromCentre = std::hypot(a.position.x(), a.position.z());
		AnchorRow row;
		row.titleText = Tr(FormatString("Anchor %zu", i + 1));
		row.detailText = Tr(FormatString("%.1f m from the center \xC2\xB7 corrects %.1f cm and %.2f\xC2\xB0",
			fromCentre, posDeltaCm, rotDeltaDeg));
		row.row = fl.Row(list);
		YGNodeStyleSetPadding(row.row, YGEdgeVertical, 16.0f);
		YGNodeStyleSetPadding(row.row, YGEdgeHorizontal, 20.0f);
		YGNodeStyleSetAlignItems(row.row, YGAlignCenter);
		YGNodeStyleSetGap(row.row, YGGutterColumn, 16.0f);
		row.icon = fl.Box(row.row, 44.0f, 44.0f);
		YGNodeRef text = fl.Column(row.row);
		YGNodeStyleSetFlexGrow(text, 1.0f);
		YGNodeStyleSetFlexShrink(text, 1.0f);
		row.title = ui::TextNode(fl, text, ui::type::Label, row.titleText);
		row.detail = ui::TextNode(fl, text, ui::type::Footnote, row.detailText);
		YGNodeStyleSetMargin(row.detail, YGEdgeTop, 2.0f);
		rows.push_back(std::move(row));
	}
	YGNodeRef useRow = nullptr, useTitle = nullptr, useDesc = nullptr, useSwitch = nullptr;
	if (!anchors.empty())
	{
		useRow = fl.Row(root);
		YGNodeStyleSetMargin(useRow, YGEdgeTop, 12.0f);
		YGNodeStyleSetMinHeight(useRow, 76.0f);
		YGNodeStyleSetPadding(useRow, YGEdgeVertical, 12.0f);
		YGNodeStyleSetPadding(useRow, YGEdgeLeft, 20.0f);
		YGNodeStyleSetPadding(useRow, YGEdgeRight, 16.0f);
		YGNodeStyleSetAlignItems(useRow, YGAlignCenter);
		YGNodeStyleSetGap(useRow, YGGutterColumn, 16.0f);
		YGNodeRef text = fl.Column(useRow);
		YGNodeStyleSetFlexGrow(text, 1.0f);
		YGNodeStyleSetFlexShrink(text, 1.0f);
		useTitle = ui::TextNode(fl, text, ui::type::RowTitle, Tr("Use field anchors"));
		useDesc = ui::TextNode(fl, text, ui::type::Footnote, Tr("Turn off to use the main calibration everywhere."));
		YGNodeStyleSetMargin(useDesc, YGEdgeTop, 2.0f);
		useSwitch = fl.Box(useRow, ui::kSwitchW, ui::kSwitchH);
	}
	YGNodeRef note = ui::TextNode(fl, root, ui::type::Footnote,
		Tr("You can save up to 8. A new anchor within 1 m of another one replaces it."));
	YGNodeStyleSetMargin(note, YGEdgeTop, 12.0f);
	YGNodeRef footer = fl.Row(root);
	YGNodeStyleSetMargin(footer, YGEdgeTop, 28.0f);
	YGNodeStyleSetJustifyContent(footer, YGJustifySpaceBetween);
	YGNodeRef clear = anchors.empty() ? fl.Box(footer, 1.0f, 52.0f)
		: fl.Box(footer, ui::PillWidth("Clear anchors", 18.0f, ui::Icon::Trash), 52.0f);
	YGNodeRef add = fl.Box(footer, ui::PillWidth("Add field anchor", 18.0f, ui::Icon::Anchor), 52.0f);
	fl.Compute(ImVec2(0.0f, 0.0f), contentW, YGUndefined);
	const float height = YGNodeLayoutGetHeight(root) + 28.0f + 32.0f;

	const FlexRect content = SheetPanel(width, height);
	if (SheetHeader(content, Tr("Field anchors")))
		CloseSheet();
	fl.Compute(content.min, contentW, YGUndefined);
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ui::DrawText(dl, ui::type::Body, fl.Rect(lead).min, fl.Rect(lead).W(), ui::col::Prose, leadText);
	const FlexRect listR = fl.Rect(list);
	ui::Card(dl, listR.min, listR.max);
	if (empty)
		ui::DrawText(dl, ui::type::Body, ImVec2(listR.min.x, listR.min.y + 20.0f), listR.W(), ui::col::Muted,
			Tr("No anchors yet."), ui::Align::Center);
	for (size_t i = 0; i < rows.size(); ++i)
	{
		const AnchorRow &row = rows[i];
		const FlexRect r = fl.Rect(row.row);
		if (i > 0)
			ui::Hairline(dl, r.min.x + 80.0f, r.max.x, r.min.y);
		const FlexRect icon = fl.Rect(row.icon);
		ui::FillRounded(dl, icon.min, icon.max, ui::col::LinkTint, 14.0f);
		ui::DrawIcon(dl, ui::Icon::Anchor, icon.Center(), 22.0f, ui::col::Link);
		ui::DrawText(dl, ui::type::Label, fl.Rect(row.title).min, fl.Rect(row.title).W(), ui::col::Text, row.titleText.c_str());
		ui::DrawText(dl, ui::type::Footnote, fl.Rect(row.detail).min, fl.Rect(row.detail).W(), ui::col::Muted, row.detailText.c_str());
	}
	if (useRow)
	{
		const FlexRect r = fl.Rect(useRow);
		ui::Card(dl, r.min, r.max);
		ui::DrawText(dl, ui::type::RowTitle, fl.Rect(useTitle).min, fl.Rect(useTitle).W(), ui::col::Text, Tr("Use field anchors"));
		ui::DrawText(dl, ui::type::Footnote, fl.Rect(useDesc).min, fl.Rect(useDesc).W(), ui::col::Muted,
			Tr("Turn off to use the main calibration everywhere."));
		// Toggled through a local: the live member changes only after the
		// transaction has persisted the candidate that carries this value.
		bool fieldEnabled = CalCtx.fieldEnabled;
		if (ui::ToggleSwitch("##fieldEnabled", fieldEnabled, fl.Rect(useSwitch).min))
		{
			if (SaveProfileFieldEdit(CalCtx,
				[&](questcal::ProfileRecord &candidate) {
					candidate.fieldEnabled = fieldEnabled;
				},
				true))
				ResyncDriverState();
		}
	}
	ui::DrawText(dl, ui::type::Footnote, fl.Rect(note).min, fl.Rect(note).W(), ui::col::Muted,
		Tr("You can save up to 8. A new anchor within 1 m of another one replaces it."));
	if (!anchors.empty() && ui::PillButton("##clearanchors", "Clear anchors", ui::Btn::Danger, fl.Rect(clear), ui::Icon::Trash))
		OpenDialog(Dialog::ClearAnchors);
	if (ui::PillButton("##addanchor", "Add field anchor", ui::Btn::Primary, fl.Rect(add), ui::Icon::Anchor))
	{
		CloseSheet();
		OpenGuide(true, false);
	}
}

// ---------------------------------------------------------------------------
// Dialogs
// ---------------------------------------------------------------------------

// The shape every confirmation shares: a round icon, a title, paragraphs and
// two buttons side by side. Laid out to find the panel's height, then drawn;
// returns which button was pressed (0 none, 1 left, 2 right).
struct DialogText
{
	std::string text;
	ImU32 colour;
};

static int ConfirmDialog(float width, ui::Icon icon, ImU32 ink, ImU32 fill, const char *title,
	const std::vector<DialogText> &paragraphs, const char *leftEnglish, ui::Btn leftKind,
	const char *rightEnglish, ui::Btn rightKind, ui::Icon rightIcon)
{
	const float contentW = width - 60.0f;
	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetAlignItems(root, YGAlignCenter);
	YGNodeStyleSetWidth(root, contentW);
	YGNodeRef iconNode = fl.Box(root, 64.0f, 64.0f);
	YGNodeRef titleNode = ui::TextNode(fl, root, ui::type::DialogTitle, title);
	YGNodeStyleSetMargin(titleNode, YGEdgeTop, 18.0f);
	std::vector<YGNodeRef> texts;
	for (size_t i = 0; i < paragraphs.size(); ++i)
	{
		YGNodeRef p = ui::TextNode(fl, root, ui::type::Body, paragraphs[i].text);
		YGNodeStyleSetMargin(p, YGEdgeTop, i == 0 ? 8.0f : 10.0f);
		texts.push_back(p);
	}
	YGNodeRef buttons = fl.Row(root);
	YGNodeStyleSetAlignSelf(buttons, YGAlignStretch);
	YGNodeStyleSetMargin(buttons, YGEdgeTop, 24.0f);
	YGNodeStyleSetGap(buttons, YGGutterColumn, 12.0f);
	YGNodeRef left = fl.Add(buttons);
	YGNodeStyleSetFlexGrow(left, 1.0f);
	YGNodeStyleSetFlexBasis(left, 0.0f);
	YGNodeStyleSetHeight(left, 48.0f);
	YGNodeRef right = fl.Add(buttons);
	YGNodeStyleSetFlexGrow(right, 1.0f);
	YGNodeStyleSetFlexBasis(right, 0.0f);
	YGNodeStyleSetHeight(right, 48.0f);
	fl.Compute(ImVec2(0.0f, 0.0f), contentW, YGUndefined);
	const float height = YGNodeLayoutGetHeight(root) + 30.0f + 26.0f;

	const FlexRect content = DialogPanel(width, height);
	fl.Compute(content.min, contentW, YGUndefined);
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ui::RoundIcon(dl, fl.Rect(iconNode).Center(), 64.0f, icon, 30.0f, ink, fill);
	// Centred across the whole panel, the way the canvas sets them.
	const FlexRect titleR = fl.Rect(titleNode);
	ui::DrawText(dl, ui::type::DialogTitle, ImVec2(content.min.x, titleR.min.y), contentW, ui::col::Text, title, ui::Align::Center);
	for (size_t i = 0; i < texts.size(); ++i)
	{
		const FlexRect r = fl.Rect(texts[i]);
		ui::DrawText(dl, ui::type::Body, ImVec2(content.min.x, r.min.y), contentW, paragraphs[i].colour,
			paragraphs[i].text.c_str(), ui::Align::Center);
	}
	int pressed = 0;
	if (ui::PillButton("##left", leftEnglish, leftKind, fl.Rect(left)))
		pressed = 1;
	if (ui::PillButton("##right", rightEnglish, rightKind, fl.Rect(right), rightIcon))
		pressed = 2;
	return pressed;
}

void BuildClearCalibrationDialog()
{
	const int pressed = ConfirmDialog(560.0f, ui::Icon::Trash, ui::col::DangerInk, ui::col::AlertTint,
		Tr("Clear this calibration?"),
		{ { Tr("Your trackers won't line up until you calibrate again. "
			"This also removes the field anchors and the headset tracker setup."), ui::col::Muted } },
		"Keep calibration", ui::Btn::Primary, "Clear calibration", ui::Btn::Danger, ui::Icon::Trash);
	if (pressed == 1 || EscapePressed())
		CloseDialog();
	else if (pressed == 2)
	{
		// The write can be refused; a destructive button that did nothing has
		// to say so instead of leaving the screen unchanged.
		if (!ClearSavedProfile(CalCtx))
			CalCtx.ReportError("Couldn't clear the calibration, so it's still saved. Restart QuestCalibrator and try again.\n",
				CalibrationContext::ErrorSource::ProfilePersistence);
		CloseDialog();
	}
}

void BuildClearAnchorsDialog()
{
	const size_t count = CalCtx.fieldAnchors.size();
	const std::string title = count == 1 ? std::string(Tr("Clear 1 field anchor?"))
		: Tr(FormatString("Clear %zu field anchors?", count));
	const int pressed = ConfirmDialog(560.0f, ui::Icon::Trash, ui::col::DangerInk, ui::col::AlertTint, title.c_str(),
		{ { Tr(count == 1 ? "That spot goes back to the main calibration. The calibration itself is kept."
			: "Those spots go back to the main calibration. The calibration itself is kept."), ui::col::Muted } },
		"Keep anchors", ui::Btn::Primary, "Clear anchors", ui::Btn::Danger, ui::Icon::Trash);
	if (pressed == 1 || EscapePressed())
		CloseDialog();
	else if (pressed == 2)
	{
		if (SaveProfileFieldEdit(CalCtx,
			[](questcal::ProfileRecord &candidate) {
				candidate.fieldAnchors.clear();
			},
			true))
			ResyncDriverState();
		CloseDialog();
	}
}

void BuildChaperoneWarningDialog()
{
	// The accept button unlocks after a short countdown, so the caveat
	// actually gets read.
	const int remain = static_cast<int>(std::ceil(5.0 - (ImGui::GetTime() - g_chapWarnOpenedAt)));
	if (remain > 0)
		ui::KeepAnimating();
	std::vector<DialogText> paragraphs = {
		{ Tr("Nova Calibrator saves and restores your SteamVR chaperone. "
			"Tracking drift can still shift it away from your real walls."), ui::col::Muted },
		{ Tr("Keep the Quest boundary on too. A protected chaperone "
			"doesn't guarantee that your play area is clear or correctly aligned."), ui::col::Muted },
		{ Tr("Before you play, check that the walls match your room and leave "
			"enough space to move safely, especially when dancing."), ui::col::Lavender },
	};
	if (!CalCtx.uiError.empty())
		paragraphs.push_back({ Tr(CalCtx.uiError.c_str()), ui::col::Alert });
	// English: the dialog's buttons translate their own labels.
	const std::string waiting = FormatString("Protect chaperone (%d s)", std::max(remain, 0));
	const int pressed = ConfirmDialog(600.0f, ui::Icon::Chaperone, ui::col::Caution, ui::col::CautionTint,
		Tr("Check your room boundaries"), paragraphs, "Cancel", ui::Btn::Secondary,
		remain > 0 ? waiting.c_str() : "Protect chaperone", remain > 0 ? ui::Btn::Waiting : ui::Btn::Primary, ui::Icon::None);
	if (pressed == 1 || EscapePressed())
		CloseDialog();
	else if (pressed == 2)
	{
		// Named for what it does: accepting the caveat is what saves the
		// chaperone.
		const bool previousAck = CalCtx.chaperoneWarningAck;
		CalCtx.chaperoneWarningAck = true;
		if (ProtectChaperone())
			CloseDialog();
		else
		{
			CalCtx.chaperoneWarningAck = previousAck;
			// A failed capture may have persisted the fail-closed chaperone
			// state immediately; keep the acknowledgement rollback consistent.
			if (!SaveSettings(CalCtx))
				CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
		}
	}
}
