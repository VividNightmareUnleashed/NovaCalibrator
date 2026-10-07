// The Settings page, the calibration editor and the credits.
#include "stdafx.h"
#include "UiInternal.h"
#include "Diagnostics.h"
#include "LocalizationTables.h"

#include <deque>
#include <functional>

// ---------------------------------------------------------------------------
// The language choice, here and on the setup's welcome
// ---------------------------------------------------------------------------

namespace
{
using questcal::i18n::Language;
const Language kLanguageOrder[] = { Language::English, Language::Italian, Language::Japanese };

// Each language written in its own words, so a player who cannot read the
// current one still finds theirs. Without a Japanese font its name would draw
// as boxes, so it is spelt out instead.
struct LanguageChoices
{
	const char *names[3] = {};
	int current = 0;
	bool japaneseFont = false;
};

LanguageChoices Languages()
{
	LanguageChoices c;
	c.japaneseFont = questcal::i18n::FontAvailable(Language::Japanese);
	c.names[0] = "English";
	c.names[1] = questcal::i18n::ItalianTable().nativeName.c_str();
	c.names[2] = c.japaneseFont ? questcal::i18n::JapaneseTable().nativeName.c_str() : "Japanese";
	// The language drawn, which -lang may set apart from the saved one.
	const Language chosen = questcal::i18n::CurrentLanguage();
	for (int i = 0; i < 3; ++i)
		if (kLanguageOrder[i] == chosen)
			c.current = i;
	return c;
}
} // namespace

float LanguagePickerWidth()
{
	const LanguageChoices c = Languages();
	return ui::PillSegmentedWidth(c.names, 3, 16.0f, 0.0f, false);
}

bool LanguagePicker(const char *id, const FlexRect &r, float lead)
{
	const LanguageChoices c = Languages();
	const int picked = ui::PillSegmented(id, c.current, c.names, 3, r, 16.0f, false, lead);
	if (picked == c.current)
		return false;
	if (kLanguageOrder[picked] == Language::Japanese && !c.japaneseFont)
	{
		CalCtx.ReportError("Japanese needs a Japanese font, and Windows doesn't have one installed. "
			"Add the Japanese Supplemental Fonts in Windows Settings > System > Optional features.\n");
		return false;
	}
	const std::string previous = CalCtx.language;
	CalCtx.language = questcal::i18n::LanguageCode(kLanguageOrder[picked]);
	SaveSettingOrRestore(CalCtx.language, previous);
	questcal::i18n::SetLanguage(questcal::i18n::LanguageFromCode(CalCtx.language));
	return true;
}

namespace
{

// ---------------------------------------------------------------------------
// Grouped rows
// ---------------------------------------------------------------------------

const ImU32 kSubInk = ui::Rgba(236, 238, 244, 0.64f);

// A page in the canvas's grouped style: a title, then sections of rows on
// rounded cards. The page is one Yoga tree, so a translation that wraps a
// sub-line grows its row and moves everything below it; each row draws its
// control once the tree is computed.
class RowPage
{
public:
	struct Row
	{
		YGNodeRef node = nullptr, title = nullptr, sub = nullptr, badge = nullptr;
		YGNodeRef lead = nullptr, controlNode = nullptr, chevron = nullptr;
		std::string titleText, subText;
		ui::TextStyle titleStyle = ui::type::RowTitle;
		ImU32 titleInk = ui::col::Text;
		const char *badgeEnglish = nullptr;
		ui::Icon icon = ui::Icon::None;   // the leading icon, or
		ImU32 dot = 0;                    // a leading status dot
		std::function<void()> onPress;    // the whole row is one button
		std::function<void(const FlexRect &)> drawControl;
		const char *tipEnglish = nullptr;
		std::string tipText;              // English built at run time
		size_t card = 0;
	};

	explicit RowPage(float width)
		: width_(width)
	{
		root_ = fl_.Root();
		YGNodeStyleSetFlexDirection(root_, YGFlexDirectionColumn);
		YGNodeStyleSetWidth(root_, width);
		YGNodeStyleSetPadding(root_, YGEdgeHorizontal, 40.0f);
	}

	void Title(const char *english)
	{
		titleText_ = Tr(english);
		title_ = ui::TextNode(fl_, root_, ui::type::PageTitle, titleText_);
	}

	void Section(const char *english, const char *badgeEnglish = nullptr)
	{
		Heading h;
		h.node = fl_.Row(root_);
		YGNodeStyleSetMargin(h.node, YGEdgeTop, headings_.empty() ? 24.0f : 28.0f);
		YGNodeStyleSetMinHeight(h.node, 32.0f);
		YGNodeStyleSetAlignItems(h.node, YGAlignCenter);
		YGNodeStyleSetGap(h.node, YGGutterColumn, 10.0f);
		h.label = Tr(english);
		h.text = ui::TextNode(fl_, h.node, ui::type::Section, h.label);
		YGNodeStyleSetFlexShrink(h.text, 1.0f);
		h.badgeEnglish = badgeEnglish;
		if (badgeEnglish)
			h.badge = fl_.Box(h.node, ui::BadgeWidth(badgeEnglish), 26.0f);
		headings_.push_back(h);
		YGNodeRef card = fl_.Column(root_);
		YGNodeStyleSetMargin(card, YGEdgeTop, 10.0f);
		cards_.push_back(card);
	}

	// A row with a title (translated), an optional sub-line and a control box
	// of the given size at its end.
	Row &Add(const std::string &title, const std::string &sub, float controlW = 0.0f, float controlH = 0.0f,
		const char *badgeEnglish = nullptr)
	{
		Row &row = NewRow();
		row.titleText = title;
		row.subText = sub;
		row.badgeEnglish = badgeEnglish;
		YGNodeStyleSetGap(row.node, YGGutterColumn, 16.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeLeft, 20.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeRight, controlW > ui::kSwitchW ? 12.0f : 16.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeVertical, sub.empty() ? 10.0f : 14.0f);
		YGNodeStyleSetMinHeight(row.node, sub.empty() ? 64.0f : 80.0f);
		YGNodeRef text = fl_.Column(row.node);
		YGNodeStyleSetFlexGrow(text, 1.0f);
		YGNodeStyleSetFlexShrink(text, 1.0f);
		YGNodeStyleSetGap(text, YGGutterRow, 2.0f);
		YGNodeRef line = fl_.Row(text);
		YGNodeStyleSetAlignItems(line, YGAlignCenter);
		YGNodeStyleSetGap(line, YGGutterColumn, 10.0f);
		row.title = ui::TextNode(fl_, line, row.titleStyle, title);
		YGNodeStyleSetFlexShrink(row.title, 1.0f);
		if (badgeEnglish)
			row.badge = fl_.Box(line, ui::BadgeWidth(badgeEnglish), 26.0f);
		if (!sub.empty())
			row.sub = ui::TextNode(fl_, text, ui::type::Footnote, sub);
		if (controlW > 0.0f)
			row.controlNode = fl_.Box(row.node, controlW, controlH);
		return row;
	}

	// A switch. value is shown; changed is called with the new one. A value
	// string (advanced numbers) sits beside the switch.
	Row &Toggle(const char *english, const char *subEnglish, bool value, const std::function<void(bool)> &changed,
		const char *badgeEnglish = nullptr, const std::string &valueText = std::string())
	{
		const ui::TextStyle valueStyle = ui::type::Callout;
		const float valueW = valueText.empty() ? 0.0f : std::ceil(ui::MeasureLine(valueStyle, valueText.c_str()).x) + 14.0f;
		Row &row = Add(Tr(english), subEnglish ? std::string(Tr(subEnglish)) : std::string(), ui::kSwitchW + valueW,
			ui::kSwitchH, badgeEnglish);
		row.drawControl = [value, changed, valueText, valueStyle, valueW](const FlexRect &r) {
			if (valueW > 0.0f)
				ui::DrawLine(ImGui::GetWindowDrawList(), ui::TextStyle{ valueStyle.weight, valueStyle.size, r.H() }, r.min,
					ui::col::Muted, valueText.c_str());
			bool v = value;
			if (ui::ToggleSwitch("##switch", v, ImVec2(r.max.x - ui::kSwitchW, r.min.y)))
				changed(v);
		};
		return row;
	}

	// A row that is one button: a leading icon, a title and, for a row that
	// opens something, a chevron.
	Row &Link(ui::Icon icon, const char *english, ImU32 ink, bool chevron, std::function<void()> pressed)
	{
		Row &row = NewRow();
		row.icon = icon;
		row.titleText = Tr(english);
		row.titleStyle = ui::type::BodyMedium;
		row.titleInk = ink;
		row.onPress = std::move(pressed);
		YGNodeStyleSetGap(row.node, YGGutterColumn, 14.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeLeft, 20.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeRight, 16.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeVertical, 8.0f);
		YGNodeStyleSetMinHeight(row.node, 52.0f);
		row.lead = fl_.Box(row.node, 20.0f, 20.0f);
		row.title = ui::TextNode(fl_, row.node, row.titleStyle, row.titleText);
		YGNodeStyleSetFlexGrow(row.title, 1.0f);
		YGNodeStyleSetFlexShrink(row.title, 1.0f);
		if (chevron)
			row.chevron = fl_.Box(row.node, 18.0f, 18.0f);
		return row;
	}

	// A state in colour behind a dot, with room for a button at the end.
	Row &Status(ImU32 dot, const std::string &text, float buttonW)
	{
		Row &row = NewRow();
		row.dot = dot;
		row.titleText = text;
		row.titleStyle = ui::type::Callout;
		row.titleInk = dot;
		YGNodeStyleSetGap(row.node, YGGutterColumn, 12.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeLeft, 20.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeRight, 12.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeVertical, 10.0f);
		YGNodeStyleSetMinHeight(row.node, 60.0f);
		row.lead = fl_.Box(row.node, 8.0f, 8.0f);
		row.title = ui::TextNode(fl_, row.node, row.titleStyle, text);
		YGNodeStyleSetFlexGrow(row.title, 1.0f);
		YGNodeStyleSetFlexShrink(row.title, 1.0f);
		if (buttonW > 0.0f)
			row.controlNode = fl_.Box(row.node, buttonW, 40.0f);
		return row;
	}

	// A sentence on its own row.
	Row &Note(const std::string &text, ImU32 ink)
	{
		Row &row = NewRow();
		row.titleText = text;
		row.titleStyle = ui::type::Callout;
		row.titleInk = ink;
		YGNodeStyleSetPadding(row.node, YGEdgeHorizontal, 20.0f);
		YGNodeStyleSetPadding(row.node, YGEdgeVertical, 14.0f);
		YGNodeStyleSetMinHeight(row.node, 52.0f);
		row.title = ui::TextNode(fl_, row.node, row.titleStyle, text);
		YGNodeStyleSetFlexGrow(row.title, 1.0f);
		YGNodeStyleSetFlexShrink(row.title, 1.0f);
		return row;
	}

	// A full-width button on its own row; label is English, as built.
	Row &Action(const std::string &label, ui::Icon icon, const std::function<void()> &pressed)
	{
		Row &row = NewRow();
		YGNodeStyleSetPadding(row.node, YGEdgeAll, 12.0f);
		row.controlNode = fl_.Add(row.node);
		YGNodeStyleSetFlexGrow(row.controlNode, 1.0f);
		YGNodeStyleSetHeight(row.controlNode, 46.0f);
		row.drawControl = [label, icon, pressed](const FlexRect &r) {
			if (ui::PillButton("##action", label.c_str(), ui::Btn::Primary, r, icon, 17.0f))
				pressed();
		};
		return row;
	}

	// Lays the page out from origin, draws it and runs its controls; returns
	// its height.
	float Finish(ImVec2 origin)
	{
		fl_.Compute(origin, width_, YGUndefined);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		if (title_)
		{
			const FlexRect t = fl_.Rect(title_);
			ui::DrawText(dl, ui::type::PageTitle, t.min, t.W() + 1.0f, ui::col::Text, titleText_.c_str());
		}
		for (const Heading &h : headings_)
		{
			const FlexRect t = fl_.Rect(h.text);
			ui::DrawText(dl, ui::type::Section, t.min, t.W() + 1.0f, ui::col::Text, h.label.c_str());
			if (h.badge)
				ui::BadgePill(dl, fl_.Rect(h.badge).min, h.badgeEnglish, ui::col::Caution, ui::col::CautionTint);
		}
		for (YGNodeRef card : cards_)
		{
			const FlexRect c = fl_.Rect(card);
			ui::Card(dl, c.min, c.max);
		}
		for (size_t i = 0; i < rows_.size(); ++i)
		{
			Row &row = rows_[i];
			const FlexRect r = fl_.Rect(row.node);
			const bool first = i == 0 || rows_[i - 1].card != row.card;
			const bool last = i + 1 == rows_.size() || rows_[i + 1].card != row.card;
			ImGui::PushID(static_cast<int>(i));
			if (row.onPress)
			{
				ImGui::SetCursorScreenPos(r.min);
				if (ImGui::InvisibleButton("##row", r.Size(), ImGuiButtonFlags_EnableNav))
					row.onPress();
				if (ImGui::IsItemHovered())
				{
					const ImDrawFlags corners = first && last ? ImDrawFlags_RoundCornersAll
						: first ? ImDrawFlags_RoundCornersTop : last ? ImDrawFlags_RoundCornersBottom : ImDrawFlags_RoundCornersNone;
					ui::FillRounded(dl, r.min, r.max, ui::Rgba(255, 255, 255, ImGui::IsItemActive() ? 0.03f : 0.05f), 20.0f, corners);
				}
				ui::FocusRing(dl, r.min, r.max, 14.0f);
			}
			if (!first)
				ui::Hairline(dl, r.min.x + 20.0f, r.max.x, r.min.y);
			if (row.lead)
			{
				const ImVec2 c = fl_.Rect(row.lead).Center();
				if (row.dot)
					dl->AddCircleFilled(c, 4.0f, row.dot, 16);
				else
					ui::DrawIcon(dl, row.icon, c, 20.0f, row.titleInk);
			}
			const FlexRect t = fl_.Rect(row.title);
			ui::DrawText(dl, row.titleStyle, t.min, t.W() + 1.0f, row.titleInk, row.titleText.c_str());
			if (row.badge)
				ui::BadgePill(dl, fl_.Rect(row.badge).min, row.badgeEnglish, ui::col::Caution, ui::col::CautionTint);
			if (row.sub)
			{
				const FlexRect s = fl_.Rect(row.sub);
				ui::DrawText(dl, ui::type::Footnote, s.min, s.W() + 1.0f, kSubInk, row.subText.c_str());
			}
			if (row.chevron)
				ui::DrawIcon(dl, ui::Icon::Chevron, fl_.Rect(row.chevron).Center(), 18.0f, ui::Rgba(236, 238, 244, 0.45f), 2.4f);
			if (row.drawControl && row.controlNode)
				row.drawControl(fl_.Rect(row.controlNode));
			if ((row.tipEnglish || !row.tipText.empty()) && ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(r.min, r.max))
				ui::Tip(row.tipEnglish ? row.tipEnglish : row.tipText.c_str());
			ImGui::PopID();
		}
		return YGNodeLayoutGetHeight(root_);
	}

private:
	struct Heading
	{
		YGNodeRef node = nullptr, text = nullptr, badge = nullptr;
		std::string label;
		const char *badgeEnglish = nullptr;
	};

	Row &NewRow()
	{
		Row &row = rows_.emplace_back();
		row.card = cards_.size() - 1;
		row.node = fl_.Row(cards_.back());
		YGNodeStyleSetAlignItems(row.node, YGAlignCenter);
		return row;
	}

	FlexLayout fl_;
	YGNodeRef root_ = nullptr, title_ = nullptr;
	std::string titleText_;
	float width_ = 0.0f;
	std::vector<Heading> headings_;
	std::vector<YGNodeRef> cards_;
	std::deque<Row> rows_;   // stable addresses while rows are added
};

void OpenUrl(const char *url)
{
	if (!g_uiPreviewMode)
		ShellExecuteA(nullptr, "open", url, nullptr, nullptr, SW_SHOWNORMAL);
}

// A setting that lives in the settings record: changed in place, rolled back
// when it cannot be saved.
std::function<void(bool)> SettingSwitch(bool &setting)
{
	return [&setting](bool value) {
		const bool previous = setting;
		setting = value;
		SaveSettingOrRestore(setting, previous);
	};
}

// ---------------------------------------------------------------------------
// General
// ---------------------------------------------------------------------------

void GeneralRows(RowPage &page)
{
	page.Section("General");
	const bool translated = questcal::i18n::CurrentLanguage() != Language::English;
	RowPage::Row &language = page.Add(Tr("Language"),
		translated ? std::string(Tr("This translation may not be accurate.")) : std::string(), LanguagePickerWidth(), 42.0f);
	language.drawControl = [](const FlexRect &r) { LanguagePicker("##language", r); };
	// A translation says it may be imperfect and where corrections go.
	if (translated)
		page.Link(ui::Icon::Globe, "Report on GitHub", ui::col::Link, false,
			[] { OpenUrl("https://github.com/VividNightmareUnleashed/NovaCalibrator/issues"); });

	page.Toggle("Advanced mode", "Shows calibration measurements, drift readings and extra settings.",
		CalCtx.uiAdvanced, SettingSwitch(CalCtx.uiAdvanced));
	// One switch for every alignment toast: a calibration that looks off,
	// continuous calibration pausing, noisy tracking.
	page.Toggle("Alignment notifications in VR",
		"Get a SteamVR notification when the alignment looks off or continuous calibration pauses.",
		CalCtx.notifyPoorCalibration, SettingSwitch(CalCtx.notifyPoorCalibration));
}

// ---------------------------------------------------------------------------
// Continuous calibration
// ---------------------------------------------------------------------------

// The trackers continuous calibration can use: every device on the target
// system but the headset. A disconnected one stays listed (it may be the
// pick) and says so.
std::vector<const VRDevice *> MountCandidates(const VRState &state)
{
	std::vector<const VRDevice *> candidates;
	for (const auto &d : state.devices)
		if (d.trackingSystem == CalCtx.targetTrackingSystem && d.deviceClass != vr::TrackedDeviceClass_HMD)
			candidates.push_back(&d);
	return candidates;
}

void PickMountTracker(const std::string &serial)
{
	if (CalCtx.continuousTrackerSerial == serial)
		return;
	// The serial and the mount offset are one edit: the learned offset
	// described the previous tracker, so both land together or not at all.
	SaveProfileFieldEdit(CalCtx,
		[&](questcal::ProfileRecord &candidate) {
			candidate.continuousTrackerSerial = serial;
			candidate.mountExtrinsic = questcal::MountExtrinsicRecord();
		});
	// The counts described the previous tracker's loop.
	CalCtx.autoCorrectionsApplied = 0;
	CalCtx.continuousReanchors = 0;
	CalCtx.continuousReanchorsUndone = 0;
	CalCtx.continuousReanchorsAcrossSolutions = 0;
}

const ui::TextStyle kPickerName{ ui::Weight::Medium, 16.0f, 42.0f };
const ui::TextStyle kPickerSerial{ ui::Weight::Regular, 16.0f, 42.0f };

// What the picker button says: the pick's name and serial, or that there is
// none yet.
struct PickerLabel
{
	std::string name, serial;
	bool placeholder = false;
};

PickerLabel CurrentPick(const std::vector<const VRDevice *> &candidates)
{
	PickerLabel label;
	for (const VRDevice *d : candidates)
		if (d->serial == CalCtx.continuousTrackerSerial)
		{
			label.name = DeviceDisplayName(*d);
			label.serial = d->serial;
			if (!d->connected)
				label.serial += std::string("  ") + Tr("Off");
			return label;
		}
	if (!CalCtx.continuousTrackerSerial.empty())
	{
		label.name = CalCtx.continuousTrackerSerial;
		label.serial = Tr("Off");
		return label;
	}
	label.name = Tr("Pick a tracker");
	label.placeholder = true;
	return label;
}

float PickerWidth(const PickerLabel &label)
{
	float w = 18.0f + ui::MeasureLine(kPickerName, label.name.c_str()).x + 10.0f + 16.0f + 12.0f;
	if (!label.serial.empty())
		w += ui::MeasureLine(kPickerSerial, label.serial.c_str()).x + 10.0f;
	return std::ceil(std::min(w, 440.0f));
}

void TrackerPicker(const FlexRect &r, const PickerLabel &label, const std::vector<const VRDevice *> &candidates)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImGui::SetCursorScreenPos(r.min);
	const bool pressed = ImGui::InvisibleButton("##picker", r.Size(), ImGuiButtonFlags_EnableNav);
	const bool hov = ImGui::IsItemHovered();
	ui::FillRounded(dl, r.min, r.max, ui::Rgba(255, 255, 255, hov ? 0.12f : 0.08f), r.H() * 0.5f);
	ui::FocusRing(dl, r.min, r.max, r.H() * 0.5f);
	float x = r.min.x + 18.0f;
	const float serialW = label.serial.empty() ? 0.0f : ui::MeasureLine(kPickerSerial, label.serial.c_str()).x + 10.0f;
	const std::string name = ui::Ellipsize(kPickerName, label.name, r.max.x - 12.0f - 16.0f - 10.0f - serialW - x);
	ui::DrawLine(dl, kPickerName, ImVec2(x, r.min.y), label.placeholder ? ui::col::Muted : ui::col::Text, name.c_str());
	x += ui::MeasureLine(kPickerName, name.c_str()).x + 10.0f;
	if (!label.serial.empty())
		ui::DrawLine(dl, kPickerSerial, ImVec2(x, r.min.y), ui::Rgba(236, 238, 244, 0.62f), label.serial.c_str());
	ui::DrawIcon(dl, ui::Icon::ChevronUpDown, ImVec2(r.max.x - 12.0f - 8.0f, r.Center().y), 16.0f,
		ui::Rgba(236, 238, 244, 0.66f), 2.4f);
	if (pressed)
		ImGui::OpenPopup("##trackers");

	if (!ui::BeginMenuPopup("##trackers", ImVec2(r.max.x, r.max.y + 6.0f), ImVec2(1.0f, 0.0f), std::max(r.W(), 380.0f)))
		return;
	if (candidates.empty())
	{
		const float w = ImGui::GetContentRegionAvail().x;
		const ImVec2 a = ImGui::GetCursorScreenPos();
		const float h = ui::DrawText(ImGui::GetWindowDrawList(), ui::type::Callout, ImVec2(a.x + 14.0f, a.y + 12.0f), w - 28.0f,
			ui::col::Muted, Tr("No trackers found. Turn on a tracker and make sure SteamVR sees it."));
		ImGui::Dummy(ImVec2(w, h + 24.0f));
	}
	for (const VRDevice *d : candidates)
	{
		ImGui::PushID(d->serial.c_str());
		const ImVec2 a = ImGui::GetCursorScreenPos();
		const ImVec2 size(ImGui::GetContentRegionAvail().x, 48.0f);
		if (ImGui::InvisibleButton("##tracker", size, ImGuiButtonFlags_EnableNav))
		{
			PickMountTracker(d->serial);
			ImGui::CloseCurrentPopup();
		}
		ImDrawList *pl = ImGui::GetWindowDrawList();
		const ImVec2 b(a.x + size.x, a.y + size.y);
		if (ImGui::IsItemHovered())
			ui::FillRounded(pl, a, b, ui::Rgba(255, 255, 255, 0.09f), 11.0f);
		ui::FocusRing(pl, a, b, 11.0f);
		if (d->serial == CalCtx.continuousTrackerSerial)
			ui::DrawIcon(pl, ui::Icon::Check, ImVec2(a.x + 14.0f + 9.0f, a.y + size.y * 0.5f), 18.0f, ui::col::Link, 2.4f);
		const ui::TextStyle nameStyle{ ui::Weight::Medium, 16.0f, size.y }, serialStyle{ ui::Weight::Regular, 15.0f, size.y };
		const std::string itemName = DeviceDisplayName(*d);
		float tx = a.x + 44.0f;
		ui::DrawLine(pl, nameStyle, ImVec2(tx, a.y), d->connected ? ui::col::Text : ui::col::Faint, itemName.c_str());
		tx += ui::MeasureLine(nameStyle, itemName.c_str()).x + 10.0f;
		const std::string serial = d->connected ? d->serial : d->serial + "  " + Tr("Off");
		ui::DrawLine(pl, serialStyle, ImVec2(tx, a.y), ui::col::Muted, serial.c_str());
		ImGui::PopID();
	}
	ui::EndMenuPopup();
}

void ContinuousRows(RowPage &page, const VRState &state)
{
	page.Section("Continuous calibration");
	page.Toggle("Continuous calibration", "Keeps devices aligned while you play, using a tracker strapped to your headset.",
		CalCtx.continuousEnabled, [](bool value) {
			SaveProfileFieldEdit(CalCtx, [&](questcal::ProfileRecord &candidate) {
				candidate.continuousEnabled = value;
			});
		});
	if (!CalCtx.continuousEnabled)
		return;

	const std::vector<const VRDevice *> candidates = MountCandidates(state);
	const PickerLabel pick = CurrentPick(candidates);
	RowPage::Row &picker = page.Add(Tr("Headset tracker"), std::string(), PickerWidth(pick), 42.0f);
	picker.drawControl = [pick, candidates](const FlexRect &r) { TrackerPicker(r, pick, candidates); };

	// How the loop meets a disagreement: both are a method and neither is
	// "on". Legacy never pauses, as the original SpaceCalibrator's continuous
	// mode does.
	static const char *const methods[] = { "Standard", "Legacy" };
	const int mode = CalCtx.continuousNoPause ? 1 : 0;
	RowPage::Row &method = page.Add(Tr("Method"), std::string(), ui::PillSegmentedWidth(methods, 2, 16.0f, 120.0f), 42.0f);
	method.drawControl = [mode](const FlexRect &r) {
		const int picked = ui::PillSegmented("##method", mode, methods, 2, r, 16.0f);
		if (picked != mode)
			SaveProfileFieldEdit(CalCtx, [&](questcal::ProfileRecord &candidate) {
				candidate.continuousNoPause = picked == 1;
			});
	};
	method.tipEnglish = "Standard pauses when the readings move away from the calibration\n"
		"and resumes once they come back or hold steady. Legacy never pauses:\n"
		"like OpenVR-SpaceCalibrator, it follows every change the headset tracker\n"
		"reports, so a bad base station fix moves your body trackers until it clears.";

	page.Toggle("Hide the headset tracker from games", nullptr, CalCtx.hideMountedTracker, [](bool value) {
		SaveProfileFieldEdit(CalCtx, [&](questcal::ProfileRecord &candidate) { candidate.hideMountedTracker = value; });
	}).tipEnglish = "Moves it far out of reach in games so full-body setups never\n"
		"mistake it for a body tracker. Calibration still sees it.";
	page.Toggle("Re-measure tracking delay while you play", nullptr, CalCtx.continuousLatencyReestimation, [](bool value) {
		SaveProfileFieldEdit(CalCtx, [&](questcal::ProfileRecord &candidate) { candidate.continuousLatencyReestimation = value; });
	}).tipEnglish = "Re-measures the delay between the two systems during play, using\n"
		"the headset tracker. Off keeps the value from calibration.";
	page.Toggle("Ask before applying each correction", nullptr, CalCtx.continuousRequireTrigger, [](bool value) {
		SaveProfileFieldEdit(CalCtx, [&](questcal::ProfileRecord &candidate) { candidate.continuousRequireTrigger = value; });
	}).tipEnglish = "Each correction waits until you pull a controller trigger.\n"
		"Recent activity on the Calibration page lists each one.";

	// What the loop needs from the player, as a button whenever the app can
	// do it for them.
	const ContinuousStatus continuous = ContinuousStatusNow();
	if (continuous == ContinuousStatus::NoTracker)
		page.Note(Tr("Strap a tracker to your headset and pick it above."), ui::col::Lavender);
	else if (continuous == ContinuousStatus::NeedsMount || continuous == ContinuousStatus::Frozen)
	{
		const std::string label = FormatString(continuous == ContinuousStatus::Frozen
			? "Recalibrate with the headset tracker (%.0f s)" : "Set up headset tracker (%.0f s)", CalCtx.CollectionSeconds());
		page.Action(label, ui::Icon::Play, [&state] { StartMountSetup(state); });
	}
}

// ---------------------------------------------------------------------------
// Calibration, updates, support
// ---------------------------------------------------------------------------

void CalibrationRows(RowPage &page)
{
	page.Section("Calibration");
	page.Toggle("Measure playspace scale",
		"Checks whether your headset and SteamVR measure distances differently. Leave it off unless you need it.",
		CalCtx.solveScale, SettingSwitch(CalCtx.solveScale), "Experimental");
	// The number is for the advanced reader; the switch is the setting.
	const std::string applied = CalCtx.uiAdvanced && CalCtx.validProfile && CalCtx.applyTimeOffset
		? FormatString("%+.1f ms", CalCtx.appliedTimeOffset * 1000.0) : std::string();
	page.Toggle("Correct for tracking delay", "Makes up for the lag between your headset and SteamVR, measured when you calibrate.",
		CalCtx.applyTimeOffset, SettingSwitch(CalCtx.applyTimeOffset), nullptr, applied);
}

// Updates never touch the network until this persisted opt-in is on.
// Checking and downloading are automatic; installing the verified package
// stays explicit, because Steam must close and Windows elevates the
// installer.
void UpdateRows(RowPage &page)
{
	page.Section("Updates");
	page.Toggle("Automatic updates", "Downloads verified updates. Close Steam before installing them.",
		CalCtx.automaticUpdates, [](bool value) {
			const bool previous = CalCtx.automaticUpdates;
			CalCtx.automaticUpdates = value;
			SaveSettingOrRestore(CalCtx.automaticUpdates, previous);
			if (CalCtx.automaticUpdates != previous && !g_uiPreviewMode)
				questcal::update::AppUpdater.SetEnabled(CalCtx.automaticUpdates);
		});
	if (!CalCtx.automaticUpdates)
		return;

	using questcal::update::State;
	const questcal::update::Snapshot update = CurrentUpdate();
	std::string status;
	ImU32 ink = ui::col::Muted;
	switch (update.state)
	{
	case State::Checking:
		status = "Checking for updates...";
		break;
	case State::Downloading:
	{
		const unsigned percent = update.totalBytes == 0 ? 0 :
			static_cast<unsigned>((update.downloadedBytes * 100) / update.totalBytes);
		status = FormatString("Downloading %s  %u%%", update.version.c_str(), percent);
		ink = ui::col::Link;
		break;
	}
	case State::UpToDate:
		status = FormatString("You're up to date (%s)", update.version.c_str());
		ink = ui::col::Good;
		break;
	case State::Ready:
		status = FormatString("Version %s is ready to install", update.version.c_str());
		ink = ui::col::Good;
		break;
	case State::Failed:
		status = update.message.empty() ? std::string("Update check failed")
			: FormatString("Update check failed: %s", update.message.c_str());
		ink = ui::col::Alert;
		break;
	case State::Prerelease:
		status = FormatString("%s is a test build and doesn't update itself", update.version.c_str());
		ink = ui::col::Caution;
		break;
	default:
		status = "Not checked yet";
		break;
	}
	const bool installReady = update.state == State::Ready;
	const bool canRetry = update.state == State::Failed || update.state == State::UpToDate || update.state == State::Idle;
	const std::string action = installReady ? FormatString("Install %s", update.version.c_str())
		: canRetry ? std::string("Check again") : std::string();
	const ui::Icon icon = installReady ? ui::Icon::Download : ui::Icon::None;
	const float buttonW = action.empty() ? 0.0f : ui::PillWidth(action.c_str(), 16.0f, icon, 20.0f);
	RowPage::Row &row = page.Status(ink, Tr(status), buttonW);
	if (!action.empty())
		row.drawControl = [action, icon, installReady](const FlexRect &r) {
			if (!ui::PillButton("##update", action.c_str(), installReady ? ui::Btn::Primary : ui::Btn::Secondary, r, icon, 16.0f))
				return;
			if (installReady)
			{
				std::string error;
				if (g_uiPreviewMode)
					return;
				if (questcal::update::AppUpdater.LaunchInstaller(error))
					RequestApplicationExit();
				else
					CalCtx.ReportError(error + "\n");
			}
			else if (!g_uiPreviewMode)
				questcal::update::AppUpdater.CheckNow();
		};
	// A test build says why it stays put; a failure already says why in the
	// status line.
	if (update.state == State::Prerelease)
		row.tipText = update.message;
}

void SupportRows(RowPage &page)
{
	page.Section("Support");
	// Extra detail in the session log while on, and one file to send,
	// written on request with personal folders taken out.
	page.Toggle("Detailed calibration logging",
		"Records extra tracking and calibration details. Saved files leave out your name and folder paths.",
		CalCtx.detailedLogging, SettingSwitch(CalCtx.detailedLogging));
	page.Link(ui::Icon::Doc, "Save diagnostics file", ui::col::Link, false, [] {
		std::string path, error;
		if (WriteDiagnosticsFile(CalCtx, path, error, vr::VRSystem(), CaptureCalibrationDiagnostics()))
		{
			CalCtx.Tell("Diagnostics saved to " + PathForLog(path), CalibrationContext::Tone::Good);
			if (!g_uiPreviewMode)
				RevealInExplorer(path);
		}
		else
			CalCtx.ReportError(error + "\n");
	});
	// The raw transform, for those who know what they are changing.
	if (CalCtx.validProfile && !CalCtx.profileUniverseUnsafe)
		page.Link(ui::Icon::Pencil, "Edit calibration (advanced)", ui::col::Text, true, [] {
			SeedTransformEditorDraft();
			CalCtx.state = CalibrationState::Editing;
		});
	page.Link(ui::Icon::Book, "Credits", ui::col::Text, true, [] { OpenSheet(Sheet::Credits); });
}

} // namespace

float BuildSettingsPage(const VRState &state, ImVec2 origin, float width)
{
	RowPage page(width);
	page.Title("Settings");
	GeneralRows(page);
	// Continuous calibration's choices live in the calibration profile.
	if (CalCtx.validProfile)
		ContinuousRows(page, state);
	CalibrationRows(page);
	UpdateRows(page);
	SupportRows(page);
	return page.Finish(origin);
}

// ---------------------------------------------------------------------------
// The calibration editor
// ---------------------------------------------------------------------------

struct TransformEditorDraft
{
	bool valid = true;
	bool rotationEdited = false;
	Eigen::Quaterniond rotationQ{ 1, 0, 0, 0 };
	Eigen::Vector3d rotationEuler{ 0, 0, 0 };
	Eigen::Vector3d translationCm{ 0, 0, 0 };
	double scale = 1.0;
};

static TransformEditorDraft g_transformDraft;

// Every path into CalibrationState::Editing calls this immediately before
// setting the state, so the draft needs no "is it seeded" flag.
void SeedTransformEditorDraft()
{
	g_transformDraft = TransformEditorDraft();
	g_transformDraft.rotationQ = CalCtx.transform.rotation;
	g_transformDraft.rotationEuler = CalCtx.transform.RotationEulerDegrees();
	g_transformDraft.translationCm = CalCtx.transform.translationMeters * 100.0;
	g_transformDraft.scale = CalCtx.transform.scale;
}

// One field of the editor: its name, the value (typed straight in) and the
// steps down and up.
static bool NumberField(const char *id, const FlexRect &r, const char *label, double &value, double step)
{
	ImGui::PushID(id);
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ui::FillRounded(dl, r.min, r.max, ui::col::Well, 14.0f);
	const float textX = r.min.x + 16.0f;
	const float buttons = 32.0f * 2.0f + 8.0f;
	ui::DrawLine(dl, ui::TextStyle{ ui::Weight::Regular, 13.0f, 17.0f }, ImVec2(textX, r.min.y + 10.0f),
		ui::Rgba(236, 238, 244, 0.66f), label);
	ImGui::SetCursorScreenPos(ImVec2(textX, r.min.y + 10.0f + 17.0f));
	ImGui::PushFont(ui::FontOf(ui::Weight::SemiBold), ui::GlyphSize(18.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
	ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(0, 0, 0, 0));
	ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, IM_COL32(0, 0, 0, 0));
	ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ui::Rgba(255, 255, 255, 0.06f));
	ImGui::SetNextItemWidth(r.max.x - 10.0f - buttons - 8.0f - textX);
	bool changed = ImGui::InputDouble("##value", &value, 0.0, 0.0, "%.8f");
	ImGui::PopStyleColor(3);
	ImGui::PopStyleVar(2);
	ImGui::PopFont();
	const ImU32 fill = ui::Rgba(255, 255, 255, 0.10f);
	const ImVec2 plus(r.max.x - 10.0f - 16.0f, r.Center().y), minus(plus.x - 32.0f - 8.0f, plus.y);
	if (ui::RoundButton("##minus", minus, 32.0f, ui::Icon::Minus, 14.0f, fill, ui::col::Text))
	{
		value -= step;
		changed = true;
	}
	if (ui::RoundButton("##plus", plus, 32.0f, ui::Icon::Plus, 14.0f, fill, ui::col::Text))
	{
		value += step;
		changed = true;
	}
	ImGui::PopID();
	return changed;
}

// The editor's sheet. Saving returns the context to None, which closes it;
// Cancel drops the draft, since entering Editing seeds it again.
void BuildEditorSheet()
{
	const FlexRect c = SheetPanel(880.0f, 560.0f);
	const bool closed = SheetHeader(c, Tr("Edit calibration"));
	ImDrawList *dl = ImGui::GetWindowDrawList();
	float y = c.min.y + 44.0f + 6.0f;
	y += ui::DrawText(dl, ui::TextStyle{ ui::Weight::Regular, 17.0f, 24.0f }, ImVec2(c.min.x, y), std::min(c.W(), 760.0f),
		ui::col::Prose, Tr("Fine-tune the calibration by hand. Nothing changes until you save."));
	const float gap = 12.0f, cellW = (c.W() - gap * 2.0f) / 3.0f;
	auto group = [&](const char *english) {
		y += 20.0f;
		ui::DrawLine(dl, ui::type::FootnoteStrong, ImVec2(c.min.x, y), ui::col::Muted, Tr(english));
		y += 20.0f + 10.0f;
	};
	auto cell = [&](int column) {
		FlexRect r;
		r.min = ImVec2(c.min.x + (cellW + gap) * static_cast<float>(column), y);
		r.max = ImVec2(r.min.x + cellW, y + 60.0f);
		return r;
	};
	TransformEditorDraft &draft = g_transformDraft;
	bool rotationEdited = false, edited = false;
	group("Rotation, degrees");
	rotationEdited |= NumberField("yaw", cell(0), Tr("Yaw"), draft.rotationEuler(1), 0.1);
	rotationEdited |= NumberField("pitch", cell(1), Tr("Pitch"), draft.rotationEuler(2), 0.1);
	rotationEdited |= NumberField("roll", cell(2), Tr("Roll"), draft.rotationEuler(0), 0.1);
	y += 60.0f;
	group("Translation, centimeters");
	edited |= NumberField("x", cell(0), "X", draft.translationCm(0), 1.0);
	edited |= NumberField("y", cell(1), "Y", draft.translationCm(1), 1.0);
	edited |= NumberField("z", cell(2), "Z", draft.translationCm(2), 1.0);
	y += 60.0f;
	group("Scale");
	edited |= NumberField("scale", cell(0), Tr("Playspace scale"), draft.scale, 0.0001);
	y += 60.0f;

	if (edited || rotationEdited)
	{
		if (rotationEdited)
		{
			draft.rotationEdited = true;
			if (draft.rotationEuler.allFinite())
				draft.rotationQ = CalibrationContext::RebuildRotationFromEuler(draft.rotationEuler);
			else
				draft.rotationQ.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
		}
		const Eigen::Vector3d candidateTranslation = draft.translationCm * 0.01;
		draft.valid = questcal::IsValidCalibrationTransform(draft.rotationQ, candidateTranslation, draft.scale);
		for (const auto &anchor : CalCtx.fieldAnchors)
			draft.valid = draft.valid && questcal::IsValidFieldAnchor(anchor.position, anchor.rotation,
				anchor.translationMeters, draft.rotationQ, candidateTranslation);
	}

	FlexRect save, cancel;
	const float saveW = std::max(230.0f, ui::PillWidth("Save calibration", 18.0f, ui::Icon::Check));
	const float cancelW = std::max(150.0f, ui::PillWidth("Cancel", 18.0f));
	save.min = ImVec2(c.max.x - saveW, c.max.y - 52.0f);
	save.max = c.max;
	cancel.min = ImVec2(save.min.x - 12.0f - cancelW, save.min.y);
	cancel.max = ImVec2(save.min.x - 12.0f, save.max.y);
	// Invalid values keep the button's label and quiet it; the red line says
	// what to fix.
	if (!draft.valid)
		ui::DrawText(dl, ui::type::Callout, ImVec2(c.min.x, y + 16.0f), cancel.min.x - 24.0f - c.min.x, ui::col::Alert,
			Tr("These values can't be applied. Use plain numbers within range. The saved calibration is unchanged."));
	if (ui::PillButton("##saveprofile", "Save calibration", draft.valid ? ui::Btn::Primary : ui::Btn::Waiting, save, ui::Icon::Check) &&
		draft.valid)
		SaveProfileEditorDraft();
	if (ui::PillButton("##cancelprofile", "Cancel", ui::Btn::Secondary, cancel) || closed)
	{
		CalCtx.state = CalibrationState::None;
		CalCtx.timeLastScan = -1e9;
	}
	if (CalCtx.state != CalibrationState::Editing)
		CloseSheet();
}

// Only for a draft the editor reported valid this frame.
void SaveProfileEditorDraft()
{
	// Persistence validates and writes a narrow profile candidate before
	// touching the live transform. Translation/scale-only edits keep the exact
	// quaternion bits; Euler conversion occurs only after a rotation edit.
	SaveProfileTransformEdit(CalCtx, g_transformDraft.rotationQ,
		g_transformDraft.translationCm * 0.01, g_transformDraft.scale,
		g_transformDraft.rotationEdited);
}

// ---------------------------------------------------------------------------
// Credits
// ---------------------------------------------------------------------------

void BuildCreditsSheet()
{
	const FlexRect c = SheetPanel(720.0f, 620.0f);
	if (SheetHeader(c, Tr("Credits")))
		CloseSheet();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	float y = c.min.y + 44.0f + 16.0f;

	// The app: its mark, name and version.
	if (const DeviceIconTex *icon = ArtTexture("ART_ICON"))
		dl->AddImage(static_cast<ImTextureID>(icon->tex), ImVec2(c.min.x, y), ImVec2(c.min.x + 56.0f, y + 56.0f));
	ui::DrawLine(dl, ui::type::CardTitle, ImVec2(c.min.x + 72.0f, y + 2.0f), ui::col::Text, "Nova Calibrator");
	ui::DrawLine(dl, ui::type::Footnote, ImVec2(c.min.x + 72.0f, y + 30.0f), ui::col::Muted,
		Tr(FormatString("Version %s", QUESTCAL_VERSION_STRING)).c_str());
	y += 56.0f + 18.0f;

	// Who made it, and what it builds on.
	y += ui::DrawText(dl, ui::type::Body, ImVec2(c.min.x, y), c.W(), ui::col::Body,
		Tr(FormatString("Nova Calibrator is made by %s and builds on %s by %s.",
			"VividNightmareUnleashed", "OpenVR-SpaceCalibrator", "pushrax")).c_str());
	y += 14.0f;
	struct LinkPill { const char *id, *english, *url; };
	static const LinkPill links[] = {
		{ "##github", "GitHub", "https://github.com/VividNightmareUnleashed" },
		{ "##jinxxy", "Jinxxy", "https://jinxxy.com/VividNightmare" },
		{ "##spacecal", "OpenVR-SpaceCalibrator", "https://github.com/pushrax/OpenVR-SpaceCalibrator" },
	};
	float x = c.min.x;
	for (const LinkPill &link : links)
	{
		FlexRect r;
		r.min = ImVec2(x, y);
		r.max = ImVec2(x + ui::PillWidth(link.english, 15.0f, ui::Icon::Globe, 16.0f), y + 36.0f);
		if (ui::PillButton(link.id, link.english, ui::Btn::Secondary, r, ui::Icon::Globe, 15.0f))
			OpenUrl(link.url);
		x = r.max.x + 10.0f;
	}
	y += 36.0f + 26.0f;

	// Where the device pictures come from, and the models the motion demos are
	// rendered from, with their licences.
	ui::DrawLine(dl, ui::type::Label, ImVec2(c.min.x, y), ui::col::Text, Tr("Pictures and models"));
	y += 23.0f + 10.0f;
	const ImVec2 wellMin(c.min.x, y), wellMax(c.max.x, c.max.y);
	ui::FillRounded(dl, wellMin, wellMax, ui::col::Well, 16.0f);
	ImGui::SetCursorScreenPos(ImVec2(wellMin.x + 6.0f, wellMin.y + 6.0f));
	ImGui::BeginChild("##credits", ImVec2(wellMax.x - wellMin.x - 12.0f, wellMax.y - wellMin.y - 12.0f), ImGuiChildFlags_None,
		ImGuiWindowFlags_NoBackground);
	const ImVec2 at = ImGui::GetCursorScreenPos();
	const float w = ImGui::GetContentRegionAvail().x - 24.0f;
	const float h = ui::DrawText(ImGui::GetWindowDrawList(), ui::type::Footnote, ImVec2(at.x + 12.0f, at.y + 8.0f), w,
		ui::col::Muted, GuideModelCredits().c_str());
	ImGui::Dummy(ImVec2(w, h + 16.0f));
	ImGui::EndChild();
}
