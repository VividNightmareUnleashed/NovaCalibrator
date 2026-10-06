// The first-launch setup: a welcome, the headset, the trackers, updates and a
// first calibration, each a page across the whole window. It shows until it
// is finished or skipped, and never to someone who already has a
// calibration.
#include "stdafx.h"
#include "UiInternal.h"

SetupStep g_setupStep = SetupStep::Welcome;

namespace
{

constexpr float kWindowW = 1200.0f, kWindowH = 800.0f;
const ImU32 kLeadInk = ui::Rgba(236, 238, 244, 0.66f);
const ui::TextStyle kLead{ ui::Weight::Regular, 19.0f, 28.0f };
const ui::TextStyle kFeatureTitle{ ui::Weight::SemiBold, 19.0f, 24.0f };
const ui::TextStyle kFeatureBody{ ui::Weight::Regular, 17.0f, 23.0f };
const ui::TextStyle kRowName{ ui::Weight::SemiBold, 19.0f, 24.0f };
const ui::TextStyle kRowSub{ ui::Weight::Regular, 16.0f, 21.0f };

// The kinds of tracker the setup asks about, told apart by the model names
// SteamVR reports for them.
struct TrackerKind
{
	const char *english;
	bool CalibrationContext::*uses;
	bool untested;
	ui::Icon glyph;
};
const TrackerKind kKinds[] = {
	{ "VIVE Tracker 3.0", &CalibrationContext::usesViveTracker3, false, ui::Icon::Tracker },
	{ "Tundra Tracker", &CalibrationContext::usesTundraTracker, true, ui::Icon::Tracker },
	{ "Index Controllers", &CalibrationContext::usesIndexControllers, false, ui::Icon::Controller },
	{ "VIVE Tracker (2018)", &CalibrationContext::usesViveTracker2018, true, ui::Icon::Tracker },
};
constexpr int kKindCount = static_cast<int>(sizeof kKinds / sizeof kKinds[0]);

// Which of the kinds a device is, or -1.
int KindOf(const VRDevice &dev)
{
	if (dev.trackingSystem != "lighthouse" || dev.deviceClass == vr::TrackedDeviceClass_HMD)
		return -1;
	std::string model = dev.model;
	for (char &c : model)
		c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
	if (model.find("tundra") != std::string::npos)
		return 1;
	if (model.find("knuckles") != std::string::npos || model.find("index") != std::string::npos)
		return 2;
	if (model.find("tracker 3.0") != std::string::npos)
		return 0;
	if (model.find("vive tracker") != std::string::npos)
		return 3;
	return -1;
}

int ConnectedOfKind(const VRState &state, int kind)
{
	int count = 0;
	for (const auto &dev : state.devices)
		if (dev.connected && KindOf(dev) == kind)
			++count;
	return count;
}

const VRDevice *FirstOfKind(const VRState &state, int kind)
{
	for (const auto &dev : state.devices)
		if (dev.connected && KindOf(dev) == kind)
			return &dev;
	return nullptr;
}

const VRDevice *Headset(const VRState &state)
{
	for (const auto &dev : state.devices)
		if (dev.deviceClass == vr::TrackedDeviceClass_HMD)
			return &dev;
	return nullptr;
}

// A kind's picture: a connected one's art, or its glyph.
void DrawKindArt(ImDrawList *dl, const VRState &state, int kind, const FlexRect &box, bool faded)
{
	if (const VRDevice *dev = FirstOfKind(state, kind))
		DrawDeviceArt(dl, *dev, box, ui::col::Muted);
	else
		ui::DrawIcon(dl, kKinds[kind].glyph, box.Center(), box.W() * 0.62f, faded ? ui::col::Faint : ui::col::Muted, 1.6f);
}

struct SetupState
{
	bool kindsSeeded = false;
	bool chosen[kKindCount] = {};
	bool pairSeeded = false;
	double demoTime = 0.0;
};
SetupState s_setup;

void Go(SetupStep step)
{
	// The motion demo's atlas is large; it stays loaded only on its page.
	if (g_setupStep == SetupStep::FirstCalibration && step != SetupStep::FirstCalibration)
		ReleaseGuideTexture();
	g_setupStep = step;
	s_setup.pairSeeded = false;
}

void FinishSetup()
{
	// The rest of this session goes on to the app even if the save fails;
	// the next launch would then ask again.
	CalCtx.onboarded = true;
	SaveSettings(CalCtx);
}

// ---------------------------------------------------------------------------
// The parts every step shares
// ---------------------------------------------------------------------------

void StepDots(ImDrawList *dl)
{
	const int count = static_cast<int>(SetupStep::Count), current = static_cast<int>(g_setupStep);
	const float w = 8.0f * static_cast<float>(count) + 10.0f * static_cast<float>(count - 1);
	float x = (kWindowW - w) * 0.5f + 4.0f;
	for (int i = 0; i < count; ++i, x += 18.0f)
		dl->AddCircleFilled(ImVec2(x, kWindowH - 36.0f - 4.0f), 4.0f, i == current ? ui::col::White : ui::Rgba(255, 255, 255, 0.26f), 12);
}

bool BackButton(ImDrawList *dl)
{
	const ui::TextStyle style{ ui::Weight::Medium, 17.0f, 44.0f };
	const char *label = Tr("Back");
	FlexRect r;
	r.min = ImVec2(32.0f, 28.0f);
	r.max = ImVec2(r.min.x + 10.0f + 18.0f + 4.0f + ui::MeasureLine(style, label).x + 18.0f, r.min.y + 44.0f);
	ImGui::SetCursorScreenPos(r.min);
	const bool pressed = ImGui::InvisibleButton("##back", r.Size(), ImGuiButtonFlags_EnableNav);
	ui::FillRounded(dl, r.min, r.max, ui::Rgba(255, 255, 255, ImGui::IsItemHovered() ? 0.12f : 0.08f), 22.0f);
	ui::FocusRing(dl, r.min, r.max, 22.0f);
	ui::DrawIcon(dl, ui::Icon::ChevronLeft, ImVec2(r.min.x + 10.0f + 9.0f, r.Center().y), 18.0f, ui::col::Text, 2.4f);
	ui::DrawLine(dl, style, ImVec2(r.min.x + 10.0f + 18.0f + 4.0f, r.min.y), ui::col::Text, label);
	return pressed || (EscapePressed() && !DialogOpen() && !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId));
}

// The page's main button, above the step dots.
bool MainButton(const char *id, const char *english, float width, ui::Icon icon = ui::Icon::None)
{
	FlexRect r;
	r.min = ImVec2((kWindowW - width) * 0.5f, kWindowH - 36.0f - 8.0f - 26.0f - 54.0f);
	r.max = ImVec2(r.min.x + width, r.min.y + 54.0f);
	return ui::PillButton(id, english, ui::Btn::Primary, r, icon, 19.0f);
}

// A page's column: centred, from top, as wide as the window.
struct Page
{
	FlexLayout fl;
	YGNodeRef root = nullptr;
	std::vector<std::pair<YGNodeRef, std::pair<ui::TextStyle, std::string>>> centred;

	explicit Page(float top)
	{
		root = fl.Root();
		YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
		YGNodeStyleSetAlignItems(root, YGAlignCenter);
		YGNodeStyleSetWidth(root, kWindowW);
		YGNodeStyleSetPadding(root, YGEdgeTop, top);
	}

	// A line of centred text (drawn by Draw).
	YGNodeRef Text(const ui::TextStyle &style, const std::string &text, float marginTop, float maxWidth)
	{
		YGNodeRef node = ui::TextNode(fl, root, style, text);
		YGNodeStyleSetMargin(node, YGEdgeTop, marginTop);
		YGNodeStyleSetMaxWidth(node, maxWidth);
		centred.push_back({ node, { style, text } });
		return node;
	}

	void Compute()
	{
		fl.Compute(ImVec2(0.0f, 0.0f), kWindowW, YGUndefined);
	}

	void DrawTexts(ImDrawList *dl, ImU32 titleInk, ImU32 bodyInk)
	{
		for (size_t i = 0; i < centred.size(); ++i)
		{
			const FlexRect r = fl.Rect(centred[i].first);
			ui::DrawText(dl, centred[i].second.first, r.min, r.W() + 1.0f, i == 0 ? titleInk : bodyInk,
				centred[i].second.second.c_str(), ui::Align::Center);
		}
	}
};

// Three short points, each an icon, a title and a sentence.
struct Feature
{
	ui::Icon icon;
	const char *title, *body;
};

struct FeatureNodes
{
	YGNodeRef icon, title, body;
	std::string titleText, bodyText;
};

std::vector<FeatureNodes> LayOutFeatures(Page &page, const Feature *features, int count, float marginTop, float gap)
{
	YGNodeRef list = page.fl.Column(page.root);
	YGNodeStyleSetWidth(list, 600.0f);
	YGNodeStyleSetMargin(list, YGEdgeTop, marginTop);
	YGNodeStyleSetGap(list, YGGutterRow, gap);
	std::vector<FeatureNodes> nodes;
	for (int i = 0; i < count; ++i)
	{
		FeatureNodes n;
		n.titleText = Tr(features[i].title);
		n.bodyText = Tr(features[i].body);
		YGNodeRef row = page.fl.Row(list);
		YGNodeStyleSetGap(row, YGGutterColumn, 18.0f);
		n.icon = page.fl.Box(row, 40.0f, 40.0f);
		YGNodeRef text = page.fl.Column(row);
		YGNodeStyleSetFlexShrink(text, 1.0f);
		YGNodeStyleSetGap(text, YGGutterRow, 3.0f);
		n.title = ui::TextNode(page.fl, text, kFeatureTitle, n.titleText);
		n.body = ui::TextNode(page.fl, text, kFeatureBody, n.bodyText);
		nodes.push_back(std::move(n));
	}
	return nodes;
}

void DrawFeatures(ImDrawList *dl, Page &page, const Feature *features, const std::vector<FeatureNodes> &nodes)
{
	for (size_t i = 0; i < nodes.size(); ++i)
	{
		ui::DrawIcon(dl, features[i].icon, page.fl.Rect(nodes[i].icon).Center(), 26.0f, ui::col::Link);
		const FlexRect t = page.fl.Rect(nodes[i].title), b = page.fl.Rect(nodes[i].body);
		ui::DrawText(dl, kFeatureTitle, t.min, t.W() + 1.0f, ui::col::Text, nodes[i].titleText.c_str());
		ui::DrawText(dl, kFeatureBody, b.min, b.W() + 1.0f, kLeadInk, nodes[i].bodyText.c_str());
	}
}

// A choice row on a setup card: art, name, sub-line, and what sits at its
// end (drawn by the caller at the returned rectangle's right).
void RowText(ImDrawList *dl, const FlexRect &row, float textX, const char *name, const char *sub, ImU32 nameInk, ImU32 subInk)
{
	if (sub && *sub)
	{
		ui::DrawLine(dl, kRowName, ImVec2(textX, row.Center().y - 24.0f + 1.0f), nameInk, name);
		ui::DrawLine(dl, kRowSub, ImVec2(textX, row.Center().y + 1.0f), subInk, sub);
	}
	else
		ui::DrawLine(dl, ui::TextStyle{ kRowName.weight, kRowName.size, row.H() }, ImVec2(textX, row.min.y), nameInk, name);
}

// ---------------------------------------------------------------------------
// The steps
// ---------------------------------------------------------------------------

void WelcomeStep()
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	// Each language in its own words, so a player who cannot read this one
	// still finds theirs.
	const float languageW = LanguagePickerWidth();
	FlexRect language;
	language.max = ImVec2(kWindowW - 32.0f, 28.0f + 44.0f);
	language.min = ImVec2(language.max.x - languageW, 28.0f);
	ui::DrawIcon(dl, ui::Icon::Globe, ImVec2(language.min.x - 20.0f, language.Center().y), 20.0f, ui::col::Muted);
	LanguagePicker("##setuplanguage", language);

	static const Feature features[] = {
		{ ui::Icon::Reticle, "Calibrate in seconds", "Hold a controller to a tracker and move both for 10 seconds." },
		{ ui::Icon::Refresh, "Stays aligned while you play", "When your headset re-centers or corrects itself, your trackers follow." },
		{ ui::Icon::Bell, "Warns you about drift", "You'll get a notification in VR if the alignment starts to slip." },
	};
	Page page(76.0f);
	YGNodeRef icon = page.fl.Box(page.root, 104.0f, 104.0f);
	page.Text(ui::type::Welcome, Tr("Welcome to Nova Calibrator"), 28.0f, 1000.0f);
	page.Text(kLead, Tr("Play with your headset and SteamVR trackers in one playspace. "
		"Nova Calibrator lines them up, so your trackers sit where your body is."), 14.0f, 640.0f);
	const auto nodes = LayOutFeatures(page, features, 3, 44.0f, 24.0f);
	page.Compute();

	const FlexRect i = page.fl.Rect(icon);
	ui::SoftShadow(dl, i.min, i.max, 24.0f, 24.0f, 10.0f, ui::Rgba(0, 0, 0, 0.35f));
	if (const DeviceIconTex *art = ArtTexture("ART_ICON"))
		dl->AddImage(static_cast<ImTextureID>(art->tex), i.min, i.max);
	page.DrawTexts(dl, ui::col::Text, kLeadInk);
	DrawFeatures(dl, page, features, nodes);

	if (MainButton("##continue", "Continue", 340.0f))
		Go(SetupStep::Headset);
}

void HeadsetStep(const VRState &state)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (BackButton(dl))
		Go(SetupStep::Welcome);
	const VRDevice *headset = Headset(state);

	Page page(72.0f);
	YGNodeRef hero = page.fl.Box(page.root, 112.0f, 112.0f);
	page.Text(ui::type::SetupTitle, Tr("Which headset do you use?"), 14.0f, 1000.0f);
	page.Text(kLead, Tr("Your headset sets the playspace. Nova Calibrator lines your trackers up with it."), 12.0f, 640.0f);
	YGNodeRef card = page.fl.Column(page.root);
	YGNodeStyleSetMargin(card, YGEdgeTop, 28.0f);
	YGNodeRef quest = page.fl.Box(card, 680.0f, 80.0f);
	static const char *const later[] = { "Steam Frame", "Pico", "Samsung Galaxy XR" };
	YGNodeRef others[3];
	for (int k = 0; k < 3; ++k)
		others[k] = page.fl.Box(card, 680.0f, 72.0f);
	page.Compute();

	// Only the Quest's own picture: no other headset's photo ships.
	const FlexRect h = page.fl.Rect(hero);
	if (headset)
		DrawDeviceArt(dl, *headset, h, ui::col::Muted);
	else
		ui::DrawIcon(dl, ui::Icon::Headset, h.Center(), 84.0f, ui::col::Muted, 1.6f);
	page.DrawTexts(dl, ui::col::Text, kLeadInk);
	const FlexRect c = page.fl.Rect(card);
	ui::Card(dl, c.min, c.max);

	// The Quest is the one headset this works with today, so it is chosen.
	const FlexRect q = page.fl.Rect(quest);
	ui::FillRounded(dl, q.min, q.max, ui::Rgba(35, 112, 232, 0.18f), 20.0f, ImDrawFlags_RoundCornersTop);
	FlexRect art;
	art.min = ImVec2(q.min.x + 16.0f - 8.0f, q.Center().y - 32.0f);
	art.max = ImVec2(art.min.x + 64.0f, art.min.y + 64.0f);
	if (headset)
		DrawDeviceArt(dl, *headset, art, ui::col::Muted);
	else
		ui::DrawIcon(dl, ui::Icon::Headset, art.Center(), 44.0f, ui::col::Muted, 1.6f);
	RowText(dl, q, q.min.x + 80.0f, "Meta Quest", Tr("Quest 2, Quest 3, Quest 3S and Quest Pro"), ui::col::Text,
		ui::Rgba(236, 238, 244, 0.68f));
	float endX = q.max.x - 22.0f;
	ui::DrawIcon(dl, ui::Icon::Check, ImVec2(endX - 11.0f, q.Center().y), 22.0f, ui::col::Link, 2.4f);
	endX -= 22.0f + 16.0f;
	if (headset && headset->connected)
	{
		const char *connected = Tr("Connected");
		const ui::TextStyle chip{ ui::Weight::SemiBold, 14.0f, 30.0f };
		const float w = 12.0f + 7.0f + 7.0f + ui::MeasureLine(chip, connected).x + 12.0f;
		const ImVec2 a(endX - w, q.Center().y - 15.0f);
		ui::FillRounded(dl, a, ImVec2(a.x + w, a.y + 30.0f), ui::Rgba(61, 214, 140, 0.16f), 15.0f);
		dl->AddCircleFilled(ImVec2(a.x + 12.0f + 3.5f, a.y + 15.0f), 3.5f, ui::col::Good, 12);
		ui::DrawLine(dl, chip, ImVec2(a.x + 12.0f + 7.0f + 7.0f, a.y), ui::col::Good, connected);
	}
	for (int k = 0; k < 3; ++k)
	{
		const FlexRect r = page.fl.Rect(others[k]);
		ui::Hairline(dl, r.min.x + 80.0f, r.max.x, r.min.y);
		ui::DrawIcon(dl, ui::Icon::Headset, ImVec2(r.min.x + 16.0f + 24.0f, r.Center().y), 40.0f, ui::col::Faint, 1.6f);
		RowText(dl, r, r.min.x + 80.0f, later[k], nullptr, ui::Rgba(236, 238, 244, 0.56f), 0);
		const char *soon = Tr("Coming later");
		const ui::TextStyle pill{ ui::Weight::SemiBold, 14.0f, 30.0f };
		const float w = ui::MeasureLine(pill, soon).x + 24.0f;
		const ImVec2 a(r.max.x - 22.0f - w, r.Center().y - 15.0f);
		ui::FillRounded(dl, a, ImVec2(a.x + w, a.y + 30.0f), ui::col::Fill, 15.0f);
		ui::DrawLine(dl, pill, ImVec2(a.x + 12.0f, a.y), ui::Rgba(236, 238, 244, 0.72f), soon);
	}

	if (MainButton("##continue", "Continue", 340.0f))
		Go(SetupStep::Trackers);
}

void TrackersStep(const VRState &state)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (BackButton(dl))
		Go(SetupStep::Headset);
	// What was chosen before, or else every kind that is switched on.
	if (!s_setup.kindsSeeded)
	{
		bool any = false;
		for (int k = 0; k < kKindCount; ++k)
			any = any || CalCtx.*kKinds[k].uses;
		for (int k = 0; k < kKindCount; ++k)
			s_setup.chosen[k] = any ? CalCtx.*kKinds[k].uses : ConnectedOfKind(state, k) > 0;
		s_setup.kindsSeeded = true;
	}

	Page page(72.0f);
	YGNodeRef hero = page.fl.Box(page.root, 340.0f, 124.0f);
	page.Text(ui::type::SetupTitle, Tr("Which trackers do you use?"), 20.0f, 1000.0f);
	page.Text(kLead, Tr("Select every kind you use."), 12.0f, 640.0f);
	YGNodeRef card = page.fl.Column(page.root);
	YGNodeStyleSetMargin(card, YGEdgeTop, 28.0f);
	YGNodeRef rows[kKindCount];
	for (int k = 0; k < kKindCount; ++k)
		rows[k] = page.fl.Box(card, 680.0f, 72.0f);
	const std::string footnote = Tr("Greyed out? Turn that tracker on first.");
	YGNodeRef note = ui::TextNode(page.fl, page.root, ui::type::Footnote, footnote);
	YGNodeStyleSetMargin(note, YGEdgeTop, 14.0f);
	YGNodeStyleSetWidth(note, 680.0f);
	page.Compute();

	// The kinds side by side: controller, tracker, tracker.
	const FlexRect h = page.fl.Rect(hero);
	const int heroKinds[3] = { 2, 0, 1 };
	const float heroSizes[3] = { 118.0f, 124.0f, 100.0f };
	float hx = h.Center().x - (118.0f + 124.0f + 100.0f - 22.0f - 16.0f) * 0.5f;
	for (int i = 0; i < 3; ++i)
	{
		FlexRect box;
		box.min = ImVec2(hx, h.max.y - heroSizes[i] - (i == 2 ? 4.0f : 0.0f));
		box.max = ImVec2(hx + heroSizes[i], box.min.y + heroSizes[i]);
		DrawKindArt(dl, state, heroKinds[i], box, false);
		hx += heroSizes[i] - (i == 0 ? 22.0f : 16.0f);
	}
	page.DrawTexts(dl, ui::col::Text, kLeadInk);
	const FlexRect c = page.fl.Rect(card);
	ui::Card(dl, c.min, c.max);
	for (int k = 0; k < kKindCount; ++k)
	{
		const FlexRect r = page.fl.Rect(rows[k]);
		const int connected = ConnectedOfKind(state, k);
		const bool available = connected > 0;
		ImGui::PushID(k);
		if (k > 0)
			ui::Hairline(dl, r.min.x + 84.0f, r.max.x, r.min.y);
		if (available)
		{
			ImGui::SetCursorScreenPos(r.min);
			if (ImGui::InvisibleButton("##kind", r.Size(), ImGuiButtonFlags_EnableNav))
				s_setup.chosen[k] = !s_setup.chosen[k];
			if (ImGui::IsItemHovered())
				ui::FillRounded(dl, r.min, r.max, ui::Rgba(255, 255, 255, 0.04f), 20.0f,
					k == 0 ? ImDrawFlags_RoundCornersTop : k + 1 == kKindCount ? ImDrawFlags_RoundCornersBottom : ImDrawFlags_RoundCornersNone);
			ui::FocusRing(dl, r.min, r.max, 14.0f);
		}
		FlexRect art;
		art.min = ImVec2(r.min.x + 16.0f - 6.0f, r.Center().y - 30.0f);
		art.max = ImVec2(art.min.x + 60.0f, art.min.y + 60.0f);
		DrawKindArt(dl, state, k, art, !available);
		const std::string sub = available ? Tr(FormatString("%d connected", connected)) : std::string(Tr("Not connected"));
		RowText(dl, r, r.min.x + 16.0f + 60.0f - 10.0f + 16.0f, Tr(kKinds[k].english), sub.c_str(),
			available ? ui::col::Text : ui::col::Faint, available ? kLeadInk : ui::Rgba(236, 238, 244, 0.40f));
		float endX = r.max.x - 20.0f;
		if (available)
		{
			const ImVec2 mark(endX - 14.0f, r.Center().y);
			if (s_setup.chosen[k])
			{
				dl->AddCircleFilled(mark, 14.0f, ui::col::Accent, 28);
				ui::DrawIcon(dl, ui::Icon::Check, mark, 16.0f, ui::col::White, 3.0f);
			}
			else
				dl->AddCircle(mark, 13.0f, ui::Rgba(236, 238, 244, 0.38f), 28, 2.0f);
			endX -= 28.0f + 16.0f;
		}
		if (kKinds[k].untested)
		{
			const float w = ui::BadgeWidth("Untested");
			ui::BadgePill(dl, ImVec2(endX - w, r.Center().y - 13.0f), "Untested",
				ui::Fade(ui::col::Caution, available ? 1.0f : 0.6f), ui::Fade(ui::col::CautionTint, available ? 1.0f : 0.6f));
		}
		ImGui::PopID();
	}
	const FlexRect n = page.fl.Rect(note);
	ui::DrawText(dl, ui::type::Footnote, n.min, n.W(), ui::col::Subtle, footnote.c_str());

	if (MainButton("##continue", "Continue", 340.0f))
	{
		for (int k = 0; k < kKindCount; ++k)
			CalCtx.*kKinds[k].uses = s_setup.chosen[k];
		SaveSettings(CalCtx);
		Go(SetupStep::Updates);
	}
}

void UpdatesStep()
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (BackButton(dl))
		Go(SetupStep::Trackers);
	static const Feature features[] = {
		{ ui::Icon::ShieldCheck, "Checked before it's offered", "Every release is signed, and you're only offered ones that pass the check." },
		{ ui::Icon::Clock, "Installs when you say so", "Steam has to be closed to install, so updates always wait for you." },
		{ ui::Icon::Globe, "Talks only to GitHub", "It only ever checks Nova Calibrator's releases on GitHub." },
	};
	Page page(72.0f);
	YGNodeRef icon = page.fl.Box(page.root, 56.0f, 56.0f);
	page.Text(ui::type::SetupTitle, Tr("Keep Nova Calibrator up to date"), 20.0f, 1000.0f);
	page.Text(kLead, Tr("New versions download in the background. You decide when to install them."), 12.0f, 620.0f);
	const auto nodes = LayOutFeatures(page, features, 3, 40.0f, 22.0f);
	const std::string noteText = Tr("You can change this anytime in Settings.");
	YGNodeRef note = ui::TextNode(page.fl, page.root, ui::type::Footnote, noteText);
	YGNodeStyleSetMargin(note, YGEdgeTop, 26.0f);
	page.Compute();

	const ImVec2 ic = page.fl.Rect(icon).Center();
	dl->AddCircle(ic, 26.0f, ui::col::Link, 48, 3.0f);
	ui::DrawIcon(dl, ui::Icon::Download, ImVec2(ic.x, ic.y - 1.0f), 30.0f, ui::col::Link, 2.4f);
	page.DrawTexts(dl, ui::col::Text, kLeadInk);
	DrawFeatures(dl, page, features, nodes);
	const FlexRect n = page.fl.Rect(note);
	ui::DrawText(dl, ui::type::Footnote, n.min, n.W() + 1.0f, ui::col::Subtle, noteText.c_str(), ui::Align::Center);

	FlexRect on, later;
	on.min = ImVec2((kWindowW - 380.0f) * 0.5f, 634.0f);
	on.max = ImVec2(on.min.x + 380.0f, on.min.y + 54.0f);
	const float laterW = ui::PillWidth("Not now", 18.0f, ui::Icon::None, 20.0f);
	later.min = ImVec2((kWindowW - laterW) * 0.5f, on.max.y + 8.0f);
	later.max = ImVec2(later.min.x + laterW, later.min.y + 44.0f);
	if (ui::PillButton("##updateson", "Turn on automatic updates", ui::Btn::Primary, on, ui::Icon::None, 19.0f))
	{
		const bool previous = CalCtx.automaticUpdates;
		CalCtx.automaticUpdates = true;
		SaveSettingOrRestore(CalCtx.automaticUpdates, previous);
		if (CalCtx.automaticUpdates && !previous && !g_uiPreviewMode)
			questcal::update::AppUpdater.SetEnabled(true);
		Go(SetupStep::FirstCalibration);
	}
	if (ui::PillButton("##notnow", "Not now", ui::Btn::Link, later, ui::Icon::None, 18.0f))
		Go(SetupStep::FirstCalibration);
}

// Where the first calibration's two devices are picked: what it is, and a
// menu of the others.
void PickerCard(const char *id, const FlexRect &r, const char *labelEnglish, const VRDevice *device,
	const std::vector<const VRDevice *> &options, bool reference)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImGui::PushID(id);
	ImGui::SetCursorScreenPos(r.min);
	const bool pressed = ImGui::InvisibleButton("##card", r.Size(), ImGuiButtonFlags_EnableNav);
	ui::FillRounded(dl, r.min, r.max, ui::Rgba(255, 255, 255, ImGui::IsItemHovered() ? 0.10f : 0.07f), 22.0f);
	ui::InnerEdge(dl, r.min, r.max, ui::col::CardEdge, 22.0f);
	ui::FocusRing(dl, r.min, r.max, 22.0f);
	FlexRect art;
	art.min = ImVec2(r.min.x + 16.0f - 8.0f, r.Center().y - 38.0f);
	art.max = ImVec2(art.min.x + 76.0f, art.min.y + 76.0f);
	if (device)
		DrawDeviceArt(dl, *device, art, ui::col::Muted);
	const float textX = art.max.x - 6.0f + 14.0f, textW = r.max.x - 18.0f - 20.0f - 14.0f - textX;
	const ui::TextStyle caption{ ui::Weight::Regular, 15.0f, 20.0f };
	ui::DrawLine(dl, caption, ImVec2(textX, r.Center().y - 33.0f), ui::col::Muted, Tr(labelEnglish));
	if (device)
	{
		std::string sub = reference ? FriendlySystemName(device->trackingSystem) : device->serial;
		if (device->battery >= 0.0f)
			sub += ", " + Tr(FormatString("%d%% battery", static_cast<int>(std::lround(std::min(device->battery, 1.0f) * 100.0f))));
		ui::DrawLine(dl, kFeatureTitle, ImVec2(textX, r.Center().y - 12.0f), ui::col::Text,
			ui::Ellipsize(kFeatureTitle, DeviceDisplayName(*device), textW).c_str());
		ui::DrawLine(dl, caption, ImVec2(textX, r.Center().y + 13.0f), ui::col::Muted, ui::Ellipsize(caption, sub, textW).c_str());
	}
	else
		ui::DrawLine(dl, kFeatureTitle, ImVec2(textX, r.Center().y - 12.0f), ui::col::Muted,
			Tr(reference ? "No controller found" : "No tracker found"));
	ui::DrawIcon(dl, ui::Icon::ChevronUpDown, ImVec2(r.max.x - 18.0f - 10.0f, r.Center().y), 20.0f, ui::Rgba(236, 238, 244, 0.66f), 2.4f);
	if (pressed)
		ImGui::OpenPopup("##devices");
	if (ui::BeginMenuPopup("##devices", ImVec2(r.min.x, r.max.y + 8.0f), ImVec2(0.0f, 0.0f), r.W()))
	{
		for (const VRDevice *option : options)
		{
			ImGui::PushID(option->id);
			const std::string name = DeviceDisplayName(*option);
			if (ui::MenuItem("##device", option == device ? ui::Icon::Check : ui::Icon::None, name.c_str()))
			{
				const uint32_t picked = static_cast<uint32_t>(option->id);
				if (reference)
					CalCtx.referenceID = picked;
				else
				{
					CalCtx.targetID = picked;
					CalCtx.pendingTargetTrackingSystem = option->trackingSystem;
				}
			}
			ImGui::PopID();
		}
		ui::EndMenuPopup();
	}
	ImGui::PopID();
}

void FirstCalibrationStep(const VRState &state)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (BackButton(dl))
		Go(SetupStep::Updates);

	// The two to hold together: one of the headset's controllers, and a
	// tracker of a kind the player said they use.
	SettlePairSystems(state);
	std::vector<const VRDevice *> controllers, trackers;
	for (const auto &dev : state.devices)
	{
		if (!dev.connected)
			continue;
		if (dev.trackingSystem == CalCtx.pendingReferenceTrackingSystem && dev.deviceClass != vr::TrackedDeviceClass_HMD)
			controllers.push_back(&dev);
		else if (dev.trackingSystem != CalCtx.pendingReferenceTrackingSystem && dev.deviceClass != vr::TrackedDeviceClass_HMD)
			trackers.push_back(&dev);
	}
	if (!s_setup.pairSeeded)
	{
		if (!controllers.empty())
			CalCtx.referenceID = static_cast<uint32_t>(controllers.front()->id);
		// A tracker of a kind they chose, else any tracker, else whatever
		// SteamVR device there is.
		const VRDevice *pick = nullptr;
		for (const VRDevice *t : trackers)
		{
			const int kind = KindOf(*t);
			if (kind >= 0 && CalCtx.*kKinds[kind].uses)
			{
				pick = t;
				break;
			}
		}
		for (const VRDevice *t : trackers)
			if (!pick && t->deviceClass == vr::TrackedDeviceClass_GenericTracker)
				pick = t;
		if (!pick && !trackers.empty())
			pick = trackers.front();
		if (pick)
		{
			CalCtx.targetID = static_cast<uint32_t>(pick->id);
			CalCtx.pendingTargetTrackingSystem = pick->trackingSystem;
		}
		s_setup.pairSeeded = true;
	}
	const VRDevice *reference = nullptr, *target = nullptr;
	for (const VRDevice *d : controllers)
		if (static_cast<uint32_t>(d->id) == CalCtx.referenceID)
			reference = d;
	for (const VRDevice *d : trackers)
		if (static_cast<uint32_t>(d->id) == CalCtx.targetID)
			target = d;

	Page page(72.0f);
	page.Text(ui::type::SetupTitle, Tr("Your first calibration"), 0.0f, 1000.0f);
	page.Text(kLead, Tr(FormatString("Hold one of your Quest controllers against a tracker. You'll move them together for %.0f seconds.",
		CalCtx.CollectionSeconds())), 12.0f, 640.0f);
	YGNodeRef pickers = page.fl.Box(page.root, 760.0f, 96.0f);
	YGNodeStyleSetMargin(pickers, YGEdgeTop, 32.0f);
	YGNodeRef demo = page.fl.Box(page.root, 760.0f, 280.0f);
	YGNodeStyleSetMargin(demo, YGEdgeTop, 24.0f);
	page.Compute();
	page.DrawTexts(dl, ui::col::Text, kLeadInk);

	const FlexRect p = page.fl.Rect(pickers);
	const float cardW = (p.W() - 14.0f * 2.0f - 40.0f) * 0.5f;
	FlexRect left, right;
	left.min = p.min;
	left.max = ImVec2(p.min.x + cardW, p.max.y);
	right.min = ImVec2(p.max.x - cardW, p.min.y);
	right.max = p.max;
	PickerCard("##hold", left, "Hold this", reference, controllers, true);
	const ImVec2 plus(p.Center().x, p.Center().y);
	dl->AddCircleFilled(plus, 20.0f, ui::Rgba(255, 255, 255, 0.10f), 32);
	ui::DrawIcon(dl, ui::Icon::Plus, plus, 18.0f, ui::Rgba(236, 238, 244, 0.80f), 2.4f);
	PickerCard("##against", right, "Against this", target, trackers, false);

	// The motion it will take, as the calibration sheet shows it.
	const FlexRect d = page.fl.Rect(demo);
	ui::FillRounded(dl, d.min, d.max, ui::Rgba(0, 0, 0, 0.20f), 24.0f);
	BOOL animate = TRUE;
	SystemParametersInfoW(SPI_GETCLIENTAREAANIMATION, 0, &animate, 0);
	if (animate && !g_uiPreviewMode)
	{
		s_setup.demoTime += std::min(ImGui::GetIO().DeltaTime, 0.05f);
		ui::KeepAnimating();
	}
	else if (s_setup.demoTime < 3.3)
		s_setup.demoTime = 3.3;
	FlexRect box = d;
	box.min.y += 8.0f;
	box.max.y -= 8.0f;
	DrawGuideAnimation(dl, box, s_setup.demoTime, GuideDemo::Wrist, false);

	const float laterW = 200.0f, startW = 300.0f, gap = 12.0f;
	FlexRect later, start;
	later.min = ImVec2((kWindowW - laterW - gap - startW) * 0.5f, kWindowH - 36.0f - 8.0f - 26.0f - 54.0f);
	later.max = ImVec2(later.min.x + laterW, later.min.y + 54.0f);
	start.min = ImVec2(later.max.x + gap, later.min.y);
	start.max = ImVec2(start.min.x + startW, later.max.y);
	if (ui::PillButton("##later", "Do it later", ui::Btn::Secondary, later, ui::Icon::None, 19.0f))
	{
		ReleaseGuideTexture();
		FinishSetup();
	}
	const bool ready = reference && target;
	if (ui::PillButton("##start", "Start calibration", ready ? ui::Btn::Primary : ui::Btn::Waiting, start, ui::Icon::Play, 19.0f) && ready)
	{
		FinishSetup();
		// Straight into the countdown: this page already showed how to move.
		OpenGuide(false, false);
		s_guide.stage = GuideStage::Countdown;
		s_guide.countdownStart = ImGui::GetTime();
	}
}

} // namespace

bool SetupShowing()
{
	return !CalCtx.onboarded && !CalCtx.validProfile;
}

void BuildSetup(const VRState &state)
{
	// Saving or a language without its font can still go wrong here.
	BuildErrorBanner(ImVec2(340.0f, 16.0f), 520.0f);
	switch (g_setupStep)
	{
	case SetupStep::Welcome: WelcomeStep(); break;
	case SetupStep::Headset: HeadsetStep(state); break;
	case SetupStep::Trackers: TrackersStep(state); break;
	case SetupStep::Updates: UpdatesStep(); break;
	case SetupStep::FirstCalibration: FirstCalibrationStep(state); break;
	case SetupStep::Count: break;
	}
	StepDots(ImGui::GetWindowDrawList());
}
