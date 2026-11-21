// The Post Processing page: the pose filters the driver runs on a device's
// poses on their way to SteamVR (PostProcessing.h), one pane for each filter,
// and the sheet that picks the devices two of them act on. The page only sets
// what the driver is asked for; nothing here touches a pose.
#include "stdafx.h"
#include "UiInternal.h"
#include "DevicePictures.h"
#include "UiPostProcessingArt.h"

FilterPane g_filterPane = FilterPane::OneEuro;

namespace
{

namespace pp = questcal::postprocess;

const ui::TextStyle kNote{ ui::Weight::Regular, 16.0f, 22.0f };
const ui::TextStyle kQuietTitle{ ui::Weight::Regular, 26.0f, 32.0f };
const ImU32 kNoteFill = ui::Rgba(255, 255, 255, 0.05f);
const ImU32 kNoteEdge = ui::Rgba(255, 255, 255, 0.06f);
const ImU32 kCautionFill = ui::Rgba(255, 179, 64, 0.10f);
const ImU32 kCautionEdge = ui::Rgba(255, 179, 64, 0.28f);
const ImU32 kChosenFill = ui::Rgba(35, 112, 232, 0.14f);
const ImU32 kAvatar = ui::Rgba(58, 61, 68);
// The card's own colour over the page, for the ring that parts two pictures.
const ImU32 kAvatarRing = ui::Rgba(47, 50, 56);

// The sizes the layout is built from.
const float kListW = 308.0f, kPaneGap = 16.0f, kPanePadX = 24.0f, kOptionGap = 12.0f;

// The settings as they were when a slider was taken hold of.
pp::Settings s_beforeSlide;

// ---------------------------------------------------------------------------
// Devices
// ---------------------------------------------------------------------------

// A device the page can show, with what it is to the filters.
struct Listed
{
	const VRDevice *dev = nullptr;
	pp::Device device;
};

// The devices that are on, the headset first, then trackers, then controllers.
// The headset tracker and virtual trackers are never offered.
std::vector<Listed> ListDevices(const VRState &state)
{
	std::vector<Listed> listed;
	for (const VRDevice &dev : state.devices)
	{
		if (!dev.connected || dev.id < 0)
			continue;
		Listed entry;
		entry.dev = &dev;
		entry.device.id = static_cast<uint32_t>(dev.id);
		entry.device.serial = dev.serial;
		entry.device.kind = pp::KindOf(dev.deviceClass, questcal::IsVirtualTrackerSystem(dev.trackingSystem),
			CalCtx.continuousEnabled && dev.serial == CalCtx.continuousTrackerSerial);
		if (pp::TakesClassic(entry.device.kind) && pp::ValidSerial(dev.serial))
			listed.push_back(entry);
	}
	const auto rank = [](const Listed &entry) {
		return entry.device.kind == pp::DeviceKind::Headset ? 0 : entry.device.kind == pp::DeviceKind::Tracker ? 1 : 2;
	};
	std::stable_sort(listed.begin(), listed.end(), [&](const Listed &a, const Listed &b) { return rank(a) < rank(b); });
	return listed;
}

std::vector<pp::Device> DevicesOf(const std::vector<Listed> &listed)
{
	std::vector<pp::Device> devices;
	devices.reserve(listed.size());
	for (const Listed &entry : listed)
		devices.push_back(entry.device);
	return devices;
}

// A device's picture at an opacity, as DrawDeviceArt draws it at full.
void DeviceArt(ImDrawList *dl, const VRDevice &dev, const FlexRect &box, float alpha)
{
	const questcal::DevicePicture picture = questcal::PictureOfDevice(dev.deviceClass == vr::TrackedDeviceClass_HMD,
		dev.controllerRole == vr::TrackedControllerRole_LeftHand, dev.model, dev.renderModel);
	if (!picture.resource || !DrawPicture(dl, picture.resource, box, picture.mirrored, false, alpha))
		DrawDeviceArt(dl, dev, box, ui::Fade(ui::col::Muted, alpha));
}

// The name a row shows. The headset is "HMD" on this page.
std::string RowName(const Listed &entry)
{
	return entry.device.kind == pp::DeviceKind::Headset ? std::string(Tr("HMD")) : DeviceDisplayName(*entry.dev);
}

// ---------------------------------------------------------------------------
// Saving
// ---------------------------------------------------------------------------

// Changes the page's settings in place, rolled back when they cannot be
// saved, and has the driver take them at the next tick.
template <typename Change>
void EditSettings(Change change)
{
	const pp::Settings previous = CalCtx.postProcessing;
	change(CalCtx.postProcessing);
	if (CalCtx.postProcessing == previous)
		return;
	SaveSettingOrRestore(CalCtx.postProcessing, previous);
	CalCtx.timeLastScan = -1e9;
}

// ---------------------------------------------------------------------------
// Text with one stronger run
// ---------------------------------------------------------------------------

// A sentence whose run between two asterisks is drawn stronger: "*Paused.*
// The rest." The marks travel with the text, so each translation places its
// own.
struct Rich
{
	std::string text;                    // without the marks
	size_t markBegin = 0, markEnd = 0;   // the run, as byte offsets into text
};

Rich ParseRich(const std::string &marked)
{
	Rich rich;
	const size_t first = marked.find('*');
	const size_t second = first == std::string::npos ? first : marked.find('*', first + 1);
	if (second == std::string::npos)
	{
		rich.text = marked;
		return rich;
	}
	rich.text = marked.substr(0, first) + marked.substr(first + 1, second - first - 1) + marked.substr(second + 1);
	rich.markBegin = first;
	rich.markEnd = second - 1;
	return rich;
}

// box is the text's node, which the layout sized by wrapping the plain text.
void DrawRich(ImDrawList *dl, const ui::TextStyle &style, const FlexRect &box, ImU32 ink, const Rich &rich,
	ui::Weight markWeight, ImU32 markInk)
{
	const char *base = rich.text.c_str();
	// Wrapped as plain text; the stronger run is a little wider, which the
	// box's padding takes. A pixel more than the box keeps a line that just
	// fits from breaking, unless that pixel changes the number of lines the
	// layout made room for.
	ui::TextLines lines = ui::WrapText(style, base, box.W() + 1.0f);
	if (std::fabs(lines.height - box.H()) > 0.5f)
		lines = ui::WrapText(style, base, box.W());
	const ui::TextStyle markStyle{ markWeight, style.size, style.lineHeight };
	const ImVec2 topLeft = box.min;
	float y = topLeft.y;
	for (const auto &line : lines.lines)
	{
		const char *cuts[4] = { line.first, std::clamp(base + rich.markBegin, line.first, line.second),
			std::clamp(base + rich.markEnd, line.first, line.second), line.second };
		float x = topLeft.x;
		for (int part = 0; part < 3; ++part)
		{
			if (cuts[part] == cuts[part + 1])
				continue;
			const bool marked = part == 1;
			const ui::TextStyle &partStyle = marked ? markStyle : style;
			ui::DrawLine(dl, partStyle, ImVec2(x, y), marked ? markInk : ink, cuts[part], cuts[part + 1]);
			x += ui::MeasureLine(partStyle, cuts[part], cuts[part + 1]).x;
		}
		y += style.lineHeight;
	}
}

// The narrowest width that wraps the text into as few lines as maxWidth does,
// so its lines are of about one length, as the design balances a note's text.
float BalancedWidth(const ui::TextStyle &style, const char *text, float maxWidth)
{
	const size_t lines = ui::WrapText(style, text, maxWidth).lines.size();
	if (lines < 2)
		return maxWidth;
	float low = maxWidth / static_cast<float>(lines), high = maxWidth;
	while (high - low > 1.0f)
	{
		const float middle = (low + high) * 0.5f;
		if (ui::WrapText(style, text, middle).lines.size() <= lines)
			high = middle;
		else
			low = middle;
	}
	return high;
}

// ---------------------------------------------------------------------------
// The page's parts: each adds its nodes to the layout and a function that
// draws them once the layout is computed
// ---------------------------------------------------------------------------

struct Build
{
	FlexLayout fl;
	ImDrawList *dl = nullptr;
	std::vector<std::function<void()>> draw;
	// A list that may be made shorter and then scrolls inside its card, so a
	// long one does not make the page scroll; and the least it is cut to.
	YGNodeRef list = nullptr;
	float listLeast = 0.0f;
};

// A rounded note with its icon; returns the column its paragraphs go in.
YGNodeRef AddNote(Build &b, YGNodeRef parent, float marginTop, ui::Icon icon, ImU32 iconInk, ImU32 fill, ImU32 edge)
{
	YGNodeRef box = b.fl.Row(parent);
	YGNodeStyleSetMargin(box, YGEdgeTop, marginTop);
	YGNodeStyleSetPadding(box, YGEdgeVertical, 11.0f);
	YGNodeStyleSetPadding(box, YGEdgeHorizontal, 16.0f);
	YGNodeStyleSetGap(box, YGGutterColumn, 12.0f);
	YGNodeStyleSetAlignItems(box, YGAlignFlexStart);
	YGNodeRef iconNode = b.fl.Box(box, 20.0f, 22.0f);
	YGNodeRef column = b.fl.Column(box);
	YGNodeStyleSetFlexGrow(column, 1.0f);
	YGNodeStyleSetFlexShrink(column, 1.0f);
	YGNodeStyleSetFlexBasis(column, 0.0f);
	b.draw.push_back([&b, box, iconNode, icon, iconInk, fill, edge] {
		const FlexRect r = b.fl.Rect(box);
		ui::FillRounded(b.dl, r.min, r.max, fill, 14.0f);
		ui::InnerEdge(b.dl, r.min, r.max, edge, 14.0f);
		ui::DrawIcon(b.dl, icon, b.fl.Rect(iconNode).Center(), 21.0f, iconInk, 2.0f);
	});
	return column;
}

// One paragraph, already translated, with its stronger run if it marks one.
YGNodeRef AddParagraph(Build &b, YGNodeRef column, const std::string &marked, float marginTop,
	ui::Weight markWeight = ui::Weight::Bold, ImU32 markInk = ui::col::Text)
{
	const auto rich = std::make_shared<Rich>(ParseRich(marked));
	YGNodeRef node = ui::TextNode(b.fl, column, kNote, rich->text);
	YGNodeStyleSetMargin(node, YGEdgeTop, marginTop);
	b.draw.push_back([&b, node, rich, markWeight, markInk] {
		DrawRich(b.dl, kNote, b.fl.Rect(node), ui::col::Text, *rich, markWeight, markInk);
	});
	return node;
}

// The pane's title, with a quieter part after it ("(Recommended)") or a
// badge ("Local").
void AddPaneTitle(Build &b, YGNodeRef pane, const char *english, const char *quietEnglish, const char *badgeEnglish)
{
	YGNodeRef row = b.fl.Add(pane);
	YGNodeStyleSetHeight(row, 32.0f);
	b.draw.push_back([&b, row, english, quietEnglish, badgeEnglish] {
		const FlexRect r = b.fl.Rect(row);
		const char *title = Tr(english);
		ui::DrawLine(b.dl, ui::type::SheetHeading, r.min, ui::col::Text, title);
		float x = r.min.x + ui::MeasureLine(ui::type::SheetHeading, title).x;
		if (quietEnglish)
		{
			x += ui::MeasureLine(kQuietTitle, " ").x;
			ui::DrawLine(b.dl, kQuietTitle, ImVec2(x, r.min.y), ui::col::Muted,
				ui::Ellipsize(kQuietTitle, Tr(quietEnglish), r.max.x - x).c_str());
		}
		if (badgeEnglish)
		{
			const char *badge = Tr(badgeEnglish);
			const ui::TextStyle style{ ui::Weight::SemiBold, 14.0f, 24.0f };
			const ImVec2 a(x + 12.0f, r.min.y + 5.0f), c(a.x + ui::MeasureLine(style, badge).x + 20.0f, a.y + 24.0f);
			ui::FillRounded(b.dl, a, c, ui::Rgba(255, 179, 64, 0.16f), 12.0f);
			ui::InnerEdge(b.dl, a, c, ui::Rgba(255, 179, 64, 0.35f), 12.0f);
			ui::DrawLine(b.dl, style, ImVec2(a.x + 10.0f, a.y), ui::col::Caution, badge);
		}
	});
}

// One of a filter's choices: its small picture, its name and, on the one a
// new install starts with, "(default)" under it.
struct Option
{
	const char *english;
	bool isDefault;
	const float *heights;
	int count;
};

bool OptionCard(const char *id, const FlexRect &r, const Option &option, bool chosen)
{
	ImGui::SetCursorScreenPos(r.min);
	const bool pressed = ImGui::InvisibleButton(id, r.Size(), ImGuiButtonFlags_EnableNav);
	const bool hov = ImGui::IsItemHovered();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (chosen)
	{
		ui::FillRounded(dl, r.min, r.max, kChosenFill, 16.0f);
		ui::InnerEdge(dl, r.min, r.max, ui::col::Accent, 16.0f, 2.0f);
	}
	else
	{
		ui::FillRounded(dl, r.min, r.max, hov ? ui::Rgba(255, 255, 255, 0.08f) : kNoteFill, 16.0f);
		ui::InnerEdge(dl, r.min, r.max, kNoteEdge, 16.0f);
	}

	// The picture: the art's line, as it lies in its 96 x 68 box.
	const ImVec2 a(r.min.x + 14.0f, r.Center().y - 34.0f);
	ui::FillRounded(dl, a, ImVec2(a.x + 96.0f, a.y + 68.0f), ui::col::Well, 10.0f);
	std::vector<ImVec2> points;
	points.reserve(static_cast<size_t>(option.count));
	for (int i = 0; i < option.count; ++i)
		points.emplace_back(a.x + 4.0f + 88.0f * static_cast<float>(i) / static_cast<float>(option.count - 1),
			a.y + option.heights[i]);
	// An opaque ink: the line's segments lie over each other, and a see-through
	// one would be brighter wherever they do.
	const ImU32 ink = chosen ? ui::col::Link : ui::Rgba(150, 152, 158);
	dl->AddPolyline(points.data(), option.count, ink, 0, 2.0f);
	// Round joins, as the design strokes the line: without them each sharp
	// turn is a thin spike.
	for (const ImVec2 &point : points)
		dl->AddCircleFilled(point, 1.0f, ink, 6);

	const float x = a.x + 96.0f + 14.0f, room = r.max.x - 12.0f - x;
	const std::string name = ui::Ellipsize(ui::type::Item, Tr(option.english), room);
	if (option.isDefault)
	{
		ui::DrawLine(dl, ui::type::Item, ImVec2(x, r.Center().y - 24.0f), ui::col::Text, name.c_str());
		const ui::TextStyle quiet{ ui::Weight::Regular, 19.0f, 24.0f };
		ui::DrawLine(dl, quiet, ImVec2(x, r.Center().y), ui::col::Muted, ui::Ellipsize(quiet, Tr("(default)"), room).c_str());
	}
	else
		ui::DrawLine(dl, ui::type::Item, ImVec2(x, r.Center().y - 12.0f), ui::col::Text, name.c_str());
	ui::FocusRing(dl, r.min, r.max, 16.0f);
	return pressed;
}

// A filter's choices, two to a row. pick takes the index of a pressed one.
void AddOptions(Build &b, YGNodeRef pane, float paneInnerW, const std::vector<Option> &options, int chosen,
	const std::function<void(int)> &pick)
{
	YGNodeRef grid = b.fl.Row(pane);
	YGNodeStyleSetMargin(grid, YGEdgeTop, 18.0f);
	YGNodeStyleSetFlexWrap(grid, YGWrapWrap);
	YGNodeStyleSetGap(grid, YGGutterAll, kOptionGap);
	const float cardW = std::floor((paneInnerW - kOptionGap) * 0.5f);
	std::vector<YGNodeRef> cards;
	cards.reserve(options.size());
	for (size_t i = 0; i < options.size(); ++i)
		cards.push_back(b.fl.Box(grid, cardW, 100.0f));
	b.draw.push_back([&b, cards, options, chosen, pick] {
		for (size_t i = 0; i < options.size(); ++i)
		{
			ImGui::PushID(static_cast<int>(i));
			if (OptionCard("##option", b.fl.Rect(cards[i]), options[i], static_cast<int>(i) == chosen))
				pick(static_cast<int>(i));
			ImGui::PopID();
		}
	});
}

// "Devices": the pictures of the filtered devices, their names, and the
// button that opens the sheet to change them.
void AddDevicesRow(Build &b, YGNodeRef pane, float paneInnerW, const std::vector<Listed> &listed)
{
	std::vector<Listed> filtered;
	for (const Listed &entry : listed)
		if (pp::Filtered(CalCtx.postProcessing, entry.device))
			filtered.push_back(entry);

	const std::string heading = Tr("Devices");
	YGNodeRef head = ui::TextNode(b.fl, pane, ui::type::Strong, heading);
	YGNodeStyleSetMargin(head, YGEdgeTop, 22.0f);
	YGNodeRef row = b.fl.Row(pane);
	YGNodeStyleSetMargin(row, YGEdgeTop, 10.0f);
	YGNodeStyleSetPadding(row, YGEdgeVertical, 12.0f);
	YGNodeStyleSetPadding(row, YGEdgeHorizontal, 16.0f);
	YGNodeStyleSetGap(row, YGGutterColumn, 16.0f);
	YGNodeStyleSetAlignItems(row, YGAlignCenter);
	const int shown = static_cast<int>((std::min)(filtered.size(), size_t{ 4 }));
	const float stackW = shown > 0 ? 38.0f + 24.0f * static_cast<float>(shown - 1) : 0.0f;
	const float buttonW = ui::PillWidth("Change", 17.0f);

	// The names when each says which device it is and they fit in two lines;
	// a count when two devices share a name or the names run longer.
	const float textW = paneInnerW - 32.0f - (shown > 0 ? stackW + 16.0f : 0.0f) - 16.0f - buttonW;
	std::string summary;
	bool distinct = true;
	for (size_t i = 0; i < filtered.size(); ++i)
	{
		const std::string name = DeviceDisplayName(*filtered[i].dev);
		for (size_t k = 0; k < i; ++k)
			distinct = distinct && DeviceDisplayName(*filtered[k].dev) != name;
		summary += (i > 0 ? ", " : "") + name;
	}
	if (filtered.empty())
		summary = Tr("No devices");
	else if (filtered.size() > 1 &&
		(!distinct || ui::WrapText(ui::type::BodyMedium, summary.c_str(), textW).lines.size() > 2))
		summary = Tr(FormatString("%d devices", static_cast<int>(filtered.size())));

	YGNodeRef stack = shown > 0 ? b.fl.Box(row, stackW, 38.0f) : nullptr;
	YGNodeRef text = ui::TextNode(b.fl, row, ui::type::BodyMedium, summary);
	YGNodeStyleSetFlexGrow(text, 1.0f);
	YGNodeStyleSetFlexShrink(text, 1.0f);
	YGNodeStyleSetFlexBasis(text, 0.0f);
	YGNodeRef button = b.fl.Box(row, buttonW, 44.0f);

	b.draw.push_back([&b, head, heading, row, stack, text, button, filtered, shown, summary] {
		const FlexRect h = b.fl.Rect(head);
		ui::DrawLine(b.dl, ui::type::Strong, h.min, ui::col::Text, heading.c_str());
		const FlexRect r = b.fl.Rect(row);
		ui::FillRounded(b.dl, r.min, r.max, kNoteFill, 16.0f);
		ui::InnerEdge(b.dl, r.min, r.max, kNoteEdge, 16.0f);
		if (stack)
		{
			const FlexRect s = b.fl.Rect(stack);
			for (int i = 0; i < shown; ++i)
			{
				const ImVec2 c(s.min.x + 19.0f + 24.0f * static_cast<float>(i), s.Center().y);
				b.dl->AddCircleFilled(c, 22.0f, kAvatarRing, 40);
				b.dl->AddCircleFilled(c, 19.0f, kAvatar, 40);
				FlexRect art;
				art.min = ImVec2(c.x - 15.0f, c.y - 15.0f);
				art.max = ImVec2(c.x + 15.0f, c.y + 15.0f);
				DeviceArt(b.dl, *filtered[static_cast<size_t>(i)].dev, art, 1.0f);
			}
		}

		const FlexRect t = b.fl.Rect(text);
		ui::DrawText(b.dl, ui::type::BodyMedium, t.min, t.W() + 1.0f, filtered.empty() ? ui::col::Muted : ui::col::Text,
			summary.c_str());

		if (ui::PillButton("##changedevices", "Change", ui::Btn::Secondary, b.fl.Rect(button), ui::Icon::None, 17.0f))
			OpenSheet(Sheet::FilterDevices);
	});
}

// ---------------------------------------------------------------------------
// The panes
// ---------------------------------------------------------------------------

void OneEuroPane(Build &b, YGNodeRef pane, float paneInnerW, const std::vector<Listed> &listed)
{
	AddPaneTitle(b, pane, "One Euro", "(Recommended)", nullptr);
	YGNodeRef note = AddNote(b, pane, 14.0f, ui::Icon::Info, ui::col::Link, kNoteFill, kNoteEdge);
	AddParagraph(b, note, Tr("This filter decreases the shake of your trackers when you stand still. It adds almost no "
		"delay when you move."), 0.0f);

	using namespace postprocessart;
	const pp::Strength strength = CalCtx.postProcessing.oneEuro;
	AddOptions(b, pane, paneInnerW, {
		{ "Off", false, OneEuroOff, static_cast<int>(std::size(OneEuroOff)) },
		{ "Light", false, OneEuroLight, static_cast<int>(std::size(OneEuroLight)) },
		{ "Medium", true, OneEuroMedium, static_cast<int>(std::size(OneEuroMedium)) },
		{ "Strong", false, OneEuroStrong, static_cast<int>(std::size(OneEuroStrong)) } },
		static_cast<int>(strength),
		[](int picked) { EditSettings([picked](pp::Settings &s) { s.oneEuro = static_cast<pp::Strength>(picked); }); });
	if (strength != pp::Strength::Off)
		AddDevicesRow(b, pane, paneInnerW, listed);
}

void SoftenJumpsPane(Build &b, YGNodeRef pane, float paneInnerW, const std::vector<Listed> &listed)
{
	AddPaneTitle(b, pane, "Soften jumps", nullptr, nullptr);
	YGNodeRef note = AddNote(b, pane, 14.0f, ui::Icon::Info, ui::col::Link, kNoteFill, kNoteEdge);
	AddParagraph(b, note, Tr("SteamVR corrects the position of a tracker many times each second. You see a large "
		"correction as a jump."), 0.0f);
	AddParagraph(b, note, Tr("With this filter, the tracker moves to its new position in a tenth of a second. During "
		"this time, the tracker is not at its measured position."), 8.0f);
	AddParagraph(b, note, Tr("The filter adds no delay to a movement, but it can make mistakes. Then a fast movement "
		"looks soft for a moment."), 8.0f);

	using namespace postprocessart;
	const bool on = CalCtx.postProcessing.softenJumps;
	AddOptions(b, pane, paneInnerW, {
		{ "Off", false, JumpsOff, static_cast<int>(std::size(JumpsOff)) },
		{ "On", true, JumpsOn, static_cast<int>(std::size(JumpsOn)) } },
		on ? 1 : 0,
		[](int picked) { EditSettings([picked](pp::Settings &s) { s.softenJumps = picked == 1; }); });
	if (on)
		AddDevicesRow(b, pane, paneInnerW, listed);
}

// One device's "Classic" value: its picture, its name, a slider in steps of
// five and the value. The values stay as they are set while "Classic" is
// paused.
void ClassicRow(const Listed &entry, const FlexRect &r)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const ImVec2 c(r.min.x + 16.0f, r.Center().y);
	dl->AddCircleFilled(c, 16.0f, kAvatar, 32);
	FlexRect art;
	art.min = ImVec2(c.x - 13.0f, c.y - 13.0f);
	art.max = ImVec2(c.x + 13.0f, c.y + 13.0f);
	DeviceArt(dl, *entry.dev, art, 1.0f);

	const float nameX = r.min.x + 32.0f + 14.0f, nameW = 136.0f;
	ui::DrawLine(dl, ui::type::BodyMedium, ImVec2(nameX, r.Center().y - 11.0f), ui::col::Text,
		ui::Ellipsize(ui::type::BodyMedium, RowName(entry), nameW).c_str());

	FlexRect track;
	track.min = ImVec2(nameX + nameW + 14.0f, r.Center().y - 14.0f);
	track.max = ImVec2(r.max.x - 56.0f - 14.0f, r.Center().y + 14.0f);
	const int percent = pp::ClassicPercent(CalCtx.postProcessing, entry.device);
	int step = percent / pp::ClassicStepPercent;
	const int steps = 100 / pp::ClassicStepPercent;

	// ImGui's slider under the page's own drawing: it brings the drag, the
	// click on the track and the keyboard.
	ImGui::SetCursorScreenPos(track.min);
	ImGui::SetNextItemWidth(track.W());
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, (std::max)(0.0f, (track.H() - ImGui::GetFontSize()) * 0.5f)));
	ImGui::PushStyleVar(ImGuiStyleVar_GrabMinSize, 24.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
	const ImVec4 clear(0.0f, 0.0f, 0.0f, 0.0f);
	for (ImGuiCol colour : { ImGuiCol_FrameBg, ImGuiCol_FrameBgHovered, ImGuiCol_FrameBgActive, ImGuiCol_SliderGrab,
		ImGuiCol_SliderGrabActive, ImGuiCol_NavHighlight })
		ImGui::PushStyleColor(colour, clear);
	const bool changed = ImGui::SliderInt("##cut", &step, 0, steps, "", ImGuiSliderFlags_NoInput | ImGuiSliderFlags_AlwaysClamp);
	ImGui::PopStyleColor(6);
	ImGui::PopStyleVar(3);
	if (ImGui::IsItemActivated())
		s_beforeSlide = CalCtx.postProcessing;
	if (changed && pp::SetClassicPercent(CalCtx.postProcessing, entry.device.serial, step * pp::ClassicStepPercent))
		CalCtx.timeLastScan = -1e9;
	// Saved once, when the slider is let go.
	if (ImGui::IsItemDeactivatedAfterEdit() && CalCtx.postProcessing != s_beforeSlide)
	{
		SaveSettingOrRestore(CalCtx.postProcessing, s_beforeSlide);
		CalCtx.timeLastScan = -1e9;
	}

	const int value = pp::ClassicPercent(CalCtx.postProcessing, entry.device);
	const float fraction = static_cast<float>(value) / 100.0f;
	const float thumbX = track.min.x + 14.0f + (track.W() - 28.0f) * fraction, cy = r.Center().y;
	dl->AddRectFilled(ImVec2(track.min.x, cy - 3.0f), ImVec2(track.max.x, cy + 3.0f), ui::Rgba(255, 255, 255, 0.14f), 3.0f);
	if (value > 0)
		dl->AddRectFilled(ImVec2(track.min.x, cy - 3.0f), ImVec2(thumbX, cy + 3.0f), ui::col::Accent, 3.0f);
	dl->AddCircleFilled(ImVec2(thumbX, cy + 1.0f), 12.5f, ui::Rgba(0, 0, 0, 0.30f), 32);
	dl->AddCircleFilled(ImVec2(thumbX, cy), 12.0f, ui::col::White, 32);
	ui::FocusRing(dl, track.min, track.max, 8.0f);

	const std::string figure = FormatString("%d%%", value);
	const ui::TextStyle figureStyle{ ui::Weight::Regular, 16.0f, 21.0f };
	ui::DrawLine(dl, figureStyle, ImVec2(r.max.x - ui::MeasureLine(figureStyle, figure.c_str()).x, cy - 10.5f),
		value > 0 ? ui::col::Text : ui::col::Muted, figure.c_str());
}

void ClassicPane(Build &b, YGNodeRef pane, const std::vector<Listed> &listed, bool paused)
{
	AddPaneTitle(b, pane, "Classic", nullptr, "Local");
	if (paused)
	{
		// The add-on's own name, as it calls itself.
		YGNodeRef alert = AddNote(b, pane, 14.0f, ui::Icon::Warn, ui::col::Caution, kNoteFill, kNoteEdge);
		AddParagraph(b, alert, Tr("*Paused.* \"OpenVR-SmoothTracking\" is running."), 0.0f, ui::Weight::SemiBold,
			ui::col::Caution);
	}
	else
	{
		YGNodeRef note = AddNote(b, pane, 14.0f, ui::Icon::Info, ui::col::Link, kNoteFill, kNoteEdge);
		AddParagraph(b, note, Tr("This filter makes tracking very fluid, but it adds more delay. It can cause motion "
			"sickness if you use it on the HMD."), 0.0f);
	}
	YGNodeRef local = AddNote(b, pane, 10.0f, ui::Icon::Info, ui::col::Caution, kCautionFill, kCautionEdge);
	AddParagraph(b, local, Tr("This filter decreases SteamVR's *local* motion prediction. Other players do not see any "
		"difference."), 0.0f);
	AddParagraph(b, local, Tr("\"OpenVR-SmoothTracking\" operates in the same way."), 8.0f);

	if (listed.empty())
		return;
	YGNodeRef card = b.fl.Column(pane);
	YGNodeStyleSetMargin(card, YGEdgeTop, 12.0f);
	YGNodeStyleSetPadding(card, YGEdgeVertical, 6.0f);
	YGNodeStyleSetPadding(card, YGEdgeHorizontal, 16.0f);
	YGNodeStyleSetGap(card, YGGutterRow, 2.0f);
	const float rowH = 34.0f, rowGap = 2.0f, padY = 6.0f;
	std::vector<YGNodeRef> rows;
	for (size_t i = 0; i < listed.size(); ++i)
	{
		YGNodeRef row = b.fl.Add(card);
		YGNodeStyleSetHeight(row, rowH);
		YGNodeStyleSetFlexShrink(row, 0.0f);
		rows.push_back(row);
	}
	// Every row when the page has the room; never fewer than three in view.
	const float rowsH = rowH * static_cast<float>(listed.size()) + rowGap * static_cast<float>(listed.size() - 1);
	b.list = card;
	b.listLeast = padY * 2.0f + rowH * 3.0f + rowGap * 2.0f;
	b.draw.push_back([&b, card, rows, listed, rowH, rowGap, padY, rowsH] {
		const FlexRect r = b.fl.Rect(card);
		ui::FillRounded(b.dl, r.min, r.max, kNoteFill, 16.0f);
		ui::InnerEdge(b.dl, r.min, r.max, kNoteEdge, 16.0f);
		if (r.H() >= padY * 2.0f + rowsH - 0.5f)
		{
			for (size_t i = 0; i < listed.size(); ++i)
			{
				ImGui::PushID(listed[i].dev->id);
				ClassicRow(listed[i], b.fl.Rect(rows[i]));
				ImGui::PopID();
			}
			return;
		}
		// The page cut the card (BuildPostProcessingPage): the rows scroll in it.
		ImGui::SetCursorScreenPos(ImVec2(r.min.x, r.min.y + padY));
		ImGui::BeginChild("##classicdevices", ImVec2(r.W(), r.H() - padY * 2.0f), ImGuiChildFlags_NavFlattened,
			ImGuiWindowFlags_NoBackground);
		const ImVec2 base = ImGui::GetCursorScreenPos();
		const float inner = ImGui::GetContentRegionAvail().x;
		for (size_t i = 0; i < listed.size(); ++i)
		{
			FlexRect row;
			row.min = ImVec2(base.x + 16.0f, base.y + (rowH + rowGap) * static_cast<float>(i));
			row.max = ImVec2(base.x + inner - 8.0f, row.min.y + rowH);
			ImGui::PushID(listed[i].dev->id);
			ClassicRow(listed[i], row);
			ImGui::PopID();
		}
		ImGui::SetCursorScreenPos(base);
		ImGui::Dummy(ImVec2(1.0f, rowsH));
		ImGui::EndChild();
	});
}

// One filter in the list at the left: its icon, its name and what it is set
// to.
bool FilterRow(const char *id, const FlexRect &r, ui::Icon icon, const char *english, bool recommended,
	const char *status, bool caution, bool chosen)
{
	ImGui::SetCursorScreenPos(r.min);
	const bool pressed = ImGui::InvisibleButton(id, r.Size(), ImGuiButtonFlags_EnableNav);
	const bool hov = ImGui::IsItemHovered();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (chosen)
		ui::FillRounded(dl, r.min, r.max, ui::col::Secondary, 14.0f);
	else if (hov)
		ui::FillRounded(dl, r.min, r.max, ui::Rgba(255, 255, 255, 0.06f), 14.0f);
	const ImVec2 badge(r.min.x + 12.0f, r.Center().y - 19.0f);
	ui::FillRounded(dl, badge, ImVec2(badge.x + 38.0f, badge.y + 38.0f), ui::col::Fill, 11.0f);
	ui::DrawIcon(dl, icon, ImVec2(badge.x + 19.0f, badge.y + 19.0f), 26.0f, ui::col::Text, 1.8f);

	const float x = badge.x + 38.0f + 12.0f, end = r.max.x - 12.0f;
	const char *name = Tr(english);
	ui::DrawLine(dl, ui::type::BodyMedium, ImVec2(x, r.Center().y - 20.0f), ui::col::Text,
		ui::Ellipsize(ui::type::BodyMedium, name, end - x).c_str());
	if (recommended)
	{
		const ui::TextStyle quiet{ ui::Weight::Regular, 17.0f, 22.0f };
		const float at = x + ui::MeasureLine(ui::type::BodyMedium, name).x + ui::MeasureLine(quiet, " ").x;
		if (at < end)
			ui::DrawLine(dl, quiet, ImVec2(at, r.Center().y - 20.0f), ui::col::Muted,
				ui::Ellipsize(quiet, Tr("(Recommended)"), end - at).c_str());
	}
	const ui::TextStyle statusStyle{ caution ? ui::Weight::SemiBold : ui::Weight::Regular, 14.0f, 18.0f };
	ui::DrawLine(dl, statusStyle, ImVec2(x, r.Center().y + 2.0f), caution ? ui::col::Caution : ui::col::Muted,
		ui::Ellipsize(statusStyle, Tr(status), end - x).c_str());
	ui::FocusRing(dl, r.min, r.max, 14.0f);
	return pressed;
}

} // namespace

float BuildPostProcessingPage(const VRState &state, ImVec2 origin, float width)
{
	Build b;
	b.dl = ImGui::GetWindowDrawList();
	const std::vector<Listed> listed = ListDevices(state);
	const pp::Settings &settings = CalCtx.postProcessing;
	// OpenVR-SmoothTracking takes the prediction down and the driver cannot get
	// behind it: "Classic" waits.
	const bool paused = CalCtx.classicPaused;
	const bool classicOn = pp::ClassicOn(settings, DevicesOf(listed));

	YGNodeRef root = b.fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, width);
	YGNodeStyleSetPadding(root, YGEdgeHorizontal, 40.0f);

	// The title, and the switch that turns every filter off at once and keeps
	// what each is set to.
	const std::string titleText = Tr("Post Processing");
	YGNodeRef head = b.fl.Row(root);
	YGNodeStyleSetAlignItems(head, YGAlignCenter);
	YGNodeStyleSetGap(head, YGGutterColumn, 24.0f);
	YGNodeRef title = ui::TextNode(b.fl, head, ui::type::PageTitle, titleText);
	YGNodeStyleSetFlexGrow(title, 1.0f);
	YGNodeRef master = b.fl.Box(head, ui::kSwitchW, ui::kSwitchH);

	// What the switch means for other add-ons that change poses: said while the
	// driver can keep it (it gives a filtered device its own driver's motion
	// back), and with the add-on's name when one is running.
	const pp::OtherAddOns others = CalCtx.otherAddOns;
	std::string promise;
	ui::TextLines promiseLines;
	YGNodeRef promiseBox = nullptr, promiseIcon = nullptr, promiseText = nullptr;
	if (others.noEffect)
	{
		promise = others.smoothTracking
			? Tr("While this switch is on, other post-processing add-ons, such as \"OpenVR-SmoothTracking\", have no "
				"effect on the devices that Nova Calibrator filters.")
			: Tr("While this switch is on, other post-processing add-ons have no effect on the devices that Nova "
				"Calibrator filters.");
		const float room = width - 80.0f - 32.0f - 20.0f - 12.0f;
		promiseLines = ui::WrapText(kNote, promise.c_str(), BalancedWidth(kNote, promise.c_str(), room));
		promiseBox = b.fl.Row(root);
		YGNodeStyleSetMargin(promiseBox, YGEdgeTop, 14.0f);
		YGNodeStyleSetPadding(promiseBox, YGEdgeVertical, 12.0f);
		YGNodeStyleSetPadding(promiseBox, YGEdgeHorizontal, 16.0f);
		YGNodeStyleSetGap(promiseBox, YGGutterColumn, 12.0f);
		YGNodeStyleSetAlignItems(promiseBox, YGAlignCenter);
		promiseIcon = b.fl.Box(promiseBox, 20.0f, 20.0f);
		promiseText = b.fl.Box(promiseBox, room, promiseLines.height);
	}

	YGNodeRef body = b.fl.Row(root);
	YGNodeStyleSetMargin(body, YGEdgeTop, 24.0f);
	YGNodeStyleSetGap(body, YGGutterColumn, kPaneGap);
	YGNodeStyleSetAlignItems(body, YGAlignFlexStart);
	YGNodeRef list = b.fl.Column(body);
	YGNodeStyleSetWidth(list, kListW);
	YGNodeStyleSetFlexShrink(list, 0.0f);
	YGNodeStyleSetPadding(list, YGEdgeAll, 8.0f);
	YGNodeStyleSetGap(list, YGGutterRow, 2.0f);
	YGNodeRef rows[3];
	for (YGNodeRef &row : rows)
	{
		row = b.fl.Add(list);
		YGNodeStyleSetHeight(row, 68.0f);
	}
	YGNodeRef pane = b.fl.Column(body);
	YGNodeStyleSetFlexGrow(pane, 1.0f);
	YGNodeStyleSetFlexShrink(pane, 1.0f);
	YGNodeStyleSetFlexBasis(pane, 0.0f);
	YGNodeStyleSetPadding(pane, YGEdgeTop, 22.0f);
	YGNodeStyleSetPadding(pane, YGEdgeHorizontal, kPanePadX);
	YGNodeStyleSetPadding(pane, YGEdgeBottom, 24.0f);
	const float paneInnerW = width - 80.0f - kListW - kPaneGap - kPanePadX * 2.0f;

	b.draw.push_back([&b, list, pane, rows, &settings, paused, classicOn] {
		const FlexRect l = b.fl.Rect(list), p = b.fl.Rect(pane);
		ui::Card(b.dl, l.min, l.max, 18.0f);
		ui::Card(b.dl, p.min, p.max, 18.0f);
		static const char *const strengths[] = { "Off", "Light", "Medium", "Strong" };
		if (FilterRow("##filteroneeuro", b.fl.Rect(rows[0]), ui::Icon::FilterOneEuro, "One Euro", true,
			strengths[static_cast<int>(settings.oneEuro)], false, g_filterPane == FilterPane::OneEuro))
			g_filterPane = FilterPane::OneEuro;
		const bool waits = paused && classicOn;
		if (FilterRow("##filterclassic", b.fl.Rect(rows[1]), ui::Icon::FilterClassic, "Classic", false,
			waits ? "Paused" : classicOn ? "On" : "Off", waits, g_filterPane == FilterPane::Classic))
			g_filterPane = FilterPane::Classic;
		if (FilterRow("##filterjumps", b.fl.Rect(rows[2]), ui::Icon::FilterJumps, "Soften jumps", false,
			settings.softenJumps ? "On" : "Off", false, g_filterPane == FilterPane::SoftenJumps))
			g_filterPane = FilterPane::SoftenJumps;
	});
	switch (g_filterPane)
	{
	case FilterPane::OneEuro: OneEuroPane(b, pane, paneInnerW, listed); break;
	case FilterPane::Classic: ClassicPane(b, pane, listed, paused); break;
	case FilterPane::SoftenJumps: SoftenJumpsPane(b, pane, paneInnerW, listed); break;
	}
	b.fl.Compute(origin, width, YGUndefined);
	if (b.list)
	{
		// What the window has under the page's top, less the space the page
		// keeps free at its end (BuildPage). A list too long for it is cut to
		// what is there, so the page stays in the window and the list scrolls.
		const float room = ImGui::GetWindowPos().y + ImGui::GetWindowHeight() - 30.0f - (origin.y + ImGui::GetScrollY());
		float over = YGNodeLayoutGetHeight(root) - room;
		if (over > 0.0f)
		{
			// The pane first gives up half of the space under the list: a list
			// that is only a little too long then stays whole.
			const float given = (std::min)(std::ceil(over), 12.0f);
			YGNodeStyleSetPadding(pane, YGEdgeBottom, 24.0f - given);
			over -= given;
			if (over > 0.0f)
				YGNodeStyleSetHeight(b.list, (std::max)(b.listLeast, std::floor(YGNodeLayoutGetHeight(b.list) - over)));
			b.fl.Compute(origin, width, YGUndefined);
		}
	}

	// ---- Drawing ----
	const FlexRect t = b.fl.Rect(title);
	ui::DrawText(b.dl, ui::type::PageTitle, t.min, t.W() + 1.0f, ui::col::Text, titleText.c_str());
	bool enabled = settings.enabled;
	if (ui::ToggleSwitch("##postprocessing", enabled, b.fl.Rect(master).min))
		EditSettings([enabled](pp::Settings &s) { s.enabled = enabled; });
	if (promiseBox)
	{
		const FlexRect r = b.fl.Rect(promiseBox);
		ui::FillRounded(b.dl, r.min, r.max, kCautionFill, 14.0f);
		ui::InnerEdge(b.dl, r.min, r.max, kCautionEdge, 14.0f);
		ui::DrawIcon(b.dl, ui::Icon::Info, b.fl.Rect(promiseIcon).Center(), 21.0f, ui::col::Caution, 2.0f);
		const FlexRect words = b.fl.Rect(promiseText);
		ui::DrawLines(b.dl, kNote, promiseLines, words.min, words.W(), ui::col::Text);
	}

	// With the switch off the filters keep their settings and take no input.
	const bool live = CalCtx.postProcessing.enabled;
	ImGui::BeginDisabled(!live);
	for (const auto &draw : b.draw)
		draw();
	ImGui::EndDisabled();
	if (!live)
	{
		// The page's own backdrop (ui::PageBackdrop's two bands) over each card,
		// as the design's 40% opacity.
		const float screenH = ImGui::GetIO().DisplaySize.y;
		const auto backdrop = [&](float y) {
			const float at = std::clamp(y / screenH, 0.0f, 1.0f);
			const ImU32 middle = ui::Rgba(36, 38, 43);
			return at < 0.55f ? ui::Mix(ui::col::BackdropTop, middle, at / 0.55f)
				: ui::Mix(middle, ui::Rgba(29, 31, 35), (at - 0.55f) / 0.45f);
		};
		for (YGNodeRef card : { list, pane })
		{
			// The shading keeps the alpha of the shape it colours.
			const FlexRect r = b.fl.Rect(card);
			const int first = b.dl->VtxBuffer.Size;
			b.dl->AddRectFilled(r.min, r.max, ui::Rgba(255, 255, 255, 0.6f), 18.0f);
			ImGui::ShadeVertsLinearColorGradientKeepAlpha(b.dl, first, b.dl->VtxBuffer.Size, r.min, ImVec2(r.min.x, r.max.y),
				backdrop(r.min.y), backdrop(r.max.y));
		}
	}
	return YGNodeLayoutGetHeight(root);
}

// ---------------------------------------------------------------------------
// The sheet: the devices "One Euro" and "Soften jumps" act on
// ---------------------------------------------------------------------------

namespace
{

bool FilterDeviceTile(const Listed &entry, const FlexRect &r, bool chosen)
{
	ImGui::SetCursorScreenPos(r.min);
	const bool pressed = ImGui::InvisibleButton("##tile", r.Size(), ImGuiButtonFlags_EnableNav);
	const bool hov = ImGui::IsItemHovered();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (chosen)
	{
		ui::FillRounded(dl, r.min, r.max, kChosenFill, 16.0f);
		ui::InnerEdge(dl, r.min, r.max, ui::col::Accent, 16.0f, 2.0f);
	}
	else
	{
		ui::FillRounded(dl, r.min, r.max, hov ? ui::Rgba(255, 255, 255, 0.09f) : ui::col::Card, 16.0f);
		ui::InnerEdge(dl, r.min, r.max, kNoteEdge, 16.0f);
	}
	FlexRect art;
	art.min = ImVec2(r.Center().x - 32.0f, r.min.y + 20.0f);
	art.max = ImVec2(art.min.x + 64.0f, art.min.y + 64.0f);
	DeviceArt(dl, *entry.dev, art, chosen ? 1.0f : 0.5f);
	const std::string name = ui::Ellipsize(ui::type::Strong, DeviceDisplayName(*entry.dev), r.W() - 20.0f);
	ui::DrawLine(dl, ui::type::Strong, ImVec2(r.Center().x - ui::MeasureLine(ui::type::Strong, name.c_str()).x * 0.5f,
		art.max.y + 8.0f), chosen ? ui::col::Text : ui::col::Muted, name.c_str());
	if (chosen)
	{
		const ImVec2 c(r.max.x - 10.0f - 12.0f, r.min.y + 10.0f + 12.0f);
		dl->AddCircleFilled(c, 12.0f, ui::col::Accent, 24);
		ui::DrawIcon(dl, ui::Icon::Check, c, 14.0f, ui::col::White, 3.0f);
	}
	else
		dl->AddCircle(ImVec2(r.max.x - 12.0f - 10.0f, r.min.y + 12.0f + 10.0f), 9.0f, ui::Rgba(236, 238, 244, 0.35f), 24, 2.0f);
	ui::FocusRing(dl, r.min, r.max, 16.0f);
	return pressed;
}

} // namespace

void BuildFilterDevicesSheet(const VRState &state)
{
	std::vector<Listed> listed;
	for (const Listed &entry : ListDevices(state))
		if (pp::TakesFilters(entry.device.kind))
			listed.push_back(entry);

	// Three tiles to a row; past four rows the tiles scroll.
	const int columns = 3;
	const float gap = 12.0f, tileH = 136.0f, contentW = 564.0f;
	const int rows = (std::max)(1, (static_cast<int>(listed.size()) + columns - 1) / columns);
	const int shownRows = (std::min)(rows, 4);
	const float gridH = tileH * static_cast<float>(shownRows) + gap * static_cast<float>(shownRows - 1);
	const float titleH = 32.0f, footerH = 44.0f;
	const FlexRect c = SheetPanel(contentW + 64.0f, 28.0f + titleH + 20.0f + gridH + 22.0f + footerH + 32.0f);
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ui::DrawLine(dl, ui::type::SheetHeading, c.min, ui::col::Text, Tr("Devices to filter"));

	const ImVec2 gridMin(c.min.x, c.min.y + titleH + 20.0f);
	if (listed.empty())
		ui::DrawText(dl, ui::type::Body, ImVec2(gridMin.x, gridMin.y + gridH * 0.5f - 12.0f), c.W(), ui::col::Muted,
			Tr("No devices connected"), ui::Align::Center);
	else
	{
		ImGui::SetCursorScreenPos(gridMin);
		// A little taller than the tiles, so the focus ring is not cut.
		ImGui::BeginChild("##filterdevices", ImVec2(c.W() + 8.0f, gridH + 8.0f), ImGuiChildFlags_NavFlattened,
			ImGuiWindowFlags_NoBackground);
		const ImVec2 origin = ImGui::GetCursorScreenPos();
		const float tileW = std::floor((c.W() - gap * static_cast<float>(columns - 1)) / static_cast<float>(columns));
		for (size_t i = 0; i < listed.size(); ++i)
		{
			const int column = static_cast<int>(i) % columns, row = static_cast<int>(i) / columns;
			FlexRect r;
			r.min = ImVec2(origin.x + (tileW + gap) * static_cast<float>(column), origin.y + (tileH + gap) * static_cast<float>(row));
			r.max = ImVec2(r.min.x + tileW, r.min.y + tileH);
			const bool chosen = pp::Filtered(CalCtx.postProcessing, listed[i].device);
			ImGui::PushID(listed[i].dev->id);
			if (FilterDeviceTile(listed[i], r, chosen))
			{
				const std::string serial = listed[i].device.serial;
				EditSettings([&serial, chosen](pp::Settings &s) { pp::SetFiltered(s, serial, !chosen); });
			}
			ImGui::PopID();
		}
		ImGui::SetCursorScreenPos(origin);
		ImGui::Dummy(ImVec2(c.W(), tileH * static_cast<float>(rows) + gap * static_cast<float>(rows - 1)));
		ImGui::EndChild();
	}

	// What the player might worry about, said once, beside the way out.
	FlexRect done;
	const float doneW = ui::PillWidth("Done", 17.0f);
	done.min = ImVec2(c.max.x - doneW, c.max.y - footerH);
	done.max = c.max;
	const ui::TextLines promise = ui::WrapText(ui::type::Footnote,
		Tr("This filter never changes the headset or the headset tracker."), done.min.x - 20.0f - c.min.x);
	ui::DrawLines(dl, ui::type::Footnote, promise, ImVec2(c.min.x, done.Center().y - promise.height * 0.5f),
		done.min.x - 20.0f - c.min.x, ui::col::Muted);
	if (ui::PillButton("##donefilterdevices", "Done", ui::Btn::Primary, done, ui::Icon::None, 17.0f) ||
		(EscapePressed() && !DialogOpen()))
		CloseSheet();
}
