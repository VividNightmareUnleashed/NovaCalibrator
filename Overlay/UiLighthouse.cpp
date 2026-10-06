// The Lighthouse page: each base station, who sees it, and what every
// lighthouse device has in view, from SteamVR's own log
// (LighthouseVisibility.h). Nothing here is measured by the app; it is the
// driver's account of its own tracking, laid out per station.
#include "stdafx.h"
#include "UiInternal.h"

namespace
{

// Stations in channel order: the cards and the table's columns keep their
// order while the counts change, and Stations() sorts by drops.
std::vector<LighthouseVisibility::Station> StationsByChannel()
{
	auto stations = CalCtx.lighthouse.Stations();
	std::sort(stations.begin(), stations.end(),
		[](const LighthouseVisibility::Station &a, const LighthouseVisibility::Station &b) {
			return a.channel < b.channel;
		});
	return stations;
}

// Only for a device with visibleKnown set.
bool Sees(const LighthouseVisibility::Device &seen, int channel)
{
	return std::find(seen.visible.begin(), seen.visible.end(), channel) != seen.visible.end();
}

// A station's drops in words.
std::string DropWords(unsigned drops)
{
	return drops == 0 ? std::string("No dropouts") : drops == 1 ? std::string("Dropped out once")
		: drops == 2 ? std::string("Dropped out twice") : FormatString("Dropped out %u times", drops);
}

const ui::TextStyle kLead{ ui::Weight::Regular, 17.0f, 24.0f };
const ui::TextStyle kColumnHead{ ui::Weight::SemiBold, 13.0f, 18.0f };
const ui::TextStyle kRowName{ ui::Weight::SemiBold, 17.0f, 22.0f };
const ui::TextStyle kRowSub{ ui::Weight::Regular, 14.0f, 18.0f };
const ImU32 kQuiet = ui::Rgba(236, 238, 244, 0.56f);

YGNodeRef Heading(FlexLayout &fl, YGNodeRef root, const std::string &text)
{
	YGNodeRef node = ui::TextNode(fl, root, ui::type::Section, text);
	YGNodeStyleSetMargin(node, YGEdgeTop, 20.0f + 3.0f);
	YGNodeStyleSetMargin(node, YGEdgeBottom, 3.0f);
	return node;
}

} // namespace

float BuildLighthousePage(const VRState &state, ImVec2 origin, float width)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();

	// The devices this page is about, the switched-on ones first.
	std::vector<const VRDevice *> devices;
	for (const auto &dev : state.devices)
		if (dev.trackingSystem == "lighthouse")
			devices.push_back(&dev);
	std::stable_sort(devices.begin(), devices.end(),
		[](const VRDevice *a, const VRDevice *b) { return a->connected && !b->connected; });

	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, width);
	YGNodeStyleSetPadding(root, YGEdgeHorizontal, 40.0f);
	const std::string titleText = Tr("Lighthouse");
	const std::string leadText = Tr("Which base stations each device can see, live from SteamVR's log.");
	YGNodeRef title = ui::TextNode(fl, root, ui::type::PageTitle, titleText);
	YGNodeRef lead = ui::TextNode(fl, root, kLead, leadText);
	YGNodeStyleSetMargin(lead, YGEdgeTop, 6.0f);
	YGNodeStyleSetMaxWidth(lead, 760.0f);
	auto drawHead = [&] {
		const FlexRect t = fl.Rect(title), l = fl.Rect(lead);
		ui::DrawText(dl, ui::type::PageTitle, t.min, t.W() + 1.0f, ui::col::Text, titleText.c_str());
		ui::DrawText(dl, kLead, l.min, l.W() + 1.0f, ui::col::Muted, leadText.c_str());
	};

	// Nothing to lay out yet: what is missing, and what brings the page to
	// life.
	std::string emptyTitle, emptyBody;
	ui::Icon emptyIcon = ui::Icon::Tracker;
	if (devices.empty())
	{
		emptyTitle = Tr("No Lighthouse devices");
		emptyBody = Tr("Turn on a tracker or controller that uses base stations.");
	}
	else if (!CalCtx.lighthouseLogAvailable)
	{
		emptyIcon = ui::Icon::Clock;
		emptyTitle = Tr("Waiting for SteamVR's log");
		emptyBody = Tr("Base stations appear here once SteamVR has written " +
			(CalCtx.lighthouseLogPath.empty() ? std::string("its vrserver log.") : CalCtx.lighthouseLogPath));
	}
	if (!emptyTitle.empty())
	{
		YGNodeRef card = fl.Column(root);
		YGNodeStyleSetMargin(card, YGEdgeTop, 24.0f);
		YGNodeStyleSetAlignItems(card, YGAlignCenter);
		YGNodeStyleSetPadding(card, YGEdgeVertical, 30.0f);
		YGNodeStyleSetPadding(card, YGEdgeHorizontal, 24.0f);
		YGNodeRef icon = fl.Box(card, 56.0f, 56.0f);
		YGNodeRef t = ui::TextNode(fl, card, ui::type::Label, emptyTitle);
		YGNodeStyleSetMargin(t, YGEdgeTop, 14.0f);
		YGNodeRef b = ui::TextNode(fl, card, ui::type::Body, emptyBody);
		YGNodeStyleSetMargin(b, YGEdgeTop, 4.0f);
		YGNodeStyleSetMaxWidth(b, 600.0f);
		fl.Compute(origin, width, YGUndefined);
		drawHead();
		const FlexRect c = fl.Rect(card);
		ui::Card(dl, c.min, c.max);
		ui::RoundIcon(dl, fl.Rect(icon).Center(), 56.0f, emptyIcon, 26.0f, ui::col::Muted, ui::col::Fill);
		ui::DrawText(dl, ui::type::Label, ImVec2(c.min.x, fl.Rect(t).min.y), c.W(), ui::col::Text, emptyTitle.c_str(), ui::Align::Center);
		const FlexRect br = fl.Rect(b);
		ui::DrawText(dl, ui::type::Body, br.min, br.W() + 1.0f, ui::col::Muted, emptyBody.c_str(), ui::Align::Center);
		return YGNodeLayoutGetHeight(root);
	}

	// ---- Layout ----
	const auto stations = StationsByChannel();
	const float contentW = width - 80.0f;
	const std::string stationsHead = Tr("Base stations"), devicesHead = Tr("What each device sees");
	YGNodeRef stationsTitle = Heading(fl, root, stationsHead);
	YGNodeRef noStations = nullptr, grid = nullptr;
	std::vector<YGNodeRef> stationCards;
	const std::string noStationsText = Tr("None named yet. Each base station appears as SteamVR reports it.");
	if (stations.empty())
	{
		noStations = ui::TextNode(fl, root, ui::type::Body, noStationsText);
		YGNodeStyleSetMargin(noStations, YGEdgeTop, 10.0f);
	}
	else
	{
		grid = fl.Row(root);
		YGNodeStyleSetMargin(grid, YGEdgeTop, 10.0f);
		YGNodeStyleSetFlexWrap(grid, YGWrapWrap);
		YGNodeStyleSetGap(grid, YGGutterAll, 14.0f);
		const int perRow = std::min(4, static_cast<int>(stations.size()));
		const float cardW = std::floor((contentW - 14.0f * static_cast<float>(perRow - 1)) / static_cast<float>(perRow));
		for (size_t i = 0; i < stations.size(); ++i)
			stationCards.push_back(fl.Box(grid, cardW, 128.0f));
	}
	YGNodeRef devicesTitle = Heading(fl, root, devicesHead);
	YGNodeRef table = fl.Column(root);
	YGNodeStyleSetMargin(table, YGEdgeTop, 10.0f);
	YGNodeRef header = fl.Box(table, contentW, 34.0f);
	std::vector<YGNodeRef> deviceRows;
	for (size_t i = 0; i < devices.size(); ++i)
		deviceRows.push_back(fl.Box(table, contentW, 56.0f));
	// What the log did for the drift monitor, and where it was read.
	const std::string driftText = Tr(CalCtx.lighthouseAttributedEvents == 0
		? std::string("Base station changes haven't affected drift checks this session.")
		: CalCtx.lighthouseAttributedEvents == 1
		? std::string("1 drift check this session was skipped because a base station had just changed.")
		: FormatString("%u drift checks this session were skipped because a base station had just changed.",
			CalCtx.lighthouseAttributedEvents));
	YGNodeRef drift = ui::TextNode(fl, root, ui::type::Footnote, driftText);
	YGNodeStyleSetMargin(drift, YGEdgeTop, 12.0f);
	YGNodeStyleSetMaxWidth(drift, 760.0f);
	const ui::TextStyle pathStyle{ ui::Weight::Regular, 14.0f, 19.0f };
	const std::string pathText = CalCtx.lighthouseLogPath.empty() ? std::string()
		: Tr(FormatString("Read from %s", CalCtx.lighthouseLogPath.c_str()));
	YGNodeRef path = pathText.empty() ? nullptr : ui::TextNode(fl, root, pathStyle, pathText);
	if (path)
		YGNodeStyleSetMargin(path, YGEdgeTop, 2.0f);
	fl.Compute(origin, width, YGUndefined);

	// ---- Drawing ----
	drawHead();
	const FlexRect st = fl.Rect(stationsTitle);
	ui::DrawText(dl, ui::type::Section, st.min, st.W() + 1.0f, ui::col::Text, stationsHead.c_str());
	if (noStations)
	{
		const FlexRect r = fl.Rect(noStations);
		ui::DrawText(dl, ui::type::Body, r.min, r.W() + 1.0f, ui::col::Muted, noStationsText.c_str());
	}

	// How many of the switched-on devices have each station in view right now.
	for (size_t i = 0; i < stations.size(); ++i)
	{
		const auto &s = stations[i];
		const FlexRect c = fl.Rect(stationCards[i]);
		int seeing = 0, reporting = 0;
		for (const VRDevice *dev : devices)
		{
			if (!dev->connected)
				continue;
			const auto *seen = CalCtx.lighthouse.Find(dev->serial);
			if (!seen || !seen->visibleKnown)
				continue;
			++reporting;
			if (Sees(*seen, s.channel))
				++seeing;
		}
		// A station no device sees is the one to look at: blocked, unplugged,
		// or aimed away.
		const bool unseen = reporting > 0 && seeing == 0;
		ui::Card(dl, c.min, c.max);
		if (unseen)
			ui::InnerEdge(dl, c.min, c.max, ui::col::Alert, 20.0f, 1.5f);
		const float x = c.min.x + 18.0f, lineY = c.min.y + 16.0f;
		ui::DrawIcon(dl, ui::Icon::Lighthouse, ImVec2(x + 11.0f, lineY + 14.0f), 22.0f, ui::col::Link);
		const std::string channel = FormatString("S-%d", s.channel);
		ui::DrawLine(dl, ui::type::CardTitle, ImVec2(x + 22.0f + 10.0f, lineY), ui::col::Text, channel.c_str());
		const std::string id = s.id != 0 ? LighthouseVisibility::IdName(s.id) : std::string(Tr("id not logged yet"));
		const ui::TextStyle idStyle{ s.id != 0 ? ui::Weight::Mono : ui::Weight::Regular, 13.0f, 28.0f };
		const float idW = ui::MeasureLine(idStyle, id.c_str()).x;
		ui::DrawLine(dl, idStyle, ImVec2(c.max.x - 18.0f - idW, lineY), ui::Rgba(236, 238, 244, 0.50f), id.c_str());
		const std::string seenLine = Tr(reporting == 0 ? std::string("No reports yet")
			: FormatString("Seen by %d of %d", seeing, reporting));
		ui::DrawLine(dl, ui::type::BodyMedium, ImVec2(x, c.max.y - 16.0f - 20.0f - 22.0f), unseen ? ui::col::Alert : ui::col::Text,
			seenLine.c_str());
		const ImU32 dropInk = s.drops == 0 ? ui::col::Good : s.drops >= 5 ? ui::col::Caution : ui::col::Muted;
		ui::DrawLine(dl, ui::type::Footnote, ImVec2(x, c.max.y - 16.0f - 20.0f), dropInk, Tr(DropWords(s.drops)).c_str());
		// A loss is a drop that left a device with no station at all.
		if (s.losses > 0 && ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(c.min, c.max))
			ui::Tip(FormatString("Dropped out %u time%s \xC2\xB7 the last one left %u time%s",
				s.drops, s.drops == 1 ? "" : "s", s.losses, s.losses == 1 ? "" : "s").c_str());
	}

	const FlexRect dt = fl.Rect(devicesTitle);
	ui::DrawText(dl, ui::type::Section, dt.min, dt.W() + 1.0f, ui::col::Text, devicesHead.c_str());
	const FlexRect tableR = fl.Rect(table);
	ui::Card(dl, tableR.min, tableR.max);

	// The columns: the device, one per station, and the figure at the end.
	const float rightW = 130.0f, colGap = 4.0f;
	const float right1 = tableR.max.x - 18.0f, right0 = right1 - rightW;
	const float stationSpan = std::max(0.0f, right0 - colGap - (tableR.min.x + 12.0f + 220.0f));
	const float colW = stations.empty() ? 0.0f
		: std::min(48.0f, (stationSpan - colGap * static_cast<float>(stations.size() - 1)) / static_cast<float>(stations.size()));
	const float stations0 = right0 - colGap - (colW + colGap) * static_cast<float>(stations.size()) + colGap;
	auto columnCentre = [&](size_t k) { return stations0 + (colW + colGap) * static_cast<float>(k) + colW * 0.5f; };

	const FlexRect head = fl.Rect(header);
	ui::DrawLine(dl, ui::TextStyle{ kColumnHead.weight, kColumnHead.size, head.H() }, ImVec2(head.min.x + 12.0f, head.min.y),
		kQuiet, Tr("Device"));
	for (size_t k = 0; k < stations.size(); ++k)
	{
		const std::string label = FormatString("S-%d", stations[k].channel);
		const float w = ui::MeasureLine(kColumnHead, label.c_str()).x;
		ui::DrawLine(dl, ui::TextStyle{ kColumnHead.weight, kColumnHead.size, head.H() }, ImVec2(columnCentre(k) - w * 0.5f, head.min.y),
			kQuiet, label.c_str());
	}
	{
		const char *label = Tr("Base stations");
		const float w = ui::MeasureLine(kColumnHead, label).x;
		ui::DrawLine(dl, ui::TextStyle{ kColumnHead.weight, kColumnHead.size, head.H() }, ImVec2(right1 - w, head.min.y), kQuiet, label);
	}
	ui::Hairline(dl, head.min.x, head.max.x, head.max.y - 1.0f);

	const int clean = CalCtx.lighthouse.Settings().cleanStations;
	for (size_t i = 0; i < devices.size(); ++i)
	{
		const VRDevice &dev = *devices[i];
		const auto *seen = CalCtx.lighthouse.Find(dev.serial);
		const bool reporting = dev.connected && seen && seen->visibleKnown;
		const FlexRect r = fl.Rect(deviceRows[i]);
		ImGui::PushID(dev.id);
		if (i > 0)
			ui::Hairline(dl, r.min.x + 68.0f, r.max.x, r.min.y);

		// The row is one hover target for its tooltip; nothing on it is
		// pressed.
		ImGui::SetCursorScreenPos(r.min);
		ImGui::InvisibleButton("##row", r.Size());
		const bool hov = ImGui::IsItemHovered();
		if (hov)
			ui::FillRounded(dl, r.min, r.max, ui::Rgba(255, 255, 255, 0.03f), 20.0f,
				i + 1 == devices.size() ? ImDrawFlags_RoundCornersBottom : ImDrawFlags_RoundCornersNone);

		FlexRect art;
		art.min = ImVec2(r.min.x + 12.0f, r.Center().y - 22.0f);
		art.max = ImVec2(art.min.x + 44.0f, art.min.y + 44.0f);
		DrawDeviceArt(dl, dev, art, ui::col::Muted);
		// The headset's tracker goes by its job unless it was named.
		const bool mounted = CalCtx.continuousEnabled && dev.serial == CalCtx.continuousTrackerSerial;
		const std::string *custom = FindDeviceName(dev.serial);
		const std::string name = custom ? *custom : mounted ? std::string(Tr("Headset tracker")) : dev.model;
		const std::string sub = name == dev.model ? dev.serial : dev.model + " \xC2\xB7 " + dev.serial;
		const float textX = art.max.x + 12.0f, textW = stations0 - colGap - textX;
		const ImU32 ink = dev.connected ? ui::col::Text : ui::col::Faint;
		ui::DrawLine(dl, kRowName, ImVec2(textX, r.Center().y - 20.0f), ink, ui::Ellipsize(kRowName, name, textW).c_str());
		ui::DrawLine(dl, kRowSub, ImVec2(textX, r.Center().y + 2.0f), kQuiet, ui::Ellipsize(kRowSub, sub, textW).c_str());

		// One dot per station, in the cards' order: filled while this device
		// has it in view.
		for (size_t k = 0; k < stations.size(); ++k)
		{
			const ImVec2 c(columnCentre(k), r.Center().y);
			if (reporting && Sees(*seen, stations[k].channel))
				dl->AddCircleFilled(c, 7.0f, ui::col::Good, 24);
			else
				dl->AddCircle(c, 6.0f, ui::Rgba(236, 238, 244, reporting ? 0.32f : 0.16f), 24, 2.0f);
		}

		// The figure at the end: how many are in view, red when too few for a
		// solid solution, with the drop count under it.
		std::string figure, detail;
		ImU32 figureInk = ui::col::Muted;
		if (!dev.connected)
			figure = "Off";
		else if (!reporting)
			figure = "Not in the log yet";
		else
		{
			const int inView = seen->InView();
			const int total = std::max(CalCtx.lighthouse.StationCount(), inView);
			figure = FormatString("%d of %d in view", inView, total);
			figureInk = inView < clean ? ui::col::Alert : ui::col::Text;
			detail = seen->drops == 0 ? std::string("No dropouts")
				: FormatString("%u dropout%s", seen->drops, seen->drops == 1 ? "" : "s");
			if (seen->losses > 0)
				detail += FormatString(", %u full loss%s", seen->losses, seen->losses == 1 ? "" : "es");
		}
		figure = Tr(figure);
		detail = Tr(detail);
		const float fw = ui::MeasureLine(kRowName, figure.c_str()).x;
		ui::DrawLine(dl, kRowName, ImVec2(right1 - fw, detail.empty() ? r.Center().y - 11.0f : r.Center().y - 20.0f),
			figureInk, figure.c_str());
		if (!detail.empty())
		{
			const float w = ui::MeasureLine(kRowSub, detail.c_str()).x;
			ui::DrawLine(dl, kRowSub, ImVec2(right1 - w, r.Center().y + 2.0f), kQuiet, detail.c_str());
		}

		if (hov && reporting)
		{
			std::string tip = "In view:";
			if (seen->InView() == 0)
				tip += " no base station";
			for (int c : seen->visible)
				tip += " " + CalCtx.lighthouse.StationName(c);
			for (uint32_t id : seen->unmappedIds)
				tip += " " + LighthouseVisibility::IdName(id);
			if (!seen->lastDisturbanceText.empty())
				tip += "\nLast change: " + seen->lastDisturbanceText;
			if (seen->bootstraps > 0)
				tip += FormatString("\nTracking restarted %u time%s", seen->bootstraps, seen->bootstraps == 1 ? "" : "s");
			ShowTip(tip.c_str(), true);
		}
		ImGui::PopID();
	}

	const FlexRect d = fl.Rect(drift);
	ui::DrawText(dl, ui::type::Footnote, d.min, d.W() + 1.0f, ui::col::Muted, driftText.c_str());
	if (path)
	{
		const FlexRect p = fl.Rect(path);
		ui::DrawText(dl, pathStyle, p.min, p.W() + 1.0f, ui::Rgba(236, 238, 244, 0.42f), pathText.c_str());
	}
	return YGNodeLayoutGetHeight(root);
}
