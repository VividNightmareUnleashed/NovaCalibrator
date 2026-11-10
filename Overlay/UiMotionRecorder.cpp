// What the Motion Recorder module draws (docs/modules.md): its page, where a
// player starts and stops a recording and plays it back; the warning a player
// reads before a new recording starts; the question before a recording is
// deleted; and the sign in the sidebar while a recording runs.
#include "stdafx.h"
#include "UiInternal.h"
#include "MotionRecorder.h"
#include "MotionRecorderText.h"

#include <ctime>
#include <string>
#include <vector>

namespace questcal::motionrecorder
{

namespace
{

using recording::Playback;

enum class Kind { Warning, Delete };
Kind s_kind = Kind::Warning;

// The page's own state: where the playback is, and how it is looked at.
double s_time = 0.0;
bool s_playing = false;
int s_speed = 1;
int s_view = 0;
constexpr double Speeds[] = { 0.5, 1.0, 2.0, 4.0 };
constexpr const char *SpeedLabels[] = { "0.5\xC3\x97", "1\xC3\x97", "2\xC3\x97", "4\xC3\x97" };

// A round icon, a title, an optional label under it, paragraphs and two
// buttons side by side: the shape of the overlay's own confirmations, with the
// paragraphs set left when there are many. Laid out to find the panel's
// height, then drawn; returns which button was pressed (0 none, 1 left,
// 2 right).
int Panel(float width, ui::Icon icon, ImU32 ink, ImU32 fill, const char *titleEnglish, const char *labelEnglish,
	const std::vector<const char *> &paragraphsEnglish, const char *leftEnglish, ui::Btn leftKind,
	const char *rightEnglish, ui::Btn rightKind, ui::Icon rightIcon)
{
	const bool list = paragraphsEnglish.size() > 1;
	const ui::TextStyle bodyStyle = list ? ui::type::Callout : ui::type::Body;
	const float contentW = width - 60.0f;
	const std::string title = Tr(titleEnglish);
	std::vector<std::string> paragraphs;
	paragraphs.reserve(paragraphsEnglish.size());
	for (const char *english : paragraphsEnglish)
		paragraphs.emplace_back(Tr(english));

	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetAlignItems(root, YGAlignCenter);
	YGNodeStyleSetWidth(root, contentW);
	YGNodeRef iconNode = fl.Box(root, 64.0f, 64.0f);
	YGNodeRef titleNode = ui::TextNode(fl, root, ui::type::DialogTitle, title);
	YGNodeStyleSetMargin(titleNode, YGEdgeTop, 18.0f);
	YGNodeRef labelNode = nullptr;
	if (labelEnglish)
	{
		labelNode = fl.Box(root, ui::BadgeWidth(labelEnglish), 26.0f);
		YGNodeStyleSetMargin(labelNode, YGEdgeTop, 10.0f);
	}
	std::vector<YGNodeRef> texts;
	texts.reserve(paragraphs.size());
	for (size_t i = 0; i < paragraphs.size(); ++i)
	{
		YGNodeRef p = ui::TextNode(fl, root, bodyStyle, paragraphs[i]);
		YGNodeStyleSetAlignSelf(p, list ? YGAlignStretch : YGAlignCenter);
		YGNodeStyleSetMargin(p, YGEdgeTop, i == 0 ? (list ? 16.0f : 8.0f) : 8.0f);
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
	const FlexRect titleR = fl.Rect(titleNode);
	ui::DrawText(dl, ui::type::DialogTitle, ImVec2(content.min.x, titleR.min.y), contentW, ui::col::Text, title.c_str(),
		ui::Align::Center);
	if (labelNode)
		ui::BadgePill(dl, fl.Rect(labelNode).min, labelEnglish, ui::col::Caution, ui::col::CautionTint);
	for (size_t i = 0; i < texts.size(); ++i)
	{
		const FlexRect r = fl.Rect(texts[i]);
		ui::DrawText(dl, bodyStyle, ImVec2(content.min.x, r.min.y), contentW, ui::col::Muted, paragraphs[i].c_str(),
			list ? ui::Align::Left : ui::Align::Center);
	}
	int pressed = 0;
	if (ui::PillButton("##left", leftEnglish, leftKind, fl.Rect(left)))
		pressed = 1;
	if (ui::PillButton("##right", rightEnglish, rightKind, fl.Rect(right), rightIcon))
		pressed = 2;
	return pressed;
}

// ---------------------------------------------------------------------------
// The view of the devices

// Which part of the body a device is on, as an index into text::DeviceNames.
enum Part
{
	Head, LeftHand, RightHand, Waist, Chest, LeftFoot, RightFoot, LeftKnee, RightKnee, LeftElbow, RightElbow,
	LeftShoulder, RightShoulder, OtherTracker, OtherController, NotABody
};

Part PartOf(const recording::DeviceInfo &info)
{
	if (info.deviceClass == static_cast<uint32_t>(vr::TrackedDeviceClass_HMD))
		return Head;
	if (info.deviceClass == static_cast<uint32_t>(vr::TrackedDeviceClass_Controller))
		return info.controllerRole == vr::TrackedControllerRole_LeftHand ? LeftHand
			: info.controllerRole == vr::TrackedControllerRole_RightHand ? RightHand : OtherController;
	if (info.deviceClass != static_cast<uint32_t>(vr::TrackedDeviceClass_GenericTracker))
		return NotABody;
	// SteamVR's role for a tracker, such as "vive_tracker_left_foot".
	static const std::pair<const char *, Part> roles[] = {
		{ "waist", Waist }, { "chest", Chest }, { "left_foot", LeftFoot }, { "right_foot", RightFoot },
		{ "left_knee", LeftKnee }, { "right_knee", RightKnee }, { "left_elbow", LeftElbow },
		{ "right_elbow", RightElbow }, { "left_shoulder", LeftShoulder }, { "right_shoulder", RightShoulder },
	};
	for (const auto &role : roles)
		if (info.controllerType.find(role.first) != std::string::npos)
			return role.second;
	return OtherTracker;
}

ImU32 InkOf(Part part)
{
	return part == Head ? ui::col::Good
		: part == LeftHand || part == RightHand || part == OtherController ? ui::col::Link : ui::col::Lavender;
}

// The devices in `box`, each a dot with its path behind it, seen from the
// front, from above or from the side. One meter is the same length in every
// view; the view follows the body across the floor.
void DrawDevices(ImDrawList *dl, const FlexRect &box, int view, const std::vector<Playback::DevicePose> &devices)
{
	ui::FillRounded(dl, box.min, box.max, ui::col::Well, 20.0f, ImDrawFlags_RoundCornersTop);
	struct Shown
	{
		const Playback::DevicePose *pose;
		Part part;
	};
	std::vector<Shown> shown;
	shown.reserve(devices.size());
	double sum[3] = { 0.0, 0.0, 0.0 };
	int tracked = 0;
	for (const Playback::DevicePose &pose : devices)
	{
		const Part part = PartOf(pose.info);
		if (part == NotABody)
			continue;
		shown.push_back({ &pose, part });
		if (pose.tracked)
		{
			++tracked;
			for (int i = 0; i < 3; ++i)
				sum[i] += pose.position[i];
		}
	}

	// Where the body stands, eased so that a step does not shake the view.
	static double centre[2] = { 0.0, 0.0 };
	static bool placed = false;
	if (tracked > 0)
	{
		const double x = sum[0] / tracked, z = sum[2] / tracked;
		const double ease = placed && !g_uiPreviewMode ? 0.08 : 1.0;
		centre[0] += (x - centre[0]) * ease;
		centre[1] += (z - centre[1]) * ease;
		placed = true;
	}

	const float scale = (box.H() - 46.0f) / 2.1f;   // pixels for one meter
	const float midX = (box.min.x + box.max.x) * 0.5f;
	const float floorY = box.max.y - 18.0f, midY = (box.min.y + box.max.y) * 0.5f + 8.0f;
	const auto project = [&](float x, float y, float z)
	{
		const float dx = x - static_cast<float>(centre[0]), dz = z - static_cast<float>(centre[1]);
		if (view == 1)
			return ImVec2(midX + dx * scale, midY + dz * scale);
		return ImVec2(midX + (view == 0 ? dx : -dz) * scale, floorY - y * scale);
	};

	dl->PushClipRect(box.min, box.max, true);
	// Lines every half meter, and the floor.
	const ImU32 gridInk = ui::Rgba(255, 255, 255, 0.05f);
	const float step = scale * 0.5f;
	for (float x = midX - std::floor((midX - box.min.x) / step) * step; x < box.max.x; x += step)
		dl->AddLine(ImVec2(std::floor(x) + 0.5f, box.min.y), ImVec2(std::floor(x) + 0.5f, box.max.y), gridInk);
	const float gridY = view == 1 ? midY : floorY;
	for (float y = gridY - std::floor((gridY - box.min.y) / step) * step; y < box.max.y; y += step)
		dl->AddLine(ImVec2(box.min.x, std::floor(y) + 0.5f), ImVec2(box.max.x, std::floor(y) + 0.5f), gridInk);
	if (view != 1)
		dl->AddLine(ImVec2(box.min.x, floorY + 0.5f), ImVec2(box.max.x, floorY + 0.5f), ui::Rgba(255, 255, 255, 0.16f));

	const auto at = [&](const Playback::DevicePose &pose)
	{
		return project(static_cast<float>(pose.position[0]), static_cast<float>(pose.position[1]),
			static_cast<float>(pose.position[2]));
	};
	const auto find = [&](Part part) -> const Playback::DevicePose *
	{
		for (const Shown &entry : shown)
			if (entry.part == part && entry.pose->tracked)
				return entry.pose;
		return nullptr;
	};
	// A faint figure between the parts that are known: it makes dots a body.
	const ImU32 boneInk = ui::Rgba(236, 238, 244, 0.16f);
	const Playback::DevicePose *head = find(Head);
	const Playback::DevicePose *hips = find(Waist) ? find(Waist) : find(Chest);
	const auto bone = [&](const Playback::DevicePose *a, const Playback::DevicePose *b)
	{
		if (a && b)
			dl->AddLine(at(*a), at(*b), boneInk, 2.0f);
	};
	bone(head, hips);
	bone(hips, find(LeftFoot));
	bone(hips, find(RightFoot));
	if (head && hips)
	{
		const ImVec2 h = at(*head), w = at(*hips);
		const ImVec2 neck(h.x + (w.x - h.x) * 0.25f, h.y + (w.y - h.y) * 0.25f);
		for (Part hand : { LeftHand, RightHand })
			if (const Playback::DevicePose *pose = find(hand))
				dl->AddLine(neck, at(*pose), boneInk, 2.0f);
	}

	for (const Shown &entry : shown)
	{
		const ImU32 ink = InkOf(entry.part);
		const auto &trail = entry.pose->trail;
		for (size_t i = 1; i < trail.size(); ++i)
		{
			const float age = static_cast<float>(i) / static_cast<float>(trail.size());
			dl->AddLine(project(trail[i - 1][0], trail[i - 1][1], trail[i - 1][2]),
				project(trail[i][0], trail[i][1], trail[i][2]), ui::Fade(ink, 0.08f + 0.5f * age), 3.0f);
		}
	}
	const bool labels = shown.size() <= 8;
	const ui::TextStyle labelStyle{ ui::Weight::Medium, 13.0f, 18.0f };
	for (const Shown &entry : shown)
	{
		const ImVec2 c = at(*entry.pose);
		const float r = entry.part == Head ? 7.0f : 6.0f;
		if (entry.pose->tracked)
		{
			dl->AddCircleFilled(c, r + 5.0f + (entry.part == Head ? 1.0f : 0.0f), ui::Fade(InkOf(entry.part), 0.18f), 32);
			dl->AddCircleFilled(c, r, InkOf(entry.part), 32);
		}
		else
			// Where it was when its tracking stopped.
			dl->AddCircle(c, r, ui::col::Faint, 32, 2.0f);
		if (labels)
		{
			// A left part's name is on the left of its dot, so that the names of
			// a pair do not cover each other.
			const bool onLeft = entry.part == LeftHand || entry.part == LeftFoot || entry.part == LeftKnee ||
				entry.part == LeftElbow || entry.part == LeftShoulder;
			const char *name = Tr(text::DeviceNames[entry.part]);
			const float x = onLeft ? c.x - r - 10.0f - ui::MeasureLine(labelStyle, name).x : c.x + r + 10.0f;
			ui::DrawLine(dl, labelStyle, ImVec2(x, c.y - 9.0f), entry.pose->tracked ? ui::col::Prose : ui::col::Faint, name);
		}
	}
	dl->PopClipRect();
}

// Minutes and seconds, with hours in front of a long recording.
std::string Clock(double seconds)
{
	const long long whole = static_cast<long long>(std::floor((std::max)(0.0, seconds)));
	if (whole >= 3600)
		return FormatString("%lld:%02lld:%02lld", whole / 3600, (whole / 60) % 60, whole % 60);
	return FormatString("%lld:%02lld", whole / 60, whole % 60);
}

// The day and the time on the player's clock, in digits so that every
// language reads it.
std::string LocalDateTime(int64_t unixMicroseconds)
{
	const time_t seconds = static_cast<time_t>(unixMicroseconds / 1000000);
	struct tm local{};
	if (localtime_s(&local, &seconds) != 0)
		return std::string();
	char text[32];
	strftime(text, sizeof text, "%Y-%m-%d %H:%M", &local);
	return text;
}

// A small pill over the view: a dot when it has one, and a line.
void ViewPill(ImDrawList *dl, ImVec2 topLeft, const std::string &label, ImU32 fill, bool dot)
{
	const ui::TextStyle style{ dot ? ui::Weight::SemiBold : ui::Weight::Medium, 14.0f, 32.0f };
	const float lead = dot ? 18.0f : 0.0f;
	const float w = 14.0f + lead + ui::MeasureLine(style, label.c_str()).x + 14.0f;
	ui::FillRounded(dl, topLeft, ImVec2(topLeft.x + w, topLeft.y + 32.0f), fill, 16.0f);
	if (dot)
		dl->AddCircleFilled(ImVec2(topLeft.x + 14.0f + 4.0f, topLeft.y + 16.0f), 4.0f, ui::col::Alert, 16);
	ui::DrawLine(dl, style, ImVec2(topLeft.x + 14.0f + lead, topLeft.y), dot ? ui::col::Text : ui::col::Body, label.c_str());
}

// The timeline: the stretches the recorder was on, each as long as it lasted,
// with a short hatched piece between two of them (the time between is not
// drawn to scale: it can be hours) and a mark where data was lost. Dragging
// moves the playback. Returns the time picked, or a negative number.
double Timeline(ImDrawList *dl, const FlexRect &r, const Playback::Summary &summary, double time)
{
	const float y = r.Center().y;
	const size_t count = summary.stretches.size();
	const float breakW = count > 1 ? (std::min)(12.0f, r.W() * 0.25f / static_cast<float>(count - 1)) : 0.0f;
	const float recordedW = r.W() - breakW * static_cast<float>(count > 1 ? count - 1 : 0);
	const double total = summary.seconds;
	// Where a stretch starts on the bar, and how long it is there.
	const auto left = [&](size_t i)
	{
		const double before = total > 0.0 ? summary.stretches[i].start / total : static_cast<double>(i) / static_cast<double>(count);
		return r.min.x + static_cast<float>(before) * recordedW + breakW * static_cast<float>(i);
	};
	const auto length = [&](size_t i)
	{
		return static_cast<float>(total > 0.0 ? summary.stretches[i].seconds / total : 1.0 / static_cast<double>(count)) * recordedW;
	};
	const auto xOf = [&](double t)
	{
		for (size_t i = count; i-- > 0;)
			if (t >= summary.stretches[i].start || i == 0)
			{
				const double into = summary.stretches[i].seconds > 0.0
					? (std::min)(1.0, (std::max)(0.0, (t - summary.stretches[i].start) / summary.stretches[i].seconds)) : 0.0;
				return left(i) + static_cast<float>(into) * length(i);
			}
		return r.min.x;
	};
	const auto timeOf = [&](float x)
	{
		for (size_t i = 0; i < count; ++i)
		{
			const float a = left(i), b = a + length(i);
			if (x <= b || i + 1 == count)
			{
				const float into = b > a ? (std::min)(1.0f, (std::max)(0.0f, (x - a) / (b - a))) : 0.0f;
				return summary.stretches[i].start + static_cast<double>(into) * summary.stretches[i].seconds;
			}
		}
		return 0.0;
	};

	const float knob = count > 0 ? xOf(time) : r.min.x;
	ui::FillRounded(dl, ImVec2(r.min.x, y - 3.0f), ImVec2(r.max.x, y + 3.0f), ui::col::Fill, 3.0f);
	// More stretches than the bar has room to tell apart are one bar.
	const bool pieces = count <= 300;
	if (!pieces)
	{
		ui::FillRounded(dl, ImVec2(r.min.x, y - 3.0f), ImVec2(r.max.x, y + 3.0f), ui::Fade(ui::col::Link, 0.38f), 3.0f);
		ui::FillRounded(dl, ImVec2(r.min.x, y - 3.0f), ImVec2(knob, y + 3.0f), ui::col::Link, 3.0f);
	}
	for (size_t i = 0; pieces && i < count; ++i)
	{
		const float a = left(i), b = a + length(i);
		ui::FillRounded(dl, ImVec2(a, y - 3.0f), ImVec2(b, y + 3.0f), ui::Fade(ui::col::Link, 0.38f), 3.0f);
		if (knob > a)
			ui::FillRounded(dl, ImVec2(a, y - 3.0f), ImVec2((std::min)(b, knob), y + 3.0f), ui::col::Link, 3.0f);
		// Off: three slanted strokes in the piece between two stretches.
		if (i + 1 < count && breakW >= 6.0f)
			for (float x = b + 1.0f; x + 4.0f <= b + breakW; x += 4.0f)
				dl->AddLine(ImVec2(x, y + 5.0f), ImVec2(x + 3.0f, y - 5.0f), ui::Rgba(236, 238, 244, 0.30f), 1.5f);
	}
	size_t marks = 0;
	for (const Playback::Loss &loss : summary.losses)
	{
		if (++marks > 200)
			break;
		const float x = std::floor(xOf(loss.time));
		ui::FillRounded(dl, ImVec2(x - 1.5f, y - 9.0f), ImVec2(x + 1.5f, y + 9.0f), ui::col::Caution, 1.5f);
	}
	dl->AddCircleFilled(ImVec2(knob, y + 1.0f), 10.0f, ui::Rgba(0, 0, 0, 0.30f), 32);
	dl->AddCircleFilled(ImVec2(knob, y), 9.0f, ui::col::White, 32);

	ImGui::SetCursorScreenPos(r.min);
	ImGui::InvisibleButton("##timeline", r.Size());
	if (ImGui::IsItemFocused())
		ui::FocusRing(dl, ImVec2(r.min.x - 6.0f, r.min.y), ImVec2(r.max.x + 6.0f, r.max.y), 14.0f);
	if (ImGui::IsItemActive() && count > 0)
		return timeOf(ImGui::GetIO().MousePos.x);
	return -1.0;
}

// A link in a card's last row. Returns whether it was pressed.
bool CardLink(ImDrawList *dl, const char *id, ImVec2 topLeft, const char *english, ImU32 ink, float &advance)
{
	const ui::TextStyle style{ ui::Weight::Medium, 17.0f, 44.0f };
	const char *label = Tr(english);
	const float w = ui::MeasureLine(style, label).x;
	advance = w;
	ImGui::SetCursorScreenPos(topLeft);
	const bool pressed = ImGui::InvisibleButton(id, ImVec2(w, 44.0f));
	if (ImGui::IsItemFocused())
		ui::FocusRing(dl, ImVec2(topLeft.x - 8.0f, topLeft.y + 4.0f), ImVec2(topLeft.x + w + 8.0f, topLeft.y + 40.0f), 12.0f);
	ui::DrawLine(dl, style, topLeft, ImGui::IsItemHovered() ? ui::Mix(ink, ui::col::White, 0.25f) : ink, label);
	return pressed;
}

void OpenRecordingFolder()
{
	const std::wstring folder = Folder();
	if (g_uiPreviewMode || folder.empty())
		return;
	// Made here so the link works before the first recording.
	CreateDirectoryW(folder.substr(0, folder.rfind(L'\\')).c_str(), nullptr);
	CreateDirectoryW(folder.c_str(), nullptr);
	ShellExecuteW(nullptr, L"open", folder.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
}

} // namespace

void OpenWarning()
{
	s_kind = Kind::Warning;
	OpenDialog(Dialog::MotionRecorder);
}

void OpenDeleteConfirm()
{
	s_kind = Kind::Delete;
	OpenDialog(Dialog::MotionRecorder);
}

void BuildDialog()
{
	if (s_kind == Kind::Warning)
	{
		// Cancel starts nothing.
		const std::vector<const char *> paragraphs(std::begin(text::Disclaimer), std::end(text::Disclaimer));
		const int pressed = Panel(680.0f, ui::Icon::Warn, ui::col::Caution, ui::col::CautionTint, text::RowTitle,
			text::PrivacyLabel, paragraphs, text::CancelButton, ui::Btn::Secondary, text::StartButton, ui::Btn::Primary,
			ui::Icon::None);
		if (pressed == 1 || EscapePressed())
			CloseDialog();
		else if (pressed == 2)
		{
			CloseDialog();
			Start();
		}
		return;
	}
	const int pressed = Panel(560.0f, ui::Icon::Trash, ui::col::DangerInk, ui::col::AlertTint, text::DeleteTitle, nullptr,
		{ text::DeleteLine }, text::KeepButton, ui::Btn::Primary, text::DeleteRecording, ui::Btn::Danger, ui::Icon::Trash);
	if (pressed == 1 || EscapePressed())
		CloseDialog();
	else if (pressed == 2)
	{
		CloseDialog();
		DeleteRecording();
	}
}

// A dot and a line in the sidebar, on every page, for as long as a recording
// runs: a player never records without seeing that they do.
void BuildSidebarSign(const ImVec2 &min, const ImVec2 &max)
{
	if (!Now().on)
		return;
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const float textW = (max.x - min.x) - 14.0f - 8.0f - 10.0f - 12.0f;
	const ui::TextLines lines = ui::WrapText(ui::type::CaptionStrong, Tr(text::Sign), textW);
	const float height = (std::max)(40.0f, lines.height + 18.0f);
	const ImVec2 end(max.x, min.y + height);
	ui::FillRounded(dl, min, end, ui::col::AlertTint, 14.0f);
	dl->AddCircleFilled(ImVec2(min.x + 14.0f + 4.0f, min.y + height * 0.5f), 4.0f, ui::col::Alert, 16);
	ui::DrawLines(dl, ui::type::CaptionStrong, lines, ImVec2(min.x + 14.0f + 8.0f + 10.0f, min.y + (height - lines.height) * 0.5f),
		textW, ui::col::Text);
}

} // namespace questcal::motionrecorder

// ---------------------------------------------------------------------------
// The page

float BuildMotionRecorderPage(const VRState &, ImVec2 origin, float width)
{
	using namespace questcal::motionrecorder;
	using questcal::recording::Playback;
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const double now = ImGui::GetTime();
	const Status status = Now();
	KeepPlayerCurrent(now);
	const Playback::Summary summary = status.on ? Playback::Summary() : Player().GetSummary();
	const bool live = status.on;
	const bool ready = !live && status.fileExists && summary.state == Playback::State::Ready && summary.samples > 0;
	const bool reading = !live && status.fileExists && summary.state == Playback::State::Reading;

	// A message in place of the view, when there is nothing to play.
	std::string noteTitle, noteBody;
	ui::Icon noteIcon = ui::Icon::Play;
	if (!live && !ready && !reading)
	{
		if (!status.fileExists || summary.state == Playback::State::Empty)
		{
			noteTitle = Tr(text::NoRecordingTitle);
			noteBody = Tr(text::NoRecordingLine);
		}
		else if (summary.state == Playback::State::Failed)
		{
			noteIcon = ui::Icon::Warn;
			noteTitle = Tr(text::BadFileTitle);
			noteBody = Tr(text::BadFileLine);
		}
		else
		{
			noteTitle = Tr(text::NoRecordingTitle);
			noteBody = Tr(text::NoSamplesLine);
		}
	}
	else if (reading)
	{
		noteIcon = ui::Icon::Clock;
		noteTitle = Tr(text::Reading);
	}

	// ---- Layout ----
	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, width);
	YGNodeStyleSetPadding(root, YGEdgeHorizontal, 40.0f);
	const float contentW = width - 80.0f;
	const std::string titleText = Tr(text::PageTitle);
	YGNodeRef title = ui::TextNode(fl, root, ui::type::PageTitle, titleText);

	YGNodeRef playCard = fl.Column(root);
	YGNodeStyleSetMargin(playCard, YGEdgeTop, 22.0f);
	YGNodeRef viewBox = nullptr, transport = nullptr, legend = nullptr, cutShort = nullptr, liveRow = nullptr;
	YGNodeRef noteIconNode = nullptr, noteTitleNode = nullptr, noteBodyNode = nullptr, noteBar = nullptr;
	const std::string liveText = Tr(text::LiveLine), cutShortText = Tr(text::CutShortLine);
	if (live || ready)
	{
		viewBox = fl.Box(playCard, contentW, 262.0f);
		if (live)
		{
			liveRow = ui::TextNode(fl, playCard, ui::type::Callout, liveText);
			YGNodeStyleSetMargin(liveRow, YGEdgeHorizontal, 20.0f);
			YGNodeStyleSetMargin(liveRow, YGEdgeVertical, 14.0f);
		}
		else
		{
			transport = fl.Box(playCard, contentW, 64.0f);
			legend = fl.Box(playCard, contentW, 34.0f);
			if (summary.cutShort)
			{
				cutShort = ui::TextNode(fl, playCard, ui::type::Caption, cutShortText);
				YGNodeStyleSetMargin(cutShort, YGEdgeHorizontal, 20.0f);
				YGNodeStyleSetMargin(cutShort, YGEdgeBottom, 14.0f);
			}
		}
	}
	else
	{
		YGNodeStyleSetAlignItems(playCard, YGAlignCenter);
		YGNodeStyleSetPadding(playCard, YGEdgeVertical, 30.0f);
		YGNodeStyleSetPadding(playCard, YGEdgeHorizontal, 24.0f);
		noteIconNode = fl.Box(playCard, 56.0f, 56.0f);
		noteTitleNode = ui::TextNode(fl, playCard, ui::type::Label, noteTitle);
		YGNodeStyleSetMargin(noteTitleNode, YGEdgeTop, 14.0f);
		if (!noteBody.empty())
		{
			noteBodyNode = ui::TextNode(fl, playCard, ui::type::Body, noteBody);
			YGNodeStyleSetMargin(noteBodyNode, YGEdgeTop, 4.0f);
			YGNodeStyleSetMaxWidth(noteBodyNode, 600.0f);
		}
		if (reading)
		{
			noteBar = fl.Box(playCard, 320.0f, 6.0f);
			YGNodeStyleSetMargin(noteBar, YGEdgeTop, 16.0f);
		}
	}

	// The card of controls: a title and a line on the left of each row, the
	// control on the right.
	YGNodeRef controls = fl.Column(root);
	YGNodeStyleSetMargin(controls, YGEdgeTop, 20.0f);
	struct ControlRow
	{
		YGNodeRef row = nullptr, title = nullptr, line = nullptr, control = nullptr;
		std::string titleText, lineText;
	};
	const auto addRow = [&](const std::string &rowTitle, const std::string &rowLine, float controlW, float controlH,
		float titleExtra)
	{
		ControlRow out;
		out.titleText = rowTitle;
		out.lineText = rowLine;
		out.row = fl.Row(controls);
		YGNodeStyleSetAlignItems(out.row, YGAlignCenter);
		YGNodeStyleSetGap(out.row, YGGutterColumn, 16.0f);
		YGNodeStyleSetPadding(out.row, YGEdgeVertical, 13.0f);
		YGNodeStyleSetPadding(out.row, YGEdgeLeft, 20.0f);
		YGNodeStyleSetPadding(out.row, YGEdgeRight, 14.0f);
		YGNodeRef words = fl.Column(out.row);
		YGNodeStyleSetFlexGrow(words, 1.0f);
		YGNodeStyleSetFlexShrink(words, 1.0f);
		YGNodeStyleSetFlexBasis(words, 0.0f);
		out.title = fl.Box(words, ui::MeasureLine(ui::type::RowTitle, rowTitle.c_str()).x + titleExtra, 26.0f);
		out.line = ui::TextNode(fl, words, ui::type::Footnote, rowLine);
		YGNodeStyleSetMargin(out.line, YGEdgeTop, 2.0f);
		out.control = fl.Box(out.row, controlW, controlH);
		return out;
	};
	const std::string recordTitle = Tr(text::RowTitle);
	const std::string recordLine = live
		? std::string(Tr(FormatString(text::StatusPattern, static_cast<int>(status.seconds / 60.0),
			static_cast<double>(status.bytes) / static_cast<double>(1ull << 30))))
		: std::string(Tr(!status.fileExists ? text::LineFirst
			: !g_uiPreviewMode && !BeginsNewRecording() ? text::LineContinues : text::LineReplaces));
	const char *recordButton = live ? text::StopButton : text::StartButton;
	const float badgeW = ui::BadgeWidth(text::PrivacyLabel);
	const ControlRow record = addRow(recordTitle, recordLine, ui::PillWidth(recordButton, 17.0f), 46.0f,
		10.0f + 20.0f + 10.0f + badgeW);
	static const char *const sizes[] = { "1 GB", "5 GB", "20 GB" };
	static_assert(CapChoiceCount == 3 && CapChoicesBytes[0] == 1ull << 30 && CapChoicesBytes[1] == 5ull << 30 &&
		CapChoicesBytes[2] == 20ull << 30, "the labels above name the choices");
	const ControlRow cap = addRow(Tr(text::CapTitle), Tr(text::CapLine),
		ui::PillSegmentedWidth(sizes, CapChoiceCount, 16.0f, 72.0f, false), 42.0f, 0.0f);
	YGNodeRef links = fl.Box(controls, contentW, 50.0f);
	fl.Compute(origin, width, YGUndefined);

	// ---- Drawing ----
	const FlexRect t = fl.Rect(title);
	ui::DrawText(dl, ui::type::PageTitle, t.min, t.W() + 1.0f, ui::col::Text, titleText.c_str());

	const FlexRect card = fl.Rect(playCard);
	ui::Card(dl, card.min, card.max);
	if (viewBox)
	{
		const FlexRect view = fl.Rect(viewBox);
		static std::vector<Playback::DevicePose> poses;
		if (live)
		{
			LivePoses(now, poses);
			ui::KeepAnimating();
		}
		else
		{
			// A preview shows the recording a third of the way in.
			static bool seeded = false;
			if (g_uiPreviewMode && !seeded)
			{
				seeded = true;
				s_time = summary.seconds * 0.335;
			}
			s_time = (std::min)(s_time, summary.seconds);
			if (s_playing)
			{
				s_time += static_cast<double>(ImGui::GetIO().DeltaTime) * Speeds[s_speed];
				if (s_time >= summary.seconds)
				{
					s_time = summary.seconds;
					s_playing = false;
				}
				ui::KeepAnimating();
			}
			Player().Frame(s_time, poses);
		}
		DrawDevices(dl, view, s_view, poses);
		if (live)
			ViewPill(dl, ImVec2(view.min.x + 16.0f, view.min.y + 14.0f), Tr(text::Live), ui::col::AlertTint, true);
		else if (const std::string when = LocalDateTime(summary.startUnixMicroseconds); !when.empty())
			ViewPill(dl, ImVec2(view.min.x + 16.0f, view.min.y + 14.0f), Tr(FormatString(text::RecordedPattern, when.c_str())),
				ui::col::Fill, false);
		const float viewsW = ui::PillSegmentedWidth(text::Views, 3, 14.0f, 56.0f);
		FlexRect views;
		views.min = ImVec2(view.max.x - 16.0f - viewsW, view.min.y + 12.0f);
		views.max = ImVec2(view.max.x - 16.0f, view.min.y + 12.0f + 36.0f);
		s_view = ui::PillSegmented("##recorderview", s_view, text::Views, 3, views, 14.0f);
	}
	if (liveRow)
	{
		const FlexRect r = fl.Rect(liveRow);
		ui::DrawText(dl, ui::type::Callout, r.min, r.W() + 1.0f, ui::col::Muted, liveText.c_str());
	}
	if (transport)
	{
		const FlexRect r = fl.Rect(transport);
		const float midY = r.min.y + 36.0f;
		if (ui::RoundButton("##recorderplay", ImVec2(r.min.x + 16.0f + 22.0f, midY), 44.0f,
			s_playing ? ui::Icon::Pause : ui::Icon::Play, 18.0f, ui::col::Accent, ui::col::White))
		{
			// At the end, the button plays the recording again from its start.
			if (!s_playing && s_time >= summary.seconds)
				s_time = 0.0;
			s_playing = !s_playing;
		}
		// The widest this line gets, so the timeline keeps its place while the
		// time counts up.
		const ui::TextStyle timeStyle{ ui::Weight::SemiBold, 16.0f, 22.0f };
		const std::string length = Clock(summary.seconds);
		// The moment and the length, in digits: no language needs words here.
		const std::string timeText = Clock(s_time) + " / " + length;
		const std::string widest = length + " / " + length;
		const float timeX = r.min.x + 16.0f + 44.0f + 16.0f;
		const float timeW = ui::MeasureLine(timeStyle, widest.c_str()).x + 6.0f;
		ui::DrawLine(dl, timeStyle, ImVec2(timeX, midY - 11.0f), ui::col::Text, timeText.c_str());

		// The speed, at the row's end: a press takes the next one.
		const ui::TextStyle speedStyle{ ui::Weight::SemiBold, 14.0f, 34.0f };
		const float speedW = 56.0f;
		const ImVec2 speedMin(r.max.x - 20.0f - speedW, midY - 17.0f), speedMax(r.max.x - 20.0f, midY + 17.0f);
		ImGui::SetCursorScreenPos(speedMin);
		if (ImGui::InvisibleButton("##recorderspeed", ImVec2(speedW, 34.0f)))
			s_speed = (s_speed + 1) % static_cast<int>(IM_ARRAYSIZE(Speeds));
		ui::FillRounded(dl, speedMin, speedMax, ImGui::IsItemHovered() ? ui::col::Secondary : ui::Rgba(255, 255, 255, 0.10f), 17.0f);
		if (ImGui::IsItemFocused())
			ui::FocusRing(dl, speedMin, speedMax, 17.0f);
		const float labelW = ui::MeasureLine(speedStyle, SpeedLabels[s_speed]).x;
		ui::DrawLine(dl, speedStyle, ImVec2(speedMin.x + (speedW - labelW) * 0.5f, speedMin.y), ui::col::Text, SpeedLabels[s_speed]);

		FlexRect bar;
		bar.min = ImVec2(timeX + timeW + 16.0f, midY - 14.0f);
		bar.max = ImVec2(speedMin.x - 20.0f, midY + 14.0f);
		const double picked = Timeline(dl, bar, summary, s_time);
		if (picked >= 0.0)
			s_time = picked;
	}
	if (legend)
	{
		const FlexRect r = fl.Rect(legend);
		const ui::TextStyle style{ ui::Weight::Regular, 14.0f, 18.0f };
		const float y = r.min.y + 2.0f;
		float x = r.min.x + 76.0f;
		const auto item = [&](const char *english, int mark)
		{
			const float cy = y + 9.0f;
			if (mark == 0)
				ui::FillRounded(dl, ImVec2(x, cy - 3.0f), ImVec2(x + 18.0f, cy + 3.0f), ui::col::Link, 3.0f);
			else if (mark == 1)
				for (float sx = x + 1.0f; sx + 4.0f <= x + 18.0f; sx += 4.0f)
					dl->AddLine(ImVec2(sx, cy + 5.0f), ImVec2(sx + 3.0f, cy - 5.0f), ui::Rgba(236, 238, 244, 0.30f), 1.5f);
			else
				ui::FillRounded(dl, ImVec2(x + 7.0f, cy - 7.0f), ImVec2(x + 10.0f, cy + 7.0f), ui::col::Caution, 1.5f);
			const char *label = Tr(english);
			ui::DrawLine(dl, style, ImVec2(x + 26.0f, y), ui::col::Muted, label);
			x += 26.0f + ui::MeasureLine(style, label).x + 22.0f;
		};
		item(text::LegendRecorded, 0);
		if (summary.stretches.size() > 1)
			item(text::LegendOff, 1);
		if (!summary.losses.empty())
			item(text::LegendLost, 2);
		const std::string figure = Tr(FormatString(text::DevicesPattern, static_cast<int>(summary.devices),
			summary.devices == 1 ? "" : "s", static_cast<double>(summary.bytes) / static_cast<double>(1ull << 30)));
		const float w = ui::MeasureLine(style, figure.c_str()).x;
		if (r.max.x - 20.0f - w > x)
			ui::DrawLine(dl, style, ImVec2(r.max.x - 20.0f - w, y), ui::col::Muted, figure.c_str());
	}
	if (cutShort)
	{
		const FlexRect r = fl.Rect(cutShort);
		ui::DrawText(dl, ui::type::Caption, r.min, r.W() + 1.0f, ui::col::Caution, cutShortText.c_str());
	}
	if (noteIconNode)
	{
		ui::RoundIcon(dl, fl.Rect(noteIconNode).Center(), 56.0f, noteIcon, 26.0f, ui::col::Muted, ui::col::Fill);
		ui::DrawText(dl, ui::type::Label, ImVec2(card.min.x, fl.Rect(noteTitleNode).min.y), card.W(), ui::col::Text,
			noteTitle.c_str(), ui::Align::Center);
		if (noteBodyNode)
		{
			const FlexRect r = fl.Rect(noteBodyNode);
			ui::DrawText(dl, ui::type::Body, r.min, r.W() + 1.0f, ui::col::Muted, noteBody.c_str(), ui::Align::Center);
		}
		if (noteBar)
		{
			const FlexRect r = fl.Rect(noteBar);
			ui::FillRounded(dl, r.min, r.max, ui::col::Fill, 3.0f);
			ui::FillRounded(dl, r.min, ImVec2(r.min.x + r.W() * static_cast<float>(summary.progress), r.max.y), ui::col::Link, 3.0f);
			ui::KeepAnimating();
		}
	}

	const FlexRect controlsR = fl.Rect(controls);
	ui::Card(dl, controlsR.min, controlsR.max);
	const auto drawRow = [&](const ControlRow &row, ImU32 lineInk)
	{
		const FlexRect titleR = fl.Rect(row.title), lineR = fl.Rect(row.line);
		ui::DrawLine(dl, ui::TextStyle{ ui::type::RowTitle.weight, ui::type::RowTitle.size, 26.0f }, titleR.min, ui::col::Text,
			row.titleText.c_str());
		ui::DrawText(dl, ui::type::Footnote, lineR.min, lineR.W() + 1.0f, lineInk, row.lineText.c_str());
		const FlexRect rowR = fl.Rect(row.row);
		ui::Hairline(dl, rowR.min.x + 20.0f, rowR.max.x, rowR.max.y);
	};
	drawRow(record, live ? ui::col::Link : ui::col::Muted);
	{
		// The warning mark and the label beside the row's title.
		const FlexRect titleR = fl.Rect(record.title);
		const float x = titleR.min.x + ui::MeasureLine(ui::type::RowTitle, record.titleText.c_str()).x + 10.0f;
		ui::DrawIcon(dl, ui::Icon::Warn, ImVec2(x + 10.0f, titleR.Center().y), 20.0f, ui::col::Caution);
		ui::BadgePill(dl, ImVec2(x + 20.0f + 10.0f, titleR.min.y), text::PrivacyLabel, ui::col::Caution, ui::col::CautionTint);
	}
	// Start asks first when the recording is a new one; stop is at once.
	if (ui::PillButton("##recordmotion", recordButton, live ? ui::Btn::Secondary : ui::Btn::Primary, fl.Rect(record.control),
		ui::Icon::None, 17.0f) && !g_uiPreviewMode)
	{
		if (live)
			Pause();
		else if (BeginsNewRecording())
			OpenWarning();
		else
			Start();
		s_playing = false;
	}
	drawRow(cap, ui::col::Muted);
	{
		// The cap is picked before a recording, not during one.
		const int choice = CapChoice();
		const int picked = ui::PillSegmented("##recordingcap", choice, sizes, CapChoiceCount, fl.Rect(cap.control), 16.0f, false);
		if (picked != choice && !live)
			SetCapChoice(picked);
	}
	{
		const FlexRect r = fl.Rect(links);
		float advance = 0.0f;
		float x = r.min.x + 20.0f;
		if (CardLink(dl, "##recordingfolder", ImVec2(x, r.min.y + 3.0f), text::OpenFolder, ui::col::Link, advance))
			OpenRecordingFolder();
		x += advance + 28.0f;
		if (status.fileExists &&
			CardLink(dl, "##recordingdelete", ImVec2(x, r.min.y + 3.0f), text::DeleteRecording, ui::col::DangerInk, advance))
			OpenDeleteConfirm();
	}
	return YGNodeLayoutGetHeight(root);
}
