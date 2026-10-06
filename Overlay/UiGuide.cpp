// The calibration sheet: get set, move and check, with the motion demo, the
// live meters and the result. The guide owns presentation state;
// CalibrationContext owns the measurement.
#include "stdafx.h"
#include "UiInternal.h"

GuideState s_guide;
static GLuint s_guideTexture = 0;
static int s_guideTextureKind = -1;

// Whether the result shows its detail lines; OpenGuide seeds it from
// advanced mode.
bool s_modalDetails = false;

void OpenGuide(bool anchor, bool mountRun)
{
	if (!g_uiPreviewMode && CalCtx.state != CalibrationState::None) return;
	s_modalDetails = CalCtx.uiAdvanced;
	s_guide = GuideState();
	s_guide.anchor = anchor;
	s_guide.mountRun = mountRun;
	s_guide.stage = GuideStage::GetSet;
	CalCtx.ClearMessages();
	s_guide.openRequested = true;
	BOOL animate = TRUE;
	if (SystemParametersInfoW(SPI_GETCLIENTAREAANIMATION, 0, &animate, 0))
		s_guide.animate = animate != FALSE;
	if (!s_guide.animate)
		s_guide.animationTime = 3.3;
}

// Mount measurement uses the headset as reference and its tracker as target.
void StartMountSetup(const VRState &state)
{
	if (!g_uiPreviewMode && CalCtx.state != CalibrationState::None) return;
	const VRDevice *hmd = nullptr;
	const VRDevice *tracker = nullptr;
	for (const auto &d : state.devices)
	{
		if (d.deviceClass == vr::TrackedDeviceClass_HMD && !hmd)
			hmd = &d;
		if (!CalCtx.continuousTrackerSerial.empty() && d.serial == CalCtx.continuousTrackerSerial)
			tracker = &d;
	}
	if (!hmd)
	{
		CalCtx.ReportError("The headset isn't showing up in SteamVR, so the tracker position can't be measured yet.\n");
		return;
	}
	if (!tracker || !tracker->connected)
	{
		CalCtx.ReportError("The headset tracker isn't connected. Switch it on, then try again.\n");
		return;
	}
	CalCtx.pendingReferenceTrackingSystem = hmd->trackingSystem;
	CalCtx.referenceID = static_cast<uint32_t>(hmd->id);
	CalCtx.pendingTargetTrackingSystem = tracker->trackingSystem;
	CalCtx.targetID = static_cast<uint32_t>(tracker->id);
	g_page = Page::Calibration;
	OpenGuide(false, true);
}

// Preview mode simulates a run without a SteamVR connection.
bool BeginGuidedRun()
{
	s_guide.metrics = questcal::GuideMetrics();
	s_guide.lastMetricsTime = 0.0;
	if (g_uiPreviewMode)
	{
		CalCtx.ClearMessages();
		CalCtx.Log(FormatString("Preview run: reference device %u, target device %u\n",
			CalCtx.referenceID, CalCtx.targetID));
		if (s_guide.demo != GuideDemo::Wrist)
		{
			CalCtx.Instruct("Look around slowly.");
			CalCtx.Note("Look left and right, then up and down. Gently tilt your head to each side. Keep the tracker in view of the base stations.");
		}
		else
		{
			CalCtx.Instruct("Keep both devices firmly together.");
			CalCtx.Note("Draw slow figure eights in the air, turning and tilting your hands as you go.");
		}
		CalCtx.Progress(0, static_cast<int>(CalCtx.CollectionSeconds() * 100.0));
		s_guide.metrics.valid = true;
		s_guide.metrics.coverage = 0.45;
		s_guide.metrics.gatedFraction = 0.08;
		s_guide.metrics.rigidityValid = true;
		s_guide.metrics.rigidityDeg = 1.4;
		return true;
	}
	return s_guide.anchor ? StartAnchorCalibration() : StartCalibration();
}

void ChooseGuideDemo(const VRState &state)
{
	s_guide.demo = s_guide.mountRun ? GuideDemo::Mounted : GuideDemo::Wrist;
	bool headsetReference = false;
	bool controllerTarget = false;
	for (const auto &device : state.devices)
	{
		if (static_cast<uint32_t>(device.id) == CalCtx.referenceID)
			headsetReference = device.deviceClass == vr::TrackedDeviceClass_HMD;
		if (static_cast<uint32_t>(device.id) == CalCtx.targetID)
			controllerTarget = device.deviceClass == vr::TrackedDeviceClass_Controller;
	}
	if (!s_guide.mountRun && headsetReference)
		s_guide.demo = controllerTarget ? GuideDemo::HeadsetContact : GuideDemo::Mounted;
}

void ReleaseGuideTexture()
{
	if (s_guideTexture)
		glDeleteTextures(1, &s_guideTexture);
	s_guideTexture = 0;
	s_guideTextureKind = -1;
}

// One atlas is resident at a time (at most 112.5 MiB). Resources are
// embedded so a moved executable cannot lose its instructions.
void DrawGuideAnimation(ImDrawList *dl, const FlexRect &box, double t, GuideDemo demo, bool caption)
{
	const int kind = static_cast<int>(demo);
	if (kind != s_guideTextureKind)
	{
		ReleaseGuideTexture();
		LoadGuideTexture(demo, &s_guideTexture);
		s_guideTextureKind = kind;
	}
	if (!s_guideTexture)
	{
		ui::DrawText(dl, ui::type::Body, ImVec2(box.min.x + 24.0f, box.min.y + 24.0f), box.W() - 48.0f, ui::col::Caution,
			Tr("Motion demos couldn't load. Reinstall QuestCalibrator to restore them."));
		return;
	}
	const bool wrist = demo == GuideDemo::Wrist;
	const int columns = wrist ? 16 : 24;
	const float frameW = wrist ? 320.0f : 192.0f;
	const float frameH = wrist ? 192.0f : 168.0f;
	const float atlasW = wrist ? 5120.0f : 4608.0f;
	const float atlasH = wrist ? 5760.0f : 5880.0f;
	// The per-axis labels of the head demos always show: without them the
	// three cells look alike. The one-line caption is optional.
	const float captionH = caption || !wrist ? 30.0f : 0.0f;
	if (demo == GuideDemo::Mounted)
		t += 2.3;
	auto drawFrame = [&](int index, ImVec2 a, ImVec2 b, float opacity)
	{
		if (opacity <= 0.0f)
			return;
		const int column = index % columns;
		const int row = index / columns;
		// Half-texel inset keeps linear filtering inside this frame.
		const ImVec2 uv0((column * frameW + 0.5f) / atlasW, (row * frameH + 0.5f) / atlasH);
		const ImVec2 uv1((column * frameW + frameW - 0.5f) / atlasW, (row * frameH + frameH - 0.5f) / atlasH);
		dl->AddImage(static_cast<ImTextureID>(s_guideTexture), a, b, uv0, uv1,
			ImGui::ColorConvertFloat4ToU32(ImVec4(1, 1, 1, opacity)));
	};
	const ui::TextStyle captionStyle{ ui::Weight::Regular, 16.0f, captionH };
	const ImU32 captionInk = ui::Rgba(236, 238, 244, 0.82f);
	if (wrist)
	{
		const float imageH = std::min(box.H() - captionH - 6.0f, (box.W() - 40.0f) * 3.0f / 5.0f);
		const float imageW = imageH * 5.0f / 3.0f;
		const ImVec2 a(box.min.x + (box.W() - imageW) * 0.5f, box.min.y + std::max(0.0f, (box.H() - captionH - imageH) * 0.5f));
		const int frame = t < 2.0 ? std::min(119, static_cast<int>(t * 60.0))
			: 120 + static_cast<int>(std::fmod(t - 2.0, 6.0) * 60.0) % 360;
		drawFrame(frame, a, ImVec2(a.x + imageW, a.y + imageH), static_cast<float>(std::clamp(t / 0.2, 0.0, 1.0)));
		if (caption)
		{
			const char *label = Tr(t < 2.0 ? "Bring the controller to the wrist tracker" : "Turn and tilt as you move in a figure eight");
			ui::DrawText(dl, captionStyle, ImVec2(box.min.x, box.max.y - captionH - 4.0f), box.W(), captionInk, label, ui::Align::Center);
		}
		return;
	}
	const float blend = static_cast<float>(std::clamp((t - 2.0) / 0.3, 0.0, 1.0));
	const float motionAlpha = blend * blend * (3.0f - 2.0f * blend);
	const float gap = 16.0f;
	const float cellW = (box.W() - gap * 2.0f) / 3.0f;
	const float imageH = std::min(box.H() - captionH - 6.0f, cellW * 7.0f / 8.0f);
	const float imageW = imageH * 8.0f / 7.0f;
	const float top = box.min.y + std::max(0.0f, (box.H() - captionH - imageH) * 0.5f);
	if (motionAlpha < 1.0f)
	{
		const float opacity = (1.0f - motionAlpha) * static_cast<float>(std::clamp(t / 0.2, 0.0, 1.0));
		const ImVec2 a(box.min.x + (box.W() - imageW) * 0.5f, top);
		drawFrame(std::min(119, static_cast<int>(t * 60.0)), a, ImVec2(a.x + imageW, a.y + imageH), opacity);
		if (caption)
		{
			const char *label = Tr(demo == GuideDemo::HeadsetContact ? "Rest the controller against the visor" : "Bring the controller to the wrist tracker");
			ui::DrawText(dl, captionStyle, ImVec2(box.min.x, box.max.y - captionH - 4.0f), box.W(), ui::Fade(captionInk, opacity),
				label, ui::Align::Center);
		}
	}
	const char *headLabels[] = { "Look left and right", "Look up and down", "Tilt side to side" };
	const int frame = static_cast<int>(std::fmod(std::max(0.0, t - 2.3), 4.0) * 60.0) % 240;
	for (int axis = 0; axis < 3; ++axis)
	{
		const float x = box.min.x + (cellW + gap) * static_cast<float>(axis);
		const ImVec2 a(x + (cellW - imageW) * 0.5f, top);
		drawFrame(120 + axis * 240 + frame, a, ImVec2(a.x + imageW, a.y + imageH), motionAlpha);
		ui::DrawText(dl, captionStyle, ImVec2(x, box.max.y - captionH - 4.0f), cellW, ui::Fade(captionInk, motionAlpha),
			Tr(headLabels[axis]), ui::Align::Center);
	}
}

// ---------------------------------------------------------------------------
// The sheet
// ---------------------------------------------------------------------------

// Titles go without a closing full stop, the way the canvas sets them; the
// messages keep theirs for the log and the activity feed.
static std::string AsTitle(const char *text)
{
	std::string s(text ? text : "");
	if (s.size() >= 3 && s.compare(s.size() - 3, 3, "\xE3\x80\x82") == 0)
		s.erase(s.size() - 3);
	else if (!s.empty() && s.back() == '.' && (s.size() < 3 || s.compare(s.size() - 3, 3, "...") != 0))
		s.pop_back();
	return s;
}

static void CloseGuide()
{
	s_guide.stage = GuideStage::Idle;
	ReleaseGuideTexture();
	CloseSheet();
}

// The two picks, as found in the device list (either may be missing).
static void FindPicks(const VRState &state, const VRDevice *&reference, const VRDevice *&target)
{
	reference = target = nullptr;
	for (const auto &d : state.devices)
	{
		if (static_cast<uint32_t>(d.id) == CalCtx.referenceID)
			reference = &d;
		if (static_cast<uint32_t>(d.id) == CalCtx.targetID)
			target = &d;
	}
}

static std::string PickName(const VRDevice *device, bool reference)
{
	return device ? DeviceDisplayName(*device) : std::string(reference ? "Reference device" : "Target device");
}

// The header row: the steps, and the button that stops everything.
static bool StepperHeader(const FlexRect &c, int current, unsigned done, int failed)
{
	ui::Stepper(ImGui::GetWindowDrawList(), c.min, current, done, failed);
	return ui::CloseButton("##cancelguide", ImVec2(c.max.x - 44.0f, c.min.y));
}

// The demo's well and the two small buttons that control it.
static void DemoWell(ImDrawList *dl, const FlexRect &area, bool controls)
{
	ui::FillRounded(dl, area.min, area.max, ui::col::Well, 22.0f);
	if (!controls)
		return;
	const ImU32 fill = ui::Rgba(255, 255, 255, 0.12f), ink = ui::Rgba(236, 238, 244, 0.90f);
	const ImVec2 pause(area.max.x - 14.0f - 20.0f, area.max.y - 12.0f - 20.0f);
	const ImVec2 replay(pause.x - 48.0f, pause.y);
	if (ui::RoundButton("##replaydemo", replay, 40.0f, ui::Icon::Refresh, 18.0f, fill, ink))
	{
		s_guide.animationTime = 0.0;
		s_guide.animate = true;
	}
	if (ImGui::IsItemHovered())
		ShowTip("Replay the demo");
	if (ui::RoundButton("##pausedemo", pause, 40.0f, s_guide.animate ? ui::Icon::Pause : ui::Icon::Play, 16.0f, fill, ink))
		s_guide.animate = !s_guide.animate;
	if (ImGui::IsItemHovered())
		ShowTip(s_guide.animate ? "Pause the demo" : "Play the demo");
}

// A sheet stage's frame: header, title, lead, the flexible area and the
// footer. The texts are the caller's, translated.
struct StageLayout
{
	FlexLayout fl;
	YGNodeRef title = nullptr, lead = nullptr, area = nullptr, extra = nullptr, footer = nullptr;
};

static void LayOutStage(StageLayout &s, const FlexRect &c, const std::string &title, const std::string &lead,
	float areaMinH, float extraH)
{
	YGNodeRef root = s.fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, c.W());
	YGNodeStyleSetHeight(root, c.H());
	s.fl.Box(root, c.W(), 44.0f);
	s.title = ui::TextNode(s.fl, root, ui::type::SheetTitle, title);
	YGNodeStyleSetMargin(s.title, YGEdgeTop, 26.0f);
	if (!lead.empty())
	{
		s.lead = ui::TextNode(s.fl, root, ui::type::Lead, lead);
		YGNodeStyleSetMargin(s.lead, YGEdgeTop, 10.0f);
		YGNodeStyleSetMaxWidth(s.lead, 760.0f);
	}
	s.area = s.fl.Add(root);
	YGNodeStyleSetMargin(s.area, YGEdgeTop, 22.0f);
	YGNodeStyleSetFlexGrow(s.area, 1.0f);
	YGNodeStyleSetFlexShrink(s.area, 1.0f);
	YGNodeStyleSetMinHeight(s.area, areaMinH);
	if (extraH > 0.0f)
	{
		s.extra = s.fl.Box(root, c.W(), extraH);
		YGNodeStyleSetMargin(s.extra, YGEdgeTop, 16.0f);
	}
	s.footer = s.fl.Box(root, c.W(), 54.0f);
	YGNodeStyleSetMargin(s.footer, YGEdgeTop, 20.0f);
	s.fl.Compute(c.min, c.W(), c.H());
}

static void DrawStageTexts(const StageLayout &s, const std::string &title, const std::string &lead)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const FlexRect t = s.fl.Rect(s.title);
	ui::DrawText(dl, ui::type::SheetTitle, t.min, t.W(), ui::col::Text, title.c_str());
	if (s.lead)
	{
		const FlexRect l = s.fl.Rect(s.lead);
		ui::DrawText(dl, ui::type::Lead, l.min, l.W(), ui::col::Prose, lead.c_str());
	}
}

// Readiness chips along the footer's left, from at, within maxWidth: a long
// device name gives way before what the chip says about it.
static void PairChips(ImDrawList *dl, ImVec2 at, float maxWidth, const VRDevice *reference, const VRDevice *target,
	bool targetSettling)
{
	struct ChipText { const char *pattern; std::string name, shown, text; ImU32 dot; };
	ChipText chips[2];
	const VRDevice *picks[2] = { reference, target };
	for (int i = 0; i < 2; ++i)
	{
		ChipText &chip = chips[i];
		chip.name = chip.shown = PickName(picks[i], i == 0);
		const bool tracking = picks[i] && picks[i]->tracking;
		const bool settling = i == 1 && targetSettling && tracking;
		chip.pattern = settling ? "%s is settling" : tracking ? "%s is tracking" : "%s isn't tracking";
		chip.dot = tracking && !settling ? ui::col::Good : ui::col::Caution;
		chip.text = Tr(FormatString(chip.pattern, chip.shown.c_str()));
	}
	const float gap = 10.0f;
	const ui::TextStyle nameStyle = ui::type::CalloutMedium;
	for (int guard = 0; guard < 64; ++guard)
	{
		if (ui::ChipWidth(chips[0].text.c_str()) + gap + ui::ChipWidth(chips[1].text.c_str()) <= maxWidth)
			break;
		// The longer name loses a little at a time.
		const float w0 = ui::MeasureLine(nameStyle, chips[0].shown.c_str()).x;
		const float w1 = ui::MeasureLine(nameStyle, chips[1].shown.c_str()).x;
		ChipText &chip = w0 >= w1 ? chips[0] : chips[1];
		const float w = std::max(w0, w1);
		if (w < 48.0f)
			break;
		chip.shown = ui::Ellipsize(nameStyle, chip.name, w - 12.0f);
		chip.text = Tr(FormatString(chip.pattern, chip.shown.c_str()));
	}
	float x = at.x;
	for (const ChipText &chip : chips)
	{
		ui::Chip(dl, ImVec2(x, at.y), chip.dot, chip.text.c_str());
		x += ui::ChipWidth(chip.text.c_str()) + gap;
	}
}

static void GetSetStage(const VRState &state, const FlexRect &c, double now)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	if (StepperHeader(c, 0, 0u, -1) || (EscapePressed() && !DialogOpen()))
	{
		CloseGuide();
		return;
	}
	const bool contact = s_guide.demo == GuideDemo::HeadsetContact;
	const bool mounted = s_guide.demo == GuideDemo::Mounted;
	const std::string title = Tr(s_guide.anchor ? "Stand where the trackers look misaligned"
		: contact ? "Hold the controller against the visor"
		: mounted ? "Keep the tracker fixed to your headset"
		: "Hold the controller against the wrist tracker");
	const std::string lead = Tr(contact
		? FormatString("Hold the controller upright, with the trigger side against the front of your headset. Keep it in place as you turn and tilt your head for %.0f seconds.", CalCtx.CollectionSeconds())
		: mounted
		? FormatString("Move your head gently for %.0f seconds, keeping your body relaxed. Keep the tracker sensors uncovered.", CalCtx.CollectionSeconds())
		: FormatString("When you start, move both in slow figure eights for %.0f seconds, turning and tilting your hands as you go.", CalCtx.CollectionSeconds()));
	StageLayout s;
	LayOutStage(s, c, title, lead, 200.0f, 0.0f);
	DrawStageTexts(s, title, lead);
	const FlexRect area = s.fl.Rect(s.area);
	DemoWell(dl, area, true);
	FlexRect demo = area;
	demo.min.y += 12.0f;
	demo.max.y -= 10.0f;
	DrawGuideAnimation(dl, demo, s_guide.animationTime, s_guide.demo);

	const VRDevice *reference = nullptr, *target = nullptr;
	FindPicks(state, reference, target);
	const bool ready = reference && reference->tracking && target && target->tracking;
	const FlexRect footer = s.fl.Rect(s.footer);
	// The action first: the chips take what room it leaves. Labels stay
	// English here; the button translates its own.
	const bool counting = s_guide.stage == GuideStage::Countdown;
	const int remain = static_cast<int>(std::ceil(kCountdownSeconds - (now - s_guide.countdownStart)));
	const std::string label = counting ? FormatString("Starting in %d...", std::max(remain, 1))
		: std::string(ready ? "Start" : "Start anyway");
	const ui::Icon icon = counting ? ui::Icon::None : ui::Icon::Play;
	const float w = std::max(210.0f, ui::PillWidth(label.c_str(), 19.0f, icon));
	FlexRect action;
	action.min = ImVec2(footer.max.x - w, footer.min.y);
	action.max = footer.max;
	PairChips(dl, ImVec2(footer.min.x, footer.min.y + (footer.H() - ui::kChipH) * 0.5f), footer.W() - w - 16.0f,
		reference, target, false);
	if (counting)
	{
		ui::PillButton("##starting", label.c_str(), ui::Btn::Waiting, action, icon, 19.0f);
		return;
	}
	if (ui::PillButton("##start", label.c_str(), ready ? ui::Btn::Primary : ui::Btn::Secondary, action, icon, 19.0f))
	{
		s_guide.stage = GuideStage::Countdown;
		s_guide.countdownStart = now;
	}
}

// The ring that counts down the measurement.
static void CountdownRing(ImDrawList *dl, ImVec2 centre, float fractionLeft, int secondsLeft)
{
	const float r = 64.0f;
	dl->AddCircle(centre, r, ui::Rgba(255, 255, 255, 0.12f), 96, 10.0f);
	const float f = std::clamp(fractionLeft, 0.0f, 1.0f);
	if (f > 0.0f)
	{
		dl->PathArcTo(centre, r, -IM_PI * 0.5f, -IM_PI * 0.5f + IM_PI * 2.0f * f, 96);
		dl->PathStroke(ui::col::Link, 10.0f);
		dl->AddCircleFilled(ImVec2(centre.x, centre.y - r), 5.0f, ui::col::Link, 16);
		const float end = -IM_PI * 0.5f + IM_PI * 2.0f * f;
		dl->AddCircleFilled(ImVec2(centre.x + std::cos(end) * r, centre.y + std::sin(end) * r), 5.0f, ui::col::Link, 16);
	}
	const std::string number = std::to_string(std::max(secondsLeft, 0));
	const ui::TextStyle big{ ui::Weight::Bold, 48.0f, 52.0f };
	const float w = ui::MeasureLine(big, number.c_str()).x;
	ui::DrawLine(dl, big, ImVec2(centre.x - w * 0.5f, centre.y - 38.0f), ui::col::Text, number.c_str());
	ui::DrawText(dl, ui::type::Caption, ImVec2(centre.x - 70.0f, centre.y + 14.0f), 140.0f, ui::col::Muted,
		Tr("seconds left"), ui::Align::Center);
}

// The device being waited for, inside a spinning ring.
static void WaitingRing(ImDrawList *dl, ImVec2 centre, const VRDevice *device, double now)
{
	const float r = 80.0f;
	dl->AddCircleFilled(centre, r, ui::Rgba(255, 255, 255, 0.04f), 96);
	dl->AddCircle(centre, r, ui::Rgba(255, 255, 255, 0.10f), 96, 5.0f);
	const float start = static_cast<float>(std::fmod(now * 4.0, IM_PI * 2.0));
	dl->PathArcTo(centre, r, start, start + IM_PI * 0.5f, 32);
	dl->PathStroke(ui::col::Link, 5.0f);
	ui::KeepAnimating();
	if (device)
	{
		FlexRect box;
		box.min = ImVec2(centre.x - 58.0f, centre.y - 58.0f);
		box.max = ImVec2(centre.x + 58.0f, centre.y + 58.0f);
		DrawDeviceArt(dl, *device, box, ui::col::Muted);
	}
}

static void RunningStage(const VRState &state, const FlexRect &c, double now)
{
	using Msg = CalibrationContext::Message;
	ImDrawList *dl = ImGui::GetWindowDrawList();
	// The run waits in Begin until its devices track and settle
	// (CalibrationTick): it says what it waits for, and nothing counts down
	// until it measures.
	const bool waiting = CalCtx.state == CalibrationState::Begin && !CalCtx.run.waitInstruction.empty();
	if (g_uiPreviewMode && !waiting)
		CalCtx.Progress(static_cast<int>((now - s_guide.countdownStart - kCountdownSeconds) * 100.0),
			static_cast<int>(CalCtx.CollectionSeconds() * 100.0));
	// Live feedback from the run's own buffers, at 5 Hz.
	if (!g_uiPreviewMode && CalCtx.state == CalibrationState::Collecting && now - s_guide.lastMetricsTime >= 0.2)
	{
		s_guide.lastMetricsTime = now;
		s_guide.metrics = questcal::ComputeGuideMetrics(CalCtx.run.referenceSamples, CalCtx.run.targetSamples);
	}
	const bool cancel = StepperHeader(c, 1, 1u, -1) || (EscapePressed() && !DialogOpen());
	if (cancel)
	{
		if (g_uiPreviewMode)
			CloseGuide();
		else
			CancelCalibration();
		return;
	}

	std::string title, lead;
	if (waiting)
	{
		title = AsTitle(Tr(CalCtx.run.waitInstruction.c_str()));
		lead = Tr(CalCtx.run.waitNote.c_str());
	}
	else
	{
		for (const auto &message : CalCtx.messages)
		{
			if (message.kind == Msg::Instruction)
				title = AsTitle(Tr(message.str.c_str()));
			else if (message.kind == Msg::Info)
				lead = Tr(message.str.c_str());
		}
	}
	const VRDevice *reference = nullptr, *target = nullptr;
	FindPicks(state, reference, target);

	StageLayout s;
	LayOutStage(s, c, title, lead, waiting ? 220.0f : 180.0f, waiting ? 0.0f : 82.0f);
	DrawStageTexts(s, title, lead);
	const FlexRect area = s.fl.Rect(s.area);
	const FlexRect footer = s.fl.Rect(s.footer);
	FlexRect cancelR;
	cancelR.min = ImVec2(footer.max.x - 160.0f, footer.min.y);
	cancelR.max = footer.max;

	if (waiting)
	{
		DemoWell(dl, area, false);
		const ImVec2 centre(area.Center().x, area.min.y + area.H() * 0.5f - 22.0f);
		WaitingRing(dl, centre, target, now);
		const char *caption = Tr(target && target->tracking ? "Settling..." : "Waiting for tracking");
		ui::DrawText(dl, ui::type::Label, ImVec2(area.min.x, centre.y + 96.0f), area.W(), ui::col::Text, caption, ui::Align::Center);
		PairChips(dl, ImVec2(footer.min.x, footer.min.y + (footer.H() - ui::kChipH) * 0.5f), cancelR.min.x - footer.min.x - 16.0f,
			reference, target, true);
		if (ui::PillButton("##cancelrun", "Cancel", ui::Btn::Secondary, cancelR, ui::Icon::None, 19.0f))
		{
			if (g_uiPreviewMode)
				CloseGuide();
			else
				CancelCalibration();
		}
		return;
	}

	// Seconds left, from the run's progress (hundredths of a second).
	float fractionLeft = 1.0f;
	int secondsLeft = static_cast<int>(CalCtx.CollectionSeconds());
	for (const auto &message : CalCtx.messages)
	{
		if (message.kind != Msg::Progress || message.target <= 0)
			continue;
		fractionLeft = 1.0f - static_cast<float>(message.progress) / static_cast<float>(message.target);
		secondsLeft = static_cast<int>(std::ceil(std::max(0.0, (message.target - message.progress) / 100.0)));
	}
	DemoWell(dl, area, false);
	FlexRect demo = area;
	demo.max.x -= 210.0f;
	demo.min.y += 8.0f;
	demo.max.y -= 6.0f;
	// The title already says how to move; the caption would repeat it.
	DrawGuideAnimation(dl, demo, s_guide.animationTime, s_guide.demo, false);
	CountdownRing(dl, ImVec2(area.max.x - 120.0f, area.Center().y), fractionLeft, secondsLeft);

	// The meters: how varied, how fast and how steady the motion is.
	const questcal::GuideMetrics &m = s_guide.metrics;
	const FlexRect extra = s.fl.Rect(s.extra);
	const float meterW = (extra.W() - 24.0f) / 3.0f;
	const char *names[3] = { Tr("Range of motion"), Tr("Movement speed"), Tr(s_guide.mountRun ? "Tracker stability" : "Grip") };
	const char *states[3] = {
		!m.valid ? "Measuring..." : m.coverage >= 0.99 ? "Enough variety" : "Keep turning and tilting",
		!m.valid ? "Measuring..." : m.gatedFraction < 0.15 ? "Good pace" : "Move more slowly",
		!m.rigidityValid ? "Measuring..." : m.rigidityDeg < 3.0 ? "Moving together"
			: s_guide.mountRun ? "Tracker is shifting on the headset" : "Hold them tighter together"
	};
	const bool good[3] = { m.valid && m.coverage >= 0.99, m.valid && m.gatedFraction < 0.15, m.rigidityValid && m.rigidityDeg < 3.0 };
	const bool measured[3] = { m.valid, m.valid, m.rigidityValid };
	const double values[3] = { m.coverage, m.valid ? 1.0 - m.gatedFraction : 0.0,
		m.rigidityValid ? std::clamp(1.0 - (m.rigidityDeg - 1.0) / 8.0, 0.0, 1.0) : 0.0 };
	for (int i = 0; i < 3; ++i)
	{
		FlexRect r;
		r.min = ImVec2(extra.min.x + (meterW + 12.0f) * static_cast<float>(i), extra.min.y);
		r.max = ImVec2(r.min.x + meterW, extra.max.y);
		const ImU32 colour = !measured[i] ? ui::col::Quiet : good[i] ? ui::col::Good : ui::col::Caution;
		ui::Meter(dl, r, names[i], Tr(states[i]), colour, static_cast<float>(values[i]));
	}
	ui::DrawLine(dl, ui::TextStyle{ ui::Weight::Regular, 16.0f, footer.H() }, footer.min, ui::col::Muted,
		Tr("Keep holding until the timer ends."));
	if (ui::PillButton("##cancelrun", "Cancel", ui::Btn::Secondary, cancelR, ui::Icon::None, 19.0f))
	{
		if (g_uiPreviewMode)
			CloseGuide();
		else
			CancelCalibration();
	}
}

// The result's big round mark: a tick, or the alert's exclamation.
static void ResultMark(ImDrawList *dl, ImVec2 centre, bool passed)
{
	dl->AddCircleFilled(centre, 54.0f, passed ? ui::Rgba(61, 214, 140, 0.12f) : ui::Rgba(255, 107, 97, 0.12f), 96);
	dl->AddCircleFilled(centre, 36.0f, passed ? ui::col::GoodDeep : ui::col::AlertDeep, 64);
	const ImU32 white = ui::col::White;
	if (passed)
	{
		const ImVec2 pts[3] = { ImVec2(centre.x - 15.0f, centre.y + 0.5f), ImVec2(centre.x - 5.0f, centre.y + 10.5f),
			ImVec2(centre.x + 15.0f, centre.y - 10.0f) };
		dl->AddPolyline(pts, 3, white, 6.0f);
		for (const ImVec2 &p : pts)
			dl->AddCircleFilled(p, 3.0f, white, 12);
	}
	else
	{
		dl->AddLine(ImVec2(centre.x, centre.y - 16.0f), ImVec2(centre.x, centre.y + 4.0f), white, 6.0f);
		dl->AddCircleFilled(ImVec2(centre.x, centre.y - 16.0f), 3.0f, white, 12);
		dl->AddCircleFilled(ImVec2(centre.x, centre.y + 4.0f), 3.0f, white, 12);
		dl->AddCircleFilled(ImVec2(centre.x, centre.y + 15.0f), 3.8f, white, 16);
	}
}

static void ResultStage(const FlexRect &c)
{
	using Msg = CalibrationContext::Message;
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const bool passed = CalCtx.lastRunPassed;
	if (StepperHeader(c, passed ? 2 : -1, 3u, passed ? -1 : 2) || (EscapePressed() && !DialogOpen()))
	{
		CloseGuide();
		return;
	}
	// The outcome block starts at the last headline; the run's own
	// instruction and note before it are not part of the result. Detail
	// lines come from anywhere, behind the toggle.
	size_t outcomeStart = 0;
	for (size_t i = 0; i < CalCtx.messages.size(); ++i)
		if (CalCtx.messages[i].kind == Msg::Headline)
			outcomeStart = i;
	std::string headline;
	std::vector<std::string> body, actions, details;
	for (size_t i = 0; i < CalCtx.messages.size(); ++i)
	{
		const auto &message = CalCtx.messages[i];
		if (message.kind == Msg::Detail)
			details.push_back(message.str);
		else if (i == outcomeStart && message.kind == Msg::Headline)
			headline = AsTitle(Tr(message.str.c_str()));
		else if (i > outcomeStart && message.kind == Msg::Info)
			body.push_back(Tr(message.str.c_str()));
		else if (i > outcomeStart && message.kind == Msg::Action)
			actions.push_back(Tr(message.str.c_str()));
	}

	// The centred column, laid out in the space between header and footer.
	const float colW = std::min(c.W(), 720.0f);
	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetAlignItems(root, YGAlignCenter);
	YGNodeStyleSetWidth(root, colW);
	YGNodeRef mark = fl.Box(root, 112.0f, 112.0f);
	YGNodeRef title = ui::TextNode(fl, root, ui::type::ResultTitle, headline);
	YGNodeStyleSetMargin(title, YGEdgeTop, 20.0f);
	std::vector<YGNodeRef> bodyNodes, actionNodes;
	for (size_t i = 0; i < body.size(); ++i)
	{
		bodyNodes.push_back(ui::TextNode(fl, root, ui::type::Lead, body[i]));
		YGNodeStyleSetMargin(bodyNodes.back(), YGEdgeTop, i == 0 ? 8.0f : 4.0f);
	}
	for (const std::string &action : actions)
	{
		actionNodes.push_back(ui::TextNode(fl, root, ui::type::Body, action));
		YGNodeStyleSetMargin(actionNodes.back(), YGEdgeTop, 4.0f);
	}
	YGNodeRef detailBox = nullptr, detailText = nullptr;
	std::string detailJoined;
	for (const std::string &d : details)
		detailJoined += (detailJoined.empty() ? "" : "\n") + d;
	while (!detailJoined.empty() && (detailJoined.back() == '\n' || detailJoined.back() == '\r'))
		detailJoined.pop_back();
	if (!details.empty() && s_modalDetails)
	{
		detailBox = fl.Column(root);
		YGNodeStyleSetMargin(detailBox, YGEdgeTop, 18.0f);
		YGNodeStyleSetPadding(detailBox, YGEdgeVertical, 12.0f);
		YGNodeStyleSetPadding(detailBox, YGEdgeHorizontal, 18.0f);
		YGNodeStyleSetMaxWidth(detailBox, colW);
		detailText = ui::TextNode(fl, detailBox, ui::type::Footnote, detailJoined);
	}
	YGNodeRef toggle = nullptr;
	const char *toggleLabel = s_modalDetails ? "Hide details" : "Show details";
	if (!details.empty())
	{
		toggle = fl.Box(root, ui::PillWidth(toggleLabel, 17.0f, ui::Icon::None, 8.0f), 32.0f);
		YGNodeStyleSetMargin(toggle, YGEdgeTop, detailBox ? 8.0f : 14.0f);
	}
	fl.Compute(ImVec2(0.0f, 0.0f), colW, YGUndefined);
	const float colH = YGNodeLayoutGetHeight(root);
	const float bandTop = c.min.y + 44.0f, bandBottom = c.max.y - 54.0f - 12.0f;
	const ImVec2 origin(c.min.x + (c.W() - colW) * 0.5f, std::max(bandTop + 8.0f, bandTop + (bandBottom - bandTop - colH) * 0.5f));
	fl.Compute(origin, colW, YGUndefined);

	ResultMark(dl, fl.Rect(mark).Center(), passed);
	const FlexRect t = fl.Rect(title);
	ui::DrawText(dl, ui::type::ResultTitle, ImVec2(origin.x, t.min.y), colW, ui::col::Text, headline.c_str(), ui::Align::Center);
	for (size_t i = 0; i < body.size(); ++i)
		ui::DrawText(dl, ui::type::Lead, ImVec2(origin.x, fl.Rect(bodyNodes[i]).min.y), colW, ui::col::Prose,
			body[i].c_str(), ui::Align::Center);
	for (size_t i = 0; i < actions.size(); ++i)
		ui::DrawText(dl, ui::type::Body, ImVec2(origin.x, fl.Rect(actionNodes[i]).min.y), colW, ui::col::Lavender,
			actions[i].c_str(), ui::Align::Center);
	if (detailBox)
	{
		const FlexRect box = fl.Rect(detailBox);
		ui::FillRounded(dl, box.min, box.max, ui::col::Well, 14.0f);
		const FlexRect text = fl.Rect(detailText);
		ui::DrawText(dl, ui::type::Footnote, text.min, text.W(), ui::col::Muted, detailJoined.c_str());
	}
	if (toggle && ui::PillButton("##details", toggleLabel, ui::Btn::Link, fl.Rect(toggle), ui::Icon::None, 17.0f))
		s_modalDetails = !s_modalDetails;

	FlexRect footer;
	footer.min = ImVec2(c.min.x, c.max.y - 54.0f);
	footer.max = c.max;
	if (passed)
	{
		FlexRect done;
		done.min = ImVec2(footer.max.x - 210.0f, footer.min.y);
		done.max = footer.max;
		if (ui::PillButton("##done", "Done", ui::Btn::Primary, done, ui::Icon::None, 19.0f))
			CloseGuide();
		return;
	}
	// Back to get-set and its motion demo, not to a log.
	const float retryW = std::max(210.0f, ui::PillWidth("Try again", 19.0f, ui::Icon::Play));
	const float closeW = std::max(150.0f, ui::PillWidth("Close", 19.0f));
	FlexRect retry, close;
	retry.min = ImVec2(footer.max.x - retryW, footer.min.y);
	retry.max = footer.max;
	close.min = ImVec2(retry.min.x - 12.0f - closeW, footer.min.y);
	close.max = ImVec2(retry.min.x - 12.0f, footer.max.y);
	if (ui::PillButton("##closeresult", "Close", ui::Btn::Secondary, close, ui::Icon::None, 19.0f))
		CloseGuide();
	if (ui::PillButton("##retry", "Try again", ui::Btn::Primary, retry, ui::Icon::Play, 19.0f))
	{
		CalCtx.ClearMessages();
		s_guide.stage = GuideStage::GetSet;
	}
}

void BuildCalibrateSheet(const VRState &state)
{
	const ImGuiIO &io = ImGui::GetIO();
	const double now = ImGui::GetTime();
	if (s_guide.stage == GuideStage::Idle)
	{
		CloseGuide();
		return;
	}
	// Stage changes the context drives: a run that ended, any way, moves to
	// the outcome; a finished countdown starts the run.
	if (s_guide.stage == GuideStage::Running && !g_uiPreviewMode && CalCtx.state == CalibrationState::None)
		s_guide.stage = GuideStage::Done;
	if (s_guide.stage == GuideStage::Running && g_uiPreviewMode &&
		now - s_guide.countdownStart > kCountdownSeconds + CalCtx.CollectionSeconds())
	{
		// Preview: a fake outcome so the result stage can be styled, a refused
		// solve under -uipreview-failed and a success otherwise.
		if (g_uiPreviewScenario == PreviewScenario::Failed)
		{
			CalCtx.lastRunPassed = false;
			CalCtx.Outcome("Calibration failed", "The devices didn't turn far enough.",
				"Make bigger turns, and keep them pressed together.",
				"Rotation coverage 0.21 of 1.00 (need 0.60)", CalibrationContext::Tone::Warn);
		}
		else
		{
			CalCtx.lastRunPassed = true;
			CalCtx.Outcome("Calibration complete", "Check in VR that your trackers line up with your body.", "", "",
				CalibrationContext::Tone::Good);
		}
		s_guide.stage = GuideStage::Done;
	}
	if (s_guide.stage == GuideStage::Done)
		ReleaseGuideTexture();
	if (s_guide.stage == GuideStage::Countdown || s_guide.stage == GuideStage::Running ||
		(s_guide.stage == GuideStage::GetSet && s_guide.animate))
		ui::KeepAnimating();
	if (s_guide.stage == GuideStage::Countdown && now - s_guide.countdownStart >= kCountdownSeconds)
	{
		if (BeginGuidedRun())
			s_guide.stage = GuideStage::Running;
		else
		{
			// StartCalibration already said why, in the banner.
			CloseGuide();
			return;
		}
	}
	if (s_guide.animate)
		s_guide.animationTime += std::min(io.DeltaTime, 0.05f);

	const FlexRect c = SheetPanel(kCalibrateSheetW, kCalibrateSheetH);
	switch (s_guide.stage)
	{
	case GuideStage::GetSet:
	case GuideStage::Countdown:
		GetSetStage(state, c, now);
		break;
	case GuideStage::Running:
		RunningStage(state, c, now);
		break;
	case GuideStage::Done:
	default:
		ResultStage(c);
		break;
	}
}
