// Rating and continuous-status logic, and the home page: the alignment hero,
// notices, tools and recent activity.
#include "stdafx.h"
#include "UiInternal.h"
#include "QualityBands.h"

// The driver's pose channel feeds every runtime monitor. Preview mode has no
// driver at all, so it must not display a fault for it.
bool PoseChannelDown()
{
	return !g_uiPreviewMode && !CalCtx.poseRingOpen;
}

// ---------------------------------------------------------------------------
// Continuous-calibration status
// ---------------------------------------------------------------------------

ContinuousStatus ContinuousStatusNow()
{
	using CA = questcal::ContinuousAlignment;

	if (!CalCtx.continuousEnabled)
		return ContinuousStatus::Off;
	if (CalCtx.continuousTrackerSerial.empty())
		return ContinuousStatus::NoTracker;
	if (!CalCtx.ContinuousArmed())
		return ContinuousStatus::NeedsMount;

	// The rest of ContinuousTick's shouldRun conjunction, asked of the same
	// predicate ContinuousTick gates on so the two cannot drift apart (a copy
	// here once missed a pending frame recovery). The tracker id is re-resolved
	// by the driver sync and reset to invalid whenever that batch fails, so a
	// sleeping tracker or a dropped pipe lands here rather than looking like a
	// loop that is still warming up. Preview mode has no driver at all and so
	// can satisfy none of these (see PoseChannelDown).
	bool running = g_uiPreviewMode || CalCtx.ContinuousShouldRun();
	if (!running)
		return ContinuousStatus::NotRunning;

	// The tracker itself before the loop's window: a loop refilling after a
	// tracking loss would say it is warming up for as long as the tracker
	// stays away, and one whose tracker SteamVR switched off would say it
	// forever. Off comes first even over a freeze, whose recalibration needs
	// the tracker on; a freeze with the tracker merely unseen keeps its screen.
	if (!CalCtx.continuousTrackerConnected)
		return ContinuousStatus::TrackerOff;
	if (!CalCtx.continuousTrackerSeen && CalCtx.continuousState != CA::State::Frozen)
		return ContinuousStatus::Coasting;
	// The loop needs the headset's stream as much as the tracker's, and a
	// streamed headset pauses on its own; the tracker is not to blame then.
	if (!CalCtx.continuousHeadsetSeen && CalCtx.continuousState != CA::State::Frozen)
		return ContinuousStatus::HeadsetUnseen;

	switch (CalCtx.continuousState)
	{
	case CA::State::Tracking: return ContinuousStatus::Tracking;
	// Both streams arrive (the checks above), yet no reading formed: the
	// pairs failed the speed gates or the alignment. Nobody is missing.
	case CA::State::Coasting: return ContinuousStatus::Gathering;
	case CA::State::Frozen:   return ContinuousStatus::Frozen;
	case CA::State::Holding:  return ContinuousStatus::Holding;
	default:                  return ContinuousStatus::Gathering;
	}
}

// The state as one or two words, for the settings row where the feature's
// name is the row title and the setup button beneath says what to do.
const char *ContinuousStateWord(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Off:        return "Off";
	case ContinuousStatus::NoTracker:  return "Needs a tracker";
	case ContinuousStatus::NeedsMount: return "Needs setup";
	case ContinuousStatus::NotRunning: return "Waiting";
	case ContinuousStatus::TrackerOff: return "Tracker off";
	case ContinuousStatus::Tracking:   return "Active";
	case ContinuousStatus::Coasting:   return "Waiting";
	case ContinuousStatus::HeadsetUnseen: return "Waiting";
	case ContinuousStatus::Frozen:     return "Paused";
	case ContinuousStatus::Holding:    return "Waiting";
	default:                           return "Warming up";
	}
}

// The home screen's line about the loop, as a whole sentence so it reads
// (and translates) as one. Written for the question the player has ("will
// this fix itself?"): waiting states resolve on their own, paused ones name
// what they need. NotRunning covers a sleeping tracker, a disabled
// calibration and a closed pose channel alike, so it claims no cause.
const char *ContinuousStatusLine(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Off:        return "Continuous calibration is off.";
	case ContinuousStatus::NoTracker:  return "Continuous calibration needs a headset tracker. Pick one in Settings.";
	case ContinuousStatus::NeedsMount: return "Continuous calibration needs the headset tracker set up. Do it in Settings.";
	case ContinuousStatus::NotRunning: return "Continuous calibration is waiting. It resumes when tracking is available.";
	case ContinuousStatus::TrackerOff: return "Continuous calibration is waiting because the headset tracker is off. Turn it back on to resume.";
	case ContinuousStatus::Tracking:   return "Continuous calibration is active.";
	case ContinuousStatus::Coasting:   return "Continuous calibration is waiting. The headset tracker isn't being seen.";
	case ContinuousStatus::HeadsetUnseen: return "Continuous calibration is waiting. The headset isn't tracking.";
	case ContinuousStatus::Frozen:
		return CalCtx.continuousFreezeFromRestart
			? "Continuous calibration is paused after a tracker restart. Recovery may need another tracker restart."
			: "Continuous calibration is paused. Readings drifted too far to correct.";
	case ContinuousStatus::Holding:    return "Continuous calibration is waiting. It resumes when tracking settles.";
	default:                           return "Continuous calibration is warming up.";
	}
}

// Whether the loop is doing its job, paused for a reason that clears itself,
// or needs the player: the colour every rendering of the status shares.
ImVec4 ContinuousStatusColor(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Tracking:   return Pal::Good;
	case ContinuousStatus::Frozen:     return Pal::Bad;
	case ContinuousStatus::Off:        return Pal::Dim;
	default:                           return Pal::Warn;
	}
}

// ---------------------------------------------------------------------------
// Rating
// ---------------------------------------------------------------------------

CalRating ComputeCalibrationRating(ContinuousStatus continuous)
{
	int r = Rating_Good;

	// Solve quality caps the rating; staleness/drift then degrade it further.
	if (CalCtx.lastResult.valid)
	{
		const questcal::SolveQuality band = questcal::JudgeSolveQuality(
			CalCtx.lastResult.rotationRmsDeg, CalCtx.lastResult.translationRmsMeters);
		const int q = band == questcal::SolveQuality::Good ? Rating_Good
			: band == questcal::SolveQuality::Decent ? Rating_Decent : Rating_Poor;
		if (q > r)
			r = q;
	}

	// A live measurement outranks the solve's history: with the headset
	// tracker running, the deviation between what it sees and the calibration
	// is the actual misalignment right now, so a paused loop with a large
	// deviation reads as the misalignment it is instead of "Usable".
	if ((continuous == ContinuousStatus::Tracking || continuous == ContinuousStatus::Frozen) &&
		CalCtx.continuousDeviation.valid)
	{
		double yaw = CalCtx.continuousDeviation.yawDeg + CalCtx.continuousDeviation.tiltDeg;
		double pos = CalCtx.continuousDeviation.posM * 100.0;
		int q =
			(yaw <= 0.75 && pos <= 2.0) ? Rating_Good :
			(yaw <= 1.5 && pos <= 4.0) ? Rating_Decent :
			(yaw <= 3.0 && pos <= 8.0) ? Rating_Poor : Rating_VeryPoor;
		if (q > r)
			r = q;
	}

	// Without the pose channel the monitors that would demote this rating
	// cannot run at all: no jump compensation, no drift evidence, and the solve
	// itself fell back to tick-rate runtime poses. Never claim Good on that;
	// and with no solve this session either, nothing has been measured at all,
	// so say so rather than asserting "Usable" while SteamVR is still starting.
	if (PoseChannelDown())
	{
		if (!CalCtx.lastResult.valid)
			return Rating_Unknown;
		if (r < Rating_Decent)
			r = Rating_Decent;
	}

	// Staleness/drift only degrade the rating when nothing is maintaining the
	// alignment; a healthy continuous loop re-measures it constantly. A frozen
	// loop gets its own line and action instead: a calibration that solved
	// well is still good when the headset tracker gets nudged.
	bool continuouslyMaintained = continuous == ContinuousStatus::Tracking;
	if (!continuouslyMaintained)
	{
		if (CalCtx.alignment == CalibrationContext::AlignmentHealth::Aging && r < Rating_Decent)
			r = Rating_Decent;
		if (CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale && r < Rating_Poor)
			r = Rating_Poor;
		if (CalCtx.driftScore >= questcal::DriftVeryPoorScore)
			r = Rating_VeryPoor;
	}

	// Solve residuals are not persisted, so a restored profile has no evidence
	// behind the quality half of this verdict. With nothing demoting it, say
	// "Not measured" rather than Good, unless the continuous loop is measuring
	// live.
	const bool measured = CalCtx.lastResult.valid ||
		((continuous == ContinuousStatus::Tracking || continuous == ContinuousStatus::Frozen) &&
			CalCtx.continuousDeviation.valid);
	if (!measured && r == Rating_Good)
		return Rating_Unknown;

	return (CalRating)r;
}

// The ladder answers "should I do anything?", so the two bad rungs read as a
// verdict a player can act on rather than a grade.
static const char *RatingLabels[] = { "Good", "Usable", "Rough", "Bad" };

const char *RatingLabel(CalRating r)
{
	return r == Rating_Unknown ? "Not measured" : RatingLabels[r];
}

ImVec4 RatingColor(CalRating r)
{
	switch (r)
	{
	// Unknown claims nothing in either direction, so it gets the neutral ink.
	case Rating_Unknown: return Pal::Dim;
	case Rating_Good:    return Pal::Good;
	case Rating_Decent:  return Pal::Warn;
	case Rating_Poor:    return Pal::Bad;
	default:             return Pal::VeryBad;
	}
}

// The recalibration nudge, derived from the rating alone so every screen gives
// the same advice. Null when there is nothing to advise.
const char *RecalibrationNudge(CalRating rating)
{
	if (rating < Rating_Poor)
		return nullptr;
	return rating == Rating_VeryPoor
		? "Your trackers are noticeably off. Recalibrate to fix them."
		: "Recalibrate to tighten the alignment.";
}

// Empty when the timestamp (0 = unknown) or the clock is unusable; each caller
// words its own fallback.
std::optional<std::string> FormatUnixAge(double unixTime)
{
	double now = static_cast<double>(std::time(nullptr));
	if (unixTime <= 0.0 || now <= 0.0)
		return std::nullopt;

	double seconds = now - unixTime;
	// Small clock corrections must not read as a future timestamp.
	if (seconds < -300.0)
		return std::string("time is in the future");

	double hours = std::max(0.0, seconds) / 3600.0;
	if (seconds < 90.0)
		return std::string("just now");
	if (hours < 1.0)
		return FormatString("%d min ago", static_cast<int>(hours * 60.0));
	if (hours < 48.0)
		return FormatString("%.1f h ago", hours);
	return FormatString("%.0f days ago", hours / 24.0);
}

// The age of the alignment, from the same base UpdateDriftScore ages from:
// the later of the manual solve and the last auto-correction. Anything shown
// beside a score-derived verdict must use that base and say which one it is.
std::optional<std::string> FormatAlignmentAge()
{
	if (CalCtx.lastAutoCorrectionUnixTime > CalCtx.calibrationUnixTime)
	{
		if (auto adjusted = FormatUnixAge(CalCtx.lastAutoCorrectionUnixTime))
			return "adjusted " + *adjusted;
	}
	if (auto calibrated = FormatUnixAge(CalCtx.calibrationUnixTime))
		return "calibrated " + *calibrated;
	return std::nullopt;
}

// Snapshot the live chaperone and arm auto-restore (the "protect" action).
// In preview mode there is no VR system to read bounds from.
bool ProtectChaperone()
{
	if (g_uiPreviewMode)
		return true;
	return LoadChaperoneBounds();
}

// ---------------------------------------------------------------------------
// The hero: the alignment's verdict, what it means and what to do
// ---------------------------------------------------------------------------

namespace
{
	enum class HeroAction { Calibrate, Recalibrate, RecalibrateMount };

	struct Hero
	{
		ui::Mark mark = ui::Mark::None;
		std::string value;          // translated
		ImU32 valueColour = ui::col::Text;
		std::string description;    // translated
		std::string advice;         // translated; empty for none
		std::vector<std::string> details;   // advanced mode, English
		HeroAction action = HeroAction::Calibrate;
		bool primary = true;        // the action carries the accent
		bool lengthPicker = true;   // the 10/20/35 s choice beside it
		bool stopContinuous = false;
	};

	// Each disable reason wants a different action, so each gets its own
	// sentence.
	std::string DisabledSentence()
	{
		using Reason = CalibrationContext::DisableReason;
		switch (CalCtx.disableReason)
		{
		case Reason::HmdMismatch:
			return FormatString("%s headset isn't connected. Calibration is off until it's back.",
				FriendlySystemName(CalCtx.referenceTrackingSystem).c_str());
		case Reason::DriverUnreachable:
			return "SteamVR isn't accepting the calibration. Restart SteamVR.";
		case Reason::DriverVersionMismatch:
			return "The app and its SteamVR driver are from different releases. Reinstall QuestCalibrator, then restart SteamVR.";
		case Reason::DriverRefusedValues:
			return "The SteamVR driver refused the calibration's values. Recalibrate.";
		case Reason::InvalidIdentity:
			return "The saved calibration doesn't match the connected hardware. Recalibrate.";
		case Reason::InvalidTransform:
			return "The saved calibration is damaged. Recalibrate.";
		case Reason::UniverseUnsafe:
			return "The headset re-centered while QuestCalibrator wasn't watching, so the saved alignment is off. Recalibrate.";
		case Reason::FrameMovesLost:
			return "SteamVR moved the base stations more often than QuestCalibrator could follow, so the saved alignment may be off. Recalibrate.";
		case Reason::None:
		default:
			// Never borrow another cause's sentence: a universe change that
			// was not observed is not something to assert.
			return "Calibration disabled. Restart SteamVR or recalibrate.";
		}
	}

	// The measurements behind the verdict, for advanced mode: quiet lines
	// under the description rather than a second opinion.
	std::vector<std::string> AdvancedDetails(ContinuousStatus continuous)
	{
		std::vector<std::string> details;
		if (CalCtx.lastResult.valid)
		{
			details.push_back(FormatString("Solve: %.2f deg / %.1f cm, %+.1f ms%s",
				CalCtx.lastResult.rotationRmsDeg, CalCtx.lastResult.translationRmsMeters * 100.0,
				CalCtx.lastResult.timeOffset * 1000.0, CalCtx.lastResult.scale != 1.0 ? ", scaled" : ""));
			if (CalCtx.solveScale)
				details.push_back(FormatString("Scale: %s (condition %.4f, uncertainty %.4f)",
					CalCtx.lastResult.scaleIdentifiable ? "identifiable" : "insufficient",
					CalCtx.lastResult.scaleCondition, CalCtx.lastResult.scaleStdDev));
		}
		if (CalCtx.jumpsCompensated > 0 || CalCtx.referenceGapEvents > 0)
			details.push_back(FormatString("Jumps: %u compensated%s", CalCtx.jumpsCompensated,
				CalCtx.referenceGapEvents > 0 ? " (tracking gaps seen; recalibrate if alignment looks off)" : ""));
		{
			const char *health = CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale ? "stale" :
				CalCtx.alignment == CalibrationContext::AlignmentHealth::Aging ? "aging" : "fresh";
			std::string line = FormatString("Drift: %s", health);
			// Evidence only when there is some.
			if (CalCtx.driftSlideEvents > 0)
				line += FormatString("; %u slip%s up to %.1f cm while standing still",
					CalCtx.driftSlideEvents, CalCtx.driftSlideEvents == 1 ? "" : "s", CalCtx.driftMaxSlideM * 100.0);
			if (CalCtx.discontinuousLossEvents > 0)
				line += FormatString("; %u tracking dropout%s with a position change",
					CalCtx.discontinuousLossEvents, CalCtx.discontinuousLossEvents == 1 ? "" : "s");
			details.push_back(line);
		}
		// Only while the loop is running: a correction count under "needs
		// setup" contradicts it, and the count is reset when the tracker
		// changes.
		const bool loopRunning = continuous == ContinuousStatus::Tracking ||
			continuous == ContinuousStatus::Coasting || continuous == ContinuousStatus::Frozen ||
			continuous == ContinuousStatus::Holding || continuous == ContinuousStatus::TrackerOff ||
			continuous == ContinuousStatus::HeadsetUnseen;
		if (loopRunning)
		{
			if (continuous == ContinuousStatus::Tracking && CalCtx.continuousDeviation.valid)
				details.push_back(FormatString(
					"Upkeep: scatter %.2f deg / %.1f cm, deviation %.2f deg / %.1f cm, %u corrections",
					CalCtx.continuousScatterRotDeg, CalCtx.continuousScatterPosM * 100.0,
					CalCtx.continuousDeviation.yawDeg + CalCtx.continuousDeviation.tiltDeg,
					CalCtx.continuousDeviation.posM * 100.0, CalCtx.autoCorrectionsApplied));
			else
				details.push_back(FormatString("Upkeep: %u corrections", CalCtx.autoCorrectionsApplied));
		}
		return details;
	}

	Hero DescribeHero()
	{
		Hero hero;
		const ContinuousStatus continuous = ContinuousStatusNow();
		if (!CalCtx.validProfile)
		{
			hero.mark = ui::Mark::None;
			hero.value = Tr("Not calibrated");
			hero.description = Tr(FormatString(
				"Your trackers won't line up with your headset until you calibrate. It takes %.0f seconds.",
				CalCtx.CollectionSeconds()));
			return hero;
		}
		if (!CalCtx.enabled)
		{
			hero.mark = ui::Mark::Off;
			hero.value = Tr("Off");
			hero.valueColour = ui::Rgba(236, 238, 244, 0.82f);
			hero.description = Tr(DisabledSentence());
			hero.action = HeroAction::Recalibrate;
			hero.primary = false;
			return hero;
		}
		if (continuous == ContinuousStatus::Frozen)
		{
			hero.mark = ui::Mark::Paused;
			hero.value = Tr("Paused");
			hero.valueColour = ui::col::Caution;
			hero.description = Tr(CalCtx.continuousFreezeFromRestart
				? "Restart the headset tracker in view of its base stations. If stable tracking stays misaligned, recalibrate."
				: "Tracking has drifted away from your calibration. Recalibrate to bring your trackers back in line.");
			hero.action = HeroAction::RecalibrateMount;
			hero.lengthPicker = false;
			hero.stopContinuous = true;
			if (CalCtx.uiAdvanced && CalCtx.continuousDeviation.valid)
				hero.details.push_back(FormatString("Difference: %.1f deg yaw, %.1f deg tilt, %.1f cm position",
					CalCtx.continuousDeviation.yawDeg, CalCtx.continuousDeviation.tiltDeg,
					CalCtx.continuousDeviation.posM * 100.0));
			return hero;
		}
		const CalRating rating = ComputeCalibrationRating(continuous);
		hero.value = Tr(RatingLabel(rating));
		switch (rating)
		{
		case Rating_Good: hero.mark = ui::Mark::Good; hero.valueColour = ui::col::Good; break;
		case Rating_Decent: hero.mark = ui::Mark::Caution; hero.valueColour = ui::col::Caution; break;
		case Rating_Poor:
		case Rating_VeryPoor: hero.mark = ui::Mark::Bad; hero.valueColour = ui::col::Alert; break;
		default: hero.mark = ui::Mark::None; break;
		}
		// What keeps the alignment, or how old it is.
		if (continuous != ContinuousStatus::Off)
		{
			std::string line = ContinuousStatusLine(continuous);
			if (continuous == ContinuousStatus::Tracking && CalCtx.autoCorrectionsApplied > 0)
				if (auto adjusted = FormatUnixAge(CalCtx.lastAutoCorrectionUnixTime))
					line += " " + FormatString("Last adjusted %s.", adjusted->c_str());
			// Translated whole, so each language joins the sentences its own way.
			hero.description = Tr(line);
		}
		else if (CalCtx.lastAutoCorrectionUnixTime > CalCtx.calibrationUnixTime && FormatUnixAge(CalCtx.lastAutoCorrectionUnixTime))
			hero.description = Tr(FormatString("Adjusted %s.", FormatUnixAge(CalCtx.lastAutoCorrectionUnixTime)->c_str()));
		else if (auto calibrated = FormatUnixAge(CalCtx.calibrationUnixTime))
			hero.description = Tr(FormatString("Calibrated %s.", calibrated->c_str()));
		if (const char *nudge = RecalibrationNudge(rating))
			hero.advice = Tr(nudge);
		hero.action = HeroAction::Recalibrate;
		hero.primary = rating >= Rating_Poor;
		if (CalCtx.uiAdvanced)
			hero.details = AdvancedDetails(continuous);
		return hero;
	}

	// A notice above the tools: an icon, what is going on, and one action.
	struct Notice
	{
		ui::Icon icon;
		ImU32 ink, fill;
		std::string title;     // translated; may be empty
		std::string text;      // translated
		const char *button = nullptr;   // English
		ui::Icon buttonIcon = ui::Icon::None;
		int id = 0;
	};

	enum NoticeId { NoticeUpdate = 1, NoticeCalibrating };
}

float BuildHomePage(const VRState &state, ImVec2 origin, float width)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	const Hero hero = DescribeHero();
	const questcal::update::Snapshot update = CurrentUpdate();

	std::vector<Notice> notices;
	if (CalCtx.state == CalibrationState::Begin || CalCtx.state == CalibrationState::Neutralizing ||
		CalCtx.state == CalibrationState::Collecting)
	{
		// The calibration sheet normally covers this; if it ever does not,
		// the same two facts are here: what is happening and how to stop it.
		if (CurrentSheet() != Sheet::Calibrate)
			notices.push_back({ ui::Icon::Reticle, ui::col::Link, ui::col::LinkTint, std::string(), Tr("Calibrating..."),
				"Cancel", ui::Icon::None, NoticeCalibrating });
	}
	if (update.state == questcal::update::State::Ready)
		notices.push_back({ ui::Icon::Download, ui::col::Good, ui::col::GoodTint,
			Tr(FormatString("Nova Calibrator %s is ready", update.version.c_str())),
			Tr("Downloaded and verified. Close Steam before you install it."), "Install", ui::Icon::Download, NoticeUpdate });
	if (PoseChannelDown())
	{
		std::string why = Tr("QuestCalibrator isn't getting tracking data from SteamVR, so it can't watch for drift. Restart SteamVR.");
		if (CalCtx.uiAdvanced)
			why += FormatString(" (host hooks: 005 %s, 006 %s)",
				CalCtx.driverPoseHookMask & protocol::PoseHook005 ? "active" : "missing",
				CalCtx.driverPoseHookMask & protocol::PoseHook006 ? "active" : "missing");
		notices.push_back({ ui::Icon::Warn, ui::col::Caution, ui::col::CautionTint, std::string(), why });
	}
	if (CalCtx.enabled && CalCtx.hookBypassingDevices != 0)
	{
		std::string why = Tr("Calibration can't move some target devices, because their tracking doesn't pass through QuestCalibrator's SteamVR driver.");
		if (CalCtx.uiAdvanced)
		{
			std::string ids;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
				if (CalCtx.hookBypassingDevices & (uint64_t{ 1 } << id))
					ids += (ids.empty() ? "" : ", ") + std::to_string(id);
			why += FormatString(" (OpenVR devices %s)", ids.c_str());
		}
		notices.push_back({ ui::Icon::Warn, ui::col::Caution, ui::col::CautionTint, std::string(), why });
	}

	// ---- Layout ----
	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionColumn);
	YGNodeStyleSetWidth(root, width);
	YGNodeStyleSetPadding(root, YGEdgeHorizontal, 40.0f);
	YGNodeRef title = ui::TextNode(fl, root, ui::type::PageTitle, Tr("Calibration"));

	YGNodeRef heroRow = fl.Row(root);
	YGNodeStyleSetMargin(heroRow, YGEdgeTop, 30.0f);
	YGNodeStyleSetAlignItems(heroRow, YGAlignCenter);
	YGNodeStyleSetGap(heroRow, YGGutterColumn, 30.0f);
	YGNodeRef mark = fl.Box(heroRow, 136.0f, 136.0f);
	YGNodeRef heroText = fl.Column(heroRow);
	YGNodeStyleSetFlexGrow(heroText, 1.0f);
	YGNodeStyleSetFlexShrink(heroText, 1.0f);
	YGNodeRef eyebrow = ui::TextNode(fl, heroText, ui::type::Eyebrow, Tr("Alignment"));
	YGNodeRef value = ui::TextNode(fl, heroText, ui::type::Display, hero.value);
	YGNodeRef description = ui::TextNode(fl, heroText, ui::type::HeroBody, hero.description);
	YGNodeStyleSetMargin(description, YGEdgeTop, 4.0f);
	YGNodeStyleSetMaxWidth(description, 600.0f);
	YGNodeRef advice = nullptr;
	if (!hero.advice.empty())
	{
		advice = ui::TextNode(fl, heroText, ui::type::Body, hero.advice);
		YGNodeStyleSetMargin(advice, YGEdgeTop, 4.0f);
	}
	std::vector<YGNodeRef> detailNodes;
	for (size_t i = 0; i < hero.details.size(); ++i)
	{
		detailNodes.push_back(ui::TextNode(fl, heroText, ui::type::Caption, Tr(hero.details[i].c_str())));
		YGNodeStyleSetMargin(detailNodes.back(), YGEdgeTop, i == 0 ? 6.0f : 0.0f);
	}
	YGNodeRef actions = fl.Row(heroText);
	YGNodeStyleSetMargin(actions, YGEdgeTop, 18.0f);
	YGNodeStyleSetAlignItems(actions, YGAlignCenter);
	YGNodeStyleSetGap(actions, YGGutterColumn, 12.0f);
	const char *actionLabel = hero.action == HeroAction::Calibrate ? "Calibrate" : "Recalibrate";
	YGNodeRef actionButton = fl.Box(actions, ui::PillWidth(actionLabel, 18.0f, ui::Icon::Play), 48.0f);
	YGNodeRef stopButton = hero.stopContinuous
		? fl.Box(actions, ui::PillWidth("Turn off continuous calibration", 18.0f), 48.0f) : nullptr;
	std::string lengthLabels[3];
	const char *lengths[3];
	for (int i = 0; i < 3; ++i)
	{
		// Labelled by length: "Slow" reads as an instruction to move slowly.
		lengthLabels[i] = FormatString("%.0f s", CalibrationContext::CollectionSecondsFor(static_cast<CalibrationContext::Speed>(i)));
		lengths[i] = lengthLabels[i].c_str();
	}
	YGNodeRef length = hero.lengthPicker ? fl.Box(actions, ui::PillSegmentedWidth(lengths, 3, 16.0f, 74.0f), 48.0f) : nullptr;
	YGNodeRef more = fl.Box(actions, 48.0f, 48.0f);

	std::vector<YGNodeRef> noticeNodes, noticeTitles, noticeTexts, noticeButtons;
	if (!notices.empty())
	{
		YGNodeRef list = fl.Column(root);
		YGNodeStyleSetMargin(list, YGEdgeTop, 30.0f);
		YGNodeStyleSetGap(list, YGGutterRow, 12.0f);
		for (const Notice &n : notices)
		{
			YGNodeRef row = fl.Row(list);
			YGNodeStyleSetMinHeight(row, 84.0f);
			YGNodeStyleSetPadding(row, YGEdgeVertical, 14.0f);
			YGNodeStyleSetPadding(row, YGEdgeHorizontal, 18.0f);
			YGNodeStyleSetAlignItems(row, YGAlignCenter);
			YGNodeStyleSetGap(row, YGGutterColumn, 16.0f);
			fl.Box(row, 44.0f, 44.0f);
			YGNodeRef text = fl.Column(row);
			YGNodeStyleSetFlexGrow(text, 1.0f);
			YGNodeStyleSetFlexShrink(text, 1.0f);
			noticeTitles.push_back(n.title.empty() ? nullptr : ui::TextNode(fl, text, ui::type::Label, n.title));
			noticeTexts.push_back(ui::TextNode(fl, text, n.title.empty() ? ui::type::Body : ui::type::Footnote, n.text));
			noticeButtons.push_back(n.button ? fl.Box(row, ui::PillWidth(n.button, 17.0f, n.buttonIcon), 44.0f) : nullptr);
			noticeNodes.push_back(row);
		}
	}

	YGNodeRef tools = fl.Column(root);
	YGNodeStyleSetMargin(tools, YGEdgeTop, notices.empty() ? 34.0f : 30.0f);
	YGNodeRef toolsTitle = ui::TextNode(fl, tools, ui::type::Section, Tr("Tools"));
	YGNodeRef tiles = fl.Row(tools);
	YGNodeStyleSetMargin(tiles, YGEdgeTop, 10.0f);
	YGNodeStyleSetGap(tiles, YGGutterColumn, 14.0f);
	YGNodeRef tileNodes[3];
	for (YGNodeRef &tile : tileNodes)
	{
		tile = fl.Add(tiles);
		YGNodeStyleSetFlexGrow(tile, 1.0f);
		YGNodeStyleSetFlexBasis(tile, 0.0f);
		YGNodeStyleSetHeight(tile, 128.0f);
	}

	// Recent activity, newest first.
	const size_t recentCount = std::min<size_t>(CalCtx.activity.size(), 3);
	YGNodeRef recent = fl.Column(root);
	YGNodeStyleSetMargin(recent, YGEdgeTop, 26.0f);
	YGNodeRef recentHead = fl.Row(recent);
	YGNodeStyleSetJustifyContent(recentHead, YGJustifySpaceBetween);
	YGNodeStyleSetAlignItems(recentHead, YGAlignCenter);
	YGNodeRef recentTitle = ui::TextNode(fl, recentHead, ui::type::Section, Tr("Recent activity"));
	YGNodeRef showAll = recentCount > 0 ? fl.Box(recentHead, ui::PillWidth("Show all", 17.0f, ui::Icon::None, 6.0f), 32.0f) : nullptr;
	YGNodeRef recentCard = fl.Column(recent);
	YGNodeStyleSetMargin(recentCard, YGEdgeTop, 10.0f);
	std::vector<YGNodeRef> entryRows, entryTexts;
	std::vector<const CalibrationContext::ActivityEntry *> entries;
	const std::vector<size_t> order = ActivityNewestFirst();
	for (size_t i = 0; i < recentCount; ++i)
	{
		const auto &entry = CalCtx.activity[order[i]];
		YGNodeRef row = fl.Row(recentCard);
		YGNodeStyleSetMinHeight(row, 54.0f);
		YGNodeStyleSetPadding(row, YGEdgeVertical, 8.0f);
		YGNodeStyleSetPadding(row, YGEdgeHorizontal, 18.0f);
		YGNodeStyleSetAlignItems(row, YGAlignCenter);
		YGNodeStyleSetGap(row, YGGutterColumn, 18.0f);
		fl.Box(row, 48.0f, 20.0f);
		YGNodeRef text = ui::TextNode(fl, row, ui::type::Body, Tr(entry.text.c_str()));
		YGNodeStyleSetFlexGrow(text, 1.0f);
		YGNodeStyleSetFlexShrink(text, 1.0f);
		entryRows.push_back(row);
		entryTexts.push_back(text);
		entries.push_back(&entry);
	}
	YGNodeRef emptyRecent = recentCount == 0 ? fl.Box(recentCard, width - 80.0f, 96.0f) : nullptr;

	fl.Compute(origin, width, YGUndefined);

	// ---- Drawing ----
	ui::DrawLine(dl, ui::type::PageTitle, fl.Rect(title).min, ui::col::Text, Tr("Calibration"));
	ui::StateMark(dl, fl.Rect(mark).Center(), hero.mark);
	ui::DrawLine(dl, ui::type::Eyebrow, fl.Rect(eyebrow).min, ui::col::Muted, Tr("Alignment"));
	ui::DrawText(dl, ui::type::Display, fl.Rect(value).min, fl.Rect(value).W(), hero.valueColour, hero.value.c_str());
	ui::DrawText(dl, ui::type::HeroBody, fl.Rect(description).min, fl.Rect(description).W(), ui::col::Body, hero.description.c_str());
	if (advice)
		ui::DrawText(dl, ui::type::Body, fl.Rect(advice).min, fl.Rect(advice).W(), ui::col::Lavender, hero.advice.c_str());
	for (size_t i = 0; i < detailNodes.size(); ++i)
		ui::DrawText(dl, ui::type::Caption, fl.Rect(detailNodes[i]).min, fl.Rect(detailNodes[i]).W(), ui::col::Quiet,
			Tr(hero.details[i].c_str()));

	if (ui::PillButton("##heroaction", actionLabel, hero.primary ? ui::Btn::Primary : ui::Btn::Secondary,
		fl.Rect(actionButton), ui::Icon::Play))
	{
		if (hero.action == HeroAction::RecalibrateMount)
			StartMountSetup(state);
		else
			OpenGuide(false, false);
	}
	if (stopButton && ui::PillButton("##stopcontinuous", "Turn off continuous calibration", ui::Btn::Secondary, fl.Rect(stopButton)))
		SaveProfileFieldEdit(CalCtx, [](questcal::ProfileRecord &candidate) {
			candidate.continuousEnabled = false;
		});
	if (length)
	{
		const FlexRect r = fl.Rect(length);
		const auto previous = CalCtx.calibrationSpeed;
		CalCtx.calibrationSpeed = static_cast<CalibrationContext::Speed>(
			ui::PillSegmented("##length", static_cast<int>(CalCtx.calibrationSpeed), lengths, 3, r, 16.0f));
		if (CalCtx.calibrationSpeed != previous)
			SaveSettingOrRestore(CalCtx.calibrationSpeed, previous);
		if (ImGui::IsMouseHoveringRect(r.min, r.max) && !ImGui::IsAnyItemActive())
			ShowTip("How long calibration collects tracking data. Longer can be more accurate.\nMove gently at every setting.");
	}
	{
		const FlexRect r = fl.Rect(more);
		if (ui::RoundButton("##pair", r.Center(), 48.0f, ui::Icon::More, 20.0f, ui::Rgba(255, 255, 255, 0.07f),
			ui::Rgba(236, 238, 244, 0.85f)))
			OpenSheet(Sheet::Pair);
		if (ImGui::IsItemHovered())
			ShowTip("Choose the pair to calibrate");
	}

	for (size_t i = 0; i < notices.size(); ++i)
	{
		const Notice &n = notices[i];
		const FlexRect r = fl.Rect(noticeNodes[i]);
		ui::Card(dl, r.min, r.max);
		ui::RoundIcon(dl, ImVec2(r.min.x + 18.0f + 22.0f, r.Center().y), 44.0f, n.icon, 22.0f, n.ink, n.fill);
		if (noticeTitles[i])
			ui::DrawText(dl, ui::type::Label, fl.Rect(noticeTitles[i]).min, fl.Rect(noticeTitles[i]).W(), ui::col::Text, n.title.c_str());
		const ui::TextStyle &style = n.title.empty() ? ui::type::Body : ui::type::Footnote;
		ui::DrawText(dl, style, fl.Rect(noticeTexts[i]).min, fl.Rect(noticeTexts[i]).W(),
			n.title.empty() ? ui::col::Text : ui::col::Muted, n.text.c_str());
		if (noticeButtons[i])
		{
			ImGui::PushID(n.id);
			const bool pressed = ui::PillButton("##notice", n.button, n.id == NoticeUpdate ? ui::Btn::Primary : ui::Btn::Secondary,
				fl.Rect(noticeButtons[i]), n.buttonIcon, 17.0f);
			ImGui::PopID();
			if (pressed && n.id == NoticeUpdate)
			{
				std::string error;
				if (questcal::update::AppUpdater.LaunchInstaller(error))
					RequestApplicationExit();
				else
					CalCtx.ReportError(error + "\n");
			}
			else if (pressed && n.id == NoticeCalibrating)
				CancelCalibration();
		}
	}

	ui::DrawLine(dl, ui::type::Section, fl.Rect(toolsTitle).min, ui::col::Text, Tr("Tools"));
	if (ui::ToolTile("##identify", fl.Rect(tileNodes[0]), ui::Icon::Identify, "Identify the pair", Tr("Makes both vibrate or blink"),
		false, false))
		StartIdentifyPulse(CalCtx.targetID, CalCtx.referenceID);
	if (ImGui::IsItemHovered())
		ShowTip("Vibrates or blinks the two selected devices so you can tell which is which.");
	{
		std::string sub;
		if (CalCtx.chaperone.valid)
		{
			auto age = FormatUnixAge(CalCtx.chaperone.copyUnixTime);
			sub = age ? Tr(FormatString("Protected %s", age->c_str())) : std::string(Tr("Protected"));
		}
		else
			sub = Tr("Not protected yet");
		if (ui::ToolTile("##chaperone", fl.Rect(tileNodes[1]), ui::Icon::Chaperone, "Chaperone", sub.c_str(), true, false))
			OpenSheet(Sheet::Chaperone);
	}
	{
		const bool anchorsAvailable = CalCtx.validProfile && !CalCtx.profileUniverseUnsafe;
		const size_t count = CalCtx.fieldAnchors.size();
		const std::string sub = !anchorsAvailable ? std::string(Tr("Calibrate first"))
			: count == 0 ? std::string(Tr("No anchors yet"))
			: Tr(FormatString("%zu anchor%s saved", count, count == 1 ? "" : "s"));
		if (ui::ToolTile("##anchors", fl.Rect(tileNodes[2]), ui::Icon::Anchor, "Field anchors", sub.c_str(), anchorsAvailable,
			!anchorsAvailable))
			OpenSheet(Sheet::Anchors);
	}

	ui::DrawLine(dl, ui::type::Section, fl.Rect(recentTitle).min, ui::col::Text, Tr("Recent activity"));
	if (showAll && ui::PillButton("##showall", "Show all", ui::Btn::Link, fl.Rect(showAll), ui::Icon::None, 17.0f))
		OpenSheet(Sheet::Activity);
	const FlexRect card = fl.Rect(recentCard);
	ui::Card(dl, card.min, card.max);
	for (size_t i = 0; i < entryRows.size(); ++i)
	{
		const FlexRect r = fl.Rect(entryRows[i]);
		if (i > 0)
			ui::Hairline(dl, r.min.x + 84.0f, r.max.x, r.min.y);
		ui::DrawLine(dl, ui::TextStyle{ ui::Weight::Regular, 15.0f, r.H() }, ImVec2(r.min.x + 18.0f, r.min.y), ui::col::Quiet,
			ActivityClock(entries[i]->unixTime).c_str());
		const FlexRect t = fl.Rect(entryTexts[i]);
		ui::DrawText(dl, ui::type::Body, t.min, t.W(), ui::col::Text, Tr(entries[i]->text.c_str()));
	}
	if (emptyRecent)
	{
		const FlexRect r = fl.Rect(emptyRecent);
		ui::DrawText(dl, ui::type::Body, ImVec2(r.min.x, r.min.y + 37.0f), r.W(), ui::col::Muted,
			Tr("Nothing yet. Calibrations and corrections will appear here."), ui::Align::Center);
	}
	return YGNodeLayoutGetHeight(root);
}
