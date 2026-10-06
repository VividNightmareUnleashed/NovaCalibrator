// The design system's controls (UiKit.h): buttons, the switch, the segmented
// pill, chips, badges, sidebar rows, tool tiles, the stepper and meters.
#include "stdafx.h"
#include "UiInternal.h"

namespace ui
{
	void KeepAnimating()
	{
		CalCtx.wantedUpdateInterval = std::min(CalCtx.wantedUpdateInterval, 1.0 / 60.0);
	}

	// An eased value kept in the window's storage under key: it moves toward
	// target with a time constant of seconds and asks for frames until it
	// arrives. A value seen for the first time starts at its target, so a
	// screenshot never catches a control mid-slide.
	static float Ease(ImGuiID key, float target, float seconds)
	{
		ImGuiStorage *storage = ImGui::GetStateStorage();
		float v = storage->GetFloat(key, target);
		const float k = 1.0f - std::exp(-ImGui::GetIO().DeltaTime / seconds);
		v += (target - v) * k;
		if (std::fabs(target - v) < 0.002f)
			v = target;
		else
			KeepAnimating();
		storage->SetFloat(key, v);
		return v;
	}

	void FocusRing(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding)
	{
		if (ImGui::IsItemFocused() && ImGui::GetIO().NavVisible)
			dl->AddRect(ImVec2(a.x - 3.0f, a.y - 3.0f), ImVec2(b.x + 3.0f, b.y + 3.0f), col::Link, rounding + 3.0f,
				ImDrawFlags_RoundCornersAll, 2.0f);
	}

	// The 1 px light along a pill's top inner edge (CSS "inset 0 1px 0").
	static void TopLight(ImDrawList *dl, const FlexRect &r, float rounding, ImU32 colour)
	{
		const float rad = std::min(rounding, r.H() * 0.5f) - 0.5f;
		dl->PathArcTo(ImVec2(r.min.x + rad + 0.5f, r.min.y + rad + 0.5f), rad, IM_PI * 1.15f, IM_PI * 1.5f, 8);
		dl->PathArcTo(ImVec2(r.max.x - rad - 0.5f, r.min.y + rad + 0.5f), rad, IM_PI * 1.5f, IM_PI * 1.85f, 8);
		dl->PathStroke(colour, 1.0f);
	}

	static float IconSizeFor(Icon icon, float fontSize)
	{
		return icon == Icon::Play ? fontSize * 0.85f : fontSize;
	}

	float PillWidth(const char *english, float fontSize, Icon icon, float padX)
	{
		const TextStyle style{ Weight::SemiBold, fontSize, fontSize };
		float w = MeasureLine(style, Tr(english)).x + padX * 2.0f;
		if (icon != Icon::None)
			w += IconSizeFor(icon, fontSize) + 10.0f;
		return std::ceil(w);
	}

	bool PillButton(const char *id, const char *english, Btn kind, const FlexRect &r, Icon icon, float fontSize)
	{
		const char *label = Tr(english);
		ImGui::SetCursorScreenPos(r.min);
		const bool clicked = ImGui::InvisibleButton(id, r.Size(), ImGuiButtonFlags_EnableNav);
		const bool hov = ImGui::IsItemHovered() && kind != Btn::Waiting;
		const bool act = ImGui::IsItemActive() && kind != Btn::Waiting;
		ImDrawList *dl = ImGui::GetWindowDrawList();
		const float rad = r.H() * 0.5f;
		ImU32 ink = col::White;
		switch (kind)
		{
		case Btn::Primary:
			SoftShadow(dl, r.min, r.max, rad, 20.0f, 8.0f, Rgba(20, 60, 140, 0.35f));
			FillRounded(dl, r.min, r.max, act ? Rgba(28, 96, 204) : hov ? Rgba(56, 128, 240) : col::Accent, rad);
			TopLight(dl, r, rad, Rgba(255, 255, 255, 0.18f));
			break;
		case Btn::Secondary:
			FillRounded(dl, r.min, r.max, act ? Rgba(255, 255, 255, 0.09f) : hov ? Rgba(255, 255, 255, 0.17f) : col::Secondary, rad);
			TopLight(dl, r, rad, Rgba(255, 255, 255, 0.08f));
			break;
		case Btn::Danger:
			FillRounded(dl, r.min, r.max, act ? Rgba(255, 107, 97, 0.12f) : hov ? Rgba(255, 107, 97, 0.23f) : col::AlertTint, rad);
			ink = col::DangerInk;
			break;
		case Btn::Waiting:
			FillRounded(dl, r.min, r.max, Rgba(35, 112, 232, 0.32f), rad);
			ink = Rgba(255, 255, 255, 0.62f);
			break;
		case Btn::Link:
			if (hov)
				FillRounded(dl, r.min, r.max, Rgba(255, 255, 255, 0.05f), rad);
			ink = hov ? Rgba(156, 196, 255) : col::Link;
			break;
		}
		TextStyle style{ kind == Btn::Link ? Weight::Medium : Weight::SemiBold, fontSize, r.H() };
		const float iconW = icon != Icon::None ? IconSizeFor(icon, fontSize) : 0.0f;
		const float gap = icon != Icon::None ? 10.0f : 0.0f;
		float textW = MeasureLine(style, label).x;
		// A translation longer than the button was laid out for shrinks to fit
		// rather than spilling past the edge; PillWidth sizes most buttons. A
		// link has no pill to keep clear of, only its hover margin.
		const float room = r.W() - (kind == Btn::Link ? 8.0f : 24.0f) - iconW - gap;
		if (textW > room && room > 0.0f)
		{
			const float k = std::max(0.72f, room / textW);
			style.size *= k;
			textW *= k;
		}
		float x = r.min.x + (r.W() - textW - iconW - gap) * 0.5f;
		if (icon != Icon::None)
		{
			DrawIcon(dl, icon, ImVec2(x + iconW * 0.5f, r.Center().y), iconW, ink, 2.4f);
			x += iconW + gap;
		}
		DrawLine(dl, style, ImVec2(x, r.min.y), ink, label);
		FocusRing(dl, r.min, r.max, rad);
		return clicked && kind != Btn::Waiting;
	}

	bool RoundButton(const char *id, ImVec2 centre, float diameter, Icon icon, float iconSize, ImU32 fill, ImU32 ink)
	{
		const ImVec2 a(centre.x - diameter * 0.5f, centre.y - diameter * 0.5f);
		const ImVec2 b(a.x + diameter, a.y + diameter);
		ImGui::SetCursorScreenPos(a);
		const bool clicked = ImGui::InvisibleButton(id, ImVec2(diameter, diameter), ImGuiButtonFlags_EnableNav);
		const bool hov = ImGui::IsItemHovered(), act = ImGui::IsItemActive();
		ImDrawList *dl = ImGui::GetWindowDrawList();
		dl->AddCircleFilled(centre, diameter * 0.5f, act ? Fade(fill, 0.8f) : hov ? Mix(fill, col::White, 0.06f) : fill, 48);
		DrawIcon(dl, icon, centre, iconSize, ink, 2.6f);
		FocusRing(dl, a, b, diameter * 0.5f);
		return clicked;
	}

	bool CloseButton(const char *id, ImVec2 topLeft)
	{
		return RoundButton(id, ImVec2(topLeft.x + 22.0f, topLeft.y + 22.0f), 44.0f, Icon::Close, 18.0f,
			Rgba(255, 255, 255, 0.10f), Rgba(236, 238, 244, 0.85f));
	}

	bool ToggleSwitch(const char *id, bool &value, ImVec2 topLeft)
	{
		ImGui::SetCursorScreenPos(topLeft);
		bool changed = false;
		if (ImGui::InvisibleButton(id, ImVec2(kSwitchW, kSwitchH), ImGuiButtonFlags_EnableNav))
		{
			value = !value;
			changed = true;
		}
		const bool hov = ImGui::IsItemHovered();
		const float t = Ease(ImGui::GetItemID(), value ? 1.0f : 0.0f, 0.04f);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		const ImVec2 b(topLeft.x + kSwitchW, topLeft.y + kSwitchH);
		const ImU32 off = Rgba(255, 255, 255, hov ? 0.21f : 0.16f);
		FillRounded(dl, topLeft, b, Mix(off, hov ? Rgba(62, 206, 134) : col::SwitchOn, t), kSwitchH * 0.5f);
		const ImVec2 knob(topLeft.x + 17.0f + 24.0f * t, topLeft.y + kSwitchH * 0.5f);
		dl->AddCircleFilled(ImVec2(knob.x, knob.y + 2.0f), 15.5f, Rgba(0, 0, 0, 0.16f), 32);
		dl->AddCircleFilled(knob, 15.0f, col::White, 32);
		FocusRing(dl, topLeft, b, kSwitchH * 0.5f);
		return changed;
	}

	float PillSegmentedWidth(const char *const items[], int count, float fontSize, float minItemW, bool translate)
	{
		const TextStyle style{ Weight::SemiBold, fontSize, fontSize };
		float itemW = minItemW;
		for (int i = 0; i < count; ++i)
			itemW = std::max(itemW, MeasureLine(style, translate ? Tr(items[i]) : items[i]).x + 32.0f);
		return std::ceil(itemW * static_cast<float>(count) + 8.0f);
	}

	int PillSegmented(const char *id, int value, const char *const items[], int count, const FlexRect &r,
		float fontSize, bool translate, float lead)
	{
		ImGui::PushID(id);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		FillRounded(dl, r.min, r.max, Rgba(255, 255, 255, 0.07f), r.H() * 0.5f);
		const float pad = 4.0f;
		const float itemW = (r.W() - pad * 2.0f - lead) / static_cast<float>(std::max(1, count));
		const float itemH = r.H() - pad * 2.0f;
		const float thumbX = Ease(ImGui::GetID("thumb"), static_cast<float>(value), 0.045f);
		if (value >= 0 && value < count)
		{
			const ImVec2 a(r.min.x + pad + lead + itemW * thumbX, r.min.y + pad);
			FillRounded(dl, a, ImVec2(a.x + itemW, a.y + itemH), Rgba(255, 255, 255, 0.18f), itemH * 0.5f);
		}
		for (int i = 0; i < count; ++i)
		{
			ImGui::PushID(i);
			const ImVec2 a(r.min.x + pad + lead + itemW * static_cast<float>(i), r.min.y + pad);
			const ImVec2 b(a.x + itemW, a.y + itemH);
			ImGui::SetCursorScreenPos(a);
			if (ImGui::InvisibleButton("item", ImVec2(itemW, itemH), ImGuiButtonFlags_EnableNav))
				value = i;
			const bool hov = ImGui::IsItemHovered();
			if (hov && i != value)
				FillRounded(dl, a, b, Rgba(255, 255, 255, 0.05f), itemH * 0.5f);
			FocusRing(dl, a, b, itemH * 0.5f);
			const char *label = translate ? Tr(items[i]) : items[i];
			const TextStyle style{ i == value ? Weight::SemiBold : Weight::Medium, fontSize, itemH };
			const float w = MeasureLine(style, label).x;
			DrawLine(dl, style, ImVec2(a.x + (itemW - w) * 0.5f, a.y), i == value ? col::White : Rgba(236, 238, 244, 0.72f), label);
			ImGui::PopID();
		}
		ImGui::PopID();
		return value;
	}

	float ChipWidth(const char *text)
	{
		return std::ceil(14.0f + 9.0f + 9.0f + MeasureLine(type::CalloutMedium, text).x + 16.0f);
	}

	void Chip(ImDrawList *dl, ImVec2 topLeft, ImU32 dot, const char *text)
	{
		const float w = ChipWidth(text);
		FillRounded(dl, topLeft, ImVec2(topLeft.x + w, topLeft.y + kChipH), col::Fill, kChipH * 0.5f);
		dl->AddCircleFilled(ImVec2(topLeft.x + 14.0f + 4.5f, topLeft.y + kChipH * 0.5f), 4.5f, dot, 16);
		DrawLine(dl, TextStyle{ Weight::Medium, 16.0f, kChipH }, ImVec2(topLeft.x + 32.0f, topLeft.y), col::Text, text);
	}

	float BadgeWidth(const char *english)
	{
		return std::ceil(MeasureLine(type::Tag, Tr(english)).x + 20.0f);
	}

	void BadgePill(ImDrawList *dl, ImVec2 topLeft, const char *english, ImU32 ink, ImU32 fill, float height)
	{
		const char *label = Tr(english);
		const float w = MeasureLine(type::Tag, label).x + 20.0f;
		FillRounded(dl, topLeft, ImVec2(topLeft.x + w, topLeft.y + height), fill, height * 0.5f);
		DrawLine(dl, TextStyle{ Weight::SemiBold, 13.0f, height }, ImVec2(topLeft.x + 10.0f, topLeft.y), ink, label);
	}

	bool NavRow(const char *id, const FlexRect &r, Icon icon, const char *english, bool active, bool disabled,
		const char *badgeEnglish, const char *disabledTip)
	{
		ImGui::SetCursorScreenPos(r.min);
		bool clicked = false;
		if (!disabled)
			clicked = ImGui::InvisibleButton(id, r.Size(), ImGuiButtonFlags_EnableNav);
		else
			ImGui::Dummy(r.Size());
		const bool hov = ImGui::IsItemHovered();
		if (disabled && hov && disabledTip)
			Tip(disabledTip);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		if (active)
			FillRounded(dl, r.min, r.max, col::Selected, 14.0f);
		else if (hov && !disabled)
			FillRounded(dl, r.min, r.max, Rgba(255, 255, 255, 0.06f), 14.0f);
		const ImU32 ink = disabled ? col::Disabled : active ? col::White : Rgba(236, 238, 244, 0.80f);
		DrawIcon(dl, icon, ImVec2(r.min.x + 14.0f + 11.0f, r.Center().y), 22.0f, ink);
		const TextStyle style{ active ? Weight::SemiBold : Weight::Medium, 17.0f, r.H() };
		const float labelX = r.min.x + 14.0f + 22.0f + 12.0f;
		float labelEnd = r.max.x - 12.0f;
		if (badgeEnglish)
		{
			const float w = BadgeWidth(badgeEnglish);
			labelEnd -= w + 8.0f;
			BadgePill(dl, ImVec2(r.max.x - 12.0f - w, r.Center().y - 12.0f), badgeEnglish,
				Rgba(236, 238, 244, 0.58f), Rgba(255, 255, 255, 0.07f), 24.0f);
		}
		// A long translation gives way to the badge rather than running under it.
		DrawLine(dl, style, ImVec2(labelX, r.min.y), ink, Ellipsize(style, Tr(english), labelEnd - labelX).c_str());
		if (!disabled)
			FocusRing(dl, r.min, r.max, 14.0f);
		return clicked;
	}

	bool ToolTile(const char *id, const FlexRect &r, Icon icon, const char *english, const char *sub,
		bool chevron, bool disabled)
	{
		ImGui::SetCursorScreenPos(r.min);
		bool clicked = false;
		if (!disabled)
			clicked = ImGui::InvisibleButton(id, r.Size(), ImGuiButtonFlags_EnableNav);
		else
			ImGui::Dummy(r.Size());
		const bool hov = ImGui::IsItemHovered() && !disabled;
		const bool act = ImGui::IsItemActive() && !disabled;
		ImDrawList *dl = ImGui::GetWindowDrawList();
		dl->AddRectFilled(r.min, r.max, act ? Rgba(255, 255, 255, 0.05f) : hov ? Rgba(255, 255, 255, 0.09f) : col::Card, 20.0f);
		InnerEdge(dl, r.min, r.max, col::CardEdge, 20.0f);
		const ImVec2 badge(r.min.x + 18.0f, r.min.y + 16.0f);
		FillRounded(dl, badge, ImVec2(badge.x + 40.0f, badge.y + 40.0f), disabled ? Rgba(255, 255, 255, 0.06f) : col::LinkTint, 12.0f);
		DrawIcon(dl, icon, ImVec2(badge.x + 20.0f, badge.y + 20.0f), 22.0f, disabled ? Rgba(236, 238, 244, 0.40f) : col::Link);
		// One line each, cut with an ellipsis rather than wrapped out of the tile.
		const float textW = r.W() - 36.0f;
		DrawLine(dl, type::Strong, ImVec2(r.min.x + 18.0f, r.min.y + 16.0f + 40.0f + 8.0f),
			disabled ? col::Faint : col::Text, Ellipsize(type::Strong, Tr(english), textW).c_str());
		if (sub && *sub)
			DrawLine(dl, type::Footnote, ImVec2(r.min.x + 18.0f, r.min.y + 16.0f + 40.0f + 8.0f + 22.0f + 4.0f),
				disabled ? Rgba(236, 238, 244, 0.40f) : col::Subtle, Ellipsize(type::Footnote, sub, textW).c_str());
		if (chevron)
			DrawIcon(dl, Icon::Chevron, ImVec2(r.max.x - 16.0f - 9.0f, r.min.y + 18.0f + 9.0f), 18.0f,
				Rgba(236, 238, 244, 0.45f), 2.4f);
		if (!disabled)
			FocusRing(dl, r.min, r.max, 20.0f);
		return clicked;
	}

	void Stepper(ImDrawList *dl, ImVec2 topLeft, int current, unsigned doneMask, int failed)
	{
		const char *const names[3] = { Tr("Get set"), Tr("Move"), Tr("Check") };
		float x = topLeft.x;
		const float cy = topLeft.y + 22.0f;
		for (int i = 0; i < 3; ++i)
		{
			if (i > 0)
			{
				dl->AddRectFilled(ImVec2(x, cy - 1.0f), ImVec2(x + 28.0f, cy + 1.0f), Rgba(255, 255, 255, 0.16f), 1.0f);
				x += 28.0f + 12.0f;
			}
			const ImVec2 c(x + 14.0f, cy);
			const bool done = (doneMask >> i) & 1u;
			ImU32 ink = Rgba(236, 238, 244, 0.62f);
			Weight weight = Weight::Medium;
			if (done)
			{
				dl->AddCircleFilled(c, 14.0f, Rgba(61, 214, 140, 0.20f), 32);
				DrawIcon(dl, Icon::Check, c, 15.0f, col::Good, 3.0f);
				ink = Rgba(236, 238, 244, 0.80f);
			}
			else if (i == failed)
			{
				dl->AddCircleFilled(c, 14.0f, Rgba(255, 107, 97, 0.20f), 32);
				DrawIcon(dl, Icon::Close, c, 14.0f, col::Alert, 3.0f);
				ink = col::DangerInk;
				weight = Weight::SemiBold;
			}
			else
			{
				const bool lit = i == current;
				dl->AddCircleFilled(c, 14.0f, lit ? col::Accent : Rgba(255, 255, 255, 0.10f), 32);
				char digit[2] = { static_cast<char>('1' + i), 0 };
				const TextStyle numberStyle{ Weight::Bold, 14.0f, 28.0f };
				const float w = MeasureLine(numberStyle, digit).x;
				DrawLine(dl, numberStyle, ImVec2(c.x - w * 0.5f, cy - 14.0f), col::White, digit);
				if (lit)
				{
					ink = col::White;
					weight = Weight::SemiBold;
				}
			}
			x += 28.0f + 10.0f;
			const TextStyle labelStyle{ weight, 16.0f, 20.0f };
			DrawLine(dl, labelStyle, ImVec2(x, cy - 10.0f), ink, names[i]);
			x += MeasureLine(labelStyle, names[i]).x + 12.0f;
		}
	}

	void Meter(ImDrawList *dl, const FlexRect &r, const char *name, const char *state, ImU32 colour, float fraction)
	{
		dl->AddRectFilled(r.min, r.max, col::Card, 16.0f);
		InnerEdge(dl, r.min, r.max, col::CardEdge, 16.0f);
		const float x = r.min.x + 16.0f;
		DrawLine(dl, type::FootnoteStrong, ImVec2(x, r.min.y + 12.0f), col::Text, name);
		DrawLine(dl, TextStyle{ Weight::Medium, 14.0f, 18.0f }, ImVec2(x, r.min.y + 34.0f), colour, state);
		const ImVec2 a(x, r.max.y - 14.0f - 6.0f), b(r.max.x - 16.0f, r.max.y - 14.0f);
		dl->AddRectFilled(a, b, Rgba(255, 255, 255, 0.10f), 3.0f);
		const float f = std::clamp(fraction, 0.0f, 1.0f);
		if (f > 0.0f)
			dl->AddRectFilled(a, ImVec2(a.x + (b.x - a.x) * f, b.y), colour, 3.0f);
	}

	void StateMark(ImDrawList *dl, ImVec2 centre, Mark mark, float scale)
	{
		auto at = [&](float x, float y) { return ImVec2(centre.x + (x - 68.0f) * scale, centre.y + (y - 68.0f) * scale); };
		ImU32 halo = Rgba(255, 255, 255, 0.05f), core = Rgba(255, 255, 255, 0.10f);
		switch (mark)
		{
		case Mark::Good: halo = Rgba(61, 214, 140, 0.09f); core = col::GoodDeep; break;
		case Mark::Caution:
		case Mark::Paused: halo = Rgba(255, 179, 64, 0.10f); core = col::CautionDeep; break;
		case Mark::Bad: halo = Rgba(255, 107, 97, 0.10f); core = col::AlertDeep; break;
		case Mark::Off: core = Rgba(255, 255, 255, 0.12f); break;
		case Mark::None: break;
		}
		const ImU32 ring = Rgba(255, 255, 255, 0.22f);
		dl->AddCircleFilled(centre, 66.0f * scale, halo, 96);
		dl->AddCircle(centre, 45.0f * scale, ring, 96, 2.5f * scale);
		const ImVec2 ticks[4][2] = { { at(68, 8), at(68, 28) }, { at(68, 108), at(68, 128) },
			{ at(8, 68), at(28, 68) }, { at(108, 68), at(128, 68) } };
		for (const auto &tick : ticks)
			dl->AddLine(tick[0], tick[1], ring, 2.5f * scale);
		dl->AddCircleFilled(centre, 28.0f * scale, core, 64);
		const ImU32 white = col::White;
		switch (mark)
		{
		case Mark::Good:
		{
			const ImVec2 pts[3] = { at(56, 68.5f), at(64, 76.5f), at(79.5f, 60.5f) };
			dl->AddPolyline(pts, 3, white, 5.0f * scale);
			for (const ImVec2 &p : pts)
				dl->AddCircleFilled(p, 2.5f * scale, white, 12);
			break;
		}
		case Mark::None:
			dl->AddCircleFilled(centre, 5.0f * scale, Rgba(255, 255, 255, 0.85f), 24);
			break;
		case Mark::Paused:
			dl->AddRectFilled(at(58, 56), at(65, 80), white, 2.0f * scale);
			dl->AddRectFilled(at(71, 56), at(78, 80), white, 2.0f * scale);
			break;
		case Mark::Caution:
		case Mark::Bad:
			dl->AddLine(at(68, 55), at(68, 71), white, 5.0f * scale);
			dl->AddCircleFilled(at(68, 55), 2.5f * scale, white, 12);
			dl->AddCircleFilled(at(68, 71), 2.5f * scale, white, 12);
			dl->AddCircleFilled(at(68, 80), 3.2f * scale, white, 16);
			break;
		case Mark::Off:
			dl->AddLine(at(56, 68), at(80, 68), Rgba(255, 255, 255, 0.85f), 5.0f * scale);
			dl->AddCircleFilled(at(56, 68), 2.5f * scale, Rgba(255, 255, 255, 0.85f), 12);
			dl->AddCircleFilled(at(80, 68), 2.5f * scale, Rgba(255, 255, 255, 0.85f), 12);
			break;
		}
	}

	void RoundIcon(ImDrawList *dl, ImVec2 centre, float diameter, Icon icon, float iconSize, ImU32 ink, ImU32 fill)
	{
		dl->AddCircleFilled(centre, diameter * 0.5f, fill, 64);
		DrawIcon(dl, icon, centre, iconSize, ink, 2.2f);
	}

	void Tip(const char *english)
	{
		ShowTip(english);
	}

	bool BeginMenuPopup(const char *id, ImVec2 anchor, ImVec2 pivot, float width)
	{
		ImGui::SetNextWindowPos(anchor, ImGuiCond_Always, pivot);
		ImGui::SetNextWindowSize(ImVec2(width, 0.0f), ImGuiCond_Always);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.0f, 6.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 16.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_PopupBorderSize, 1.0f);
		const bool open = ImGui::BeginPopup(id, ImGuiWindowFlags_NoMove);
		ImGui::PopStyleVar(3);
		if (open)
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 2.0f));
		return open;
	}

	void EndMenuPopup()
	{
		ImGui::PopStyleVar();
		ImGui::EndPopup();
	}

	bool MenuItem(const char *id, Icon icon, const char *english, ImU32 ink)
	{
		const ImVec2 a = ImGui::GetCursorScreenPos();
		const ImVec2 size(ImGui::GetContentRegionAvail().x, kMenuItemH);
		const bool pressed = ImGui::InvisibleButton(id, size, ImGuiButtonFlags_EnableNav);
		const ImVec2 b(a.x + size.x, a.y + size.y);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		if (ImGui::IsItemHovered())
			FillRounded(dl, a, b, Rgba(255, 255, 255, ImGui::IsItemActive() ? 0.06f : 0.09f), 11.0f);
		FocusRing(dl, a, b, 11.0f);
		if (icon != Icon::None)
			DrawIcon(dl, icon, ImVec2(a.x + 14.0f + 9.0f, a.y + size.y * 0.5f), 18.0f, ink);
		DrawLine(dl, TextStyle{ Weight::Medium, 16.0f, size.y }, ImVec2(a.x + 44.0f, a.y), ink,
			Ellipsize(TextStyle{ Weight::Medium, 16.0f, size.y }, Tr(english), size.x - 56.0f).c_str());
		if (pressed)
			ImGui::CloseCurrentPopup();
		return pressed;
	}
}
