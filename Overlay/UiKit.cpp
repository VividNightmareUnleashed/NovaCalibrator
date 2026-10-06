// The design system's type, surfaces and line icons (UiKit.h). The controls
// built on them are in UiControls.cpp.
#include "stdafx.h"
#include "UiKit.h"

#include <imgui/imgui_internal.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace ui
{
	ImU32 Fade(ImU32 colour, float alphaMul)
	{
		const float a = static_cast<float>((colour >> IM_COL32_A_SHIFT) & 0xFF) * std::clamp(alphaMul, 0.0f, 1.0f);
		return (colour & ~IM_COL32_A_MASK) | (static_cast<ImU32>(a + 0.5f) << IM_COL32_A_SHIFT);
	}

	ImU32 Mix(ImU32 a, ImU32 b, float t)
	{
		t = std::clamp(t, 0.0f, 1.0f);
		ImU32 out = 0;
		for (int shift = 0; shift < 32; shift += 8)
		{
			const float ca = static_cast<float>((a >> shift) & 0xFF), cb = static_cast<float>((b >> shift) & 0xFF);
			out |= static_cast<ImU32>(ca + (cb - ca) * t + 0.5f) << shift;
		}
		return out;
	}

	// -----------------------------------------------------------------------
	// Type
	// -----------------------------------------------------------------------

	static ImFont *s_fonts[5] = {};

	void SetFont(Weight weight, ImFont *font)
	{
		s_fonts[static_cast<int>(weight)] = font;
	}

	ImFont *FontOf(Weight weight)
	{
		ImFont *font = s_fonts[static_cast<int>(weight)];
		// Without a monospace face Windows has, Inter sets it.
		if (!font && weight == Weight::Mono)
			font = s_fonts[static_cast<int>(Weight::Regular)];
		return font ? font : ImGui::GetFont();
	}

	// Inter's ascent and descent span 2478 of its 2048 units per em. ImGui
	// scales a font so that span fills the size it is given, a browser so the
	// em does; this turns the canvas's sizes into ImGui's.
	float GlyphSize(float cssSize)
	{
		return cssSize * (2478.0f / 2048.0f);
	}

	ImVec2 MeasureLine(const TextStyle &style, const char *text, const char *end)
	{
		if (!text)
			return ImVec2(0.0f, style.lineHeight);
		const ImVec2 size = FontOf(style.weight)->CalcTextSizeA(GlyphSize(style.size), FLT_MAX, 0.0f, text, end);
		return ImVec2(size.x, style.lineHeight);
	}

	std::string Ellipsize(const TextStyle &style, const std::string &text, float width)
	{
		if (MeasureLine(style, text.c_str()).x <= width)
			return text;
		const char *ellipsis = "\xE2\x80\xA6";
		std::string out = text;
		while (!out.empty())
		{
			// Back off one UTF-8 character (its continuation bytes, then its
			// lead byte), and any space it leaves.
			while (!out.empty() && (static_cast<unsigned char>(out.back()) & 0xC0) == 0x80)
				out.pop_back();
			if (!out.empty())
				out.pop_back();
			while (!out.empty() && out.back() == ' ')
				out.pop_back();
			if (!out.empty() && MeasureLine(style, (out + ellipsis).c_str()).x <= width)
				return out + ellipsis;
		}
		return ellipsis;
	}

	// Japanese is set without spaces and may wrap between any two characters,
	// except that a line may not start with closing punctuation, a small kana
	// or the long-vowel mark, nor end with an opening bracket. A space beside a
	// Japanese character is spacing rather than a place to break, so a number
	// and its counter, or a product name and its particle, stay together.
	namespace
	{
		bool IsCjk(unsigned int c)
		{
			return (c >= 0x3000 && c <= 0x30FF) || (c >= 0x3400 && c <= 0x4DBF) || (c >= 0x4E00 && c <= 0x9FFF) ||
				(c >= 0xF900 && c <= 0xFAFF) || (c >= 0xFF00 && c <= 0xFFEF);
		}

		bool NoLineStart(unsigned int c)
		{
			static const unsigned int closing[] = {
				0x3001, 0x3002, 0xFF0C, 0xFF0E, 0x30FB, 0xFF1A, 0xFF1B, 0xFF1F, 0xFF01, 0x30FC,   // 、。，．・：；？！ー
				0x300D, 0x300F, 0xFF09, 0xFF3D, 0xFF5D, 0x3009, 0x300B, 0x3011, 0x3015,           // closing brackets
				0x2026, 0x2025, 0x3005, 0x303B, 0x309D, 0x309E, 0x30FD, 0x30FE,                   // … ‥ and repeat marks
				0x3041, 0x3043, 0x3045, 0x3047, 0x3049, 0x3063, 0x3083, 0x3085, 0x3087, 0x308E,   // small hiragana
				0x3095, 0x3096,
				0x30A1, 0x30A3, 0x30A5, 0x30A7, 0x30A9, 0x30C3, 0x30E3, 0x30E5, 0x30E7, 0x30EE,   // small katakana
				0x30F5, 0x30F6,
				')', ']', '}', '.', ',', ':', ';', '!', '?', '%',
			};
			return std::find(std::begin(closing), std::end(closing), c) != std::end(closing);
		}

		bool NoLineEnd(unsigned int c)
		{
			static const unsigned int opening[] = { 0x300C, 0x300E, 0xFF08, 0xFF3B, 0xFF5B, 0x3008, 0x300A, 0x3010, 0x3014,
				'(', '[', '{' };
			return std::find(std::begin(opening), std::end(opening), c) != std::end(opening);
		}

		bool HasCjk(const char *s, const char *end)
		{
			while (s < end)
			{
				unsigned int c = 0;
				const int n = ImTextCharFromUtf8(&c, s, end);
				if (IsCjk(c))
					return true;
				s += std::max(n, 1);
			}
			return false;
		}

		// Where the line starting at s ends, for text with Japanese in it.
		const char *CjkWrapPosition(ImFont *font, float size, const char *s, const char *end, float maxWidth)
		{
			// Baked at a rounded size, so advances scale to the size asked for,
			// as ImGui's own measuring does.
			ImFontBaked *baked = font->GetFontBaked(size);
			const float scale = size / baked->Size;
			const char *lastBreak = nullptr;
			unsigned int beforePrev = 0, prev = 0;
			float width = 0.0f;
			for (const char *p = s; p < end;)
			{
				unsigned int c = 0;
				const int n = std::max(ImTextCharFromUtf8(&c, p, end), 1);
				if (p > s && c != ' ')
				{
					const bool allowed = prev == ' '
						? beforePrev != 0 && !IsCjk(beforePrev) && !IsCjk(c)
						: (IsCjk(prev) || IsCjk(c)) && !NoLineStart(c) && !NoLineEnd(prev);
					if (allowed)
						lastBreak = p;
				}
				width += baked->GetCharAdvance(static_cast<ImWchar>(c)) * scale;
				if (width > maxWidth && p > s)
					return lastBreak ? lastBreak : p;
				beforePrev = prev;
				prev = c;
				p += n;
			}
			return end;
		}
	}

	namespace
	{
		using Line = std::pair<const char *, const char *>;

		// One paragraph (no newline in it) broken into lines that fit width.
		std::vector<Line> WrapParagraph(ImFont *font, float size, const char *s, const char *end, float width, bool cjk)
		{
			std::vector<Line> lines;
			while (s < end)
			{
				const char *lineEnd = end;
				if (width > 0.0f)
				{
					lineEnd = cjk ? CjkWrapPosition(font, size, s, end, width)
						: font->CalcWordWrapPosition(size, s, end, width);
					if (lineEnd <= s)
					{
						unsigned int c = 0;
						lineEnd = s + std::max(1, ImTextCharFromUtf8(&c, s, end));
					}
				}
				const char *trimmed = lineEnd;
				while (trimmed > s && (trimmed[-1] == ' ' || trimmed[-1] == '\t'))
					--trimmed;
				lines.emplace_back(s, trimmed);
				s = lineEnd;
				while (s < end && (*s == ' ' || *s == '\t'))
					++s;
			}
			return lines;
		}

		// A single short word alone on a paragraph's last line reads as an
		// accident, so the line above gives it a neighbour when that keeps the
		// paragraph's line count (CSS's text-wrap: pretty, in its simplest
		// form).
		void AvoidOrphan(std::vector<Line> &lines, ImFont *font, float size, const char *s, const char *end, float width)
		{
			if (lines.size() < 2 || width <= 0.0f)
				return;
			const Line last = lines.back();
			if (std::find(last.first, last.second, ' ') != last.second)
				return;
			const float lastW = font->CalcTextSizeA(size, FLT_MAX, 0.0f, last.first, last.second).x;
			if (lastW > width * 0.25f)
				return;
			const Line &above = lines[lines.size() - 2];
			const char *space = above.second;
			while (space > above.first && space[-1] != ' ')
				--space;
			if (space <= above.first)
				return;
			const float moved = font->CalcTextSizeA(size, FLT_MAX, 0.0f, space, above.second).x;
			std::vector<Line> retry = WrapParagraph(font, size, s, end, width - moved - 1.0f, false);
			if (retry.size() == lines.size())
				lines = std::move(retry);
		}
	}

	TextLines WrapText(const TextStyle &style, const char *text, float maxWidth)
	{
		TextLines out;
		if (!text)
			return out;
		ImFont *font = FontOf(style.weight);
		const float glyph = GlyphSize(style.size);
		const char *const end = text + std::strlen(text);
		const char *s = text;
		while (s <= end)
		{
			// A newline always ends a line, so a paragraph break survives.
			const char *newline = static_cast<const char *>(std::memchr(s, '\n', static_cast<size_t>(end - s)));
			const char *paragraphEnd = newline ? newline : end;
			if (s == paragraphEnd)
				out.lines.emplace_back(s, s);
			const bool cjk = maxWidth > 0.0f && HasCjk(s, paragraphEnd);
			std::vector<Line> lines = WrapParagraph(font, glyph, s, paragraphEnd, maxWidth, cjk);
			if (!cjk)
				AvoidOrphan(lines, font, glyph, s, paragraphEnd, maxWidth);
			for (const Line &line : lines)
			{
				out.lines.push_back(line);
				out.width = std::max(out.width, font->CalcTextSizeA(glyph, FLT_MAX, 0.0f, line.first, line.second).x);
			}
			if (!newline)
				break;
			s = newline + 1;
			if (s == end)
			{
				out.lines.emplace_back(s, s);
				break;
			}
		}
		out.height = static_cast<float>(out.lines.size()) * style.lineHeight;
		return out;
	}

	void DrawLine(ImDrawList *dl, const TextStyle &style, ImVec2 topLeft, ImU32 colour, const char *text, const char *end)
	{
		if (!text || (end && end <= text))
			return;
		const float glyph = GlyphSize(style.size);
		// The glyph box sits centred in the line box, as a browser's half-leading
		// puts it; whole pixels keep the text sharp.
		const ImVec2 at(std::floor(topLeft.x + 0.5f), std::floor(topLeft.y + (style.lineHeight - glyph) * 0.5f + 0.5f));
		dl->AddText(FontOf(style.weight), glyph, at, colour, text, end);
	}

	void DrawLines(ImDrawList *dl, const TextStyle &style, const TextLines &lines, ImVec2 topLeft,
		float boxWidth, ImU32 colour, Align align)
	{
		float y = topLeft.y;
		for (const auto &line : lines.lines)
		{
			float x = topLeft.x;
			if (align != Align::Left)
			{
				const float w = MeasureLine(style, line.first, line.second).x;
				x += align == Align::Center ? (boxWidth - w) * 0.5f : boxWidth - w;
			}
			DrawLine(dl, style, ImVec2(x, y), colour, line.first, line.second);
			y += style.lineHeight;
		}
	}

	float DrawText(ImDrawList *dl, const TextStyle &style, ImVec2 topLeft, float width, ImU32 colour,
		const char *text, Align align)
	{
		const TextLines lines = WrapText(style, text, width);
		DrawLines(dl, style, lines, topLeft, width, colour, align);
		return lines.height;
	}

	YGNodeRef TextNode(FlexLayout &fl, YGNodeRef parent, const TextStyle &style, const std::string &text)
	{
		return fl.Measured(parent, [style, text](float maxWidth) {
			const TextLines lines = WrapText(style, text.c_str(), maxWidth >= FLT_MAX * 0.5f ? 0.0f : maxWidth);
			return ImVec2(std::ceil(lines.width), lines.height);
		});
	}

	// -----------------------------------------------------------------------
	// Surfaces
	// -----------------------------------------------------------------------

	void FillRounded(ImDrawList *dl, ImVec2 a, ImVec2 b, ImU32 colour, float rounding, ImDrawFlags flags)
	{
		dl->AddRectFilled(a, b, colour, rounding, flags);
	}

	void FillGradient(ImDrawList *dl, ImVec2 a, ImVec2 b, ImU32 top, ImU32 bottom, float rounding, ImDrawFlags flags)
	{
		const int first = dl->VtxBuffer.Size;
		dl->AddRectFilled(a, b, IM_COL32_WHITE, rounding, flags);
		ImGui::ShadeVertsLinearColorGradientKeepAlpha(dl, first, dl->VtxBuffer.Size, a, ImVec2(a.x, b.y), top, bottom);
	}

	void InnerEdge(ImDrawList *dl, ImVec2 a, ImVec2 b, ImU32 colour, float rounding, float thickness)
	{
		const float h = thickness * 0.5f;
		dl->AddRect(ImVec2(a.x + h, a.y + h), ImVec2(b.x - h, b.y - h), colour, std::max(0.0f, rounding - h),
			ImDrawFlags_RoundCornersAll, thickness);
	}

	void SoftShadow(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding, float blur, float dy, ImU32 colour)
	{
		// Stacked translucent layers, from just inside the shape to blur past
		// its edge: a falloff close enough to a Gaussian at this size.
		const int layers = 12;
		const ImU32 layer = Fade(colour, 1.6f / static_cast<float>(layers));
		for (int i = 0; i < layers; ++i)
		{
			const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(layers);
			const float grow = -blur * 0.35f + blur * 1.25f * t;
			dl->AddRectFilled(ImVec2(a.x - grow, a.y - grow + dy), ImVec2(b.x + grow, b.y + grow + dy), layer,
				std::max(0.0f, rounding + grow));
		}
	}

	void RadialGlow(ImDrawList *dl, ImVec2 centre, ImVec2 radius, ImU32 inner, int segments)
	{
		const ImU32 outer = inner & ~IM_COL32_A_MASK;
		const ImVec2 uv = dl->_Data->TexUvWhitePixel;
		dl->PrimReserve(segments * 3, segments + 1);
		const ImDrawIdx base = static_cast<ImDrawIdx>(dl->_VtxCurrentIdx);
		dl->PrimWriteVtx(centre, uv, inner);
		for (int i = 0; i < segments; ++i)
		{
			const float angle = 2.0f * IM_PI * static_cast<float>(i) / static_cast<float>(segments);
			dl->PrimWriteVtx(ImVec2(centre.x + std::cos(angle) * radius.x, centre.y + std::sin(angle) * radius.y), uv, outer);
		}
		for (int i = 0; i < segments; ++i)
		{
			dl->PrimWriteIdx(base);
			dl->PrimWriteIdx(static_cast<ImDrawIdx>(base + 1 + i));
			dl->PrimWriteIdx(static_cast<ImDrawIdx>(base + 1 + (i + 1) % segments));
		}
	}

	void Card(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding)
	{
		dl->AddRectFilled(a, b, col::Card, rounding);
		InnerEdge(dl, a, b, col::CardEdge, rounding);
	}

	void Hairline(ImDrawList *dl, float x0, float x1, float y)
	{
		dl->AddRectFilled(ImVec2(x0, y), ImVec2(x1, y + 1.0f), col::Hairline);
	}

	void PageBackdrop(ImDrawList *dl, ImVec2 a, ImVec2 b)
	{
		// linear-gradient(165deg, #2E3137 0%, #24262B 55%, #1D1F23 100%): nearly
		// vertical at this aspect, so two vertical bands.
		const float split = a.y + (b.y - a.y) * 0.55f;
		const ImU32 top = Rgba(46, 49, 55), middle = Rgba(36, 38, 43), bottom = Rgba(29, 31, 35);
		dl->AddRectFilledMultiColor(a, ImVec2(b.x, split), top, top, middle, middle);
		dl->AddRectFilledMultiColor(ImVec2(a.x, split), b, middle, middle, bottom, bottom);
		// radial-gradient(900px 620px at 16% -14%, white 10%, transparent 70%).
		const ImVec2 size(b.x - a.x, b.y - a.y);
		RadialGlow(dl, ImVec2(a.x + size.x * 0.16f, a.y - size.y * 0.14f), ImVec2(900.0f * 0.7f, 620.0f * 0.7f),
			Rgba(255, 255, 255, 0.10f), 96);
	}

	void PanelSurface(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding)
	{
		SoftShadow(dl, a, b, rounding, 80.0f, 30.0f, Rgba(0, 0, 0, 0.55f));
		FillGradient(dl, a, b, col::SheetTop, col::SheetBottom, rounding);
		InnerEdge(dl, a, b, col::SheetEdge, rounding);
	}

	// -----------------------------------------------------------------------
	// Icons
	// -----------------------------------------------------------------------

	namespace
	{
		// One drawn part of an icon on the 24-unit grid: a path (SVG path data)
		// stroked or filled, a circle or a rounded rectangle.
		struct IconPart
		{
			enum Kind { Stroke, Fill, Circle, Dot, Box, Block } kind;
			const char *path;
			float x, y, w, h, r;
			float stroke;   // multiplies the icon's stroke width; 0 keeps it
		};

		IconPart P(const char *d, float stroke = 0.0f) { return { IconPart::Stroke, d, 0, 0, 0, 0, 0, stroke }; }
		IconPart F(const char *d) { return { IconPart::Fill, d, 0, 0, 0, 0, 0, 0 }; }
		IconPart O(float cx, float cy, float r, float stroke = 0.0f) { return { IconPart::Circle, nullptr, cx, cy, 0, 0, r, stroke }; }
		IconPart D(float cx, float cy, float r) { return { IconPart::Dot, nullptr, cx, cy, 0, 0, r, 0 }; }
		IconPart B(float x, float y, float w, float h, float r) { return { IconPart::Box, nullptr, x, y, w, h, r, 0 }; }
		IconPart K(float x, float y, float w, float h, float r) { return { IconPart::Block, nullptr, x, y, w, h, r, 0 }; }

		const char *const kShield = "M12 3l7 2.8v5.4c0 4.4-2.9 7.8-7 9.8-4.1-2-7-5.4-7-9.8V5.8z";

		std::vector<IconPart> IconParts(Icon icon)
		{
			switch (icon)
			{
			case Icon::Reticle: return { O(12, 12, 7), P("M12 2.5v4M12 17.5v4M2.5 12h4M17.5 12h4"), D(12, 12, 1.6f) };
			case Icon::Identify: return { P("M12 3v3M12 18v3M3 12h3M18 12h3"), O(12, 12, 5.5f), D(12, 12, 1.5f) };
			case Icon::Chaperone: return { P(kShield), P("M9.67 3.93v15.71M14.33 3.93v15.71M5 9h14M5.42 14h13.16", 0.7f) };
			case Icon::Anchor: return { P("M12 13.5s5-4.6 5-8.3a5 5 0 0 0-10 0c0 3.7 5 8.3 5 8.3z"), O(12, 5.4f, 1.6f),
				P("M5.5 16.5c-1.6.6-2.5 1.4-2.5 2.2 0 1.8 4 3.3 9 3.3s9-1.5 9-3.3c0-.8-.9-1.6-2.5-2.2") };
			case Icon::Lighthouse: return { B(4.5f, 4, 15, 16, 3.5f), O(12, 12.5f, 3.6f), P("M8.5 6.9h7") };
			case Icon::Smoothing: return { P("M3 14c2.2-5 4.4-7 6.4-7 3.2 0 2.8 10 6.2 10 1.8 0 3.4-2 5.4-6") };
			case Icon::Chevron: return { P("M9.5 5.5L16 12l-6.5 6.5") };
			case Icon::ChevronLeft: return { P("M14.5 5.5L8 12l6.5 6.5") };
			case Icon::ChevronUpDown: return { P("M8 9.5l4-4 4 4M8 14.5l4 4 4-4") };
			case Icon::Close: return { P("M6 6l12 12M18 6L6 18") };
			case Icon::Check: return { P("M5 12.5l4.5 4.5L19 7.5") };
			case Icon::Plus: return { P("M12 6v12M6 12h12") };
			case Icon::Minus: return { P("M6 12h12") };
			case Icon::Trash: return { P("M4.5 7h15M9.5 7V4.8h5V7M6.5 7l1 12.5h9l1-12.5"), P("M10.2 10.5v6M13.8 10.5v6") };
			case Icon::Download: return { P("M12 4v10.5M7.5 10l4.5 4.5 4.5-4.5"), P("M5 19.5h14") };
			case Icon::Doc: return { P("M7 3.5h7l4 4V20a.5.5 0 0 1-.5.5h-10A.5.5 0 0 1 7 20z"), P("M14 3.5V8h4"), P("M9.5 12.5h6M9.5 16h6") };
			case Icon::Pencil: return { P("M5 19l1-4.2L15.6 5.2a1.8 1.8 0 0 1 2.5 0l.7.7a1.8 1.8 0 0 1 0 2.5L9.2 18z"), P("M13.8 7l3.2 3.2") };
			case Icon::Info: return { O(12, 12, 9), P("M12 11v5.5"), D(12, 7.8f, 1.1f) };
			case Icon::Warn: return { P("M12 4.2L21 19.5H3z"), P("M12 10v4.4"), D(12, 16.9f, 1.0f) };
			case Icon::Refresh: return { P("M20 11a8 8 0 0 0-14.3-4.6"), P("M5 3.5v3.4h3.4"), P("M4 13a8 8 0 0 0 14.3 4.6"), P("M19 20.5v-3.4h-3.4") };
			case Icon::Power: return { P("M12 3.5v8"), P("M7.2 6.6a7.5 7.5 0 1 0 9.6 0") };
			case Icon::Pause: return { K(6.5f, 5, 3.6f, 14, 1.2f), K(13.9f, 5, 3.6f, 14, 1.2f) };
			case Icon::Play: return { F("M7 4.5v15a1 1 0 0 0 1.5.86l12.4-7.5a1 1 0 0 0 0-1.72L8.5 3.64A1 1 0 0 0 7 4.5z") };
			case Icon::Book: return { P("M5 4.5h9.5a3 3 0 0 1 3 3v12H8a3 3 0 0 1-3-3z"), P("M5 16.5a3 3 0 0 1 3-3h9.5") };
			case Icon::Globe: return { O(12, 12, 9), P("M3 12h18"), P("M12 3a14 14 0 0 1 0 18a14 14 0 0 1 0-18z") };
			case Icon::Bell: return { P("M18 16.5H6l1.4-1.9v-4.1a4.6 4.6 0 0 1 9.2 0v4.1z"), P("M10.2 19.3a1.9 1.9 0 0 0 3.6 0") };
			case Icon::Clock: return { O(12, 12, 8.5f), P("M12 7.5V12l3 2") };
			case Icon::ShieldCheck: return { P(kShield), P("M8.6 12.2l2.4 2.4 4.4-4.6") };
			case Icon::Headset: return { P("M4.5 8h15a2 2 0 0 1 2 2v4.5a3 3 0 0 1-3 3h-3l-1.7-2.3a1.4 1.4 0 0 0-2.3 0l-1.7 2.3h-3a3 3 0 0 1-3-3V10a2 2 0 0 1 2-2z") };
			case Icon::Tracker: return { P("M12 4.5l6.5 3.75v7.5L12 19.5l-6.5-3.75v-7.5z"), D(12, 12, 1.8f) };
			case Icon::Controller: return { O(9.5f, 8.5f, 5.5f), P("M12.6 13l4 7") };
			case Icon::More: return { D(6, 12, 1.7f), D(12, 12, 1.7f), D(18, 12, 1.7f) };
			case Icon::Replay: return { P("M4.5 12a7.5 7.5 0 1 0 2.2-5.3"), P("M4.5 4.5v4.2h4.2") };
			case Icon::DownloadCircle: return { O(12, 12, 9), P("M12 7.2v8.6M8.2 12.4l3.8 3.8 3.8-3.8") };
			default: return {};
			}
		}

		// SVG path data to polylines on the grid. Handles the commands the icons
		// use, absolute and relative: M L H V C S Q A Z.
		struct SubPath
		{
			std::vector<ImVec2> points;
			std::vector<bool> corner;   // a point where segments meet at an angle
			bool closed = false;
		};

		class PathReader
		{
		public:
			explicit PathReader(const char *d) : p(d) {}

			bool Command(char &cmd)
			{
				Skip();
				if (!*p)
					return false;
				if (std::isalpha(static_cast<unsigned char>(*p)))
				{
					cmd = *p++;
					return true;
				}
				return HasNumber();   // an implicit repeat of the last command
			}
			bool HasNumber()
			{
				Skip();
				return *p == '-' || *p == '+' || *p == '.' || std::isdigit(static_cast<unsigned char>(*p));
			}
			float Number()
			{
				Skip();
				char *end = nullptr;
				const float v = std::strtof(p, &end);
				p = end && end != p ? end : p + 1;
				return v;
			}
			float Flag()
			{
				Skip();
				const float v = *p == '1' ? 1.0f : 0.0f;
				if (*p)
					++p;
				return v;
			}

		private:
			void Skip()
			{
				while (*p == ' ' || *p == ',' || *p == '\n' || *p == '\t')
					++p;
			}
			const char *p;
		};

		void AddArc(SubPath &sp, ImVec2 from, float rx, float ry, float rotationDeg, bool large, bool sweep, ImVec2 to)
		{
			// Endpoint to centre parameterisation (SVG 1.1, appendix F.6).
			if (rx == 0.0f || ry == 0.0f)
			{
				sp.points.push_back(to);
				sp.corner.push_back(true);
				return;
			}
			rx = std::fabs(rx);
			ry = std::fabs(ry);
			const float phi = rotationDeg * IM_PI / 180.0f;
			const float cs = std::cos(phi), sn = std::sin(phi);
			const float dx = (from.x - to.x) * 0.5f, dy = (from.y - to.y) * 0.5f;
			const float x1 = cs * dx + sn * dy, y1 = -sn * dx + cs * dy;
			const float lambda = (x1 * x1) / (rx * rx) + (y1 * y1) / (ry * ry);
			if (lambda > 1.0f)
			{
				rx *= std::sqrt(lambda);
				ry *= std::sqrt(lambda);
			}
			const float num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1;
			const float den = rx * rx * y1 * y1 + ry * ry * x1 * x1;
			float k = den > 0.0f ? std::sqrt(std::max(0.0f, num / den)) : 0.0f;
			if (large == sweep)
				k = -k;
			const float cx1 = k * rx * y1 / ry, cy1 = -k * ry * x1 / rx;
			const float cx = cs * cx1 - sn * cy1 + (from.x + to.x) * 0.5f;
			const float cy = sn * cx1 + cs * cy1 + (from.y + to.y) * 0.5f;
			auto angle = [](float ux, float uy, float vx, float vy) {
				const float a = std::atan2(ux * vy - uy * vx, ux * vx + uy * vy);
				return a;
			};
			const float theta = angle(1.0f, 0.0f, (x1 - cx1) / rx, (y1 - cy1) / ry);
			float delta = angle((x1 - cx1) / rx, (y1 - cy1) / ry, (-x1 - cx1) / rx, (-y1 - cy1) / ry);
			if (!sweep && delta > 0.0f)
				delta -= 2.0f * IM_PI;
			else if (sweep && delta < 0.0f)
				delta += 2.0f * IM_PI;
			const int steps = std::max(2, static_cast<int>(std::ceil(std::fabs(delta) / (IM_PI / 16.0f))));
			for (int i = 1; i <= steps; ++i)
			{
				const float t = theta + delta * static_cast<float>(i) / static_cast<float>(steps);
				const float ex = rx * std::cos(t), ey = ry * std::sin(t);
				sp.points.emplace_back(cs * ex - sn * ey + cx, sn * ex + cs * ey + cy);
				sp.corner.push_back(i == steps);
			}
		}

		std::vector<SubPath> ParsePath(const char *d)
		{
			std::vector<SubPath> out;
			PathReader in(d);
			char cmd = 'M';
			ImVec2 cur(0, 0), start(0, 0), lastControl(0, 0);
			char lastCmd = 0;
			auto line = [&](ImVec2 to) {
				if (out.empty())
					out.emplace_back();
				out.back().points.push_back(to);
				out.back().corner.push_back(true);
				cur = to;
			};
			while (in.Command(cmd))
			{
				const bool rel = std::islower(static_cast<unsigned char>(cmd)) != 0;
				const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(cmd)));
				const ImVec2 base = rel ? cur : ImVec2(0, 0);
				switch (c)
				{
				case 'M':
				{
					const float x = in.Number() + base.x, y = in.Number() + base.y;
					out.emplace_back();
					out.back().points.emplace_back(x, y);
					out.back().corner.push_back(true);
					cur = start = ImVec2(x, y);
					// Further pairs after a move are lines.
					cmd = rel ? 'l' : 'L';
					while (in.HasNumber())
					{
						const ImVec2 b2 = rel ? cur : ImVec2(0, 0);
						const float lx = in.Number() + b2.x, ly = in.Number() + b2.y;
						line(ImVec2(lx, ly));
					}
					break;
				}
				case 'L':
				{
					const float x = in.Number() + base.x, y = in.Number() + base.y;
					line(ImVec2(x, y));
					break;
				}
				case 'H':
					line(ImVec2(in.Number() + (rel ? cur.x : 0.0f), cur.y));
					break;
				case 'V':
					line(ImVec2(cur.x, in.Number() + (rel ? cur.y : 0.0f)));
					break;
				case 'C':
				case 'S':
				case 'Q':
				{
					ImVec2 c1, c2, to;
					if (c == 'S')
					{
						const bool smooth = lastCmd == 'C' || lastCmd == 'S';
						c1 = smooth ? ImVec2(2.0f * cur.x - lastControl.x, 2.0f * cur.y - lastControl.y) : cur;
						c2.x = in.Number() + base.x; c2.y = in.Number() + base.y;
						to.x = in.Number() + base.x; to.y = in.Number() + base.y;
					}
					else
					{
						c1.x = in.Number() + base.x; c1.y = in.Number() + base.y;
						if (c == 'C')
						{
							c2.x = in.Number() + base.x; c2.y = in.Number() + base.y;
						}
						to.x = in.Number() + base.x; to.y = in.Number() + base.y;
						if (c == 'Q')
						{
							// As a cubic: controls two thirds of the way to the quad's.
							const ImVec2 q = c1;
							c1 = ImVec2(cur.x + (q.x - cur.x) * 2.0f / 3.0f, cur.y + (q.y - cur.y) * 2.0f / 3.0f);
							c2 = ImVec2(to.x + (q.x - to.x) * 2.0f / 3.0f, to.y + (q.y - to.y) * 2.0f / 3.0f);
						}
					}
					if (out.empty())
						out.emplace_back();
					const ImVec2 p0 = cur;
					const int steps = 12;
					for (int i = 1; i <= steps; ++i)
					{
						const float t = static_cast<float>(i) / static_cast<float>(steps), u = 1.0f - t;
						const float w0 = u * u * u, w1 = 3.0f * u * u * t, w2 = 3.0f * u * t * t, w3 = t * t * t;
						out.back().points.emplace_back(w0 * p0.x + w1 * c1.x + w2 * c2.x + w3 * to.x,
							w0 * p0.y + w1 * c1.y + w2 * c2.y + w3 * to.y);
						out.back().corner.push_back(i == steps);
					}
					lastControl = c2;
					cur = to;
					break;
				}
				case 'A':
				{
					const float rx = in.Number(), ry = in.Number(), rot = in.Number();
					const bool large = in.Flag() != 0.0f, sweep = in.Flag() != 0.0f;
					const float x = in.Number() + base.x, y = in.Number() + base.y;
					if (out.empty())
						out.emplace_back();
					AddArc(out.back(), cur, rx, ry, rot, large, sweep, ImVec2(x, y));
					cur = ImVec2(x, y);
					break;
				}
				case 'Z':
					if (!out.empty())
						out.back().closed = true;
					cur = start;
					break;
				default:
					break;
				}
				lastCmd = c;
			}
			return out;
		}

		struct ParsedPart
		{
			IconPart part;
			std::vector<SubPath> paths;
		};

		const std::vector<ParsedPart> &Parsed(Icon icon)
		{
			static std::vector<ParsedPart> cache[static_cast<int>(Icon::Count)];
			static bool ready[static_cast<int>(Icon::Count)] = {};
			const int i = static_cast<int>(icon);
			if (!ready[i])
			{
				for (const IconPart &part : IconParts(icon))
				{
					ParsedPart parsed{ part, {} };
					if (part.path)
						parsed.paths = ParsePath(part.path);
					cache[i].push_back(std::move(parsed));
				}
				ready[i] = true;
			}
			return cache[i];
		}

		void StrokePolyline(ImDrawList *dl, const std::vector<ImVec2> &pts, const std::vector<bool> &corner, bool closed,
			ImU32 colour, float thickness)
		{
			if (pts.size() < 2)
				return;
			dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), colour, thickness, closed ? ImDrawFlags_Closed : 0);
			// Round caps and joins, which ImGui's polylines do not draw.
			const float r = thickness * 0.5f;
			for (size_t i = 0; i < pts.size(); ++i)
			{
				const bool end = !closed && (i == 0 || i + 1 == pts.size());
				if (end || (corner[i] && thickness > 1.2f))
					dl->AddCircleFilled(pts[i], r, colour, 8);
			}
		}

		void Gear(ImDrawList *dl, ImVec2 origin, float scale, ImU32 colour, float thickness)
		{
			const ImVec2 c(origin.x + 12.0f * scale, origin.y + 12.0f * scale);
			dl->AddCircle(c, 6.0f * scale, colour, 32, thickness);
			dl->AddCircle(c, 2.3f * scale, colour, 16, thickness);
			for (int t = 0; t < 8; ++t)
			{
				const float a = static_cast<float>(t) * IM_PI / 4.0f;
				const float cs = std::cos(a), sn = std::sin(a);
				const ImVec2 local[4] = { { -1.3f, -9.7f }, { 1.3f, -9.7f }, { 1.3f, -5.8f }, { -1.3f, -5.8f } };
				ImVec2 q[4];
				for (int k = 0; k < 4; ++k)
					q[k] = ImVec2(c.x + (local[k].x * cs - local[k].y * sn) * scale, c.y + (local[k].x * sn + local[k].y * cs) * scale);
				dl->AddConvexPolyFilled(q, 4, colour);
			}
		}
	}

	void DrawIcon(ImDrawList *dl, Icon icon, ImVec2 centre, float size, ImU32 colour, float stroke)
	{
		if (icon == Icon::None)
			return;
		const float scale = size / 24.0f;
		const ImVec2 origin(centre.x - 12.0f * scale, centre.y - 12.0f * scale);
		auto map = [&](ImVec2 p) { return ImVec2(origin.x + p.x * scale, origin.y + p.y * scale); };
		if (icon == Icon::Gear)
		{
			Gear(dl, origin, scale, colour, stroke * scale);
			return;
		}
		std::vector<ImVec2> pts;
		for (const ParsedPart &parsed : Parsed(icon))
		{
			const IconPart &part = parsed.part;
			const float thickness = stroke * scale * (part.stroke > 0.0f ? part.stroke : 1.0f);
			switch (part.kind)
			{
			case IconPart::Stroke:
			case IconPart::Fill:
				for (const SubPath &sp : parsed.paths)
				{
					pts.clear();
					for (const ImVec2 &p : sp.points)
						pts.push_back(map(p));
					if (part.kind == IconPart::Fill)
						dl->AddConvexPolyFilled(pts.data(), static_cast<int>(pts.size()), colour);
					else
						StrokePolyline(dl, pts, sp.corner, sp.closed, colour, thickness);
				}
				break;
			case IconPart::Circle:
				dl->AddCircle(map(ImVec2(part.x, part.y)), part.r * scale, colour, 0, thickness);
				break;
			case IconPart::Dot:
				dl->AddCircleFilled(map(ImVec2(part.x, part.y)), part.r * scale, colour, 0);
				break;
			case IconPart::Box:
				dl->AddRect(map(ImVec2(part.x, part.y)), map(ImVec2(part.x + part.w, part.y + part.h)), colour,
					part.r * scale, ImDrawFlags_RoundCornersAll, thickness);
				break;
			case IconPart::Block:
				dl->AddRectFilled(map(ImVec2(part.x, part.y)), map(ImVec2(part.x + part.w, part.y + part.h)), colour,
					part.r * scale);
				break;
			}
		}
	}
}
