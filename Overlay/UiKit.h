#pragma once

// The Nova Calibrator design system: colour tokens, the Inter weights, wrapped
// text, soft surfaces, line icons and the controls every screen is built from.
// The design canvas is drawn at the overlay's own 1200x800, so its CSS pixels
// are these pixels; the sizes below are copied from it.
//
// Controls that take an English label translate it themselves (Tr), like the
// older widgets; text drawn directly takes text the caller has translated.

#include <imgui/imgui.h>
#include "UiLayout.h"

#include <string>
#include <utility>
#include <vector>

namespace ui
{
	// -----------------------------------------------------------------------
	// Colour
	// -----------------------------------------------------------------------

	constexpr ImU32 Rgba(int r, int g, int b, float a = 1.0f)
	{
		return IM_COL32(r, g, b, static_cast<int>(a * 255.0f + 0.5f));
	}
	// The same colour with its alpha multiplied by alphaMul.
	ImU32 Fade(ImU32 colour, float alphaMul);
	// From a (t = 0) to b (t = 1), alpha included.
	ImU32 Mix(ImU32 a, ImU32 b, float t);
	// Keeps frames coming at 60 Hz while something on screen is still moving;
	// the main loop otherwise sleeps up to a second between frames.
	void KeepAnimating();

	namespace col
	{
		// Ink, from strongest to quietest.
		constexpr ImU32 Text = Rgba(244, 245, 247);
		constexpr ImU32 White = Rgba(255, 255, 255);
		constexpr ImU32 Body = Rgba(236, 238, 244, 0.78f);     // the hero's description
		constexpr ImU32 Prose = Rgba(236, 238, 244, 0.72f);    // sheet paragraphs
		constexpr ImU32 Muted = Rgba(236, 238, 244, 0.66f);    // subtitles and sub-lines
		constexpr ImU32 Subtle = Rgba(236, 238, 244, 0.62f);
		constexpr ImU32 Quiet = Rgba(236, 238, 244, 0.56f);    // times, serials
		constexpr ImU32 Faint = Rgba(236, 238, 244, 0.45f);
		constexpr ImU32 Disabled = Rgba(236, 238, 244, 0.42f);

		constexpr ImU32 Accent = Rgba(35, 112, 232);           // the primary button
		constexpr ImU32 Link = Rgba(106, 167, 255);
		constexpr ImU32 Good = Rgba(61, 214, 140);
		constexpr ImU32 GoodDeep = Rgba(31, 164, 99);
		constexpr ImU32 Caution = Rgba(255, 179, 64);
		constexpr ImU32 CautionDeep = Rgba(201, 132, 21);
		constexpr ImU32 Alert = Rgba(255, 107, 97);
		constexpr ImU32 AlertDeep = Rgba(217, 83, 74);
		constexpr ImU32 DangerInk = Rgba(255, 138, 128);
		constexpr ImU32 Lavender = Rgba(179, 166, 255);        // advice: what to do next
		constexpr ImU32 SwitchOn = Rgba(48, 194, 122);

		constexpr ImU32 Card = Rgba(255, 255, 255, 0.06f);
		constexpr ImU32 CardEdge = Rgba(255, 255, 255, 0.05f);
		constexpr ImU32 Hairline = Rgba(255, 255, 255, 0.08f);
		constexpr ImU32 Well = Rgba(0, 0, 0, 0.22f);           // demo areas and fields
		constexpr ImU32 Fill = Rgba(255, 255, 255, 0.08f);     // chips and quiet fills
		constexpr ImU32 Secondary = Rgba(255, 255, 255, 0.12f);
		constexpr ImU32 Selected = Rgba(255, 255, 255, 0.14f); // the current page in the sidebar
		constexpr ImU32 Sidebar = Rgba(0, 0, 0, 0.16f);
		constexpr ImU32 BackdropTop = Rgba(46, 49, 55);        // the page backdrop at its top edge
		constexpr ImU32 Scrim = Rgba(12, 13, 16, 0.68f);       // behind sheets and dialogs
		constexpr ImU32 SheetTop = Rgba(58, 61, 68);
		constexpr ImU32 SheetBottom = Rgba(47, 50, 56);
		constexpr ImU32 SheetEdge = Rgba(255, 255, 255, 0.09f);
		constexpr ImU32 LinkTint = Rgba(106, 167, 255, 0.14f); // behind a blue icon
		constexpr ImU32 GoodTint = Rgba(61, 214, 140, 0.14f);
		constexpr ImU32 CautionTint = Rgba(255, 179, 64, 0.14f);
		constexpr ImU32 AlertTint = Rgba(255, 107, 97, 0.16f);
	}

	// -----------------------------------------------------------------------
	// Type
	// -----------------------------------------------------------------------

	// Mono is the design's monospace face for measurements and ids, at its
	// regular weight.
	enum class Weight { Regular, Medium, SemiBold, Bold, Mono };

	// A CSS size and line height in pixels. Lines are spaced lineHeight apart
	// and the glyphs sit in each line box the way a browser places them.
	struct TextStyle
	{
		Weight weight;
		float size;
		float lineHeight;
	};

	namespace type
	{
		constexpr TextStyle Display { Weight::Bold, 46.0f, 52.0f };       // the hero's verdict
		constexpr TextStyle Welcome { Weight::Bold, 42.0f, 48.0f };
		constexpr TextStyle SetupTitle { Weight::Bold, 40.0f, 46.0f };
		constexpr TextStyle PageTitle { Weight::Bold, 34.0f, 40.0f };
		constexpr TextStyle ResultTitle { Weight::Bold, 32.0f, 38.0f };
		constexpr TextStyle SheetTitle { Weight::Bold, 30.0f, 36.0f };
		constexpr TextStyle SheetHeading { Weight::Bold, 26.0f, 32.0f };
		constexpr TextStyle DialogTitle { Weight::Bold, 24.0f, 30.0f };
		constexpr TextStyle CardTitle { Weight::Bold, 22.0f, 28.0f };
		constexpr TextStyle Section { Weight::SemiBold, 20.0f, 26.0f };
		constexpr TextStyle Lead { Weight::Regular, 19.0f, 27.0f };       // under a sheet's title
		constexpr TextStyle HeroBody { Weight::Regular, 19.0f, 26.0f };
		constexpr TextStyle Intro { Weight::Regular, 19.0f, 28.0f };      // under a setup title
		constexpr TextStyle RowTitle { Weight::Medium, 18.0f, 23.0f };
		constexpr TextStyle Label { Weight::SemiBold, 18.0f, 23.0f };
		constexpr TextStyle Item { Weight::SemiBold, 19.0f, 24.0f };       // a device or choice
		constexpr TextStyle Body { Weight::Regular, 17.0f, 24.0f };
		constexpr TextStyle BodyMedium { Weight::Medium, 17.0f, 22.0f };
		constexpr TextStyle Strong { Weight::SemiBold, 17.0f, 22.0f };
		constexpr TextStyle Eyebrow { Weight::SemiBold, 17.0f, 22.0f };    // "Alignment"
		constexpr TextStyle Callout { Weight::Regular, 16.0f, 21.0f };
		constexpr TextStyle CalloutMedium { Weight::Medium, 16.0f, 21.0f };
		constexpr TextStyle Footnote { Weight::Regular, 15.0f, 20.0f };
		constexpr TextStyle FootnoteStrong { Weight::SemiBold, 15.0f, 20.0f };
		constexpr TextStyle Caption { Weight::Regular, 14.0f, 18.0f };
		constexpr TextStyle CaptionStrong { Weight::SemiBold, 14.0f, 18.0f };
		constexpr TextStyle Tag { Weight::SemiBold, 13.0f, 18.0f };
	}

	void SetFont(Weight weight, ImFont *font);
	ImFont *FontOf(Weight weight);
	// ImGui sizes a font by its ascent-to-descent height, a browser by its em.
	float GlyphSize(float cssSize);

	// One line, no wrapping: its advance width and line height.
	ImVec2 MeasureLine(const TextStyle &style, const char *text, const char *end = nullptr);

	// The longest start of text that fits width on one line, with an ellipsis
	// when it had to be cut.
	std::string Ellipsize(const TextStyle &style, const std::string &text, float width);

	// Text broken into lines that fit maxWidth (no limit when maxWidth <= 0),
	// at word boundaries, or between characters in Japanese, following its
	// line-breaking rules.
	struct TextLines
	{
		std::vector<std::pair<const char *, const char *>> lines;
		float width = 0.0f;
		float height = 0.0f;
	};
	TextLines WrapText(const TextStyle &style, const char *text, float maxWidth);

	enum class Align { Left, Center, Right };
	void DrawLines(ImDrawList *dl, const TextStyle &style, const TextLines &lines, ImVec2 topLeft,
		float boxWidth, ImU32 colour, Align align = Align::Left);
	// Wraps text to width and draws it; returns the height it took.
	float DrawText(ImDrawList *dl, const TextStyle &style, ImVec2 topLeft, float width, ImU32 colour,
		const char *text, Align align = Align::Left);
	// One line with its line box's top-left corner at topLeft.
	void DrawLine(ImDrawList *dl, const TextStyle &style, ImVec2 topLeft, ImU32 colour, const char *text,
		const char *end = nullptr);

	// A Yoga leaf sized to text wrapped at whatever width the layout gives it.
	// The text is copied, so a temporary is fine.
	YGNodeRef TextNode(FlexLayout &fl, YGNodeRef parent, const TextStyle &style, const std::string &text);

	// -----------------------------------------------------------------------
	// Surfaces
	// -----------------------------------------------------------------------

	void FillRounded(ImDrawList *dl, ImVec2 a, ImVec2 b, ImU32 colour, float rounding, ImDrawFlags flags = 0);
	// A vertical gradient from top to bottom, rounded like FillRounded.
	void FillGradient(ImDrawList *dl, ImVec2 a, ImVec2 b, ImU32 top, ImU32 bottom, float rounding,
		ImDrawFlags flags = 0);
	// A 1 px line just inside the shape's edge (CSS "inset 0 0 0 1px").
	void InnerEdge(ImDrawList *dl, ImVec2 a, ImVec2 b, ImU32 colour, float rounding, float thickness = 1.0f);
	// A soft shadow under a rounded shape: blur is the CSS blur radius, dy its
	// downward offset.
	void SoftShadow(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding, float blur, float dy, ImU32 colour);
	// An elliptical glow fading from inner at the centre to nothing at radius.
	void RadialGlow(ImDrawList *dl, ImVec2 centre, ImVec2 radius, ImU32 inner, int segments = 64);
	// The quiet raised card most content sits on.
	void Card(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding = 20.0f);
	void Hairline(ImDrawList *dl, float x0, float x1, float y);
	// The window's own backdrop: a dark gradient lit from the top left.
	void PageBackdrop(ImDrawList *dl, ImVec2 a, ImVec2 b);
	// A sheet's or dialog's raised panel.
	void PanelSurface(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding);

	// -----------------------------------------------------------------------
	// Icons: line art on a 24-unit grid, as the canvas draws them
	// -----------------------------------------------------------------------

	enum class Icon
	{
		None,
		Reticle, Identify, Chaperone, Anchor, Lighthouse, Smoothing, Gear,
		Chevron, ChevronLeft, ChevronUpDown, Close, Check, Plus, Minus,
		Trash, Download, Doc, Pencil, Info, Warn, Refresh, Power, Pause, Play,
		Book, Globe, Bell, Clock, ShieldCheck, Headset, Tracker, Controller, More,
		Replay, DownloadCircle,
		Count
	};
	// size is the drawn width of the 24-unit grid; stroke is in grid units.
	void DrawIcon(ImDrawList *dl, Icon icon, ImVec2 centre, float size, ImU32 colour, float stroke = 2.0f);

	// -----------------------------------------------------------------------
	// Controls
	// -----------------------------------------------------------------------

	// Keyboard focus, drawn the same on every hand-painted control.
	void FocusRing(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding);

	enum class Btn { Primary, Secondary, Danger, Waiting, Link };
	// The width a pill needs for its label in the current language.
	float PillWidth(const char *english, float fontSize, Icon icon = Icon::None, float padX = 22.0f);
	// A rounded button filling r. Waiting is the look of a button that is not
	// ready yet: it never reports a press.
	bool PillButton(const char *id, const char *english, Btn kind, const FlexRect &r,
		Icon icon = Icon::None, float fontSize = 18.0f);
	// A round icon button, as the close button and the steppers' - and +.
	bool RoundButton(const char *id, ImVec2 centre, float diameter, Icon icon, float iconSize,
		ImU32 fill, ImU32 ink);
	bool CloseButton(const char *id, ImVec2 topLeft);

	// The 58 x 34 switch. Returns whether value changed.
	bool ToggleSwitch(const char *id, bool &value, ImVec2 topLeft);
	constexpr float kSwitchW = 58.0f;
	constexpr float kSwitchH = 34.0f;

	// A row of choices in a rounded track; the chosen one carries a lit pill.
	float PillSegmentedWidth(const char *const items[], int count, float fontSize, float minItemW, bool translate = true);
	// lead leaves room inside the track, before the first choice, for an icon
	// the caller draws.
	int PillSegmented(const char *id, int value, const char *const items[], int count, const FlexRect &r,
		float fontSize = 16.0f, bool translate = true, float lead = 0.0f);

	// A status pill with a coloured dot. The text is the caller's, translated.
	float ChipWidth(const char *text);
	void Chip(ImDrawList *dl, ImVec2 topLeft, ImU32 dot, const char *text);
	constexpr float kChipH = 40.0f;

	// A small label pill ("Experimental", "Untested", "Soon").
	float BadgeWidth(const char *english);
	void BadgePill(ImDrawList *dl, ImVec2 topLeft, const char *english, ImU32 ink, ImU32 fill, float height = 26.0f);

	// A sidebar link: icon and label, lit when active. A disabled link shows
	// its tip on hover and an optional badge at its end.
	bool NavRow(const char *id, const FlexRect &r, Icon icon, const char *english, bool active, bool disabled,
		const char *badgeEnglish = nullptr, const char *disabledTip = nullptr);

	// A tool tile: icon badge, title and sub-line (translated by the caller).
	bool ToolTile(const char *id, const FlexRect &r, Icon icon, const char *english, const char *sub,
		bool chevron, bool disabled);

	// The calibration steps: Get set, Move, Check. current is lit; a step in
	// doneMask has a tick and failed (-1 for none) a cross.
	void Stepper(ImDrawList *dl, ImVec2 topLeft, int current, unsigned doneMask, int failed);

	// A live measurement: name, state in colour, and a filled bar.
	void Meter(ImDrawList *dl, const FlexRect &r, const char *name, const char *state, ImU32 colour, float fraction);

	// The big round state mark: rings and a coloured core.
	enum class Mark { Good, Caution, None, Paused, Bad, Off };
	void StateMark(ImDrawList *dl, ImVec2 centre, Mark mark, float scale = 1.0f);

	// The round tinted icon at the top of a dialog.
	void RoundIcon(ImDrawList *dl, ImVec2 centre, float diameter, Icon icon, float iconSize, ImU32 ink, ImU32 fill);

	// A tooltip beside the pointer (or under the focused control).
	void Tip(const char *english);

	// A menu of actions dropped from a control: ImGui::OpenPopup(id) opens
	// it; while BeginMenuPopup returns true, add MenuItems and close it with
	// EndMenuPopup. pivot places the menu's corner at anchor ((1, 0) hangs it
	// from its top-right corner).
	bool BeginMenuPopup(const char *id, ImVec2 anchor, ImVec2 pivot, float width);
	void EndMenuPopup();
	// One action in the menu; pressing it also closes the menu.
	bool MenuItem(const char *id, Icon icon, const char *english, ImU32 ink = col::Text);
	constexpr float kMenuItemH = 44.0f;
}
