// The ImGui theme, the vector device glyphs, textures and the tooltip.
#include "stdafx.h"
#include "UiInternal.h"

// ---------------------------------------------------------------------------
// Theme
// ---------------------------------------------------------------------------

// What ImGui's own widgets look like: the text fields, popups, tooltips and
// scrollbars the hand-painted controls leave to it, in the design kit's
// colours.
void ApplyTheme()
{
	ImGuiStyle &s = ImGui::GetStyle();

	s.WindowPadding     = ImVec2(28, 24);
	s.FramePadding      = ImVec2(14, 10);
	s.ItemSpacing       = ImVec2(12, 8);
	s.ItemInnerSpacing  = ImVec2(8, 6);
	s.WindowRounding    = 0.0f;
	s.FrameRounding     = 9.0f;
	s.PopupRounding     = 14.0f;
	s.ChildRounding     = 12.0f;
	s.GrabRounding      = 9.0f;
	s.ScrollbarRounding = 8.0f;
	s.ScrollbarSize     = 10.0f;
	s.WindowBorderSize  = 0.0f;
	s.ChildBorderSize   = 0.0f;
	s.PopupBorderSize   = 1.0f;
	s.FrameBorderSize   = 1.0f;

	auto v = [](ImU32 colour) { return ImGui::ColorConvertU32ToFloat4(colour); };
	const ImVec4 accent = v(ui::col::Accent);
	ImVec4 *c = s.Colors;
	c[ImGuiCol_Text]                 = v(ui::col::Text);
	c[ImGuiCol_TextDisabled]         = v(ui::col::Faint);
	c[ImGuiCol_WindowBg]             = v(ui::Rgba(29, 31, 35));
	c[ImGuiCol_ChildBg]              = ImVec4(0, 0, 0, 0);
	// Tooltips and menus: the colour of the sheets' panels.
	c[ImGuiCol_PopupBg]              = v(ui::Rgba(52, 55, 62, 0.98f));
	c[ImGuiCol_Border]               = v(ui::col::SheetEdge);
	c[ImGuiCol_BorderShadow]         = ImVec4(0, 0, 0, 0);
	c[ImGuiCol_FrameBg]              = v(ui::col::Fill);
	c[ImGuiCol_FrameBgHovered]       = v(ui::col::Secondary);
	c[ImGuiCol_FrameBgActive]        = v(ui::col::Selected);
	// The window is a fixed 1200x800 and a long page overflows it, so the
	// scrollbar is the only sign that more exists below the fold. It has to be
	// visible at rest.
	c[ImGuiCol_ScrollbarBg]          = ImVec4(1, 1, 1, 0.0f);
	c[ImGuiCol_ScrollbarGrab]        = ImVec4(1, 1, 1, 0.34f);
	c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(1, 1, 1, 0.46f);
	c[ImGuiCol_ScrollbarGrabActive]  = ImVec4(1, 1, 1, 0.56f);
	c[ImGuiCol_CheckMark]            = accent;
	c[ImGuiCol_SliderGrab]           = accent;
	c[ImGuiCol_SliderGrabActive]     = accent;
	c[ImGuiCol_Button]               = v(ui::col::Fill);
	c[ImGuiCol_ButtonHovered]        = v(ui::col::Secondary);
	c[ImGuiCol_ButtonActive]         = v(ui::col::Selected);
	c[ImGuiCol_Header]               = ImVec4(accent.x, accent.y, accent.z, 0.35f);
	c[ImGuiCol_HeaderHovered]        = ImVec4(accent.x, accent.y, accent.z, 0.25f);
	c[ImGuiCol_HeaderActive]         = ImVec4(accent.x, accent.y, accent.z, 0.45f);
	c[ImGuiCol_Separator]            = v(ui::col::Hairline);
	c[ImGuiCol_TextSelectedBg]       = ImVec4(accent.x, accent.y, accent.z, 0.35f);
	// The scrim behind sheets and dialogs: the page stays legible as context
	// but stops competing for attention.
	c[ImGuiCol_ModalWindowDimBg]     = v(ui::col::Scrim);
	c[ImGuiCol_NavHighlight]         = accent;
}

// ---------------------------------------------------------------------------
// Device glyphs (line style, s = half-extent in pixels), for a device with
// no picture of its own
// ---------------------------------------------------------------------------

void IconHMD(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 a = ImVec2(c.x - s, c.y - s * 0.60f);
	ImVec2 b = ImVec2(c.x + s, c.y + s * 0.44f);
	dl->PathRect(a, b, s * 0.34f);
	dl->PathStroke(col, 2.2f, ImDrawFlags_Closed);
	dl->AddCircleFilled(ImVec2(c.x - s * 0.42f, c.y - s * 0.06f), s * 0.16f, col, 12);
	dl->AddCircleFilled(ImVec2(c.x + s * 0.42f, c.y - s * 0.06f), s * 0.16f, col, 12);
}

void IconController(ImDrawList *dl, ImVec2 c, float s, ImU32 col, bool leftHand)
{
	float m = leftHand ? -1.0f : 1.0f;
	// Handle first, then the tracking ring drawn over it (same color, so the
	// overlap disappears and the silhouette reads as one piece).
	ImVec2 h0 = ImVec2(c.x + m * s * 0.16f, c.y - s * 0.10f);
	ImVec2 h1 = ImVec2(c.x + m * s * 0.34f, c.y + s * 0.78f);
	float th = s * 0.38f;
	dl->AddLine(h0, h1, col, th);
	dl->AddCircleFilled(h0, th * 0.5f, col, 12);
	dl->AddCircleFilled(h1, th * 0.5f, col, 12);
	ImVec2 ringC = ImVec2(c.x - m * s * 0.10f, c.y - s * 0.32f);
	dl->AddCircle(ringC, s * 0.52f, col, 24, 2.4f);
}

void IconTracker(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.68f, col, 24, 2.2f);
	dl->AddCircleFilled(c, s * 0.15f, col, 10);
	for (int i = 0; i < 3; ++i)
	{
		float a = -IM_PI * 0.5f + (float)i * (2.0f * IM_PI / 3.0f);
		dl->AddCircleFilled(ImVec2(c.x + cosf(a) * s * 0.40f, c.y + sinf(a) * s * 0.40f), s * 0.09f, col, 8);
	}
}

// ---------------------------------------------------------------------------
// SteamVR device icon textures
// ---------------------------------------------------------------------------

// Owns one COM interface for the rest of its scope, so each acquisition below
// is one line plus one guard.
template<typename T>
struct ComScoped
{
	ComScoped() = default;
	ComScoped(const ComScoped &) = delete;
	ComScoped &operator=(const ComScoped &) = delete;
	~ComScoped() { if (ptr) ptr->Release(); }

	T **Put() { return &ptr; }
	T *Get() const { return ptr; }
	T *operator->() const { return ptr; }

	T *ptr = nullptr;
};

static bool DecodeTexture(IWICImagingFactory *factory, IWICBitmapDecoder *decoder,
                         GLuint *outTex, int *outW, int *outH, bool guide)
{
	ComScoped<IWICBitmapFrameDecode> frame;
	if (FAILED(decoder->GetFrame(0, frame.Put())))
		return false;

	ComScoped<IWICFormatConverter> conv;
	if (FAILED(factory->CreateFormatConverter(conv.Put())))
		return false;
	if (FAILED(conv->Initialize(frame.Get(), GUID_WICPixelFormat32bppRGBA,
		WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom)))
		return false;

	// Device icons come from driver folders and get a small allocation bound.
	// An embedded guide must be one of the two atlas layouts UiGuide.cpp's UVs
	// assume.
	UINT w = 0, h = 0;
	if (FAILED(conv->GetSize(&w, &h)))
		return false;
	const bool validDimensions = guide
		? ((w == 4608 && h == 5880) || (w == 5120 && h == 5760))
		: (w > 0 && h > 0 && w <= 1024 && h <= 1024);
	if (!validDimensions)
		return false;

	std::vector<unsigned char> pixels((size_t)w * h * 4);
	if (FAILED(conv->CopyPixels(nullptr, w * 4, (UINT)pixels.size(), pixels.data())))
		return false;

	GLuint tex = 0;
	glGenTextures(1, &tex);
	if (!tex)
		return false;
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
	GLint allocatedWidth = 0;
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &allocatedWidth);
	if (allocatedWidth != static_cast<GLint>(w))
	{
		glDeleteTextures(1, &tex);
		return false;
	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	*outTex = tex;
	*outW = (int)w;
	*outH = (int)h;
	return true;
}

bool LoadTextureFromFile(const char *path, GLuint *outTex, int *outW, int *outH)
{
	static const HRESULT comInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
	(void)comInit;
	ComScoped<IWICImagingFactory> factory;
	if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
		IID_PPV_ARGS(factory.Put()))))
		return false;
	wchar_t wpath[MAX_PATH];
	if (!MultiByteToWideChar(CP_UTF8, 0, path, -1, wpath, MAX_PATH))
		return false;
	ComScoped<IWICBitmapDecoder> decoder;
	if (FAILED(factory->CreateDecoderFromFilename(wpath, nullptr, GENERIC_READ,
		WICDecodeMetadataCacheOnDemand, decoder.Put())))
		return false;
	return DecodeTexture(factory.Get(), decoder.Get(), outTex, outW, outH, false);
}

// A built-in PNG (an RCDATA resource) decoded into a texture: a guide atlas,
// whose layout is checked, or a small piece of art.
static bool LoadResourceTexture(const char *name, bool guide, GLuint *outTex, int *outW, int *outH)
{
	HRSRC resource = FindResourceA(nullptr, name, MAKEINTRESOURCEA(10));
	if (!resource)
		return false;
	HGLOBAL loaded = LoadResource(nullptr, resource);
	auto bytes = static_cast<BYTE *>(LockResource(loaded));
	DWORD size = SizeofResource(nullptr, resource);
	if (!bytes || size == 0)
		return false;
	static const HRESULT comInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
	(void)comInit;
	ComScoped<IWICImagingFactory> factory;
	if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
		IID_PPV_ARGS(factory.Put()))))
		return false;
	ComScoped<IWICStream> stream;
	if (FAILED(factory->CreateStream(stream.Put())) ||
		FAILED(stream->InitializeFromMemory(bytes, size)))
		return false;
	ComScoped<IWICBitmapDecoder> decoder;
	if (FAILED(factory->CreateDecoderFromStream(stream.Get(), nullptr,
		WICDecodeMetadataCacheOnDemand, decoder.Put())))
		return false;
	return DecodeTexture(factory.Get(), decoder.Get(), outTex, outW, outH, guide);
}

bool LoadGuideTexture(GuideDemo demo, GLuint *outTex)
{
	const char *name = demo == GuideDemo::Mounted ? "GUIDE_HEADSET"
		: demo == GuideDemo::HeadsetContact ? "GUIDE_CONTACT" : "GUIDE_HANDHELD";
	int width = 0, height = 0;
	return LoadResourceTexture(name, true, outTex, &width, &height);
}

// Kept for the life of the process, like the device icons; loaded lazily on
// the render thread (GL context current).
const DeviceIconTex *ArtTexture(const char *resource)
{
	static std::map<std::string, DeviceIconTex> cache;
	auto it = cache.find(resource);
	if (it == cache.end())
	{
		DeviceIconTex t;
		if (!LoadResourceTexture(resource, false, &t.tex, &t.w, &t.h))
			t.failed = true;
		it = cache.emplace(resource, t).first;
	}
	return it->second.failed ? nullptr : &it->second;
}

void DrawDeviceArt(ImDrawList *dl, const VRDevice &dev, const FlexRect &box, ImU32 fallbackInk)
{
	const DeviceIconTex *tex = GetDeviceIconTex(dev.iconPath);
	if (!tex)
	{
		DeviceIcon(dl, dev, box.Center(), std::min(box.W(), box.H()) * 0.32f, fallbackInk);
		return;
	}
	const float scale = std::min(box.W() / static_cast<float>(tex->w), box.H() / static_cast<float>(tex->h));
	const ImVec2 half(static_cast<float>(tex->w) * scale * 0.5f, static_cast<float>(tex->h) * scale * 0.5f);
	const ImVec2 c = box.Center();
	dl->AddImage(static_cast<ImTextureID>(tex->tex), ImVec2(c.x - half.x, c.y - half.y), ImVec2(c.x + half.x, c.y + half.y));
}

const std::string &GuideModelCredits()
{
	static const std::string credits = []() -> std::string {
		HRSRC resource = FindResourceA(nullptr, "GUIDE_CREDITS", MAKEINTRESOURCEA(10));
		if (resource)
		{
			const char *bytes = static_cast<const char *>(LockResource(LoadResource(nullptr, resource)));
			if (bytes)
				return std::string(bytes, SizeofResource(nullptr, resource));
		}
		return "Motion demo credits couldn't load. Reinstall QuestCalibrator to restore them.";
	}();
	return credits;
}

// Keyed by absolute path; loaded lazily on the render thread (GL context current).
static std::map<std::string, DeviceIconTex> s_deviceIconCache;

const DeviceIconTex *GetDeviceIconTex(const std::string &path)
{
	if (path.empty())
		return nullptr;

	auto it = s_deviceIconCache.find(path);
	if (it == s_deviceIconCache.end())
	{
		DeviceIconTex t;
		if (!LoadTextureFromFile(path.c_str(), &t.tex, &t.w, &t.h))
			t.failed = true;
		it = s_deviceIconCache.emplace(path, t).first;
	}
	return it->second.failed ? nullptr : &it->second;
}

bool FileExists(const std::string &path)
{
	DWORD attrs = GetFileAttributesA(path.c_str());
	return attrs != INVALID_FILE_ATTRIBUTES && !(attrs & FILE_ATTRIBUTE_DIRECTORY);
}

// SteamVR ships icons in 1x and 2x; prefer the sharper one when present.
std::string Prefer2x(const std::string &path)
{
	size_t dot = path.find_last_of('.');
	if (dot != std::string::npos)
	{
		std::string hi = path.substr(0, dot) + "_2x" + path.substr(dot);
		if (FileExists(hi))
			return hi;
	}
	return path;
}

// ---------------------------------------------------------------------------
// Widgets
// ---------------------------------------------------------------------------


// Escape as the keyboard form of a modal's Cancel / Keep / Close.
bool EscapePressed()
{
	return ImGui::IsKeyPressed(ImGuiKey_Escape, false);
}

void ShowTip(const char *english, bool leftOfCursor)
{
	const char *text = Tr(english);
	// Nothing behind a modal may raise a tooltip: a rect-based hover test does
	// not know the modal is there, and a tooltip window appearing takes focus
	// from it.
	if (ImGuiWindow *modal = ImGui::GetTopMostPopupModal())
	{
		if (ImGui::GetCurrentWindowRead()->RootWindow != modal->RootWindow)
			return;
	}
	ImGuiIO &io = ImGui::GetIO();
	ImFont *font = ui::FontOf(ui::Weight::Regular);
	const float size = ui::GlyphSize(16.0f);
	const ImVec2 pad(14.0f, 10.0f);
	const ImVec2 textSize = font->CalcTextSizeA(size, FLT_MAX, 0.0f, text);
	const float w = textSize.x + pad.x * 2.0f + 2.0f, h = textSize.y + pad.y * 2.0f;
	// Above the cursor in the lower half of the window, and never past the
	// right edge: the window is fixed-size, so a tooltip raised near the
	// edge would otherwise be cut mid-sentence.
	// Raised by keyboard focus rather than the pointer, the tip anchors to
	// the control itself: the mouse may be parked over something else.
	const ImVec2 itemMin = ImGui::GetItemRectMin(), itemMax = ImGui::GetItemRectMax();
	const bool byFocus = io.NavVisible && !ImGui::IsMouseHoveringRect(itemMin, itemMax);
	const ImVec2 at = byFocus ? ImVec2(itemMin.x, itemMax.y) : io.MousePos;
	// Tools inside a list ask for the tip beside the cursor, so it never
	// lands on the row beneath.
	float x = leftOfCursor ? at.x - w - 16.0f
		: std::min(at.x + 16.0f, io.DisplaySize.x - w - 12.0f);
	float y = leftOfCursor ? at.y - h * 0.5f
		: at.y > io.DisplaySize.y * 0.5f ? (byFocus ? itemMin.y : at.y) - h - 8.0f : at.y + (byFocus ? 8.0f : 20.0f);
	ImGui::SetNextWindowPos(ImVec2(std::max(8.0f, x), y));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, pad);
	ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 12.0f);
	ImGui::PushFont(font, size);
	ImGui::BeginTooltip();
	ImGui::TextUnformatted(text);
	ImGui::EndTooltip();
	ImGui::PopFont();
	ImGui::PopStyleVar(2);
}

std::string FormatString(const char *fmt, ...)
{
	char buf[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof buf, fmt, args);
	va_end(args);
	return std::string(buf);
}
