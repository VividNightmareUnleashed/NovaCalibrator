#pragma once

// A small 3D picture of tracked devices for the Motion Recorder's page: a
// floor grid and each device as SteamVR's own render model at its position
// and orientation, seen from a camera that turns about the body. It is drawn
// into a texture of its own, which the page then places as an image.
//
// A device whose model SteamVR does not have (or while SteamVR is not there,
// as in a preview) is a plain rounded shape in its colour.

#include <string>
#include <vector>

namespace questcal::sceneview
{

enum class Shape { Headset, Controller, Tracker };

struct Item
{
	float position[3] = { 0.0f, 0.0f, 0.0f };          // meters
	float rotation[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // w x y z
	std::string renderModel;                           // SteamVR's name; empty for none
	Shape shape = Shape::Tracker;                      // what stands in for a missing model
	float colour[3] = { 1.0f, 1.0f, 1.0f };            // its edge light, and the stand-in's colour
	bool tracked = true;
};

// The camera looks at `target` from `distance` meters away. `yaw` turns it
// about the vertical (0 is from +z, looking along -z); `pitch` lifts it.
struct Camera
{
	float yaw = 0.0f;
	float pitch = 0.0f;
	float distance = 3.0f;
	float target[3] = { 0.0f, 1.0f, 0.0f };
};

// Draws the floor and the items into a texture of width by height pixels and
// returns its OpenGL name, or 0 when the picture cannot be made. The texture's
// first row is the picture's bottom row. Needs the window's OpenGL context;
// it leaves the context's state as it found it.
unsigned Render(int width, int height, const Camera &camera, const std::vector<Item> &items);

// Where a point of the scene is in that picture, in pixels from its top-left
// corner. False when the point is behind the camera.
bool Project(int width, int height, const Camera &camera, const float point[3], float &x, float &y);

inline bool ModelNameLetter(char c)
{
	return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
}

// Whether a render model name read from a recording may be given to SteamVR:
// a plain name, with an optional "{driver}" in front, and nothing that could
// be a path. A recording is untrusted input.
inline bool PlainModelName(const std::string &name)
{
	if (name.empty() || name.size() > 128)
		return false;
	size_t at = 0;
	if (name[0] == '{')
	{
		const size_t close = name.find('}');
		if (close == std::string::npos || close == 1)
			return false;
		for (size_t i = 1; i < close; ++i)
			if (!ModelNameLetter(name[i]))
				return false;
		at = close + 1;
	}
	if (at >= name.size() || name[at] == '.')
		return false;
	for (; at < name.size(); ++at)
	{
		const char c = name[at];
		const bool allowed = ModelNameLetter(c) || c == '-' || c == '.';
		if (!allowed)
			return false;
		if (c == '.' && at + 1 < name.size() && name[at + 1] == '.')
			return false;
	}
	return true;
}

} // namespace questcal::sceneview
