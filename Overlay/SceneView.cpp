#include "stdafx.h"
#include "SceneView.h"

#include <GL/gl3w.h>
#include <openvr.h>

#include <algorithm>
#include <cmath>
#include <map>

namespace questcal::sceneview
{

namespace
{

// ---------------------------------------------------------------------------
// Matrices, column-major as OpenGL takes them

struct Mat4
{
	float m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
};

Mat4 Multiply(const Mat4 &a, const Mat4 &b)
{
	Mat4 out;
	for (int column = 0; column < 4; ++column)
		for (int row = 0; row < 4; ++row)
		{
			float sum = 0.0f;
			for (int k = 0; k < 4; ++k)
				sum += a.m[k * 4 + row] * b.m[column * 4 + k];
			out.m[column * 4 + row] = sum;
		}
	return out;
}

constexpr float FieldOfView = 0.70f;   // radians, top to bottom
constexpr float NearPlane = 0.05f;
constexpr float FarPlane = 60.0f;

void EyeOf(const Camera &camera, float eye[3])
{
	const float c = std::cos(camera.pitch);
	eye[0] = camera.target[0] + camera.distance * std::sin(camera.yaw) * c;
	eye[1] = camera.target[1] + camera.distance * std::sin(camera.pitch);
	eye[2] = camera.target[2] + camera.distance * std::cos(camera.yaw) * c;
}

// World to clip space for a picture of this shape.
Mat4 ViewProjection(int width, int height, const Camera &camera)
{
	float eye[3];
	EyeOf(camera, eye);
	// The camera's axes: f looks at the target, r is to its right, u is up.
	float f[3] = { camera.target[0] - eye[0], camera.target[1] - eye[1], camera.target[2] - eye[2] };
	const float fl = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
	for (float &v : f)
		v /= fl > 1e-6f ? fl : 1.0f;
	// Straight down has no "up" of its own: lean it a hair.
	float r[3] = { -f[2], 0.0f, f[0] };
	float rl = std::sqrt(r[0] * r[0] + r[2] * r[2]);
	if (rl < 1e-4f)
	{
		r[0] = std::cos(camera.yaw);
		r[2] = -std::sin(camera.yaw);
		rl = 1.0f;
	}
	r[0] /= rl;
	r[2] /= rl;
	const float u[3] = { r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0] };

	Mat4 view;
	view.m[0] = r[0]; view.m[4] = r[1]; view.m[8] = r[2];
	view.m[1] = u[0]; view.m[5] = u[1]; view.m[9] = u[2];
	view.m[2] = -f[0]; view.m[6] = -f[1]; view.m[10] = -f[2];
	view.m[12] = -(r[0] * eye[0] + r[1] * eye[1] + r[2] * eye[2]);
	view.m[13] = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
	view.m[14] = f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2];

	Mat4 projection;
	const float t = 1.0f / std::tan(FieldOfView * 0.5f);
	const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
	projection.m[0] = t / aspect;
	projection.m[5] = t;
	projection.m[10] = (FarPlane + NearPlane) / (NearPlane - FarPlane);
	projection.m[11] = -1.0f;
	projection.m[14] = 2.0f * FarPlane * NearPlane / (NearPlane - FarPlane);
	projection.m[15] = 0.0f;
	return Multiply(projection, view);
}

// A position and a unit quaternion (w x y z) as a model matrix.
Mat4 ModelMatrix(const float position[3], const float q[4])
{
	float w = q[0], x = q[1], y = q[2], z = q[3];
	const float n = std::sqrt(w * w + x * x + y * y + z * z);
	if (!(n > 1e-6f) || !std::isfinite(n))
	{
		w = 1.0f;
		x = y = z = 0.0f;
	}
	else
	{
		w /= n; x /= n; y /= n; z /= n;
	}
	Mat4 out;
	out.m[0] = 1 - 2 * (y * y + z * z); out.m[1] = 2 * (x * y + w * z); out.m[2] = 2 * (x * z - w * y);
	out.m[4] = 2 * (x * y - w * z); out.m[5] = 1 - 2 * (x * x + z * z); out.m[6] = 2 * (y * z + w * x);
	out.m[8] = 2 * (x * z + w * y); out.m[9] = 2 * (y * z - w * x); out.m[10] = 1 - 2 * (x * x + y * y);
	out.m[12] = position[0]; out.m[13] = position[1]; out.m[14] = position[2];
	return out;
}

// ---------------------------------------------------------------------------
// OpenGL objects, made on first use and kept

const char *const MeshVertex = R"(#version 150
in vec3 aPosition;
in vec3 aNormal;
in vec2 aUv;
uniform mat4 uViewProjection;
uniform mat4 uModel;
uniform vec3 uScale;
out vec3 vNormal;
out vec3 vWorld;
out vec2 vUv;
void main()
{
	vec4 world = uModel * vec4(aPosition * uScale, 1.0);
	vWorld = world.xyz;
	vNormal = mat3(uModel) * (aNormal / uScale);
	vUv = aUv;
	gl_Position = uViewProjection * world;
}
)";

// A lit surface with a soft edge light in the device's colour, so that a
// black tracker still reads against the dark floor.
const char *const MeshFragment = R"(#version 150
in vec3 vNormal;
in vec3 vWorld;
in vec2 vUv;
uniform sampler2D uTexture;
uniform int uTextured;
uniform vec3 uColour;
uniform vec3 uEye;
uniform float uDim;
out vec4 oColour;
void main()
{
	vec3 n = normalize(vNormal);
	vec3 toEye = normalize(uEye - vWorld);
	if (dot(n, toEye) < 0.0)
		n = -n;
	float light = 0.42 + 0.58 * max(dot(n, normalize(vec3(0.35, 0.85, 0.40))), 0.0);
	vec3 base = uTextured != 0 ? texture(uTexture, vUv).rgb : uColour;
	float edge = pow(1.0 - max(dot(n, toEye), 0.0), 2.0);
	vec3 colour = base * light + uColour * (0.10 + 0.45 * edge);
	oColour = vec4(mix(colour, vec3(0.36, 0.37, 0.40), uDim), 1.0);
}
)";

const char *const LineVertex = R"(#version 150
in vec3 aPosition;
in vec4 aColour;
uniform mat4 uViewProjection;
out vec4 vColour;
void main()
{
	vColour = aColour;
	gl_Position = uViewProjection * vec4(aPosition, 1.0);
}
)";

const char *const LineFragment = R"(#version 150
in vec4 vColour;
out vec4 oColour;
void main()
{
	oColour = vColour;
}
)";

GLuint Compile(GLenum kind, const char *source)
{
	const GLuint shader = glCreateShader(kind);
	glShaderSource(shader, 1, &source, nullptr);
	glCompileShader(shader);
	GLint ok = GL_FALSE;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
	if (ok == GL_TRUE)
		return shader;
	glDeleteShader(shader);
	return 0;
}

GLuint Link(const char *vertex, const char *fragment, const char *const attributes[], int attributeCount)
{
	const GLuint vs = Compile(GL_VERTEX_SHADER, vertex), fs = Compile(GL_FRAGMENT_SHADER, fragment);
	GLuint program = 0;
	if (vs && fs)
	{
		program = glCreateProgram();
		glAttachShader(program, vs);
		glAttachShader(program, fs);
		for (int i = 0; i < attributeCount; ++i)
			glBindAttribLocation(program, static_cast<GLuint>(i), attributes[i]);
		glLinkProgram(program);
		GLint ok = GL_FALSE;
		glGetProgramiv(program, GL_LINK_STATUS, &ok);
		if (ok != GL_TRUE)
		{
			glDeleteProgram(program);
			program = 0;
		}
	}
	if (vs)
		glDeleteShader(vs);
	if (fs)
		glDeleteShader(fs);
	return program;
}

struct Vertex
{
	float position[3];
	float normal[3];
	float uv[2];
};

struct Mesh
{
	enum class State { Loading, Ready, Failed };
	State state = State::Loading;
	GLuint vao = 0, vbo = 0, ebo = 0, texture = 0;
	GLsizei indices = 0;
	// SteamVR's model while its texture is still on the way.
	vr::RenderModel_t *waiting = nullptr;
};

void Upload(Mesh &mesh, const Vertex *vertices, size_t vertexCount, const uint16_t *indices, size_t indexCount)
{
	glGenVertexArrays(1, &mesh.vao);
	glBindVertexArray(mesh.vao);
	glGenBuffers(1, &mesh.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
	glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertexCount * sizeof(Vertex)), vertices, GL_STATIC_DRAW);
	glGenBuffers(1, &mesh.ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indexCount * sizeof(uint16_t)), indices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<const void *>(offsetof(Vertex, position)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<const void *>(offsetof(Vertex, normal)));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<const void *>(offsetof(Vertex, uv)));
	mesh.indices = static_cast<GLsizei>(indexCount);
	mesh.state = Mesh::State::Ready;
}

struct Scene
{
	bool tried = false, usable = false;
	GLuint meshProgram = 0, lineProgram = 0;
	Mesh sphere;
	GLuint lineVao = 0, lineVbo = 0;
	// The picture: drawn with several samples a pixel where the driver has
	// them, then resolved into `texture`.
	int width = 0, height = 0, samples = 0;
	GLuint drawFbo = 0, drawColour = 0, drawDepth = 0, resolveFbo = 0, texture = 0;
	std::map<std::string, Mesh> models;
};
Scene g;

// More names than any rig has are a file's invention.
constexpr size_t MaxModels = 48;

bool Prepare()
{
	if (g.tried)
		return g.usable;
	g.tried = true;
	static const char *const meshAttributes[] = { "aPosition", "aNormal", "aUv" };
	static const char *const lineAttributes[] = { "aPosition", "aColour" };
	g.meshProgram = Link(MeshVertex, MeshFragment, meshAttributes, 3);
	g.lineProgram = Link(LineVertex, LineFragment, lineAttributes, 2);
	if (!g.meshProgram || !g.lineProgram)
		return false;

	// The stand-in for a device with no model: a ball, which each shape then
	// stretches to its own proportions.
	constexpr int rings = 12, segments = 20;
	std::vector<Vertex> vertices;
	std::vector<uint16_t> indices;
	vertices.reserve((rings + 1) * (segments + 1));
	for (int ring = 0; ring <= rings; ++ring)
		for (int segment = 0; segment <= segments; ++segment)
		{
			const float v = static_cast<float>(ring) / rings, u = static_cast<float>(segment) / segments;
			const float polar = v * 3.14159265f, around = u * 6.2831853f;
			Vertex vertex{};
			vertex.position[0] = std::sin(polar) * std::cos(around);
			vertex.position[1] = std::cos(polar);
			vertex.position[2] = std::sin(polar) * std::sin(around);
			std::copy(vertex.position, vertex.position + 3, vertex.normal);
			vertex.uv[0] = u;
			vertex.uv[1] = v;
			vertices.push_back(vertex);
		}
	indices.reserve(rings * segments * 6);
	for (int ring = 0; ring < rings; ++ring)
		for (int segment = 0; segment < segments; ++segment)
		{
			const int a = ring * (segments + 1) + segment, b = a + segments + 1;
			for (int index : { a, b, a + 1, a + 1, b, b + 1 })
				indices.push_back(static_cast<uint16_t>(index));
		}
	Upload(g.sphere, vertices.data(), vertices.size(), indices.data(), indices.size());

	glGenVertexArrays(1, &g.lineVao);
	glBindVertexArray(g.lineVao);
	glGenBuffers(1, &g.lineVbo);
	glBindBuffer(GL_ARRAY_BUFFER, g.lineVbo);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), nullptr);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), reinterpret_cast<const void *>(3 * sizeof(float)));
	g.usable = true;
	return true;
}

bool Resize(int width, int height)
{
	if (g.texture && width == g.width && height == g.height)
		return true;
	if (g.texture)
	{
		glDeleteFramebuffers(1, &g.drawFbo);
		glDeleteFramebuffers(1, &g.resolveFbo);
		glDeleteRenderbuffers(1, &g.drawColour);
		glDeleteRenderbuffers(1, &g.drawDepth);
		glDeleteTextures(1, &g.texture);
		g.texture = 0;
	}
	GLint most = 0;
	glGetIntegerv(GL_MAX_SAMPLES, &most);
	g.samples = (std::min)(4, (std::max)(0, static_cast<int>(most)));
	if (g.samples < 2)
		g.samples = 0;

	glGenTextures(1, &g.texture);
	glBindTexture(GL_TEXTURE_2D, g.texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glGenFramebuffers(1, &g.resolveFbo);
	glBindFramebuffer(GL_FRAMEBUFFER, g.resolveFbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g.texture, 0);

	glGenFramebuffers(1, &g.drawFbo);
	glBindFramebuffer(GL_FRAMEBUFFER, g.drawFbo);
	glGenRenderbuffers(1, &g.drawColour);
	glBindRenderbuffer(GL_RENDERBUFFER, g.drawColour);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, g.samples, GL_RGBA8, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, g.drawColour);
	glGenRenderbuffers(1, &g.drawDepth);
	glBindRenderbuffer(GL_RENDERBUFFER, g.drawDepth);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, g.samples, GL_DEPTH_COMPONENT24, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, g.drawDepth);
	const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
	g.width = width;
	g.height = height;
	return complete;
}

// SteamVR's model of this name, once it has arrived: its mesh, then its
// texture, each asked for again every frame until SteamVR has it.
Mesh *ModelOf(const std::string &name)
{
	if (!PlainModelName(name))
		return nullptr;
	auto found = g.models.find(name);
	if (found == g.models.end())
	{
		if (g.models.size() >= MaxModels)
			return nullptr;
		found = g.models.emplace(name, Mesh()).first;
	}
	Mesh &mesh = found->second;
	if (mesh.state != Mesh::State::Loading)
		return mesh.state == Mesh::State::Ready ? &mesh : nullptr;
	vr::IVRRenderModels *models = vr::VRRenderModels();
	if (!models)
		return nullptr;
	if (!mesh.waiting)
	{
		vr::RenderModel_t *model = nullptr;
		const vr::EVRRenderModelError error = models->LoadRenderModel_Async(name.c_str(), &model);
		if (error == vr::VRRenderModelError_Loading)
			return nullptr;
		if (error != vr::VRRenderModelError_None || !model || model->unVertexCount == 0 || model->unTriangleCount == 0)
		{
			if (model)
				models->FreeRenderModel(model);
			mesh.state = Mesh::State::Failed;
			return nullptr;
		}
		mesh.waiting = model;
	}
	vr::RenderModel_TextureMap_t *map = nullptr;
	if (mesh.waiting->diffuseTextureId >= 0)
	{
		const vr::EVRRenderModelError error = models->LoadTexture_Async(mesh.waiting->diffuseTextureId, &map);
		if (error == vr::VRRenderModelError_Loading)
			return nullptr;
		if (error != vr::VRRenderModelError_None)
			map = nullptr;
	}
	static_assert(sizeof(vr::RenderModel_Vertex_t) == sizeof(Vertex), "SteamVR's vertex is position, normal, uv");
	Upload(mesh, reinterpret_cast<const Vertex *>(mesh.waiting->rVertexData), mesh.waiting->unVertexCount,
		mesh.waiting->rIndexData, static_cast<size_t>(mesh.waiting->unTriangleCount) * 3);
	if (map && map->rubTextureMapData && map->unWidth > 0 && map->unHeight > 0)
	{
		glGenTextures(1, &mesh.texture);
		glBindTexture(GL_TEXTURE_2D, mesh.texture);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, map->unWidth, map->unHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE,
			map->rubTextureMapData);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		glGenerateMipmap(GL_TEXTURE_2D);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	if (map)
		models->FreeTexture(map);
	models->FreeRenderModel(mesh.waiting);
	mesh.waiting = nullptr;
	return &mesh;
}

// The floor: lines every half meter around the target, fading with distance.
void FloorLines(const Camera &camera, std::vector<float> &out)
{
	constexpr float step = 0.5f, reach = 5.0f;
	const float cx = std::floor(camera.target[0] / step) * step, cz = std::floor(camera.target[2] / step) * step;
	const auto vertex = [&](float x, float z, float alpha)
	{
		out.insert(out.end(), { x, 0.0f, z, 1.0f, 1.0f, 1.0f, alpha });
	};
	for (float offset = -reach; offset <= reach + 0.01f; offset += step)
	{
		const float fade = 0.11f * (1.0f - std::abs(offset) / (reach + step));
		// Each line in two halves, so that it fades toward both of its ends.
		vertex(cx + offset, cz - reach, 0.0f); vertex(cx + offset, cz, fade);
		vertex(cx + offset, cz, fade); vertex(cx + offset, cz + reach, 0.0f);
		vertex(cx - reach, cz + offset, 0.0f); vertex(cx, cz + offset, fade);
		vertex(cx, cz + offset, fade); vertex(cx + reach, cz + offset, 0.0f);
	}
}

} // namespace

bool Project(int width, int height, const Camera &camera, const float point[3], float &x, float &y)
{
	const Mat4 vp = ViewProjection(width, height, camera);
	const float cx = vp.m[0] * point[0] + vp.m[4] * point[1] + vp.m[8] * point[2] + vp.m[12];
	const float cy = vp.m[1] * point[0] + vp.m[5] * point[1] + vp.m[9] * point[2] + vp.m[13];
	const float cw = vp.m[3] * point[0] + vp.m[7] * point[1] + vp.m[11] * point[2] + vp.m[15];
	if (!(cw > NearPlane))
		return false;
	x = (cx / cw * 0.5f + 0.5f) * static_cast<float>(width);
	y = (0.5f - cy / cw * 0.5f) * static_cast<float>(height);
	return true;
}

unsigned Render(int width, int height, const Camera &camera, const std::vector<Item> &items)
{
	if (width < 8 || height < 8 || width > 4096 || height > 4096)
		return 0;

	// What the window's own drawing set, put back at the end.
	GLint framebuffer = 0, program = 0, vertexArray = 0, arrayBuffer = 0, texture = 0, activeTexture = 0, viewport[4] = {};
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &framebuffer);
	glGetIntegerv(GL_CURRENT_PROGRAM, &program);
	glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vertexArray);
	glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &arrayBuffer);
	glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTexture);
	glActiveTexture(GL_TEXTURE0);
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture);
	glGetIntegerv(GL_VIEWPORT, viewport);
	const GLboolean depth = glIsEnabled(GL_DEPTH_TEST), cull = glIsEnabled(GL_CULL_FACE), blend = glIsEnabled(GL_BLEND),
		scissor = glIsEnabled(GL_SCISSOR_TEST);
	const auto restore = [&]
	{
		glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(framebuffer));
		glUseProgram(static_cast<GLuint>(program));
		glBindVertexArray(static_cast<GLuint>(vertexArray));
		glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(arrayBuffer));
		glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture));
		glActiveTexture(static_cast<GLenum>(activeTexture));
		glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
		(depth ? glEnable : glDisable)(GL_DEPTH_TEST);
		(cull ? glEnable : glDisable)(GL_CULL_FACE);
		(blend ? glEnable : glDisable)(GL_BLEND);
		(scissor ? glEnable : glDisable)(GL_SCISSOR_TEST);
	};

	if (!Prepare() || !Resize(width, height))
	{
		restore();
		return 0;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, g.drawFbo);
	glViewport(0, 0, width, height);
	glDisable(GL_SCISSOR_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(GL_TRUE);
	// The colour of the page's well, as a solid: an edge that fades into
	// nothing would darken where the samples are averaged.
	glClearColor(34.0f / 255.0f, 36.0f / 255.0f, 41.0f / 255.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	const Mat4 vp = ViewProjection(width, height, camera);
	float eye[3];
	EyeOf(camera, eye);

	static std::vector<float> lines;
	lines.clear();
	FloorLines(camera, lines);
	glEnable(GL_BLEND);
	glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glUseProgram(g.lineProgram);
	glUniformMatrix4fv(glGetUniformLocation(g.lineProgram, "uViewProjection"), 1, GL_FALSE, vp.m);
	glBindVertexArray(g.lineVao);
	glBindBuffer(GL_ARRAY_BUFFER, g.lineVbo);
	glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(lines.size() * sizeof(float)), lines.data(), GL_STREAM_DRAW);
	glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size() / 7));
	glDisable(GL_BLEND);

	glUseProgram(g.meshProgram);
	glUniformMatrix4fv(glGetUniformLocation(g.meshProgram, "uViewProjection"), 1, GL_FALSE, vp.m);
	glUniform3fv(glGetUniformLocation(g.meshProgram, "uEye"), 1, eye);
	glUniform1i(glGetUniformLocation(g.meshProgram, "uTexture"), 0);
	const GLint modelAt = glGetUniformLocation(g.meshProgram, "uModel"), scaleAt = glGetUniformLocation(g.meshProgram, "uScale"),
		colourAt = glGetUniformLocation(g.meshProgram, "uColour"), texturedAt = glGetUniformLocation(g.meshProgram, "uTextured"),
		dimAt = glGetUniformLocation(g.meshProgram, "uDim");
	for (const Item &item : items)
	{
		if (!std::isfinite(item.position[0]) || !std::isfinite(item.position[1]) || !std::isfinite(item.position[2]))
			continue;
		const Mesh *mesh = item.renderModel.empty() ? nullptr : ModelOf(item.renderModel);
		// Every device is drawn half as large again as it is: at its true size
		// a tracker is a few pixels in a picture that shows a whole body.
		constexpr float enlarge = 1.5f;
		float scale[3] = { enlarge, enlarge, enlarge };
		if (!mesh)
		{
			// Meters: a headset, a controller's grip, a tracker's puck.
			mesh = &g.sphere;
			const float sizes[3][3] = { { 0.095f, 0.058f, 0.070f }, { 0.030f, 0.030f, 0.085f }, { 0.046f, 0.024f, 0.046f } };
			for (int i = 0; i < 3; ++i)
				scale[i] = sizes[static_cast<int>(item.shape)][i] * enlarge;
		}
		glUniformMatrix4fv(modelAt, 1, GL_FALSE, ModelMatrix(item.position, item.rotation).m);
		glUniform3fv(scaleAt, 1, scale);
		glUniform3fv(colourAt, 1, item.colour);
		glUniform1i(texturedAt, mesh->texture ? 1 : 0);
		glUniform1f(dimAt, item.tracked ? 0.0f : 0.65f);
		glBindTexture(GL_TEXTURE_2D, mesh->texture);
		glBindVertexArray(mesh->vao);
		glDrawElements(GL_TRIANGLES, mesh->indices, GL_UNSIGNED_SHORT, nullptr);
	}

	glBindFramebuffer(GL_READ_FRAMEBUFFER, g.drawFbo);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, g.resolveFbo);
	glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
	restore();
	return g.texture;
}

} // namespace questcal::sceneview
