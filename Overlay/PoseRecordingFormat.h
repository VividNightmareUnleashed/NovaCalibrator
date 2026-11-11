#pragma once

// The pose recording file: every pose sample the recorder received. Format
// version 1 (raw mode) holds each sample as it was received, 189 bytes.
// Format version 2 (compact mode) holds the same records, and most samples in
// blocks of a few bits each: position to a micrometer, rotation to a
// microradian, single-precision values bit for bit ("Compact samples" below).
// docs/recording-format.md is the contract a reader in another language is
// written from; this header is the same layout as code, and
// Tests/PoseRecordingTests.cpp holds the proof that the two agree
// (Tests/recordings has files with what a correct reader reports for each).
//
// This header needs the C++17 standard library and nothing else: no Nova
// Calibrator, SteamVR or UI header. Another program can compile it as it is.
//
// Every number is little-endian and nothing is padded. A double is its 64
// IEEE-754 bits, so NaN payloads and negative zero come back as they went in.
// A string is one length byte and that many bytes of UTF-8.
//
//   header   magic "NOVAPOSE", u32 format version, u32 mode, (version 2: u32
//            velocity steps in a meter a second, u32 angular velocity steps
//            in a radian a second,) i64 clock ticks per second, i64 clock
//            count at the start, i64 wall clock at the start (microseconds
//            since 1970-01-01 UTC), u32 device count, string writer, then
//            that many devices
//   device   u32 id, u32 device class, i32 controller role, strings label,
//            model, tracking system, controller type
//   record   u8 kind, u16 payload size, payload:
//     1 sample    186 bytes, see Sample
//     2 gap       i64 count when read, u64 samples lost, u64 size of the whole
//                 hole so far
//     3 boundary  i64 count when read, u64 running boundary count
//     4 device    one device: first seen, or changed
//     5 end       u8 reason, i64 count, i64 wall clock, u64 samples in the file
//     6 pause     u8 cause, i64 count, i64 wall clock: recording stops here
//     7 resume    u8 cause, i64 count, i64 wall clock, string trigger:
//                 recording goes on here, in the same file
//     8 alignment u8 applied, i64 count, f64 x 4 rotation (w x y z), f64 x 3
//                 translation in meters, f64 scale, string tracking system:
//                 the calibration Nova Calibrator gives that system's devices
//                 from here on. The samples stay as the driver made them.
//     9 model     u32 device id, string render model: SteamVR's name of the
//                 model that shows the device, for a playback that draws it
//    10 mark      i64 count, i64 wall clock, u32 device id, string name
//    11 samples   version 2: u16 sample count, then that many samples as bits
//    12 sync      version 2: no payload; a reader forgets every device here
//
// A reader never joins the samples on the two sides of a gap, a boundary or a
// pause. It skips a record kind it does not know by its size, and refuses a
// format version it does not know. A file read back is untrusted input:
// Reader refuses what does not fit this layout and never reads past what a
// size field allows. Device ids and tracking results come back as recorded.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <istream>
#include <string>
#include <utility>
#include <vector>

namespace questcal::recording
{

constexpr char Magic[8] = { 'N', 'O', 'V', 'A', 'P', 'O', 'S', 'E' };
constexpr uint32_t FormatVersion = 1;
constexpr uint32_t ModeRaw = 1;
// Version 2 is always compact mode, and version 1 always raw mode.
constexpr uint32_t FormatVersionCompact = 2;
constexpr uint32_t ModeCompact = 2;
constexpr size_t MaxHeaderDevices = 64;
constexpr size_t MaxStringBytes = 255;
constexpr size_t RecordPrefixBytes = 3;
constexpr size_t SamplePayloadBytes = 186;
constexpr size_t GapPayloadBytes = 24;
constexpr size_t BoundaryPayloadBytes = 16;
constexpr size_t EndPayloadBytes = 25;
constexpr size_t PausePayloadBytes = 17;
// A resume with a full trigger string, and a device with four full strings:
// the largest records this version writes.
constexpr size_t MaxResumePayloadBytes = 17 + 1 + MaxStringBytes;
constexpr size_t MaxDevicePayloadBytes = 12 + 4 * (1 + MaxStringBytes);
constexpr size_t MaxMarkPayloadBytes = 20 + 1 + MaxStringBytes;

enum RecordKind : uint8_t
{
	KindSample = 1,
	KindGap = 2,
	KindBoundary = 3,
	KindDevice = 4,
	KindEnd = 5,
	KindPause = 6,
	KindResume = 7,
	KindAlignment = 8,
	KindModel = 9,
	KindMark = 10,
	KindSamples = 11,   // version 2 only
	KindSync = 12,      // version 2 only
};

// Why the recorder closed the file for good. Stored as a byte; a newer
// writer may use a reason this header does not name.
enum class EndReason : uint8_t
{
	SizeLimit = 1,      // the file reached its size cap
	DiskSpaceLow = 2,   // the drive fell under the free-space floor
	SteamVrQuit = 3,    // SteamVR quit or its server ended
	AppExit = 4,        // the recording program closed
	WriteError = 5,     // a write failed; the file may lack this record
};

// Why recording paused, and why it went on. Stored as a byte.
enum class PauseCause : uint8_t
{
	Player = 1,   // the player turned recording off, then on again
	Clip = 2,     // "Automatic": a clip ended, and a clip began
};

// The names the recorder gives what it marks (docs/recording-format.md, "10:
// mark"). A newer writer may use a name this header does not list.
namespace markname
{
constexpr const char *Player = "player";
constexpr const char *UniverseJump = "universe-jump";
constexpr const char *BaseStationMoved = "base-station-moved";
constexpr const char *TrackingLost = "tracking-lost";
constexpr const char *DeviceSlid = "device-slid";
constexpr const char *TrackingJumped = "tracking-jumped";
constexpr const char *CalibrationStopped = "calibration-stopped";
constexpr const char *CalibrationRealigned = "calibration-realigned";
} // namespace markname

// The device id of a mark that is about no one device.
constexpr uint32_t NoDevice = 0xFFFFFFFF;

inline const char *EndReasonText(uint8_t reason)
{
	switch (static_cast<EndReason>(reason))
	{
	case EndReason::SizeLimit:
		return "the file reached its size limit";
	case EndReason::DiskSpaceLow:
		return "the disk was almost full";
	case EndReason::SteamVrQuit:
		return "SteamVR quit";
	case EndReason::AppExit:
		return "the recording program closed";
	case EndReason::WriteError:
		return "a write failed";
	}
	return "a reason this reader does not know";
}

// One pose as the driver captured it, before any calibration: the file's
// sample record, field for field and in this order.
struct Sample
{
	int64_t qpc = 0;                  // clock count at capture
	uint32_t deviceId = 0xFFFFFFFF;
	uint32_t trackingResult = 0;      // vr::ETrackingResult
	bool poseValid = false;
	bool connected = false;
	double poseTimeOffset = 0.0;      // seconds
	double worldFromDriverRotation[4] = { 1.0, 0.0, 0.0, 0.0 };   // w x y z
	double worldFromDriverTranslation[3] = { 0.0, 0.0, 0.0 };     // meters
	double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };                  // w x y z, driver space
	double position[3] = { 0.0, 0.0, 0.0 };                       // meters, driver space
	double velocity[3] = { 0.0, 0.0, 0.0 };                       // m/s
	double angularVelocity[3] = { 0.0, 0.0, 0.0 };                // rad/s
};

// A device as the file names it. The label is made up for one recording and
// says nothing about the hardware: a file holds no serial number.
struct DeviceInfo
{
	uint32_t deviceId = 0xFFFFFFFF;
	uint32_t deviceClass = 0;      // vr::ETrackedDeviceClass
	int32_t controllerRole = 0;    // vr::ETrackedControllerRole hint
	std::string label;
	std::string model;
	std::string trackingSystem;
	std::string controllerType;    // SteamVR's, such as "vive_tracker_waist"

	bool operator==(const DeviceInfo &other) const
	{
		return deviceId == other.deviceId && deviceClass == other.deviceClass &&
			controllerRole == other.controllerRole && label == other.label && model == other.model &&
			trackingSystem == other.trackingSystem && controllerType == other.controllerType;
	}
	bool operator!=(const DeviceInfo &other) const { return !(*this == other); }
};

struct Header
{
	uint32_t version = FormatVersion;
	uint32_t mode = ModeRaw;
	// Version 2 only: how fine a block's velocities are, as steps in one meter
	// a second and in one radian a second (10000: steps of 100 micrometers a
	// second). A sample stored whole is never changed by them.
	uint32_t velocityScale = 0;
	uint32_t angularScale = 0;
	int64_t qpcFrequency = 0;
	int64_t startQpc = 0;
	int64_t startUnixMicroseconds = 0;
	std::string writer;   // the program and version that wrote the file
	std::vector<DeviceInfo> devices;
};

struct Gap
{
	int64_t qpc = 0;
	uint64_t lost = 0;
	uint64_t holeSize = 0;
};

struct Boundary
{
	int64_t qpc = 0;
	uint64_t streamBoundaries = 0;
};

struct End
{
	uint8_t reason = 0;
	int64_t qpc = 0;
	int64_t unixMicroseconds = 0;
	uint64_t samples = 0;
};

struct Pause
{
	uint8_t cause = 0;
	int64_t qpc = 0;
	int64_t unixMicroseconds = 0;
};

struct Resume
{
	uint8_t cause = 0;
	int64_t qpc = 0;
	int64_t unixMicroseconds = 0;
	std::string trigger;   // what started a clip; empty for the player's own resume
};

// The calibration in use for one tracking system's devices: a position in
// that system's space, p, is shown to SteamVR at rotation * (scale * p) +
// translation while `applied` is true. A sample is never changed by it.
struct Alignment
{
	bool applied = false;
	int64_t qpc = 0;
	double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };   // w x y z
	double translation[3] = { 0.0, 0.0, 0.0 };     // meters
	double scale = 1.0;
	std::string trackingSystem;

	bool SameValues(const Alignment &other) const
	{
		return applied == other.applied && trackingSystem == other.trackingSystem &&
			std::memcmp(rotation, other.rotation, sizeof rotation) == 0 &&
			std::memcmp(translation, other.translation, sizeof translation) == 0 &&
			std::memcmp(&scale, &other.scale, sizeof scale) == 0;
	}
};

// SteamVR's name of the model that shows a device, such as
// "{htc}vr_tracker_vive_3_0". It names a shape, not a device.
struct Model
{
	uint32_t deviceId = 0xFFFFFFFF;
	std::string renderModel;
};

// Something worth finding again happened at this moment: a tracking problem
// Nova Calibrator saw, or the player's own "save this".
struct Mark
{
	int64_t qpc = 0;
	int64_t unixMicroseconds = 0;
	uint32_t deviceId = NoDevice;   // the device it is about, or NoDevice
	std::string name;
};

// One record. `kind` says which member holds it; a kind this header does not
// know keeps its payload bytes in `unknown`. A block of samples (kind 11 in a
// version 2 file) comes back as the samples it holds, in `samples`, with its
// payload as stored in `block`. In a version 1 file kinds 11 and 12 are kinds
// the format does not know: `samples` is then empty.
struct Record
{
	uint8_t kind = 0;
	Sample sample;
	std::vector<Sample> samples;
	std::string block;
	Gap gap;
	Boundary boundary;
	DeviceInfo device;
	End end;
	Pause pause;
	Resume resume;
	Alignment alignment;
	Model model;
	Mark mark;
	std::string unknown;
};

// ---------------------------------------------------------------------------
// Writing

inline void PutU8(std::string &out, uint8_t value)
{
	out.push_back(static_cast<char>(value));
}

inline void PutU16(std::string &out, uint16_t value)
{
	PutU8(out, static_cast<uint8_t>(value & 0xFFu));
	PutU8(out, static_cast<uint8_t>(value >> 8));
}

inline void PutU32(std::string &out, uint32_t value)
{
	for (int shift = 0; shift < 32; shift += 8)
		PutU8(out, static_cast<uint8_t>((value >> shift) & 0xFFu));
}

inline void PutU64(std::string &out, uint64_t value)
{
	for (int shift = 0; shift < 64; shift += 8)
		PutU8(out, static_cast<uint8_t>((value >> shift) & 0xFFu));
}

inline void PutI64(std::string &out, int64_t value)
{
	PutU64(out, static_cast<uint64_t>(value));
}

inline void PutF64(std::string &out, double value)
{
	uint64_t bits = 0;
	std::memcpy(&bits, &value, sizeof bits);
	PutU64(out, bits);
}

// Cut to what the length byte holds; the cut may fall inside a UTF-8 sequence.
inline void PutString(std::string &out, const std::string &text)
{
	const size_t size = text.size() < MaxStringBytes ? text.size() : MaxStringBytes;
	PutU8(out, static_cast<uint8_t>(size));
	out.append(text, 0, size);
}

inline void PutRecordPrefix(std::string &out, uint8_t kind, size_t payloadBytes)
{
	PutU8(out, kind);
	PutU16(out, static_cast<uint16_t>(payloadBytes));
}

inline void PutDeviceFields(std::string &out, const DeviceInfo &device)
{
	PutU32(out, device.deviceId);
	PutU32(out, device.deviceClass);
	PutU32(out, static_cast<uint32_t>(device.controllerRole));
	PutString(out, device.label);
	PutString(out, device.model);
	PutString(out, device.trackingSystem);
	PutString(out, device.controllerType);
}

// At most MaxHeaderDevices devices are written; the rest get device records.
inline void AppendHeader(std::string &out, const Header &header)
{
	out.append(Magic, sizeof Magic);
	PutU32(out, header.version);
	PutU32(out, header.mode);
	if (header.version == FormatVersionCompact)
	{
		PutU32(out, header.velocityScale);
		PutU32(out, header.angularScale);
	}
	PutI64(out, header.qpcFrequency);
	PutI64(out, header.startQpc);
	PutI64(out, header.startUnixMicroseconds);
	const size_t devices = header.devices.size() < MaxHeaderDevices ? header.devices.size() : MaxHeaderDevices;
	PutU32(out, static_cast<uint32_t>(devices));
	PutString(out, header.writer);
	for (size_t i = 0; i < devices; ++i)
		PutDeviceFields(out, header.devices[i]);
}

inline void AppendSample(std::string &out, const Sample &sample)
{
	PutRecordPrefix(out, KindSample, SamplePayloadBytes);
	PutI64(out, sample.qpc);
	PutU32(out, sample.deviceId);
	PutU32(out, sample.trackingResult);
	PutU8(out, sample.poseValid ? 1 : 0);
	PutU8(out, sample.connected ? 1 : 0);
	PutF64(out, sample.poseTimeOffset);
	for (double value : sample.worldFromDriverRotation)
		PutF64(out, value);
	for (double value : sample.worldFromDriverTranslation)
		PutF64(out, value);
	for (double value : sample.rotation)
		PutF64(out, value);
	for (double value : sample.position)
		PutF64(out, value);
	for (double value : sample.velocity)
		PutF64(out, value);
	for (double value : sample.angularVelocity)
		PutF64(out, value);
}

inline void AppendGap(std::string &out, const Gap &gap)
{
	PutRecordPrefix(out, KindGap, GapPayloadBytes);
	PutI64(out, gap.qpc);
	PutU64(out, gap.lost);
	PutU64(out, gap.holeSize);
}

inline void AppendBoundary(std::string &out, const Boundary &boundary)
{
	PutRecordPrefix(out, KindBoundary, BoundaryPayloadBytes);
	PutI64(out, boundary.qpc);
	PutU64(out, boundary.streamBoundaries);
}

inline void AppendDevice(std::string &out, const DeviceInfo &device)
{
	std::string payload;
	PutDeviceFields(payload, device);
	PutRecordPrefix(out, KindDevice, payload.size());
	out += payload;
}

inline void AppendEnd(std::string &out, const End &end)
{
	PutRecordPrefix(out, KindEnd, EndPayloadBytes);
	PutU8(out, end.reason);
	PutI64(out, end.qpc);
	PutI64(out, end.unixMicroseconds);
	PutU64(out, end.samples);
}

inline void AppendPause(std::string &out, const Pause &pause)
{
	PutRecordPrefix(out, KindPause, PausePayloadBytes);
	PutU8(out, pause.cause);
	PutI64(out, pause.qpc);
	PutI64(out, pause.unixMicroseconds);
}

inline void AppendResume(std::string &out, const Resume &resume)
{
	std::string payload;
	PutU8(payload, resume.cause);
	PutI64(payload, resume.qpc);
	PutI64(payload, resume.unixMicroseconds);
	PutString(payload, resume.trigger);
	PutRecordPrefix(out, KindResume, payload.size());
	out += payload;
}

inline void AppendAlignment(std::string &out, const Alignment &alignment)
{
	std::string payload;
	PutU8(payload, alignment.applied ? 1 : 0);
	PutI64(payload, alignment.qpc);
	for (double value : alignment.rotation)
		PutF64(payload, value);
	for (double value : alignment.translation)
		PutF64(payload, value);
	PutF64(payload, alignment.scale);
	PutString(payload, alignment.trackingSystem);
	PutRecordPrefix(out, KindAlignment, payload.size());
	out += payload;
}

inline void AppendModel(std::string &out, const Model &model)
{
	std::string payload;
	PutU32(payload, model.deviceId);
	PutString(payload, model.renderModel);
	PutRecordPrefix(out, KindModel, payload.size());
	out += payload;
}

inline void AppendMark(std::string &out, const Mark &mark)
{
	std::string payload;
	PutI64(payload, mark.qpc);
	PutI64(payload, mark.unixMicroseconds);
	PutU32(payload, mark.deviceId);
	PutString(payload, mark.name);
	PutRecordPrefix(out, KindMark, payload.size());
	out += payload;
}

// Version 2: a reader forgets every device here, so the next sample of each
// device is one stored whole.
inline void AppendSync(std::string &out)
{
	PutRecordPrefix(out, KindSync, 0);
}

// Any record Reader returned, as the bytes it was read from.
inline void AppendRecord(std::string &out, const Record &record)
{
	switch (record.kind)
	{
	case KindSamples:
		// A block that was read as samples. Without samples the kind was one
		// the file's version does not know.
		if (record.samples.empty())
		{
			PutRecordPrefix(out, record.kind, record.unknown.size());
			out += record.unknown;
			break;
		}
		PutRecordPrefix(out, KindSamples, record.block.size());
		out += record.block;
		break;
	case KindSample:
		AppendSample(out, record.sample);
		break;
	case KindGap:
		AppendGap(out, record.gap);
		break;
	case KindBoundary:
		AppendBoundary(out, record.boundary);
		break;
	case KindDevice:
		AppendDevice(out, record.device);
		break;
	case KindEnd:
		AppendEnd(out, record.end);
		break;
	case KindPause:
		AppendPause(out, record.pause);
		break;
	case KindResume:
		AppendResume(out, record.resume);
		break;
	case KindAlignment:
		AppendAlignment(out, record.alignment);
		break;
	case KindModel:
		AppendModel(out, record.model);
		break;
	case KindMark:
		AppendMark(out, record.mark);
		break;
	default:
		PutRecordPrefix(out, record.kind, record.unknown.size());
		out += record.unknown;
		break;
	}
}

// ---------------------------------------------------------------------------
// Reading

namespace detail
{

// Bytes already in memory: a record's payload.
struct MemoryInput
{
	const char *at = nullptr;
	const char *end = nullptr;

	bool Bytes(char *to, size_t count)
	{
		if (static_cast<size_t>(end - at) < count)
			return false;
		std::memcpy(to, at, count);
		at += count;
		return true;
	}
};

// Bytes from the file, counting what was taken.
struct StreamInput
{
	std::istream &in;
	uint64_t taken = 0;

	bool Bytes(char *to, size_t count)
	{
		in.read(to, static_cast<std::streamsize>(count));
		const size_t got = static_cast<size_t>(in.gcount());
		taken += got;
		return got == count;
	}
};

template <typename Input>
bool GetU8(Input &in, uint8_t &value)
{
	char byte = 0;
	if (!in.Bytes(&byte, 1))
		return false;
	value = static_cast<uint8_t>(byte);
	return true;
}

template <typename Input>
bool GetU32(Input &in, uint32_t &value)
{
	char bytes[4];
	if (!in.Bytes(bytes, sizeof bytes))
		return false;
	value = 0;
	for (int i = 3; i >= 0; --i)
		value = (value << 8) | static_cast<uint8_t>(bytes[i]);
	return true;
}

template <typename Input>
bool GetU64(Input &in, uint64_t &value)
{
	char bytes[8];
	if (!in.Bytes(bytes, sizeof bytes))
		return false;
	value = 0;
	for (int i = 7; i >= 0; --i)
		value = (value << 8) | static_cast<uint8_t>(bytes[i]);
	return true;
}

template <typename Input>
bool GetI64(Input &in, int64_t &value)
{
	uint64_t bits = 0;
	if (!GetU64(in, bits))
		return false;
	std::memcpy(&value, &bits, sizeof value);
	return true;
}

template <typename Input>
bool GetF64(Input &in, double &value)
{
	uint64_t bits = 0;
	if (!GetU64(in, bits))
		return false;
	std::memcpy(&value, &bits, sizeof value);
	return true;
}

template <typename Input>
bool GetString(Input &in, std::string &text)
{
	uint8_t size = 0;
	if (!GetU8(in, size))
		return false;
	text.resize(size);
	return size == 0 || in.Bytes(&text[0], size);
}

template <typename Input>
bool GetDevice(Input &in, DeviceInfo &device)
{
	uint32_t role = 0;
	if (!GetU32(in, device.deviceId) || !GetU32(in, device.deviceClass) || !GetU32(in, role))
		return false;
	std::memcpy(&device.controllerRole, &role, sizeof role);
	return GetString(in, device.label) && GetString(in, device.model) &&
		GetString(in, device.trackingSystem) && GetString(in, device.controllerType);
}

// False when the payload ends early or a flag byte is not 0 or 1.
inline bool GetSample(MemoryInput &in, Sample &sample)
{
	uint8_t valid = 0, connected = 0;
	if (!GetI64(in, sample.qpc) || !GetU32(in, sample.deviceId) || !GetU32(in, sample.trackingResult) ||
		!GetU8(in, valid) || !GetU8(in, connected) || valid > 1 || connected > 1)
		return false;
	sample.poseValid = valid == 1;
	sample.connected = connected == 1;
	bool read = GetF64(in, sample.poseTimeOffset);
	for (double &value : sample.worldFromDriverRotation)
		read = read && GetF64(in, value);
	for (double &value : sample.worldFromDriverTranslation)
		read = read && GetF64(in, value);
	for (double &value : sample.rotation)
		read = read && GetF64(in, value);
	for (double &value : sample.position)
		read = read && GetF64(in, value);
	for (double &value : sample.velocity)
		read = read && GetF64(in, value);
	for (double &value : sample.angularVelocity)
		read = read && GetF64(in, value);
	return read;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Compact samples (format version 2)
//
// A version 2 file holds every record of version 1, and two more. A sample is
// stored whole (kind 1, as in version 1) when it is a device's first since a
// reader last forgot the device, and when it does not fit a block. Every other
// sample is a few bits in a block (kind 11): what it differs by from a guess
// made of the device's sample before it. docs/recording-format.md, "Compact
// samples", is the layout bit for bit; CodeSample below is in its order.
//
// A guess is whole-number arithmetic on values the writer and every reader
// hold alike, so two readers on any machine give the same doubles. Floating
// point is at the edges only, one operation for a value: a multiplication
// that turns a double into steps, and a division that turns steps back.

namespace compact
{

constexpr int64_t PositionScale = 1000000;   // steps in one meter
constexpr int64_t RotationScale = 2000000;   // steps in 1, for each of a rotation's four numbers
// What a header's two velocity scales may be.
constexpr uint32_t MinVelocityScale = 1000;
constexpr uint32_t MaxVelocityScale = 10000000;
// The clock rates a compact file may have: a pose time is kept to one count.
constexpr int64_t MinClockRate = 1000000;
constexpr int64_t MaxClockRate = 1000000000;
// The largest values a block holds, in steps or counts. A sample with a
// larger one is stored whole.
constexpr int64_t PositionLimit = int64_t{ 1 } << 40;
constexpr int64_t RotationLimit = int64_t{ 1 } << 22;
constexpr int64_t VelocityLimit = int64_t{ 1 } << 30;
constexpr int64_t OffsetLimit = int64_t{ 1 } << 40;
constexpr int64_t CountLimit = int64_t{ 1 } << 58;
constexpr int64_t PeriodLimit = int64_t{ 1 } << 30;
// The devices a reader keeps between two syncs. One more has its samples
// stored whole.
constexpr size_t MaxDevices = 64;
constexpr unsigned MaxParameter = 40;
constexpr unsigned MaxCodeLength = 61;
constexpr uint64_t MaxContextStep = uint64_t{ 1 } << 40;
// What a writer puts in one block at most; a reader takes any block. One
// sample is never more than MaxSampleBytes in a block.
constexpr size_t MaxBlockSamples = 1024;
constexpr size_t MaxBlockBytes = 60000;
constexpr size_t MaxSampleBytes = 320;
// A writer has a reader forget every device this often, so that a reader can
// start in the middle of a file.
constexpr int64_t SyncSeconds = 2;

struct Parameters
{
	int64_t clockRate = 0;
	uint32_t velocityScale = 0;
	uint32_t angularScale = 0;
};

inline bool Valid(const Parameters &parameters)
{
	return parameters.clockRate >= MinClockRate && parameters.clockRate <= MaxClockRate &&
		parameters.velocityScale >= MinVelocityScale && parameters.velocityScale <= MaxVelocityScale &&
		parameters.angularScale >= MinVelocityScale && parameters.angularScale <= MaxVelocityScale;
}

// A sample's thirteen pose numbers in the file's order: rotation w x y z,
// position, velocity, angular velocity.
constexpr int PoseNumbers = 13;

inline void PoseOf(const Sample &sample, double pose[PoseNumbers])
{
	for (int i = 0; i < 4; ++i)
		pose[i] = sample.rotation[i];
	for (int i = 0; i < 3; ++i)
	{
		pose[4 + i] = sample.position[i];
		pose[7 + i] = sample.velocity[i];
		pose[10 + i] = sample.angularVelocity[i];
	}
}

inline void SetPose(Sample &sample, const double pose[PoseNumbers])
{
	for (int i = 0; i < 4; ++i)
		sample.rotation[i] = pose[i];
	for (int i = 0; i < 3; ++i)
	{
		sample.position[i] = pose[4 + i];
		sample.velocity[i] = pose[7 + i];
		sample.angularVelocity[i] = pose[10 + i];
	}
}

inline double ScaleOf(const Parameters &parameters, int number)
{
	return number < 4 ? static_cast<double>(RotationScale) : number < 7 ? static_cast<double>(PositionScale)
		: number < 10 ? static_cast<double>(parameters.velocityScale) : static_cast<double>(parameters.angularScale);
}

inline int64_t LimitOf(int number)
{
	return number < 4 ? RotationLimit : number < 7 ? PositionLimit : VelocityLimit;
}

// `value` in steps of 1 / `scale`: the nearest whole number, a half upward.
// False when it is not a number or is more than `limit` steps from zero. The
// product is compared, never added to: a compiler that joins a multiplication
// and an addition into one operation cannot change the answer.
inline bool Steps(double value, double scale, int64_t limit, int64_t &steps)
{
	const double product = value * scale;
	if (!(std::fabs(product) <= static_cast<double>(limit)))
		return false;
	const double whole = std::floor(product);
	steps = static_cast<int64_t>(whole) + (product >= whole + 0.5 ? 1 : 0);
	return true;
}

// Whether a sample can be in a block, and can be what the device's next
// sample is guessed from: every number in range.
inline bool Usable(const Sample &sample, const Parameters &parameters)
{
	int64_t steps = 0;
	if (sample.qpc < -CountLimit || sample.qpc > CountLimit ||
		!Steps(sample.poseTimeOffset, static_cast<double>(parameters.clockRate), OffsetLimit, steps))
		return false;
	double pose[PoseNumbers];
	PoseOf(sample, pose);
	for (int i = 0; i < PoseNumbers; ++i)
		if (!Steps(pose[i], ScaleOf(parameters, i), LimitOf(i), steps))
			return false;
	return true;
}

// The 32 bits of `value` as a single-precision number. False when it is not
// exactly one (or is not finite). By its bits, so that the answer does not
// depend on how a processor is set to treat very small numbers.
inline bool SingleBits(double value, uint32_t &bits)
{
	uint64_t wide = 0;
	std::memcpy(&wide, &value, sizeof wide);
	const uint32_t sign = (wide >> 63) != 0 ? 0x80000000u : 0u;
	const int exponent = static_cast<int>((wide >> 52) & 0x7FF);
	const uint64_t fraction = wide & ((uint64_t{ 1 } << 52) - 1);
	if (exponent == 0x7FF)
		return false;
	if (exponent == 0)
	{
		// Zero. A double this small and not zero is under every single.
		bits = sign;
		return fraction == 0;
	}
	const int power = exponent - 1023;
	if (power > 127)
		return false;
	if (power >= -126)
	{
		bits = sign | (static_cast<uint32_t>(power + 127) << 23) | static_cast<uint32_t>(fraction >> 29);
		return (fraction & ((uint64_t{ 1 } << 29) - 1)) == 0;
	}
	if (power < -149)
		return false;
	// A single without the leading one: its 23 bits count 2^-149.
	const int shift = -97 - power;
	const uint64_t significand = fraction | (uint64_t{ 1 } << 52);
	bits = sign | static_cast<uint32_t>(significand >> shift);
	return (significand & ((uint64_t{ 1 } << shift) - 1)) == 0;
}

// The double a single-precision number's 32 bits stand for. False for bits
// that are not a finite number.
inline bool SingleValue(uint32_t bits, double &value)
{
	const uint64_t sign = static_cast<uint64_t>(bits >> 31) << 63;
	const uint32_t exponent = (bits >> 23) & 0xFF;
	const uint64_t fraction = bits & 0x7FFFFFu;
	if (exponent == 0xFF)
		return false;
	uint64_t wide = sign;
	if (exponent != 0)
		wide |= ((static_cast<uint64_t>(exponent) + 896) << 52) | (fraction << 29);
	else if (fraction != 0)
	{
		int top = 22;
		while (((fraction >> top) & 1) == 0)
			--top;
		wide |= (static_cast<uint64_t>(top - 149 + 1023) << 52) | ((fraction << (52 - top)) & ((uint64_t{ 1 } << 52) - 1));
	}
	std::memcpy(&value, &wide, sizeof value);
	return true;
}

// How many bits `value` takes: 0 for 0, 3 for 4 to 7.
inline unsigned BitLength(uint64_t value)
{
	unsigned length = 0;
	for (unsigned step = 32; step > 0; step >>= 1)
		if ((value >> step) != 0)
		{
			length += step;
			value >>= step;
		}
	return length + static_cast<unsigned>(value);
}

// a / b rounded down, for b over zero.
inline int64_t FloorDivide(int64_t a, int64_t b)
{
	const int64_t quotient = a / b;
	return a % b < 0 ? quotient - 1 : quotient;
}

// The largest whole number whose square is not over `value` (0 or more).
inline int64_t SquareRoot(int64_t value)
{
	int64_t root = static_cast<int64_t>(std::sqrt(static_cast<double>(value)));
	while (root * root > value)
		--root;
	while ((root + 1) * (root + 1) <= value)
		++root;
	return root;
}

inline uint64_t Zigzag(int64_t value)
{
	return (static_cast<uint64_t>(value) << 1) ^ (value < 0 ? ~uint64_t{ 0 } : uint64_t{ 0 });
}

inline int64_t Unzigzag(uint64_t code)
{
	return static_cast<int64_t>(code >> 1) ^ -static_cast<int64_t>(code & 1);
}

// How large the numbers of one kind have been, which sets how the next one is
// written: eight times a running mean. Writer and reader keep it alike.
struct Context
{
	uint64_t sum = 128;

	unsigned Parameter() const
	{
		const unsigned length = BitLength(sum);
		return length <= 5 ? 0 : length - 5 > MaxParameter ? MaxParameter : length - 5;
	}

	void Add(uint64_t number)
	{
		sum += (number < MaxContextStep ? number : MaxContextStep) - (sum >> 3);
	}
};

constexpr uint64_t FirstContextSum = 128;
constexpr uint64_t FirstSingleSum = uint64_t{ 8 } << 16;
constexpr uint64_t FirstCaptureSum = uint64_t{ 8 } << 10;

// What writer and reader keep of one device between two syncs.
struct Device
{
	uint32_t id = 0xFFFFFFFF;
	Sample previous;        // as a reader has it
	// The writer's own: the previous sample's pose numbers as they were given
	// to it, which is what a repeat is a repeat of. A reader has no use for it.
	double given[PoseNumbers] = {};
	bool usable = false;    // `previous` can be guessed from
	bool single = false;    // its thirteen pose numbers are single-precision numbers
	int64_t period = 0;     // counts from one pose time to the next; 0: not known yet
	// For a pose number in steps: 6 velocities, 3 positions, 4 rotation
	// numbers by their place, and the rotation's largest number. One set for a
	// sample whose pose time stayed, one for a sample whose pose time moved on.
	Context steps[2][14];
	Context time[2];
	// For a single-precision number: the pose time offset, then the thirteen.
	Context singles[14];

	void Start(const Sample &sample, const Parameters &parameters)
	{
		previous = sample;
		usable = Usable(sample, parameters);
		PoseOf(sample, given);
		single = true;
		for (double value : given)
		{
			uint32_t bits = 0;
			single = single && SingleBits(value, bits);
		}
		period = 0;
		for (Context &context : steps[0])
			context.sum = FirstContextSum;
		for (Context &context : steps[1])
			context.sum = FirstContextSum;
		for (Context &context : time)
			context.sum = FirstContextSum;
		for (Context &context : singles)
			context.sum = FirstSingleSum;
	}
};

// Everything a block's bits are read against. A sync, a resume and the header
// empty it; a sample stored whole starts its device.
struct Model
{
	Parameters parameters;
	// In the order of each device's first whole sample: a block names a
	// device by its place here.
	std::vector<Device> devices;
	int64_t lastCount = 0;   // the capture count of the sample before, of any device
	Context capture{ FirstCaptureSum };

	void Reset()
	{
		devices.clear();
		lastCount = 0;
		capture.sum = FirstCaptureSum;
	}

	Device *Find(uint32_t id)
	{
		for (Device &device : devices)
			if (device.id == id)
				return &device;
		return nullptr;
	}

	void Whole(const Sample &sample)
	{
		lastCount = sample.qpc;
		Device *device = Find(sample.deviceId);
		if (!device)
		{
			if (devices.size() >= MaxDevices)
				return;
			devices.emplace_back();
			device = &devices.back();
			device->id = sample.deviceId;
		}
		device->Start(sample, parameters);
	}
};

// Bits, the first in the lowest bit of a byte.
class BitWriter
{
public:
	// The lowest `count` bits of `value`, lowest first. `count` up to 64.
	void Bits(uint64_t value, unsigned count)
	{
		while (count > 0)
		{
			const unsigned take = count < 32 ? count : 32;
			pending_ |= (value & ((uint64_t{ 1 } << take) - 1)) << held_;
			held_ += take;
			value = take < 32 ? value >> take : (value >> 16) >> 16;
			count -= take;
			while (held_ >= 8)
			{
				bytes_.push_back(static_cast<char>(pending_ & 0xFF));
				pending_ >>= 8;
				held_ -= 8;
			}
		}
	}

	size_t Bytes() const { return bytes_.size() + (held_ > 0 ? 1 : 0); }

	// The bytes written, the last one filled up with zero bits; then empty.
	void TakeInto(std::string &out)
	{
		if (held_ > 0)
			bytes_.push_back(static_cast<char>(pending_ & 0xFF));
		out += bytes_;
		bytes_.clear();
		pending_ = 0;
		held_ = 0;
	}

private:
	std::string bytes_;
	uint64_t pending_ = 0;
	unsigned held_ = 0;
};

class BitReader
{
public:
	BitReader(const char *data, size_t size) : data_{ reinterpret_cast<const unsigned char *>(data) }, end_{ size * 8 } { }

	// False when fewer than `count` bits are left. `count` up to 64.
	bool Bits(unsigned count, uint64_t &value)
	{
		value = 0;
		if (count > end_ - at_)
			return false;
		for (unsigned got = 0; got < count;)
		{
			const unsigned inByte = static_cast<unsigned>(at_ & 7);
			const unsigned take = 8 - inByte < count - got ? 8 - inByte : count - got;
			const uint64_t piece = (static_cast<uint64_t>(data_[at_ >> 3]) >> inByte) & ((uint64_t{ 1 } << take) - 1);
			value |= piece << got;
			got += take;
			at_ += take;
		}
		return true;
	}

	// The zero bits in front of the next one bit, which is taken too. False
	// when the bits end first or there are more than `limit` zeros.
	bool Zeros(unsigned limit, unsigned &count)
	{
		count = 0;
		for (;;)
		{
			if (at_ >= end_)
				return false;
			const bool one = ((data_[at_ >> 3] >> (at_ & 7)) & 1) != 0;
			++at_;
			if (one)
				return true;
			if (++count > limit)
				return false;
		}
	}

	// What is left is less than a byte of zero bits: the end of a block.
	bool AtEnd()
	{
		uint64_t rest = 0;
		const size_t left = end_ - at_;
		return left < 8 && Bits(static_cast<unsigned>(left), rest) && rest == 0;
	}

private:
	const unsigned char *data_;
	size_t end_;
	size_t at_ = 0;
};

// The two sides of CodeSample. Each call writes its value, or reads it.
struct Writing
{
	static constexpr bool Writes = true;
	BitWriter &bits;

	bool Flag(bool &flag)
	{
		bits.Bits(flag ? 1 : 0, 1);
		return true;
	}

	bool Whole(uint64_t &value, unsigned count)
	{
		bits.Bits(value, count);
		return true;
	}

	// `value` as its distance from `guess`. The distance is under 2^60.
	bool Step(Context &context, int64_t guess, int64_t &value)
	{
		const uint64_t number = Zigzag(value - guess);
		const unsigned parameter = context.Parameter();
		const uint64_t high = (number >> parameter) + 1;
		const unsigned length = BitLength(high);
		bits.Bits(0, length - 1);
		bits.Bits(1, 1);
		bits.Bits(high, length - 1);
		bits.Bits(number, parameter);
		context.Add(number);
		return true;
	}
};

struct Reading
{
	static constexpr bool Writes = false;
	BitReader &bits;

	bool Flag(bool &flag)
	{
		uint64_t bit = 0;
		if (!bits.Bits(1, bit))
			return false;
		flag = bit != 0;
		return true;
	}

	bool Whole(uint64_t &value, unsigned count)
	{
		return bits.Bits(count, value);
	}

	bool Step(Context &context, int64_t guess, int64_t &value)
	{
		const unsigned parameter = context.Parameter();
		unsigned zeros = 0;
		uint64_t high = 0, low = 0;
		if (!bits.Zeros(MaxCodeLength - parameter, zeros) || !bits.Bits(zeros, high) || !bits.Bits(parameter, low))
			return false;
		const uint64_t number = ((((uint64_t{ 1 } << zeros) | high) - 1) << parameter) | low;
		context.Add(number);
		value = guess + Unzigzag(number);
		return true;
	}
};

template <typename Coder>
bool CodeDouble(Coder &coder, double &value)
{
	uint64_t bits = 0;
	std::memcpy(&bits, &value, sizeof bits);
	if (!coder.Whole(bits, 64))
		return false;
	std::memcpy(&value, &bits, sizeof value);
	return true;
}

// A single-precision number as the distance of its 32 bits from the bits of
// the value before it (from 0 when that is not a single-precision number).
template <typename Coder>
bool CodeSingle(Coder &coder, Context &context, double before, double &value)
{
	uint32_t base = 0, bits = 0;
	if (!SingleBits(before, base))
		base = 0;
	if constexpr (Coder::Writes)
		SingleBits(value, bits);
	int64_t number = bits;
	if (!coder.Step(context, base, number) || number < 0 || number > 0xFFFFFFFFll)
		return false;
	return SingleValue(static_cast<uint32_t>(number), value);
}

// One sample of `device` to or from a block, behind the bits that name the
// device. Writing: `sample` is the one to store, which is usable, of a device
// whose previous sample is usable, at a count within 2^59 of the last; it
// comes back as a reader will have it. Reading: `sample` comes back as read,
// or false when the bits are not a sample. Either way the model moves on.
template <typename Coder>
bool CodeSample(Coder &coder, Model &model, Device &device, Sample &sample)
{
	const Parameters &parameters = model.parameters;
	const Sample previous = device.previous;
	const double rate = static_cast<double>(parameters.clockRate);
	if (model.lastCount < -CountLimit || model.lastCount > CountLimit)
		return false;

	// The numbers are the document's; 1, the device, is the caller's.
	// 2. The capture count, from the count of the sample before it in the file.
	if (!coder.Step(model.capture, model.lastCount, sample.qpc) || sample.qpc < -CountLimit || sample.qpc > CountLimit)
		return false;
	model.lastCount = sample.qpc;

	// 3. What is unusual about it. One bit says that nothing is.
	double was[PoseNumbers], now[PoseNumbers];
	PoseOf(previous, was);
	PoseOf(sample, now);
	bool status = false, moved = false, single = device.single, repeat = false;
	if constexpr (Coder::Writes)
	{
		status = sample.trackingResult != previous.trackingResult || sample.poseValid != previous.poseValid ||
			sample.connected != previous.connected;
		moved = std::memcmp(sample.worldFromDriverRotation, previous.worldFromDriverRotation,
				sizeof sample.worldFromDriverRotation) != 0 ||
			std::memcmp(sample.worldFromDriverTranslation, previous.worldFromDriverTranslation,
				sizeof sample.worldFromDriverTranslation) != 0;
		single = true;
		for (double value : now)
		{
			uint32_t bits = 0;
			single = single && SingleBits(value, bits);
		}
		// A repeat is the sample before it again as it was given, bit for
		// bit; a reader then has its own previous numbers again.
		repeat = std::memcmp(now, device.given, sizeof now) == 0;
		std::memcpy(device.given, now, sizeof now);
	}
	bool plain = !status && !moved && single == device.single && !repeat;
	if (!coder.Flag(plain))
		return false;
	if (!plain && (!coder.Flag(status) || !coder.Flag(moved) || !coder.Flag(single) || !coder.Flag(repeat)))
		return false;

	sample.deviceId = device.id;
	if (status)
	{
		uint64_t valid = sample.poseValid ? 1 : 0, connected = sample.connected ? 1 : 0, result = sample.trackingResult;
		if (!coder.Whole(valid, 1) || !coder.Whole(connected, 1) || !coder.Whole(result, 32))
			return false;
		sample.poseValid = valid != 0;
		sample.connected = connected != 0;
		sample.trackingResult = static_cast<uint32_t>(result);
	}
	else
	{
		sample.poseValid = previous.poseValid;
		sample.connected = previous.connected;
		sample.trackingResult = previous.trackingResult;
	}
	if (moved)
	{
		for (double &value : sample.worldFromDriverRotation)
			if (!CodeDouble(coder, value))
				return false;
		for (double &value : sample.worldFromDriverTranslation)
			if (!CodeDouble(coder, value))
				return false;
	}
	else
	{
		std::memcpy(sample.worldFromDriverRotation, previous.worldFromDriverRotation, sizeof sample.worldFromDriverRotation);
		std::memcpy(sample.worldFromDriverTranslation, previous.worldFromDriverTranslation,
			sizeof sample.worldFromDriverTranslation);
	}

	// 4. The pose time offset. For a sample of single-precision numbers, as one
	// (or as its 64 bits when it is none). Otherwise as the pose time, the
	// capture count plus the offset in counts: it stays, or goes on by the
	// device's period.
	int64_t wasOffset = 0;
	if (!Steps(previous.poseTimeOffset, rate, OffsetLimit, wasOffset))
		return false;
	const int64_t wasPoseTime = previous.qpc + wasOffset;
	int64_t elapsed = 0;   // counts from the previous pose time to this one
	bool onward = false;   // the pose time moved on
	if (single)
	{
		bool wide = false;
		if constexpr (Coder::Writes)
		{
			uint32_t bits = 0;
			wide = !SingleBits(sample.poseTimeOffset, bits);
		}
		if (!coder.Flag(wide))
			return false;
		if (wide ? !CodeDouble(coder, sample.poseTimeOffset)
			: !CodeSingle(coder, device.singles[0], previous.poseTimeOffset, sample.poseTimeOffset))
			return false;
	}
	else
	{
		int64_t offset = 0;
		unsigned periods = 0;
		if constexpr (Coder::Writes)
		{
			if (!Steps(sample.poseTimeOffset, rate, OffsetLimit, offset))
				return false;
			elapsed = sample.qpc + offset - wasPoseTime;
			if (offset == wasOffset)
				periods = 4;
			else if (device.period == 0)
				periods = elapsed > parameters.clockRate / 2000 ? 1 : 0;
			else
			{
				// The count of periods that leaves the least to write.
				int64_t least = elapsed < 0 ? -elapsed : elapsed;
				for (unsigned count = 1; count <= 3; ++count)
				{
					const int64_t miss = elapsed - static_cast<int64_t>(count) * device.period;
					if ((miss < 0 ? -miss : miss) < least)
					{
						least = miss < 0 ? -miss : miss;
						periods = count;
					}
				}
			}
		}
		// As many one bits as periods, then a zero bit unless there are four.
		unsigned ones = 0;
		while (ones < 4)
		{
			bool more = periods > ones;
			if (!coder.Flag(more))
				return false;
			if (!more)
				break;
			++ones;
		}
		periods = ones;
		onward = periods > 0;
		if (periods == 4)
		{
			// The offset in counts is the previous sample's.
			offset = wasOffset;
			elapsed = sample.qpc + offset - wasPoseTime;
		}
		else
		{
			if (!coder.Step(device.time[onward ? 1 : 0], static_cast<int64_t>(periods) * device.period, elapsed))
				return false;
			offset = wasPoseTime + elapsed - sample.qpc;
			if (offset < -OffsetLimit || offset > OffsetLimit)
				return false;
			if (periods >= 1 && device.period == 0)
			{
				if (elapsed > 0 && elapsed <= PeriodLimit)
					device.period = elapsed;
			}
			else if (periods == 1)
			{
				// The period follows what it missed by, a little at a time.
				const int64_t miss = elapsed - device.period;
				const int64_t size = miss < 0 ? -miss : miss;
				const int64_t change = size >> 2 < 8 ? size >> 2 : 8;
				device.period += miss < 0 ? -change : change;
				if (device.period < 1)
					device.period = 1;
			}
		}
		sample.poseTimeOffset = static_cast<double>(offset) / rate;
	}

	// 5. The thirteen pose numbers.
	if (repeat)
		std::memcpy(now, was, sizeof now);
	else if (single)
	{
		for (int i = 0; i < PoseNumbers; ++i)
			if (!CodeSingle(coder, device.singles[1 + i], was[i], now[i]))
				return false;
	}
	else
	{
		int64_t from[PoseNumbers] = {}, to[PoseNumbers] = {};
		for (int i = 0; i < PoseNumbers; ++i)
		{
			if (!Steps(was[i], ScaleOf(parameters, i), LimitOf(i), from[i]))
				return false;
			if constexpr (Coder::Writes)
			{
				if (!Steps(now[i], ScaleOf(parameters, i), LimitOf(i), to[i]))
					return false;
			}
		}
		Context *contexts = device.steps[onward ? 1 : 0];
		// The two velocities, each number from the one before it.
		for (int i = 0; i < 6; ++i)
			if (!coder.Step(contexts[i], from[7 + i], to[7 + i]) || to[7 + i] < -VelocityLimit || to[7 + i] > VelocityLimit)
				return false;
		// The time the pose moved on by, in microseconds, when it is a time a
		// guess can be made over.
		const int64_t micros = elapsed > 0 && elapsed <= parameters.clockRate / 4 ? elapsed * 1000000 / parameters.clockRate : 0;
		// Position: the previous one moved on by the mean of the two velocities.
		const int64_t velocityScale = parameters.velocityScale;
		for (int i = 0; i < 3; ++i)
		{
			const int64_t guess = from[4 + i] +
				FloorDivide((from[7 + i] + to[7 + i]) * micros + velocityScale, 2 * velocityScale);
			if (!coder.Step(contexts[6 + i], guess, to[4 + i]) || to[4 + i] < -PositionLimit || to[4 + i] > PositionLimit)
				return false;
		}
		// Rotation: the previous one turned on by the new angular velocity, taken
		// in the device's own frame. Its largest number comes last, guessed from
		// the other three: the four keep their length.
		const int64_t w = from[0], x = from[1], y = from[2], z = from[3];
		const int64_t a = to[10], b = to[11], c = to[12];
		const int64_t turn[4] = { -(x * a + y * b + z * c), w * a + y * c - z * b, w * b + z * a - x * c,
			w * c + x * b - y * a };
		int largest = 0;
		for (int i = 1; i < 4; ++i)
			if ((from[i] < 0 ? -from[i] : from[i]) > (from[largest] < 0 ? -from[largest] : from[largest]))
				largest = i;
		int64_t rest = w * w + x * x + y * y + z * z;
		const int64_t angularScale = parameters.angularScale;
		for (int i = 0; i < 4; ++i)
		{
			if (i == largest)
				continue;
			const int64_t guess = from[i] + FloorDivide(FloorDivide(turn[i], angularScale) * micros + 1000000, 2000000);
			if (!coder.Step(contexts[9 + i], guess, to[i]) || to[i] < -RotationLimit || to[i] > RotationLimit)
				return false;
			rest -= to[i] * to[i];
		}
		const int64_t root = rest > 0 ? SquareRoot(rest) : 0;
		if (!coder.Step(contexts[13], from[largest] < 0 ? -root : root, to[largest]) || to[largest] < -RotationLimit ||
			to[largest] > RotationLimit)
			return false;
		for (int i = 0; i < PoseNumbers; ++i)
			now[i] = static_cast<double>(to[i]) / ScaleOf(parameters, i);
	}
	SetPose(sample, now);

	device.previous = sample;
	device.usable = Usable(sample, parameters);
	device.single = single;
	return true;
}

// A block's payload as its samples. False when it is not a block that fits
// the model: the model is then of no more use.
inline bool ReadBlock(Model &model, const char *data, size_t size, std::vector<Sample> &samples)
{
	samples.clear();
	if (size < 2)
		return false;
	const size_t count = static_cast<unsigned char>(data[0]) | (static_cast<size_t>(static_cast<unsigned char>(data[1])) << 8);
	if (count == 0)
		return false;
	BitReader bits(data + 2, size - 2);
	Reading coder{ bits };
	for (size_t i = 0; i < count; ++i)
	{
		// 1. The device, by its place among those that have a whole sample.
		uint64_t place = 0;
		if (model.devices.empty() || !bits.Bits(BitLength(model.devices.size() - 1), place) || place >= model.devices.size())
			return false;
		Device &device = model.devices[static_cast<size_t>(place)];
		Sample sample;
		if (!device.usable || !CodeSample(coder, model, device, sample))
			return false;
		samples.push_back(sample);
	}
	return bits.AtEnd();
}

} // namespace compact

// The samples of a version 2 file, written as they come: whole (kind 1) where
// a reader needs one whole, in blocks (kind 11) otherwise, with a sync record
// (kind 12) every compact::SyncSeconds of capture time. A block stays open
// across calls: Flush before any other record goes into the same bytes.
class CompactWriter
{
public:
	explicit CompactWriter(const compact::Parameters &parameters)
	{
		model_.parameters = parameters;
	}

	// The next sample. It may add nothing to `out` yet.
	void Append(std::string &out, const Sample &sample)
	{
		const bool counted = sample.qpc >= -compact::CountLimit && sample.qpc <= compact::CountLimit;
		const int64_t interval = model_.parameters.clockRate * compact::SyncSeconds;
		// A sync when the time is up, and when the clock went far back.
		if (timed_ && counted && (sample.qpc - syncCount_ >= interval || syncCount_ - sample.qpc >= interval))
			Sync(out);
		if (!timed_ && counted)
		{
			timed_ = true;
			syncCount_ = sample.qpc;
		}

		compact::Device *device = model_.Find(sample.deviceId);
		const bool fits = device && device->usable && compact::Usable(sample, model_.parameters) &&
			model_.lastCount >= -compact::CountLimit && model_.lastCount <= compact::CountLimit;
		if (!fits)
		{
			Flush(out);
			AppendSample(out, sample);
			model_.Whole(sample);
			return;
		}
		const size_t place = static_cast<size_t>(device - model_.devices.data());
		bits_.Bits(place, compact::BitLength(model_.devices.size() - 1));
		compact::Writing coder{ bits_ };
		Sample stored = sample;
		compact::CodeSample(coder, model_, *device, stored);
		++count_;
		if (count_ >= compact::MaxBlockSamples || bits_.Bytes() >= compact::MaxBlockBytes)
			Flush(out);
	}

	// Closes the open block into `out`.
	void Flush(std::string &out)
	{
		if (count_ == 0)
			return;
		PutRecordPrefix(out, KindSamples, 2 + bits_.Bytes());
		PutU16(out, static_cast<uint16_t>(count_));
		bits_.TakeInto(out);
		count_ = 0;
	}

	// The bytes Flush would add now.
	size_t Pending() const
	{
		return count_ == 0 ? 0 : RecordPrefixBytes + 2 + bits_.Bytes();
	}

	// A sync record, unless nothing was written since the last: the next
	// sample of every device is stored whole, so a file can be cut here.
	void Sync(std::string &out)
	{
		Flush(out);
		if (!model_.devices.empty())
			AppendSync(out);
		Restart();
	}

	// Behind a header or a resume record, where a reader forgets every device
	// by itself. Flush first.
	void Restart()
	{
		model_.Reset();
		timed_ = false;
	}

private:
	compact::Model model_;
	compact::BitWriter bits_;
	size_t count_ = 0;
	int64_t syncCount_ = 0;
	bool timed_ = false;
};

// Whole records as a version 1 file holds them, added to `out` as a version 2
// file holds them. False when `raw` is not whole records.
inline bool AppendCompact(std::string &out, CompactWriter &writer, const std::string &raw)
{
	size_t at = 0;
	while (raw.size() - at >= RecordPrefixBytes)
	{
		const uint8_t kind = static_cast<uint8_t>(raw[at]);
		const size_t size = static_cast<uint8_t>(raw[at + 1]) | (static_cast<size_t>(static_cast<uint8_t>(raw[at + 2])) << 8);
		if (raw.size() - at - RecordPrefixBytes < size)
			return false;
		const char *payload = raw.data() + at + RecordPrefixBytes;
		Sample sample;
		detail::MemoryInput in{ payload, payload + size };
		if (kind == KindSample && size == SamplePayloadBytes && detail::GetSample(in, sample))
			writer.Append(out, sample);
		else
		{
			writer.Flush(out);
			out.append(raw, at, RecordPrefixBytes + size);
			if (kind == KindResume)
				writer.Restart();
		}
		at += RecordPrefixBytes + size;
	}
	return at == raw.size();
}

// Reads a recording from a binary stream, front to back, one record at a
// time, so a file of any size takes one record's memory.
class Reader
{
public:
	enum class Next
	{
		Record,      // `record` holds the next one
		EndOfFile,   // the file ended on a record's edge
		Truncated,   // the file ended inside a record: the recorder was cut short
		Invalid,     // the bytes are not a recording; Error() says why
	};

	explicit Reader(std::istream &in) : input_{ in } { }

	// First call. False when the header is cut short or is not this format;
	// Error() says which.
	bool ReadHeader(Header &header)
	{
		header = Header();
		char magic[sizeof Magic];
		if (!input_.Bytes(magic, sizeof magic))
			return Refuse("the file is shorter than a recording header");
		if (std::memcmp(magic, Magic, sizeof Magic) != 0)
			return Refuse("the file does not start with NOVAPOSE");
		uint32_t devices = 0;
		if (!detail::GetU32(input_, header.version) || !detail::GetU32(input_, header.mode))
			return Refuse("the file is shorter than a recording header");
		if (header.version != FormatVersion && header.version != FormatVersionCompact)
			return Refuse("the recording has format version " + std::to_string(header.version) +
				", and this reader knows versions " + std::to_string(FormatVersion) + " and " +
				std::to_string(FormatVersionCompact));
		const bool compact = header.version == FormatVersionCompact;
		if (header.mode != (compact ? ModeCompact : ModeRaw))
			return Refuse(compact ? "the recording is not in compact mode" : "the recording is not in raw mode");
		if (compact && (!detail::GetU32(input_, header.velocityScale) || !detail::GetU32(input_, header.angularScale)))
			return Refuse("the file is shorter than a recording header");
		if (!detail::GetI64(input_, header.qpcFrequency) || !detail::GetI64(input_, header.startQpc) ||
			!detail::GetI64(input_, header.startUnixMicroseconds) || !detail::GetU32(input_, devices))
			return Refuse("the file is shorter than a recording header");
		if (header.qpcFrequency <= 0)
			return Refuse("the recording's clock rate is not positive");
		model_.parameters = { header.qpcFrequency, header.velocityScale, header.angularScale };
		if (compact && !compact::Valid(model_.parameters))
			return Refuse("the recording's clock rate or velocity scales are not ones a compact recording has");
		if (devices > MaxHeaderDevices)
			return Refuse("the header lists more than 64 devices");
		if (!detail::GetString(input_, header.writer))
			return Refuse("the file is shorter than a recording header");
		header.devices.resize(devices);
		for (DeviceInfo &device : header.devices)
			if (!detail::GetDevice(input_, device))
				return Refuse("the file ends inside the header's device table");
		headerRead_ = true;
		compact_ = compact;
		model_.Reset();
		consumed_ = input_.taken;
		return true;
	}

	// After ReadHeader succeeded. Anything but Record is final: every later
	// call answers the same.
	Next Read(Record &record)
	{
		if (!headerRead_ && state_ == Next::Record)
			Refuse("the header was not read");
		if (state_ != Next::Record)
			return state_;

		char prefix[RecordPrefixBytes];
		input_.in.read(prefix, sizeof prefix);
		const size_t got = static_cast<size_t>(input_.in.gcount());
		input_.taken += got;
		if (got == 0)
			return state_ = Next::EndOfFile;
		if (got < sizeof prefix)
			return state_ = Next::Truncated;
		const uint8_t kind = static_cast<uint8_t>(prefix[0]);
		const size_t size = static_cast<uint8_t>(prefix[1]) | (static_cast<size_t>(static_cast<uint8_t>(prefix[2])) << 8);
		if (kind == 0)
		{
			Refuse("a record has kind 0: the file has a hole or is not a recording");
			return state_;
		}
		payload_.resize(size);
		if (size > 0 && !input_.Bytes(&payload_[0], size))
			return state_ = Next::Truncated;

		record.kind = kind;
		record.samples.clear();
		detail::MemoryInput in{ payload_.data(), payload_.data() + payload_.size() };
		bool fits = true;
		// The two kinds only a compact file has are kinds a raw file does not
		// know: there they are skipped as any other.
		switch (!compact_ && (kind == KindSamples || kind == KindSync) ? 0 : kind)
		{
		case KindSample:
			fits = size == SamplePayloadBytes && detail::GetSample(in, record.sample);
			// A whole sample is where a block's guesses for its device start.
			if (fits && compact_)
				model_.Whole(record.sample);
			break;
		case KindSamples:
			record.block = payload_;
			fits = compact::ReadBlock(model_, payload_.data(), payload_.size(), record.samples);
			break;
		case KindSync:
			fits = size == 0;
			record.unknown.clear();
			model_.Reset();
			break;
		case KindGap:
			fits = size == GapPayloadBytes && detail::GetI64(in, record.gap.qpc) &&
				detail::GetU64(in, record.gap.lost) && detail::GetU64(in, record.gap.holeSize);
			break;
		case KindBoundary:
			fits = size == BoundaryPayloadBytes && detail::GetI64(in, record.boundary.qpc) &&
				detail::GetU64(in, record.boundary.streamBoundaries);
			break;
		case KindDevice:
			// Exactly one device: bytes left over would not be written back.
			fits = detail::GetDevice(in, record.device) && in.at == in.end;
			break;
		case KindEnd:
			fits = size == EndPayloadBytes && detail::GetU8(in, record.end.reason) &&
				detail::GetI64(in, record.end.qpc) && detail::GetI64(in, record.end.unixMicroseconds) &&
				detail::GetU64(in, record.end.samples);
			break;
		case KindPause:
			fits = size == PausePayloadBytes && detail::GetU8(in, record.pause.cause) &&
				detail::GetI64(in, record.pause.qpc) && detail::GetI64(in, record.pause.unixMicroseconds);
			break;
		case KindResume:
			fits = detail::GetU8(in, record.resume.cause) && detail::GetI64(in, record.resume.qpc) &&
				detail::GetI64(in, record.resume.unixMicroseconds) &&
				detail::GetString(in, record.resume.trigger) && in.at == in.end;
			// A stretch reads by itself: its blocks never reach back over this.
			model_.Reset();
			break;
		case KindAlignment:
		{
			// The flag is 0 or 1, as a sample's flags are.
			uint8_t applied = 0;
			fits = detail::GetU8(in, applied) && applied <= 1 && detail::GetI64(in, record.alignment.qpc);
			for (double &value : record.alignment.rotation)
				fits = fits && detail::GetF64(in, value);
			for (double &value : record.alignment.translation)
				fits = fits && detail::GetF64(in, value);
			fits = fits && detail::GetF64(in, record.alignment.scale) &&
				detail::GetString(in, record.alignment.trackingSystem) && in.at == in.end;
			record.alignment.applied = applied == 1;
			break;
		}
		case KindModel:
			fits = detail::GetU32(in, record.model.deviceId) && detail::GetString(in, record.model.renderModel) &&
				in.at == in.end;
			break;
		case KindMark:
			fits = detail::GetI64(in, record.mark.qpc) && detail::GetI64(in, record.mark.unixMicroseconds) &&
				detail::GetU32(in, record.mark.deviceId) && detail::GetString(in, record.mark.name) && in.at == in.end;
			break;
		default:
			record.unknown = payload_;
			break;
		}
		if (!fits)
		{
			Refuse("a record of kind " + std::to_string(kind) + " and size " + std::to_string(size) +
				" does not fit its layout, " + std::to_string(consumed_) + " bytes in");
			return state_;
		}
		consumed_ = input_.taken;
		return Next::Record;
	}

	// Goes on from the record that starts `offset` bytes into the file, for a
	// caller that read the file before and kept where its records start. In a
	// compact file only a record that Fresh() was true in front of can be gone
	// to: a block behind any other place has nothing to be read against. The
	// stream must be one that can seek. False when it cannot go there, and
	// before ReadHeader succeeded: the header says how the records are read.
	bool SeekToRecord(uint64_t offset)
	{
		if (!headerRead_)
			return Refuse("the header was not read");
		input_.in.clear();
		input_.in.seekg(static_cast<std::streamoff>(offset));
		if (!input_.in)
			return Refuse("the file has no byte " + std::to_string(offset));
		input_.taken = offset;
		consumed_ = offset;
		error_.clear();
		state_ = Next::Record;
		model_.Reset();
		return true;
	}

	// Whether reading could start at the next record with nothing of what came
	// before: always in a raw file; in a compact file behind the header, a
	// sync or a resume, until the next sample.
	bool Fresh() const { return model_.devices.empty(); }

	// Why the header or a record was refused; empty otherwise.
	const std::string &Error() const { return error_; }
	// Bytes of the header and of every record returned so far.
	uint64_t BytesRead() const { return consumed_; }

private:
	bool Refuse(std::string why)
	{
		error_ = std::move(why);
		state_ = Next::Invalid;
		return false;
	}

	detail::StreamInput input_;
	std::string payload_;
	std::string error_;
	uint64_t consumed_ = 0;
	Next state_ = Next::Record;
	bool headerRead_ = false;
	// A compact file's blocks are read against this.
	bool compact_ = false;
	compact::Model model_;
};

} // namespace questcal::recording
