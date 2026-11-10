#pragma once

// The pose recording file, format version 1, raw mode: every pose sample the
// recorder received, written as it was received. docs/recording-format.md is
// the contract a reader in another language is written from; this header is
// the same layout as code, and Tests/PoseRecordingTests.cpp holds the proof
// that the two agree (Tests/recordings has files with what a correct reader
// reports for each).
//
// This header needs the C++17 standard library and nothing else: no Nova
// Calibrator, SteamVR or UI header. Another program can compile it as it is.
//
// Every number is little-endian and nothing is padded. A double is its 64
// IEEE-754 bits, so NaN payloads and negative zero come back as they went in.
// A string is one length byte and that many bytes of UTF-8.
//
//   header   magic "NOVAPOSE", u32 format version, u32 mode, i64 clock ticks
//            per second, i64 clock count at the start, i64 wall clock at the
//            start (microseconds since 1970-01-01 UTC), u32 device count,
//            string writer, then that many devices
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
//
// A reader never joins the samples on the two sides of a gap, a boundary or a
// pause. It skips a record kind it does not know by its size, and refuses a
// format version it does not know. A file read back is untrusted input:
// Reader refuses what does not fit this layout and never reads past what a
// size field allows. Device ids and tracking results come back as recorded.

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
	Clip = 2,     // reserved: one clip ended and another began ("Automatic")
};

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

// One record. `kind` says which member holds it; a kind this header does not
// know keeps its payload bytes in `unknown`.
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

struct Record
{
	uint8_t kind = 0;
	Sample sample;
	Gap gap;
	Boundary boundary;
	DeviceInfo device;
	End end;
	Pause pause;
	Resume resume;
	Alignment alignment;
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

// Any record Reader returned, as the bytes it was read from.
inline void AppendRecord(std::string &out, const Record &record)
{
	switch (record.kind)
	{
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
		if (!detail::GetU32(input_, header.version) || !detail::GetU32(input_, header.mode) ||
			!detail::GetI64(input_, header.qpcFrequency) || !detail::GetI64(input_, header.startQpc) ||
			!detail::GetI64(input_, header.startUnixMicroseconds) || !detail::GetU32(input_, devices))
			return Refuse("the file is shorter than a recording header");
		if (header.version != FormatVersion)
			return Refuse("the recording has format version " + std::to_string(header.version) +
				", and this reader knows version " + std::to_string(FormatVersion));
		if (header.mode != ModeRaw)
			return Refuse("the recording is not in raw mode");
		if (header.qpcFrequency <= 0)
			return Refuse("the recording's clock rate is not positive");
		if (devices > MaxHeaderDevices)
			return Refuse("the header lists more than 64 devices");
		if (!detail::GetString(input_, header.writer))
			return Refuse("the file is shorter than a recording header");
		header.devices.resize(devices);
		for (DeviceInfo &device : header.devices)
			if (!detail::GetDevice(input_, device))
				return Refuse("the file ends inside the header's device table");
		headerRead_ = true;
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
		detail::MemoryInput in{ payload_.data(), payload_.data() + payload_.size() };
		bool fits = true;
		switch (kind)
		{
		case KindSample:
			fits = size == SamplePayloadBytes && detail::GetSample(in, record.sample);
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
	// caller that read the file before and kept where its records start. The
	// stream must be one that can seek. False when it cannot go there.
	bool SeekToRecord(uint64_t offset)
	{
		input_.in.clear();
		input_.in.seekg(static_cast<std::streamoff>(offset));
		if (!input_.in)
			return Refuse("the file has no byte " + std::to_string(offset));
		input_.taken = offset;
		consumed_ = offset;
		error_.clear();
		state_ = Next::Record;
		headerRead_ = true;
		return true;
	}

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
};

} // namespace questcal::recording
