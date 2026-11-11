// The pose recording file (Overlay/PoseRecordingFormat.h), the recorder that
// writes it from the pose stream hub (Overlay/PoseRecorder.h) and the playback
// that reads it (Overlay/RecordingPlayback.h): what goes in comes
// back bit for bit; a file that was cut short or is not a recording is told
// apart from a whole one; the recorder keeps the hub's order, its gaps and its
// boundaries inside its size and disk limits; nothing published while it is
// paused reaches the file; there is one recording, and a new one replaces it
// only once it has data; the file names no hardware; and a playback shows
// each device where its samples put it, on a timeline of the recorded time.
#include "../Overlay/Localization.h"
#include "../Overlay/MotionRecorderText.h"
#include "../Overlay/PoseRecorder.h"
#include "../Overlay/PoseRecordingFormat.h"
#include "../Overlay/PoseRecordingListing.h"
#include "../Overlay/RecordingPlayback.h"
#include "../Overlay/RecordingTriggers.h"
#include "../Overlay/SceneView.h"
#include "RecordingProperty.h"
#include "ReferenceRecordings.h"

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using Check = void (*)(const char *, bool, const char *);
using namespace questcal;
using namespace questcal::recording;

double FromBits(uint64_t bits)
{
	double value = 0.0;
	std::memcpy(&value, &bits, sizeof value);
	return value;
}

uint64_t Bits(double value)
{
	uint64_t bits = 0;
	std::memcpy(&bits, &value, sizeof bits);
	return bits;
}

// A sample as the file holds it: equal strings are equal in every bit.
std::string SampleBytes(const Sample &sample)
{
	std::string bytes;
	AppendSample(bytes, sample);
	return bytes;
}

// A pose as the driver hands it to the hub.
protocol::DevicePoseSample MakePose(uint32_t id, int64_t qpc)
{
	const double t = static_cast<double>(qpc) * 1e-3;
	protocol::DevicePoseSample pose;
	pose.sampleTimeQpc = qpc;
	pose.deviceId = id;
	pose.trackingResult = 200;
	pose.poseIsValid = true;
	pose.deviceIsConnected = true;
	pose.poseTimeOffset = -0.011;
	pose.worldFromDriverRotation = { 0.5, 0.5, -0.5, 0.5 };
	pose.worldFromDriverTranslation[1] = 1.0 / 3.0;
	pose.rotation = { std::cos(t), 0.0, std::sin(t), 0.0 };
	pose.position[0] = 0.1 * id + std::sin(t * 0.7);
	pose.position[1] = 1.0 + 0.01 * std::cos(t * 1.3);
	pose.position[2] = -2.0 / 7.0;
	pose.velocity[0] = 0.7 * std::cos(t * 0.7);
	pose.angularVelocity[1] = 2.0;
	return pose;
}

// The same pose as the file's sample: written here field by field, apart from
// the recorder's own conversion.
Sample AsSample(const protocol::DevicePoseSample &pose)
{
	Sample sample;
	sample.qpc = pose.sampleTimeQpc;
	sample.deviceId = pose.deviceId;
	sample.trackingResult = pose.trackingResult;
	sample.poseValid = pose.poseIsValid;
	sample.connected = pose.deviceIsConnected;
	sample.poseTimeOffset = pose.poseTimeOffset;
	const vr::HmdQuaternion_t &w = pose.worldFromDriverRotation, &r = pose.rotation;
	const double world[4] = { w.w, w.x, w.y, w.z }, turn[4] = { r.w, r.x, r.y, r.z };
	std::copy(world, world + 4, sample.worldFromDriverRotation);
	std::copy(turn, turn + 4, sample.rotation);
	std::copy(pose.worldFromDriverTranslation, pose.worldFromDriverTranslation + 3, sample.worldFromDriverTranslation);
	std::copy(pose.position, pose.position + 3, sample.position);
	std::copy(pose.velocity, pose.velocity + 3, sample.velocity);
	std::copy(pose.angularVelocity, pose.angularVelocity + 3, sample.angularVelocity);
	return sample;
}

Sample MakeSample(uint32_t id, int64_t qpc)
{
	return AsSample(MakePose(id, qpc));
}

RecordedDevice MakeDevice(uint32_t id, uint32_t deviceClass, const char *serial, const char *model,
	const char *system, const char *controllerType = "")
{
	RecordedDevice device;
	device.deviceId = id;
	device.deviceClass = deviceClass;
	device.serial = serial;
	device.model = model;
	device.trackingSystem = system;
	device.controllerType = controllerType;
	return device;
}

const char *const HeadsetSerial = "1WMHH000X00000";
const char *const TrackerSerial = "LHR-A3C36EA5";
const char *const FootSerial = "LHR-77E5A211";

RecordedDevice Headset()
{
	return MakeDevice(0, 1, HeadsetSerial, "Oculus Quest2", "oculus");
}

RecordedDevice Tracker()
{
	return MakeDevice(3, 3, TrackerSerial, "VIVE Tracker 3.0 MV", "lighthouse", "vive_tracker_waist");
}

// What the file says of a device: everything but the serial, and a label.
bool Describes(const DeviceInfo &written, const RecordedDevice &device)
{
	bool hex = written.label.size() == 8;
	for (char c : written.label)
		hex = hex && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
	return hex && written.deviceId == device.deviceId && written.deviceClass == device.deviceClass &&
		written.controllerRole == device.controllerRole && written.model == device.model &&
		written.trackingSystem == device.trackingSystem && written.controllerType == device.controllerType;
}

struct ReadBack
{
	bool headerRead = false;
	Header header;
	std::vector<Record> records;
	Reader::Next end = Reader::Next::Invalid;
	std::string error;
	uint64_t bytes = 0;
};

ReadBack ReadAll(std::istream &in)
{
	ReadBack back;
	Reader reader(in);
	back.headerRead = reader.ReadHeader(back.header);
	if (back.headerRead)
	{
		Record record;
		for (back.end = reader.Read(record); back.end == Reader::Next::Record; back.end = reader.Read(record))
			back.records.push_back(record);
	}
	back.error = reader.Error();
	back.bytes = reader.BytesRead();
	return back;
}

ReadBack ReadBytes(const std::string &bytes)
{
	std::istringstream in(bytes, std::ios::in | std::ios::binary);
	return ReadAll(in);
}

std::string FileContents(const std::wstring &path)
{
	std::ifstream in(path.c_str(), std::ios::in | std::ios::binary);
	std::ostringstream bytes;
	bytes << in.rdbuf();
	return bytes.str();
}

ReadBack ReadFile(const std::wstring &path)
{
	return ReadBytes(FileContents(path));
}

bool Exists(const std::wstring &path)
{
	std::error_code failed;
	return std::filesystem::is_regular_file(path, failed);
}

// One letter for each record, in file order.
std::string KindsOf(const std::vector<Record> &records)
{
	std::string kinds;
	for (const Record &record : records)
		kinds += record.kind == KindSample ? 'S' : record.kind == KindGap ? 'G' : record.kind == KindBoundary ? 'B'
			: record.kind == KindDevice ? 'D' : record.kind == KindEnd ? 'E' : record.kind == KindPause ? 'P'
			: record.kind == KindResume ? 'R' : record.kind == KindAlignment ? 'A' : record.kind == KindModel ? 'M'
			: record.kind == KindMark ? 'K' : record.kind == KindSamples && !record.samples.empty() ? 'C'
			: record.kind == KindSync ? 'Y' : '?';
	return kinds;
}

// Every sample a file holds, whole ones and those of a compact file's blocks,
// in file order.
std::vector<Sample> SamplesOf(const ReadBack &back)
{
	std::vector<Sample> samples;
	for (const Record &record : back.records)
	{
		if (record.kind == KindSample)
			samples.push_back(record.sample);
		else if (record.kind == KindSamples)
			samples.insert(samples.end(), record.samples.begin(), record.samples.end());
	}
	return samples;
}

// "" when a compact file's samples are `expected`, in order, each within the
// format's steps of it (RecordingProperty.h); otherwise what is wrong.
std::string StoredAre(const ReadBack &back, const std::vector<Sample> &expected)
{
	const std::vector<Sample> stored = SamplesOf(back);
	if (stored.size() != expected.size())
		return "the file holds " + std::to_string(stored.size()) + " samples, not " + std::to_string(expected.size());
	const compact::Parameters parameters{ back.header.qpcFrequency, back.header.velocityScale, back.header.angularScale };
	for (size_t i = 0; i < stored.size(); ++i)
	{
		const std::string why = StoredWithinSteps(expected[i], stored[i], parameters);
		if (!why.empty())
			return why + " (sample " + std::to_string(i) + ")";
	}
	return "";
}

// ---------------------------------------------------------------------------
// The format

void FormatRoundTrip(Check check)
{
	const Header header = reference::FileHeader();

	// Every value a double or an id can hold that a careless codec loses.
	Sample edge;
	edge.qpc = (std::numeric_limits<int64_t>::min)();
	edge.deviceId = 0xFFFFFFFFu;
	edge.trackingResult = 0xFFFFFFFEu;
	edge.poseValid = false;
	edge.connected = true;
	edge.poseTimeOffset = -0.0;
	edge.worldFromDriverRotation[0] = FromBits(0x7FF8DEADBEEF0001ull);   // a quiet NaN with a payload
	edge.worldFromDriverRotation[1] = FromBits(0x7FF0000000000001ull);   // a signalling NaN
	edge.worldFromDriverRotation[2] = FromBits(0xFFF8000000000000ull);   // a negative NaN
	edge.worldFromDriverRotation[3] = -std::numeric_limits<double>::infinity();
	edge.worldFromDriverTranslation[0] = DBL_MAX;
	edge.worldFromDriverTranslation[1] = -DBL_MAX;
	edge.worldFromDriverTranslation[2] = 4.9e-324;
	edge.rotation[0] = -0.0;
	edge.rotation[1] = 0.0;
	edge.rotation[2] = DBL_MIN;
	edge.position[0] = 0.1;
	edge.position[1] = 1e-300;
	edge.position[2] = -1e300;
	edge.velocity[0] = -0.0;
	edge.velocity[1] = std::numeric_limits<double>::quiet_NaN();
	edge.velocity[2] = std::numeric_limits<double>::infinity();
	edge.angularVelocity[0] = 1.0 / 3.0;
	const Sample plain = MakeSample(3, 1234567);
	const Gap gap{ -5, 0xFFFFFFFFFFFFFFFFull, 17 };
	const Boundary boundary{ 987654351012, 3 };
	const DeviceInfo late = reference::Device(63, 2, -2, "0badf00d", "Knuckles \xE5\x8F\xB3", "lighthouse", "knuckles");
	const Pause pause{ 1, 987654355012, 1794230000500000 };
	const Resume resume{ 2, 987654357012, 1794230000700000, "a trigger" };
	const End end{ static_cast<uint8_t>(EndReason::SteamVrQuit), 987654361012, -1, 2 };

	std::string file;
	AppendHeader(file, header);
	AppendSample(file, edge);
	AppendGap(file, gap);
	AppendBoundary(file, boundary);
	AppendDevice(file, late);
	AppendPause(file, pause);
	AppendResume(file, resume);
	AppendSample(file, plain);
	AppendEnd(file, end);

	const ReadBack back = ReadBytes(file);
	const bool headerSame = back.headerRead && back.header.version == FormatVersion && back.header.mode == ModeRaw &&
		back.header.qpcFrequency == header.qpcFrequency && back.header.startQpc == header.startQpc &&
		back.header.startUnixMicroseconds == header.startUnixMicroseconds && back.header.writer == header.writer &&
		back.header.devices == header.devices;
	const bool shape = KindsOf(back.records) == "SGBDPRSE" && back.end == Reader::Next::EndOfFile &&
		back.bytes == file.size() && back.error.empty();
	bool samplesSame = false, edgesKept = false, othersSame = false, writesBack = false;
	if (shape)
	{
		const Sample &read = back.records[0].sample;
		samplesSame = SampleBytes(read) == SampleBytes(edge) && SampleBytes(back.records[6].sample) == SampleBytes(plain) &&
			read.qpc == edge.qpc && read.deviceId == edge.deviceId && read.trackingResult == edge.trackingResult &&
			!read.poseValid && read.connected;
		edgesKept = Bits(read.worldFromDriverRotation[0]) == 0x7FF8DEADBEEF0001ull &&
			Bits(read.worldFromDriverRotation[1]) == 0x7FF0000000000001ull &&
			Bits(read.worldFromDriverRotation[2]) == 0xFFF8000000000000ull &&
			Bits(read.poseTimeOffset) == 0x8000000000000000ull && Bits(read.rotation[0]) == 0x8000000000000000ull &&
			Bits(read.rotation[1]) == 0 && Bits(read.velocity[0]) == 0x8000000000000000ull &&
			std::isnan(read.velocity[1]) && read.worldFromDriverTranslation[2] == 4.9e-324;
		const Record &g = back.records[1], &b = back.records[2], &d = back.records[3], &p = back.records[4],
			&r = back.records[5], &e = back.records[7];
		othersSame = g.gap.qpc == gap.qpc && g.gap.lost == gap.lost && g.gap.holeSize == gap.holeSize &&
			b.boundary.qpc == boundary.qpc && b.boundary.streamBoundaries == boundary.streamBoundaries &&
			d.device == late && p.pause.cause == pause.cause && p.pause.qpc == pause.qpc &&
			p.pause.unixMicroseconds == pause.unixMicroseconds && r.resume.cause == resume.cause &&
			r.resume.qpc == resume.qpc && r.resume.unixMicroseconds == resume.unixMicroseconds &&
			r.resume.trigger == resume.trigger && e.end.reason == end.reason && e.end.qpc == end.qpc &&
			e.end.unixMicroseconds == end.unixMicroseconds && e.end.samples == end.samples;
		std::string again;
		AppendHeader(again, back.header);
		for (const Record &record : back.records)
			AppendRecord(again, record);
		writesBack = again == file;
	}
	char detail[192];
	snprintf(detail, sizeof detail, "header %d, records %s end %d, samples %d, NaN and -0 bits %d, other records %d, rewrite %d",
		headerSame, KindsOf(back.records).c_str(), static_cast<int>(back.end), samplesSame, edgesKept, othersSame,
		writesBack);
	check("pose recording: a file reads back bit for bit, NaN payloads and negative zero included",
		headerSame && shape && samplesSame && edgesKept && othersSame && writesBack, detail);
}

// A recorder that is ended without warning leaves a file cut anywhere. Every
// cut reads as the whole records in front of it and then says "cut short", or
// "end of file" on a record's edge: never as a file that is not a recording.
void CutAnywhere(Check check)
{
	std::string file;
	AppendHeader(file, reference::FileHeader());
	const size_t headerBytes = file.size();
	std::vector<size_t> edges = { file.size() };
	const auto edge = [&] { edges.push_back(file.size()); };
	AppendSample(file, MakeSample(0, 100));
	edge();
	AppendGap(file, { 150, 4, 4 });
	edge();
	AppendDevice(file, reference::Devices()[1]);
	edge();
	AppendPause(file, { 1, 160, 1794230000000160 });
	edge();
	AppendResume(file, { 1, 170, 1794230000000170, "" });
	edge();
	AppendSample(file, MakeSample(3, 200));
	edge();
	AppendEnd(file, { static_cast<uint8_t>(EndReason::AppExit), 300, 1794230000000300, 2 });
	edge();

	size_t wrong = 0, firstWrong = 0;
	for (size_t cut = 0; cut <= file.size(); ++cut)
	{
		const ReadBack back = ReadBytes(file.substr(0, cut));
		bool right;
		if (cut < headerBytes)
			right = !back.headerRead && !back.error.empty();
		else
		{
			size_t whole = 0;
			bool onEdge = false;
			for (size_t i = 0; i < edges.size(); ++i)
			{
				if (i > 0 && edges[i] <= cut)
					++whole;
				onEdge = onEdge || edges[i] == cut;
			}
			right = back.headerRead && back.records.size() == whole && back.error.empty() &&
				back.end == (onEdge ? Reader::Next::EndOfFile : Reader::Next::Truncated) &&
				back.bytes == edges[whole];
		}
		if (!right && wrong++ == 0)
			firstWrong = cut;
	}
	char detail[128];
	snprintf(detail, sizeof detail, "%zu cuts of a %zu-byte file, %zu read wrongly (first at byte %zu)",
		file.size() + 1, file.size(), wrong, firstWrong);
	check("pose recording: a file cut at any byte reads up to its last whole record", wrong == 0, detail);
}

void UnknownKinds(Check check)
{
	const Sample sample = MakeSample(3, 100);
	std::string file;
	AppendHeader(file, reference::FileHeader());
	PutRecordPrefix(file, 77, 4);
	file += "abcd";
	AppendSample(file, sample);
	PutRecordPrefix(file, 200, 0);
	AppendEnd(file, { static_cast<uint8_t>(EndReason::AppExit), 300, 0, 1 });

	const ReadBack back = ReadBytes(file);
	const bool read = back.headerRead && back.end == Reader::Next::EndOfFile && back.records.size() == 4 &&
		back.records[0].kind == 77 && back.records[0].unknown == "abcd" &&
		back.records[1].kind == KindSample && SampleBytes(back.records[1].sample) == SampleBytes(sample) &&
		back.records[2].kind == 200 && back.records[2].unknown.empty() && back.records[3].kind == KindEnd;
	char detail[96];
	snprintf(detail, sizeof detail, "%zu records, end %d", back.records.size(), static_cast<int>(back.end));
	check("pose recording: a record of a kind this version does not know is skipped by its size", read, detail);
}

void Refusals(Check check)
{
	std::string good;
	AppendHeader(good, reference::FileHeader());
	const size_t headerBytes = good.size();
	std::string sample;
	AppendSample(sample, MakeSample(3, 100));

	struct Case
	{
		const char *name;
		std::string bytes;
		size_t wholeRecords;   // returned before the refusal
	};
	std::vector<Case> cases;
	auto header = [&](const char *name, const std::function<void(Header &)> &change)
	{
		Header h = reference::FileHeader();
		change(h);
		std::string bytes;
		AppendHeader(bytes, h);
		cases.push_back({ name, bytes + sample, 0 });
	};
	auto record = [&](const char *name, const std::string &broken)
	{
		cases.push_back({ name, good + sample + broken + sample, 1 });
	};
	auto sized = [&](const char *name, uint8_t kind, size_t size)
	{
		std::string broken;
		PutRecordPrefix(broken, kind, size);
		broken += std::string(size, '\0');
		record(name, broken);
	};

	cases.push_back({ "an empty file", "", 0 });
	cases.push_back({ "ten bytes", good.substr(0, 10), 0 });
	std::string magic = good + sample;
	magic[3] = 'a';
	cases.push_back({ "another magic", magic, 0 });
	header("a newer format version", [](Header &h) { h.version = FormatVersionCompact + 1; });
	header("another mode", [](Header &h) { h.mode = 0; });
	header("a raw file that says it is compact", [](Header &h) { h.mode = ModeCompact; });
	header("a clock rate of zero", [](Header &h) { h.qpcFrequency = 0; });
	header("a negative clock rate", [](Header &h) { h.qpcFrequency = -10000000; });
	// More devices than the header holds, which AppendHeader itself never writes.
	std::string crowded = good;
	crowded[40] = static_cast<char>(MaxHeaderDevices + 1);
	cases.push_back({ "a device table past 64", crowded + sample, 0 });

	std::string bad;
	PutRecordPrefix(bad, 0, 0);
	record("a record of kind 0", bad);
	bad.clear();
	PutRecordPrefix(bad, KindSample, SamplePayloadBytes - 1);
	bad += sample.substr(RecordPrefixBytes, SamplePayloadBytes - 1);
	record("a short sample", bad);
	bad.clear();
	PutRecordPrefix(bad, KindSample, SamplePayloadBytes + 1);
	bad += sample.substr(RecordPrefixBytes) + "x";
	record("a long sample", bad);
	bad = sample;
	bad[RecordPrefixBytes + 16] = 2;   // pose valid
	record("a pose-valid flag of 2", bad);
	bad = sample;
	bad[RecordPrefixBytes + 17] = static_cast<char>(0xFF);   // connected
	record("a connected flag of 255", bad);
	sized("a short gap", KindGap, GapPayloadBytes - 1);
	sized("a long boundary", KindBoundary, BoundaryPayloadBytes + 1);
	sized("a short end", KindEnd, EndPayloadBytes - 1);
	sized("a long pause", KindPause, PausePayloadBytes + 1);
	sized("a resume with no trigger string", KindResume, 17);
	std::string resume;
	AppendResume(resume, { 1, 5, 6, "abc" });
	bad = resume.substr(0, 1);
	PutU16(bad, static_cast<uint16_t>(resume.size() - RecordPrefixBytes + 1));
	bad += resume.substr(RecordPrefixBytes) + "x";
	record("a resume with a byte left over", bad);
	std::string device;
	PutDeviceFields(device, reference::Devices()[1]);
	bad.clear();
	PutRecordPrefix(bad, KindDevice, device.size() + 1);
	bad += device + "x";
	record("a device with a byte left over", bad);
	bad.clear();
	std::string overrun = device;
	overrun[12] = static_cast<char>(0xFF);   // the label's length, past the payload
	PutRecordPrefix(bad, KindDevice, overrun.size());
	bad += overrun;
	record("a device string past its record", bad);

	std::string accepted;
	for (const Case &c : cases)
	{
		const ReadBack back = ReadBytes(c.bytes);
		const bool headerCase = c.wholeRecords == 0 && c.bytes.substr(0, headerBytes) != good;
		const bool refused = headerCase
			? !back.headerRead && back.records.empty() && !back.error.empty()
			: back.headerRead && back.end == Reader::Next::Invalid && back.records.size() == c.wholeRecords &&
				!back.error.empty() && back.bytes == headerBytes + c.wholeRecords * sample.size();
		if (!refused)
			accepted += std::string(accepted.empty() ? "" : ", ") + c.name;
	}
	const std::string detail = std::to_string(cases.size()) + " malformed files" +
		(accepted.empty() ? "" : ", not refused: " + accepted);
	check("pose recording: what does not fit the layout is refused with a reason", accepted.empty(), detail.c_str());
}

// ---------------------------------------------------------------------------
// The reference recordings and the reader's properties

std::string Resource(const std::string &name)
{
	const std::wstring wide(name.begin(), name.end());
	const HRSRC found = FindResourceW(nullptr, wide.c_str(), MAKEINTRESOURCEW(10));
	const HGLOBAL loaded = found ? LoadResource(nullptr, found) : nullptr;
	const void *bytes = loaded ? LockResource(loaded) : nullptr;
	return bytes ? std::string(static_cast<const char *>(bytes), SizeofResource(nullptr, found)) : std::string();
}

// Tests/recordings holds what the generator makes, and its listings. With
// the environment variable NOVA_RECORDER_WRITE_REFERENCE naming a folder, the
// files are written there instead (tools/check-recording-readers.ps1
// -WriteReference): the way to change them on purpose, with
// docs/recording-format.md.
void ReferenceRecordings(Check check)
{
	char folder[MAX_PATH] = {};
	size_t length = 0;
	const bool write = getenv_s(&length, folder, sizeof folder, "NOVA_RECORDER_WRITE_REFERENCE") == 0 && length > 1;
	std::string wrong;
	const std::vector<reference::File> files = reference::Files();
	for (const reference::File &file : files)
	{
		std::istringstream in(file.bytes, std::ios::in | std::ios::binary);
		const std::string listing = Listing(in);
		if (write)
		{
			const std::filesystem::path base = std::filesystem::path(folder) / file.name;
			std::ofstream(base.string() + ".novapose", std::ios::binary) << file.bytes;
			std::ofstream(base.string() + ".txt", std::ios::binary) << listing;
			continue;
		}
		std::string resource = "REFERENCE_" + file.name;
		for (char &c : resource)
			c = c == '-' ? '_' : static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
		if (Resource(resource) != file.bytes)
			wrong += (wrong.empty() ? "" : ", ") + file.name + ".novapose";
		if (Resource(resource + "_LISTING") != listing)
			wrong += (wrong.empty() ? "" : ", ") + file.name + ".txt";
	}
	const std::string detail = std::to_string(files.size()) + " files" +
		(write ? std::string(" written to ") + folder : wrong.empty() ? "" : ", not as in Tests/recordings: " + wrong);
	check("pose recording: the generator makes the reference recordings byte for byte", wrong.empty(), detail.c_str());

	// What the listings must say, whatever is in Tests/recordings.
	const auto listingOf = [&](size_t index)
	{
		std::istringstream in(files[index].bytes, std::ios::in | std::ios::binary);
		return Listing(in);
	};
	const auto has = [](const std::string &text, const char *line) { return text.find(line) != std::string::npos; };
	const std::string clean = listingOf(0), holes = listingOf(1), paused = listingOf(2), cut = listingOf(3),
		newer = listingOf(4), aligned = listingOf(5), clips = listingOf(6), compact = listingOf(7);
	// The compact recording: whole samples at its start, behind its sync and
	// behind its resume, and where a pose fits no block; the rest in blocks.
	const bool saidCompact = has(compact, "novapose listing 2\nformat: version 2, mode 2\n"
			"velocity steps: 10000 in a meter a second, 10000 in a radian a second\n") &&
		has(compact, "gap after sample 218: lost 40, hole 40, ") && has(compact, "\nsync after sample 218\n") &&
		has(compact, "mark after sample 278: name \"tracking-lost\", device 3, ") &&
		has(compact, "pause after sample 278: cause 1, ") && has(compact, "resume after sample 278: cause 1, ") &&
		has(compact, "end after sample 318: reason 4, ") && has(compact, "samples: 318\nsamples in blocks: 305\n") &&
		has(compact, "samples of device 5: 145, valid 143, ") && has(compact, "\nfile: complete\n") &&
		!has(clean, "samples in blocks") && has(clean, "samples: 218\nsample digest: ");
	check("compact recording: a listing says what a correct reader finds in the compact reference recording", saidCompact,
		saidCompact ? "" : compact.c_str());
	const bool said = has(clean, "samples: 218\n") && has(clean, "samples of device 0: 18, valid 18, ") &&
		has(clean, "samples of device 5: 100, valid 98, ") && has(clean, "end after sample 218: reason 4, ") &&
		has(clean, "\nfile: complete\n") &&
		has(holes, "gap after sample 80: lost 40, hole 40, ") && has(holes, "boundary after sample 150: boundaries 1, ") &&
		has(holes, "gap after sample 150: lost 1, hole 1, ") &&
		has(holes, "device record after sample 180: device 7: class 0, role 0, label \"\"") &&
		has(holes, "label \"55aa55aa\", model \"Controller model\"") &&
		has(paused, "pause after sample 100: cause 1, ") && has(paused, "resume after sample 100: cause 1, trigger \"\", ") &&
		has(paused, "samples: 160\n") &&
		has(cut, "samples: 217\n") && has(cut, "\nfile: cut short inside a record after ") && !has(cut, "end after") &&
		has(newer, "unknown record after sample 10: kind 77, 5 bytes\n") &&
		has(newer, "resume after sample 20: cause 2, trigger \"example trigger\", ") &&
		has(aligned, "alignment after sample 0: system \"lighthouse\", applied 1, count 5000000000, "
			"rotation 0.5 0.5 -0.5 0.5, translation 1.5 -0.25 2, scale 1\n") &&
		has(aligned, "alignment after sample 20: system \"lighthouse\", applied 0, ") && has(aligned, ", scale 1.25\n") &&
		has(aligned, "model after sample 0: device 3, render model \"{htc}vr_tracker_vive_3_0\"\n") &&
		has(clips, "pause after sample 0: cause 2, ") &&
		has(clips, "resume after sample 0: cause 2, trigger \"base-station-moved\", ") &&
		has(clips, "mark after sample 20: name \"base-station-moved\", device 4294967295, ") &&
		has(clips, "resume after sample 30: cause 2, trigger \"tracking-lost\", ") &&
		has(clips, "mark after sample 45: name \"tracking-lost\", device 3, ") &&
		has(clips, "mark after sample 50: name \"player\", device 4294967295, ") && has(clips, "samples: 55\n") &&
		has(clips, "\nfile: complete\n");
	check("pose recording: a listing says what a correct reader finds in each reference recording", said,
		said ? "" : clean.c_str());
}

void ReaderProperties(Check check)
{
	const std::vector<std::string> seeds = PropertySeeds();
	std::mt19937 rng(0x5EED1234u);
	std::string failure;
	int seedFailures = 0, failures = 0;
	const auto run = [&](const std::string &input)
	{
		std::string why;
		try
		{
			why = CheckRecording(reinterpret_cast<const uint8_t *>(input.data()), input.size());
		}
		catch (...)
		{
			why = "the reader threw";
		}
		if (!why.empty() && failure.empty())
			failure = why;
		return why.empty();
	};
	for (const std::string &seed : seeds)
		seedFailures += run(seed) ? 0 : 1;
	const int mutations = 4000;
	for (int i = 0; i < mutations; ++i)
		failures += run(Mutate(seeds[rng() % seeds.size()], seeds, rng)) ? 0 : 1;
	char detail[256];
	snprintf(detail, sizeof detail, "%zu seeds (%d fail), %d mutations (%d fail)%s%s", seeds.size(), seedFailures,
		mutations, failures, failure.empty() ? "" : ": ", failure.c_str());
	check("pose recording: the reader keeps its properties on mutated files", seedFailures == 0 && failures == 0, detail);
}

// ---------------------------------------------------------------------------
// Compact recordings (format version 2)

// Some seconds of a headset and three trackers much as they really send: the
// headset's numbers single-precision ones, a tracker's pose time staying for
// one sample in two, noise on every number, the capture counts uneven.
std::vector<Sample> MotionStream(double seconds)
{
	std::mt19937 rng(20261111u);
	const auto noise = [&](double size) { return (static_cast<double>(rng() % 20001u) - 10000.0) * 1e-4 * size; };
	const auto single = [](double value) { return static_cast<double>(static_cast<float>(value)); };
	std::vector<Sample> stream;
	int64_t count = reference::StartCount + 1000;
	const int ticks = static_cast<int>(seconds * 250.0);
	for (int tick = 0; tick < ticks; ++tick)
	{
		const double t = tick * 0.004;
		// A pose every four milliseconds, captured five to seven later.
		const int64_t tickCount = reference::StartCount + static_cast<int64_t>(tick) * 40000;
		for (uint32_t id : { 1u, 2u, 4u })
		{
			const int64_t poseTime = tickCount + static_cast<int64_t>(rng() % 201u);
			const int copies = rng() % 3u != 0 ? 2 : 1;
			for (int copy = 0; copy < copies; ++copy)
			{
				count = (std::max)(count + 100 + static_cast<int64_t>(rng() % 400u),
					tickCount + 50000 + copy * 9000 + static_cast<int64_t>(id) * 2500 + static_cast<int64_t>(rng() % 2000u));
				Sample sample;
				sample.qpc = count;
				sample.deviceId = id;
				sample.trackingResult = 200;
				sample.poseValid = true;
				sample.connected = true;
				sample.poseTimeOffset = (static_cast<double>(poseTime - count) + noise(0.4)) / 1e7;
				const double fromDriver[4] = { 0.5, 0.5, -0.5, 0.5 }, shift[3] = { 0.1, -1.5, 2.25 };
				std::copy(fromDriver, fromDriver + 4, sample.worldFromDriverRotation);
				std::copy(shift, shift + 3, sample.worldFromDriverTranslation);
				const double angle = 0.8 * std::sin(0.5 * t + id);
				sample.rotation[0] = std::cos(0.5 * angle) + noise(5e-5);
				sample.rotation[1] = 0.6 * std::sin(0.5 * angle) + noise(5e-5);
				sample.rotation[2] = 0.8 * std::sin(0.5 * angle) + noise(5e-5);
				sample.rotation[3] = noise(5e-5);
				sample.position[0] = 0.3 * id + 0.4 * std::sin(1.3 * t + id) + noise(3e-5);
				sample.position[1] = 1.0 + 0.2 * std::cos(0.9 * t) + noise(3e-5);
				sample.position[2] = -0.5 + 0.3 * std::sin(0.7 * t + 1.0) + noise(3e-5);
				sample.velocity[0] = 0.52 * std::cos(1.3 * t + id) + noise(4e-4);
				sample.velocity[1] = -0.18 * std::sin(0.9 * t) + noise(4e-4);
				sample.velocity[2] = 0.21 * std::cos(0.7 * t + 1.0) + noise(4e-4);
				sample.angularVelocity[0] = 0.24 * std::cos(0.5 * t + id) + noise(5e-3);
				sample.angularVelocity[1] = 0.32 * std::cos(0.5 * t + id) + noise(5e-3);
				sample.angularVelocity[2] = noise(5e-3);
				stream.push_back(sample);
			}
		}
		if (tick % 4 == 0)
		{
			count = (std::max)(count + 100 + static_cast<int64_t>(rng() % 400u),
				tickCount + 75000 + static_cast<int64_t>(rng() % 2000u));
			Sample sample;
			sample.qpc = count;
			sample.deviceId = 0;
			sample.trackingResult = 200;
			sample.poseValid = true;
			sample.connected = true;
			sample.poseTimeOffset = single(-6e-6 - static_cast<double>(rng() % 30u) * 1e-7);
			const double turn = 0.3 * std::sin(0.4 * t);
			sample.rotation[0] = single(std::cos(turn));
			sample.rotation[1] = single(0.1 * std::sin(turn));
			sample.rotation[2] = single(std::sin(turn));
			sample.rotation[3] = single(noise(1e-3));
			sample.position[0] = single(0.05 * std::sin(0.8 * t) + noise(2e-4));
			sample.position[1] = single(1.7 + 0.01 * std::sin(2.0 * t));
			sample.position[2] = single(0.2 + noise(2e-4));
			for (int i = 0; i < 3; ++i)
			{
				sample.velocity[i] = single(noise(6e-3));
				sample.angularVelocity[i] = single(noise(3e-2));
			}
			stream.push_back(sample);
		}
	}
	return stream;
}

// The same stream with one of every sample a block treats in its own way, or
// cannot hold at all.
std::vector<Sample> OddMotionStream(double seconds)
{
	std::vector<Sample> stream = MotionStream(seconds);
	const auto earlier = [&](size_t at)
	{
		for (size_t i = at; i-- > 0;)
			if (stream[i].deviceId == stream[at].deviceId)
				return i;
		return at;
	};
	const auto next = [&](size_t from, bool headset)
	{
		size_t at = from;
		while ((stream[at].deviceId == 0) != headset)
			++at;
		return at;
	};
	// Not a number, too far away, a count from nowhere: stored whole.
	stream[next(100, false)].velocity[1] = std::numeric_limits<double>::quiet_NaN();
	stream[next(200, false)].position[2] = 5e9;
	stream[300].qpc = (std::numeric_limits<int64_t>::min)() + 5;
	stream[next(1200, false)].poseTimeOffset = 1e9;
	// The sample before it again, number for number.
	const size_t repeat = next(400, false), before = earlier(repeat);
	std::copy(stream[before].rotation, stream[before].rotation + 4, stream[repeat].rotation);
	std::copy(stream[before].position, stream[before].position + 3, stream[repeat].position);
	std::copy(stream[before].velocity, stream[before].velocity + 3, stream[repeat].velocity);
	std::copy(stream[before].angularVelocity, stream[before].angularVelocity + 3, stream[repeat].angularVelocity);
	// Tracking lost and back, a driver frame that moves, a device of another id.
	for (size_t i = 500; i < 520; ++i)
		if (stream[i].deviceId == 2)
		{
			stream[i].trackingResult = 101;
			stream[i].poseValid = false;
		}
	for (size_t i = 600; i < stream.size(); ++i)
		if (stream[i].deviceId == 2)
			stream[i].worldFromDriverTranslation[1] = -1.25;
	stream[next(900, false)].deviceId = 0xFFFFFFFEu;
	// A headset pose time offset that no single holds, and a negative zero.
	stream[next(700, true)].poseTimeOffset = -6.1234567e-6;
	stream[next(1300, true)].position[0] = -0.0;
	// A tracker sample of single-precision numbers, the other way round of a
	// turn, and a rotation that is none.
	Sample &still = stream[next(800, false)];
	const double upright[4] = { 1.0, 0.0, 0.0, 0.0 };
	std::copy(upright, upright + 4, still.rotation);
	for (int i = 0; i < 3; ++i)
		still.position[i] = still.velocity[i] = still.angularVelocity[i] = 0.0;
	for (double &number : stream[next(1000, false)].rotation)
		number = -number;
	for (double &number : stream[next(1100, false)].rotation)
		number = 0.0;
	return stream;
}

// A stream as a recorder would write it: the samples, with a gap, a mark, and
// a pause and resume that states the devices again, then an end.
std::string RecordsOf(const std::vector<Sample> &stream)
{
	std::string records;
	for (size_t i = 0; i < stream.size(); ++i)
	{
		if (i == stream.size() / 3)
			AppendGap(records, { stream[i].qpc, 12, 12 });
		if (i == stream.size() / 2)
			AppendMark(records, { stream[i].qpc, reference::TimeAt(stream[i].qpc), 2, markname::TrackingLost });
		if (i == stream.size() * 2 / 3)
		{
			AppendPause(records, { 1, stream[i].qpc - 20, reference::TimeAt(stream[i].qpc - 20) });
			AppendResume(records, { 1, stream[i].qpc - 10, reference::TimeAt(stream[i].qpc - 10), std::string() });
			for (const DeviceInfo &device : reference::Devices())
				AppendDevice(records, device);
		}
		AppendSample(records, stream[i]);
	}
	AppendEnd(records, { static_cast<uint8_t>(EndReason::AppExit), stream.back().qpc + 1000,
		reference::TimeAt(stream.back().qpc + 1000), stream.size() });
	return records;
}

std::string RawFileOf(const std::string &records)
{
	std::string file;
	AppendHeader(file, reference::FileHeader());
	return file + records;
}

std::string CompactFileOf(const std::string &records, uint32_t velocityScale)
{
	Header header = reference::FileHeader();
	header.version = FormatVersionCompact;
	header.mode = ModeCompact;
	header.velocityScale = header.angularScale = velocityScale;
	std::string file;
	AppendHeader(file, header);
	CompactWriter writer({ header.qpcFrequency, velocityScale, velocityScale });
	AppendCompact(file, writer, records);
	writer.Flush(file);
	return file;
}

// Every record that is no sample, with how many samples are in front of it.
std::string OtherRecordsOf(const ReadBack &back)
{
	std::string others;
	size_t samples = 0;
	for (const Record &record : back.records)
	{
		if (record.kind == KindSample)
			++samples;
		else if (record.kind == KindSamples)
			samples += record.samples.size();
		else if (record.kind != KindSync)
		{
			others += std::to_string(samples) + ":";
			AppendRecord(others, record);
		}
	}
	return others;
}

void CompactRoundTrip(Check check)
{
	const std::vector<Sample> stream = OddMotionStream(6.0);
	const std::string records = RecordsOf(stream);
	const ReadBack raw = ReadBytes(RawFileOf(records));
	std::string wrong, sizes;
	bool others = true, repeats = true;
	for (uint32_t scale : { 10000u, 100000u })
	{
		const std::string file = CompactFileOf(records, scale);
		const ReadBack back = ReadBytes(file);
		std::string why = !back.headerRead || back.end != Reader::Next::EndOfFile ? "the file is not read to its end: " + back.error
			: back.header.version != FormatVersionCompact || back.header.velocityScale != scale ? "the header is another"
			: StoredAre(back, stream);
		size_t inBlocks = 0, changed = 0;
		const std::vector<Sample> stored = SamplesOf(back);
		for (const Record &record : back.records)
			inBlocks += record.kind == KindSamples ? record.samples.size() : 0;
		for (size_t i = 0; i < stored.size() && i < stream.size(); ++i)
			changed += SampleBytes(stored[i]) != SampleBytes(stream[i]) ? 1 : 0;
		// The test is of blocks: nearly every sample is in one, and the trackers'
		// are stored in steps.
		if (why.empty() && (inBlocks * 10 < stream.size() * 9 || changed * 2 < stream.size()))
			why = "too few samples are in blocks or in steps for this to be a test of them";
		if (why.empty() && file.size() * 4 > raw.bytes)
			why = "the compact file is more than a quarter of the raw one";
		if (!why.empty() && wrong.empty())
			wrong = "at " + std::to_string(scale) + " steps: " + why;
		others = others && OtherRecordsOf(back) == OtherRecordsOf(raw);
		// The sample that repeated the one before it still does, number for number.
		for (size_t i = 400; i < 410 && i < stored.size(); ++i)
			if (stream[i].deviceId != 0)
			{
				size_t before = i;
				while (before-- > 0 && stored[before].deviceId != stored[i].deviceId) { }
				double now[compact::PoseNumbers], was[compact::PoseNumbers];
				compact::PoseOf(stored[i], now);
				compact::PoseOf(stored[before], was);
				repeats = repeats && std::memcmp(now, was, sizeof now) == 0;
				break;
			}
		char text[96];
		snprintf(text, sizeof text, "%s%u steps: %.2f bytes a sample, %zu in blocks", sizes.empty() ? "" : "; ", scale,
			static_cast<double>(file.size()) / static_cast<double>(stream.size()), inBlocks);
		sizes += text;
	}
	const std::string detail = std::to_string(stream.size()) + " samples, raw " + std::to_string(raw.bytes) + " bytes; " +
		sizes + (wrong.empty() ? "" : "; " + wrong);
	check("compact recording: every sample reads back within its steps, and bit for bit when it is of single-precision "
		"numbers or fits no block", wrong.empty() && repeats, detail.c_str());
	check("compact recording: the records that are no samples stay as they are, in their places", others,
		others ? "a gap, a mark, a pause, a resume, three devices, an end" : "they differ from the raw file's");

	// The reference recording against the raw records it was made from.
	const std::string reference = StoredAre(ReadBytes(reference::CompactRecording(true)),
		SamplesOf(ReadBytes(reference::CompactRecording(false))));
	check("compact recording: the reference recording holds its raw twin's samples within its steps", reference.empty(),
		reference.c_str());
}

// Reading can start where a compact file's blocks reach back to nothing:
// behind the header, a sync and a resume. From each such place the rest of
// the file reads as it does from the start; from any other, a block is
// refused, not misread.
void CompactStartsAnywhere(Check check)
{
	const std::string file = CompactFileOf(RecordsOf(OddMotionStream(6.0)), 10000);
	std::vector<uint64_t> places, blockPlaces;
	std::vector<size_t> samplesBefore;
	std::vector<Sample> all;
	size_t syncs = 0;
	Reader::Next end = Reader::Next::Invalid;
	{
		std::istringstream in(file, std::ios::in | std::ios::binary);
		Reader reader(in);
		Header header;
		Record record;
		reader.ReadHeader(header);
		for (;;)
		{
			const uint64_t offset = reader.BytesRead();
			const bool fresh = reader.Fresh();
			end = reader.Read(record);
			if (end != Reader::Next::Record)
				break;
			if (fresh)
			{
				places.push_back(offset);
				samplesBefore.push_back(all.size());
			}
			syncs += record.kind == KindSync ? 1 : 0;
			if (record.kind == KindSample)
				all.push_back(record.sample);
			else if (record.kind == KindSamples)
			{
				blockPlaces.push_back(offset);
				all.insert(all.end(), record.samples.begin(), record.samples.end());
			}
		}
	}
	const auto readFrom = [&](uint64_t offset, std::vector<Sample> &samples)
	{
		std::istringstream in(file, std::ios::in | std::ios::binary);
		Reader reader(in);
		Header header;
		Record record;
		reader.ReadHeader(header);
		reader.SeekToRecord(offset);
		Reader::Next state = reader.Read(record);
		for (; state == Reader::Next::Record; state = reader.Read(record))
		{
			if (record.kind == KindSample)
				samples.push_back(record.sample);
			else if (record.kind == KindSamples)
				samples.insert(samples.end(), record.samples.begin(), record.samples.end());
		}
		return state;
	};
	size_t wrong = 0;
	for (size_t i = 0; i < places.size(); ++i)
	{
		std::vector<Sample> rest;
		bool same = readFrom(places[i], rest) == Reader::Next::EndOfFile && rest.size() == all.size() - samplesBefore[i];
		for (size_t k = 0; same && k < rest.size(); ++k)
			same = SampleBytes(rest[k]) == SampleBytes(all[samplesBefore[i] + k]);
		wrong += same ? 0 : 1;
	}
	// A block in the middle of a stretch has nothing to be read against.
	size_t misread = 0;
	for (size_t i = 0; i < blockPlaces.size(); i += 7)
	{
		std::vector<Sample> rest;
		misread += readFrom(blockPlaces[i], rest) == Reader::Next::Invalid && rest.empty() ? 0 : 1;
	}
	// The header says how the records are read: no going anywhere without it.
	std::istringstream unread(file, std::ios::in | std::ios::binary);
	Reader headless(unread);
	const bool needsHeader = !places.empty() && !headless.SeekToRecord(places.back()) && !headless.Error().empty();
	char detail[240];
	snprintf(detail, sizeof detail, "%zu samples, %zu syncs, %zu places to start (%zu read wrongly), %zu blocks (%zu of those "
		"tried were not refused), end %d, a reader without the header refuses to go there %d", all.size(), syncs,
		places.size(), wrong, blockPlaces.size(), misread, static_cast<int>(end), needsHeader);
	// Six seconds: a sync every two, and the resume.
	check("compact recording: a reading can start behind the header, a sync and a resume, and nowhere inside a stretch",
		end == Reader::Next::EndOfFile && syncs >= 2 && places.size() > syncs + 1 && wrong == 0 &&
			!blockPlaces.empty() && misread == 0 && needsHeader, detail);
}

void CompactCutAnywhere(Check check)
{
	const std::string file = reference::CompactRecording(true);
	// Where each record ends, by its size alone.
	std::vector<size_t> edges;
	{
		std::istringstream in(file, std::ios::in | std::ios::binary);
		Reader reader(in);
		Header header;
		reader.ReadHeader(header);
		edges.push_back(static_cast<size_t>(reader.BytesRead()));
	}
	const size_t headerBytes = edges[0];
	while (edges.back() + RecordPrefixBytes <= file.size())
		edges.push_back(edges.back() + RecordPrefixBytes + (static_cast<uint8_t>(file[edges.back() + 1]) |
			(static_cast<size_t>(static_cast<uint8_t>(file[edges.back() + 2])) << 8)));
	size_t wrong = 0, firstWrong = 0, blocksCut = 0;
	for (size_t cut = 0; cut <= file.size(); ++cut)
	{
		const ReadBack back = ReadBytes(file.substr(0, cut));
		bool right;
		if (cut < headerBytes)
			right = !back.headerRead && !back.error.empty();
		else
		{
			size_t whole = 0;
			bool onEdge = false;
			for (size_t i = 0; i < edges.size(); ++i)
			{
				if (i > 0 && edges[i] <= cut)
					++whole;
				onEdge = onEdge || edges[i] == cut;
			}
			right = back.headerRead && back.records.size() == whole && back.error.empty() &&
				back.end == (onEdge ? Reader::Next::EndOfFile : Reader::Next::Truncated) && back.bytes == edges[whole];
			blocksCut += !onEdge && static_cast<uint8_t>(file[edges[whole]]) == KindSamples ? 1 : 0;
		}
		if (!right && wrong++ == 0)
			firstWrong = cut;
	}
	char detail[160];
	snprintf(detail, sizeof detail, "%zu cuts of a %zu-byte file with %zu records, %zu of them inside a block, %zu read "
		"wrongly (first at byte %zu)", file.size() + 1, file.size(), edges.size() - 1, blocksCut, wrong, firstWrong);
	check("compact recording: a file cut at any byte reads up to its last whole record",
		wrong == 0 && edges.back() == file.size() && blocksCut > 100, detail);
}

void CompactRefusals(Check check)
{
	Header header = reference::FileHeader();
	header.version = FormatVersionCompact;
	header.mode = ModeCompact;
	header.velocityScale = header.angularScale = reference::CompactVelocityScale;
	std::string good;
	AppendHeader(good, header);
	// Three whole samples of three devices, and blocks that fit behind them.
	const std::vector<Sample> whole = BlockWholeSamples();
	std::string start = good;
	for (const Sample &sample : whole)
		AppendSample(start, sample);
	const std::vector<std::string> blocks = BlockSeeds();
	const std::string &one = blocks.back();
	const auto block = [](const std::string &payload)
	{
		std::string record;
		PutRecordPrefix(record, KindSamples, payload.size());
		return record + payload;
	};

	struct Case
	{
		const char *name;
		std::string bytes;
		size_t wholeRecords;   // returned before the refusal; none for a header
		bool inHeader;
	};
	std::vector<Case> cases;
	const auto head = [&](const char *name, const std::function<void(Header &)> &change)
	{
		Header changed = header;
		change(changed);
		std::string bytes;
		AppendHeader(bytes, changed);
		cases.push_back({ name, bytes, 0, true });
	};
	head("a compact file that says it is raw", [](Header &h) { h.mode = ModeRaw; });
	head("a velocity scale of zero", [](Header &h) { h.velocityScale = 0; });
	head("a velocity scale under the least", [](Header &h) { h.velocityScale = compact::MinVelocityScale - 1; });
	head("an angular scale over the most", [](Header &h) { h.angularScale = compact::MaxVelocityScale + 1; });
	head("a clock too slow for a pose time", [](Header &h) { h.qpcFrequency = compact::MinClockRate - 1; });
	head("a clock too fast", [](Header &h) { h.qpcFrequency = compact::MaxClockRate + 1; });

	std::string resume, sync, payload;
	AppendResume(resume, { 1, 5000002000, 0, std::string() });
	AppendSync(sync);
	cases.push_back({ "a block with no whole sample in front of it", good + block(one), 0, false });
	cases.push_back({ "a block behind a resume", start + resume + block(one), 4, false });
	cases.push_back({ "a block behind a sync", start + sync + block(one), 4, false });
	payload = one;
	payload[0] = 0;
	cases.push_back({ "a block that says it holds no sample", start + block(payload), 3, false });
	payload[0] = 2;
	cases.push_back({ "a block that says it holds two samples and holds one", start + block(payload), 3, false });
	cases.push_back({ "a block of one byte", start + block(one.substr(0, 1)), 3, false });
	cases.push_back({ "a block cut one byte short", start + block(one.substr(0, one.size() - 1)), 3, false });
	cases.push_back({ "a block with a byte left over", start + block(one + std::string(1, '\0')), 3, false });
	cases.push_back({ "a block of ones", start + block(std::string("\x01\x00", 2) + std::string(40, '\xFF')), 3, false });
	cases.push_back({ "a block of zeros", start + block(std::string("\x01\x00", 2) + std::string(40, '\0')), 3, false });
	std::string synced;
	PutRecordPrefix(synced, KindSync, 1);
	cases.push_back({ "a sync with a payload", start + synced + "x", 3, false });
	// The block's device has a sample that is not a number as its last.
	std::string unusable = good;
	for (Sample sample : whole)
	{
		if (sample.deviceId == 3)
			sample.velocity[0] = std::numeric_limits<double>::quiet_NaN();
		AppendSample(unusable, sample);
	}
	cases.push_back({ "a block for a device whose last sample is not a number", unusable + block(one), 3, false });

	std::string accepted;
	for (const Case &c : cases)
	{
		const ReadBack back = ReadBytes(c.bytes);
		const bool refused = c.inHeader ? !back.headerRead && back.records.empty() && !back.error.empty()
			: back.headerRead && back.end == Reader::Next::Invalid && back.records.size() == c.wholeRecords &&
				!back.error.empty();
		if (!refused)
			accepted += std::string(accepted.empty() ? "" : ", ") + c.name;
	}
	// What is taken: the blocks themselves, and in a raw file the two kinds as
	// kinds it does not know.
	const ReadBack fits = ReadBytes(start + block(blocks.front()) + sync + start.substr(good.size()) + block(one));
	const bool taken = fits.end == Reader::Next::EndOfFile && KindsOf(fits.records) == "SSSCYSSSC" &&
		fits.records[3].samples.size() > 1 && fits.records[8].samples.size() == 1;
	std::string old;
	AppendHeader(old, reference::FileHeader());
	AppendSample(old, whole[0]);
	old += block(one) + synced + "x";
	AppendSample(old, whole[1]);
	const ReadBack skipped = ReadBytes(old);
	std::string again;
	AppendHeader(again, skipped.header);
	for (const Record &record : skipped.records)
		AppendRecord(again, record);
	const bool unknown = skipped.end == Reader::Next::EndOfFile && skipped.records.size() == 4 &&
		skipped.records[1].kind == KindSamples && skipped.records[1].samples.empty() && skipped.records[1].unknown == one &&
		skipped.records[2].kind == KindSync && skipped.records[2].unknown == "x" && again == old;
	const std::string detail = std::to_string(cases.size()) + " malformed files" +
		(accepted.empty() ? "" : ", not refused: " + accepted) + (taken ? "" : "; good blocks are not taken (" +
			KindsOf(fits.records) + " " + fits.error + ")") + (unknown ? "" : "; a raw file's kinds 11 and 12 are not skipped");
	check("compact recording: a block that does not fit what is in front of it is refused, and a raw file skips the two "
		"new kinds", accepted.empty() && taken && unknown, detail.c_str());
}

// ---------------------------------------------------------------------------
// The recorder

struct TempFolder
{
	std::filesystem::path path;

	TempFolder()
	{
		static int made = 0;
		path = std::filesystem::temp_directory_path() /
			("NovaCalibrator-recording-tests-" + std::to_string(GetCurrentProcessId()) + "-" + std::to_string(++made));
		std::filesystem::create_directories(path);
	}
	~TempFolder()
	{
		std::error_code ignored;
		std::filesystem::remove_all(path, ignored);
	}
	std::wstring Dir() const { return path.wstring(); }
	std::wstring File() const { return PoseRecorder::FilePath(path.wstring()); }
	size_t Files() const
	{
		size_t files = 0;
		for (const auto &entry : std::filesystem::directory_iterator(path))
			files += entry.is_regular_file() ? 1 : 0;
		return files;
	}
};

// The drive as the recorder under test sees it: its free space, and, while
// held, a disk that does not answer (the recorder reads the free space from
// its own thread, where it also writes).
std::atomic<uint64_t> DiskFreeBytes{ 0 };
std::atomic<bool> DiskHeld{ false };
std::atomic<bool> DiskWaiting{ false };

bool FakeFreeDiskBytes(const std::wstring &, uint64_t &freeBytes)
{
	while (DiskHeld.load())
	{
		DiskWaiting.store(true);
		Sleep(1);
	}
	freeBytes = DiskFreeBytes.load();
	return true;
}

PoseRecordingLimits TestLimits()
{
	DiskFreeBytes.store(1ull << 30);
	DiskHeld.store(false);
	DiskWaiting.store(false);
	PoseRecordingLimits limits;
	limits.maxFileBytes = 64ull << 20;
	limits.minFreeDiskBytes = 1ull << 20;
	limits.freeDiskBytes = &FakeFreeDiskBytes;
	limits.diskCheckIntervalMs = 0;
	return limits;
}

bool WaitFor(const std::function<bool()> &holds, DWORD timeoutMs = 20000)
{
	const ULONGLONG started = GetTickCount64();
	while (!holds())
	{
		if (GetTickCount64() - started > timeoutMs)
			return false;
		Sleep(1);
	}
	return true;
}

bool WaitForSamples(const PoseRecorder &recorder, uint64_t samples)
{
	return WaitFor([&] { return recorder.GetProgress().samples >= samples; });
}

bool Finish(PoseRecorder &recorder, PoseRecorder::Finished &finished)
{
	return WaitFor([&] { return recorder.TakeFinished(finished); });
}

// A hub and the poses sent through it, in order.
struct Feed
{
	PoseStreamHub hub;
	std::vector<Sample> sent;

	void Send(uint32_t id)
	{
		const protocol::DevicePoseSample pose = MakePose(id, 1000 + static_cast<int64_t>(sent.size()) * 7);
		sent.push_back(AsSample(pose));
		hub.AppendSampleForTest(pose);
	}
};

// Whether the file's samples are exactly `expected`, in order.
bool SamplesAre(const ReadBack &back, const std::vector<Sample> &expected)
{
	size_t next = 0;
	for (const Record &record : back.records)
		if (record.kind == KindSample)
		{
			if (next >= expected.size() || SampleBytes(record.sample) != SampleBytes(expected[next]))
				return false;
			++next;
		}
	return next == expected.size();
}

void RecorderKeepsTheStream(Check check)
{
	TempFolder folder;
	Feed feed;
	// Before the recording: not part of it.
	feed.hub.AppendSampleForTest(MakePose(0, 1));

	PoseRecorder recorder;
	std::string why;
	const auto started = recorder.Start(feed.hub, folder.Dir(), "test writer 9.8.7", { Headset(), Tracker() },
		TestLimits(), &why);
	bool reached = started == PoseRecorder::StartResult::Started;
	feed.Send(0);
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 2);
	feed.hub.AppendGapForTest(5);
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 3);
	feed.hub.AppendSessionBoundaryForTest();
	feed.Send(0);
	reached = reached && WaitForSamples(recorder, 4);
	// A device nothing has described yet, then its description, then a change
	// to a device the header has.
	feed.Send(7);
	reached = reached && WaitForSamples(recorder, 5);
	// The UI thread's news and the driver's poses are not ordered against each
	// other; the test waits for each device record so the file's order is one
	// it can name.
	auto describe = [&](const std::vector<RecordedDevice> &devices)
	{
		const uint64_t before = recorder.GetProgress().bytes;
		recorder.SetDevices(devices);
		return WaitFor([&] { return recorder.GetProgress().bytes > before; });
	};
	const RecordedDevice late = MakeDevice(7, 3, FootSerial, "Tundra Tracker", "lighthouse", "vive_tracker_left_foot");
	reached = reached && describe({ Headset(), Tracker(), late });
	feed.Send(7);
	reached = reached && WaitForSamples(recorder, 6);
	RecordedDevice changed = Tracker();
	changed.controllerType = "vive_tracker_chest";
	reached = reached && describe({ Headset(), changed, late });
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 7);
	// Nothing changed: no record.
	recorder.SetDevices({ Headset(), changed, late });
	feed.Send(0);
	reached = reached && WaitForSamples(recorder, 8);

	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);
	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const std::string kinds = KindsOf(back.records);
	const bool shape = reached && closed && back.headerRead && back.end == Reader::Next::EndOfFile &&
		kinds == "SSGSBGSDSDSDSSP";

	LARGE_INTEGER frequency{};
	QueryPerformanceFrequency(&frequency);
	const int64_t nowMicroseconds = static_cast<int64_t>(std::time(nullptr)) * 1000000;
	const bool stream = shape && SamplesAre(back, feed.sent) && back.header.qpcFrequency == frequency.QuadPart &&
		back.header.writer == "test writer 9.8.7" && back.header.devices.size() == 2 &&
		Describes(back.header.devices[0], Headset()) && Describes(back.header.devices[1], Tracker()) &&
		std::abs(back.header.startUnixMicroseconds - nowMicroseconds) < 3600ll * 1000000;
	char detail[256];
	snprintf(detail, sizeof detail, "start %d (%s), reached %d, closed %d, records %s, end %d %s", static_cast<int>(started),
		why.c_str(), reached, closed, kinds.c_str(), static_cast<int>(back.end), back.error.c_str());
	check("pose recorder: the file holds the hub's stream in order, bit for bit", stream, detail);

	const bool holes = shape && back.records[2].gap.lost == 5 && back.records[2].gap.holeSize == 5 &&
		back.records[4].boundary.streamBoundaries == 1 && back.records[5].gap.lost == 1 &&
		back.records[5].gap.holeSize == 1 && back.records[2].gap.qpc >= back.header.startQpc &&
		back.records[4].boundary.qpc >= back.records[2].gap.qpc;
	check("pose recorder: a gap and a session boundary are records where the hub reported them", holes,
		shape ? "gap of 5, then a boundary and its one-count gap" : detail);

	DeviceInfo unknown;
	unknown.deviceId = 7;
	const bool described = shape && back.records[7].device == unknown && Describes(back.records[9].device, late) &&
		Describes(back.records[11].device, changed) &&
		back.records[11].device.label == back.header.devices[1].label;
	check("pose recorder: a device gets a record when first seen and when what is known of it changes", described,
		shape ? "an empty entry before its first sample, then the description, then the change" : detail);

	// The file names no hardware: a label made up for the recording stands
	// where a serial would, a different one for each device.
	const bool anonymous = shape && bytes.find(HeadsetSerial) == std::string::npos &&
		bytes.find(TrackerSerial) == std::string::npos && bytes.find(FootSerial) == std::string::npos &&
		bytes.find("LHR-") == std::string::npos &&
		back.header.devices[0].label != back.header.devices[1].label &&
		back.records[9].device.label != back.header.devices[1].label;
	check("pose recorder: the file holds labels made up for the recording and no serial number", anonymous,
		shape ? back.header.devices[1].label.c_str() : detail);

	const bool paused = shape && finished.paused && finished.saved && finished.closedCleanly && finished.error.empty() &&
		finished.samples == feed.sent.size() && finished.lost == 6 && finished.bytes == bytes.size() &&
		finished.bytes == back.bytes && finished.path == folder.File() && !Exists(folder.File() + L".new") &&
		back.records.back().pause.cause == static_cast<uint8_t>(PauseCause::Player) &&
		back.records.back().pause.qpc >= back.header.startQpc &&
		std::abs(back.records.back().pause.unixMicroseconds - nowMicroseconds) < 3600ll * 1000000 &&
		!recorder.GetProgress().recording && recorder.CanResume(folder.Dir());
	snprintf(detail, sizeof detail, "paused %d saved %d clean %d samples %llu lost %llu bytes %llu of %zu [%s]",
		finished.paused, finished.saved, finished.closedCleanly, static_cast<unsigned long long>(finished.samples),
		static_cast<unsigned long long>(finished.lost), static_cast<unsigned long long>(finished.bytes), bytes.size(),
		finished.error.c_str());
	check("pose recorder: a pause closes the file with a pause record and its wall-clock time", paused, detail);
}

// The privacy rule: nothing published while recording is off reaches the
// file, and turning it on again goes on in the same file behind a resume
// record.
void RecorderPauseAndResume(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	const auto started = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, TestLimits());
	feed.Send(0);
	feed.Send(3);
	bool reached = started == PoseRecorder::StartResult::Started && WaitForSamples(recorder, 2);
	recorder.RequestPause();
	// Published the moment after the switch went off, while the recorder's
	// thread may not even have noticed yet, and all through the pause.
	std::vector<int64_t> offCounts;
	for (int i = 0; i < 300; ++i)
	{
		const protocol::DevicePoseSample off = MakePose(i % 2 ? 3 : 0, 500000 + i);
		offCounts.push_back(off.sampleTimeQpc);
		feed.hub.AppendSampleForTest(off);
	}
	PoseRecorder::Finished first;
	reached = reached && Finish(recorder, first);
	const std::string afterPause = FileContents(folder.File());
	for (int i = 0; i < 300; ++i)
	{
		const protocol::DevicePoseSample off = MakePose(3, 600000 + i);
		offCounts.push_back(off.sampleTimeQpc);
		feed.hub.AppendSampleForTest(off);
	}
	// The player names a device's role in SteamVR while recording is off.
	RecordedDevice changed = Tracker();
	changed.controllerType = "vive_tracker_chest";
	const bool canResume = recorder.CanResume(folder.Dir());
	const auto resumed = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), changed }, TestLimits());
	feed.Send(3);
	feed.Send(0);
	reached = reached && WaitForSamples(recorder, 4);
	recorder.RequestPause();
	PoseRecorder::Finished second;
	reached = reached && Finish(recorder, second);

	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const std::string kinds = KindsOf(back.records);
	bool offAbsent = true;
	for (const Record &record : back.records)
		if (record.kind == KindSample)
			offAbsent = offAbsent && std::find(offCounts.begin(), offCounts.end(), record.sample.qpc) == offCounts.end();
	char detail[256];
	snprintf(detail, sizeof detail, "reached %d, records %s, %zu poses published while off, absent %d", reached,
		kinds.c_str(), offCounts.size(), offAbsent);
	check("pose recorder: nothing published while recording is off reaches the file",
		reached && offAbsent && SamplesAre(back, feed.sent) && kinds == "SSPRDDSSP" && first.samples == 2, detail);

	const bool sameFile = reached && canResume && resumed == PoseRecorder::StartResult::Resumed &&
		bytes.size() > afterPause.size() && bytes.compare(0, afterPause.size(), afterPause) == 0 &&
		folder.Files() == 1 && back.end == Reader::Next::EndOfFile && kinds == "SSPRDDSSP" &&
		back.records[3].resume.cause == static_cast<uint8_t>(PauseCause::Player) &&
		back.records[3].resume.trigger.empty() && back.records[3].resume.qpc > back.records[2].pause.qpc &&
		back.records[3].resume.unixMicroseconds >= back.records[2].pause.unixMicroseconds &&
		Describes(back.records[4].device, Headset()) && Describes(back.records[5].device, changed) &&
		back.records[4].device.label == back.header.devices[0].label &&
		back.records[5].device.label == back.header.devices[1].label &&
		second.paused && second.samples == 4 && second.bytes == bytes.size() && second.seconds >= first.seconds;
	snprintf(detail, sizeof detail, "can resume %d, start %d, %zu then %zu bytes, %zu file(s), samples %llu",
		canResume, static_cast<int>(resumed), afterPause.size(), bytes.size(), folder.Files(),
		static_cast<unsigned long long>(second.samples));
	check("pose recorder: a resume goes on in the same file behind a resume record and every device again",
		sameFile, detail);
}

// At most one recording. A new one is written beside the old one and takes
// its place when its first sample is written: until a new recording holds
// something, the old one is not lost.
void RecorderOneRecording(Check check)
{
	TempFolder folder;
	Feed feed;
	char detail[256];

	// Yesterday's recording, ended.
	{
		PoseRecorder yesterday;
		yesterday.Start(feed.hub, folder.Dir(), "yesterday", { Headset() }, TestLimits());
		feed.Send(0);
		WaitForSamples(yesterday, 1);
		yesterday.StopAndWait(EndReason::AppExit, 20000);
	}
	const std::string old = FileContents(folder.File());
	const bool oldWhole = KindsOf(ReadBytes(old).records) == "SE";

	// Recording is turned on and off again before SteamVR sends a pose.
	{
		PoseRecorder recorder;
		const bool resumable = recorder.CanResume(folder.Dir());
		const auto started = recorder.Start(feed.hub, folder.Dir(), "today", { Headset() }, TestLimits());
		const bool beside = Exists(folder.File() + L".new") && FileContents(folder.File()) == old;
		recorder.RequestPause();
		PoseRecorder::Finished finished;
		const bool closed = Finish(recorder, finished);
		snprintf(detail, sizeof detail, "old whole %d, resumable %d, start %d, written beside %d, closed %d, saved %d, %zu file(s)",
			oldWhole, resumable, static_cast<int>(started), beside, closed, finished.saved, folder.Files());
		check("pose recorder: a recording that got no data replaces nothing and leaves no file",
			oldWhole && !resumable && started == PoseRecorder::StartResult::Started && beside && closed &&
				!finished.saved && FileContents(folder.File()) == old &&
				folder.Files() == 1 && !recorder.CanResume(folder.Dir()), detail);
	}

	// Now with data: the new recording takes the old one's place at its first
	// sample.
	{
		PoseRecorder recorder;
		recorder.Start(feed.hub, folder.Dir(), "today", { Headset() }, TestLimits());
		const bool keptUntilData = FileContents(folder.File()) == old;
		const size_t before = feed.sent.size();
		feed.Send(0);
		feed.Send(0);
		const bool reached = WaitForSamples(recorder, 2);
		recorder.RequestPause();
		PoseRecorder::Finished finished;
		const bool closed = Finish(recorder, finished);
		const ReadBack back = ReadFile(folder.File());
		const std::vector<Sample> today(feed.sent.begin() + static_cast<std::ptrdiff_t>(before), feed.sent.end());
		snprintf(detail, sizeof detail, "kept until data %d, reached %d, closed %d, writer %s, records %s, %zu file(s)",
			keptUntilData, reached, closed, back.header.writer.c_str(), KindsOf(back.records).c_str(), folder.Files());
		check("pose recorder: a new recording replaces the old one when its first sample is written",
			keptUntilData && reached && closed && finished.saved && back.header.writer == "today" &&
				KindsOf(back.records) == "SSP" && SamplesAre(back, today) && folder.Files() == 1, detail);

		// The paused file is changed behind the recorder's back: it is no longer
		// the file the recorder left, so the next start is a new recording.
		{
			std::ofstream append(folder.File().c_str(), std::ios::binary | std::ios::app);
			append << "x";
		}
		const bool resumable = recorder.CanResume(folder.Dir());
		const auto again = recorder.Start(feed.hub, folder.Dir(), "after a change", { Headset() }, TestLimits());
		feed.Send(0);
		const bool reachedAgain = WaitForSamples(recorder, 1);
		recorder.RequestPause();
		PoseRecorder::Finished after;
		const bool closedAgain = Finish(recorder, after);
		const ReadBack fresh = ReadFile(folder.File());
		snprintf(detail, sizeof detail, "resumable %d, start %d, reached %d, closed %d, writer %s, records %s", resumable,
			static_cast<int>(again), reachedAgain, closedAgain, fresh.header.writer.c_str(), KindsOf(fresh.records).c_str());
		check("pose recorder: a paused file that changed is not gone on with",
			!resumable && again == PoseRecorder::StartResult::Started && reachedAgain && closedAgain &&
				fresh.header.writer == "after a change" && KindsOf(fresh.records) == "SP" && after.samples == 1 &&
				fresh.header.devices[0].label != back.header.devices[0].label, detail);
	}
}

void RecorderSizeCap(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	PoseRecordingLimits limits = TestLimits();
	limits.maxFileBytes = 4096;
	const auto started = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, limits);
	const size_t offered = 200;
	for (size_t i = 0; i < offered; ++i)
		feed.Send(i % 2 ? 3 : 0);
	// No stop is asked for: the cap ends it.
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);
	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const uint64_t size = bytes.size();
	size_t samples = 0;
	bool inOrder = closed && back.headerRead && back.end == Reader::Next::EndOfFile && !back.records.empty() &&
		back.records.back().kind == KindEnd;
	for (size_t i = 0; inOrder && i + 1 < back.records.size(); ++i)
		inOrder = back.records[i].kind == KindSample && samples < feed.sent.size() &&
			SampleBytes(back.records[i].sample) == SampleBytes(feed.sent[samples++]);
	const size_t sampleBytes = RecordPrefixBytes + SamplePayloadBytes;
	const bool capped = inOrder && !finished.paused && finished.reason == EndReason::SizeLimit && finished.closedCleanly &&
		size <= limits.maxFileBytes && size + sampleBytes > limits.maxFileBytes && samples > 0 && samples < offered &&
		finished.samples == samples && back.records.back().end.samples == samples &&
		back.records.back().end.reason == static_cast<uint8_t>(EndReason::SizeLimit) &&
		!recorder.CanResume(folder.Dir());
	char detail[160];
	snprintf(detail, sizeof detail, "start %d, closed %d, reason %d, %zu of %zu samples in %llu bytes under a cap of %llu",
		static_cast<int>(started), closed, static_cast<int>(finished.reason), samples, offered,
		static_cast<unsigned long long>(size), static_cast<unsigned long long>(limits.maxFileBytes));
	check("pose recorder: the size cap ends the file at the last sample that fits", capped, detail);
}

void RecorderDiskFloor(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	PoseRecordingLimits limits = TestLimits();
	DiskFreeBytes.store(limits.minFreeDiskBytes - 1);
	std::string why;
	const auto refused = recorder.Start(feed.hub, folder.Dir(), "test", { Headset() }, limits, &why);
	char detail[192];
	snprintf(detail, sizeof detail, "start %d (%s), %zu file(s) made", static_cast<int>(refused), why.c_str(),
		folder.Files());
	check("pose recorder: a recording does not start on a disk under the floor",
		refused == PoseRecorder::StartResult::DiskSpaceLow && !why.empty() && folder.Files() == 0 &&
			!recorder.GetProgress().recording, detail);

	DiskFreeBytes.store(limits.minFreeDiskBytes);
	const auto started = recorder.Start(feed.hub, folder.Dir(), "test", { Headset() }, limits, &why);
	for (int i = 0; i < 3; ++i)
		feed.Send(0);
	const bool reached = WaitForSamples(recorder, 3);
	// Something else fills the disk.
	DiskFreeBytes.store(limits.minFreeDiskBytes - 1);
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);
	const ReadBack back = ReadFile(folder.File());
	const bool floored = started == PoseRecorder::StartResult::Started && reached && closed && !finished.paused &&
		finished.reason == EndReason::DiskSpaceLow && finished.closedCleanly &&
		back.end == Reader::Next::EndOfFile && KindsOf(back.records) == "SSSE" &&
		back.records.back().end.reason == static_cast<uint8_t>(EndReason::DiskSpaceLow);
	snprintf(detail, sizeof detail, "start %d, reached %d, closed %d, reason %d, records %s", static_cast<int>(started),
		reached, closed, static_cast<int>(finished.reason), KindsOf(back.records).c_str());
	check("pose recorder: a disk that falls under the floor ends the recording cleanly", floored, detail);
}

// The recorder's thread stuck on its disk while the driver keeps publishing:
// the hub takes every sample at once, and the file later says how many the
// recorder was too late for.
void RecorderStalledDisk(Check check)
{
	TempFolder folder;
	Feed feed;
	const uint64_t history = 64;
	feed.hub.SetGeometryForTest(history, 8);
	PoseRecorder recorder;
	const auto started = recorder.Start(feed.hub, folder.Dir(), "test", { Headset() }, TestLimits());
	for (int i = 0; i < 4; ++i)
		feed.Send(0);
	bool reached = started == PoseRecorder::StartResult::Started && WaitForSamples(recorder, 4);

	DiskHeld.store(true);
	reached = reached && WaitFor([] { return DiskWaiting.load(); });
	const size_t during = 1000;
	for (size_t i = 0; i < during; ++i)
		feed.Send(0);
	// All published, and the recorder has none of them.
	const bool hubWentOn = reached && recorder.GetProgress().samples == 4 &&
		feed.hub.ReadDiagnostics().devices[0].received == feed.sent.size();
	DiskHeld.store(false);

	const uint64_t lost = during - history;
	reached = reached && WaitFor([&] { return recorder.GetProgress().samples == 4 + history; });
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);
	const ReadBack back = ReadFile(folder.File());
	uint64_t samples = 0, gapLost = 0;
	bool newest = closed && back.end == Reader::Next::EndOfFile;
	for (const Record &record : back.records)
	{
		if (record.kind == KindGap)
			gapLost += record.gap.lost;
		if (record.kind != KindSample)
			continue;
		// The four from before the stall, then the newest the history kept.
		const size_t index = samples < 4 ? static_cast<size_t>(samples)
			: feed.sent.size() - history + static_cast<size_t>(samples - 4);
		newest = newest && index < feed.sent.size() && SampleBytes(record.sample) == SampleBytes(feed.sent[index]);
		++samples;
	}
	char detail[192];
	snprintf(detail, sizeof detail, "hub went on %d; %llu samples and %llu lost of %zu sent (expected %llu lost), records %.24s...",
		hubWentOn, static_cast<unsigned long long>(samples), static_cast<unsigned long long>(gapLost), feed.sent.size(),
		static_cast<unsigned long long>(lost), KindsOf(back.records).c_str());
	check("pose recorder: a stalled disk costs the recording a gap and never holds the hub",
		hubWentOn && reached && newest && samples == 4 + history && gapLost == lost && finished.lost == lost &&
			samples + gapLost == feed.sent.size(), detail);
}

void RecorderExits(Check check)
{
	char detail[192];
	const int64_t nowMicroseconds = static_cast<int64_t>(std::time(nullptr)) * 1000000;

	// SteamVR quits: the caller waits for the file.
	{
		TempFolder folder;
		Feed feed;
		PoseRecorder recorder;
		recorder.Start(feed.hub, folder.Dir(), "test", { Headset() }, TestLimits());
		feed.Send(0);
		const bool reached = WaitForSamples(recorder, 1);
		const bool waited = recorder.StopAndWait(EndReason::SteamVrQuit, 20000);
		// Closed when the wait returns, before anything takes the result.
		const ReadBack back = ReadFile(folder.File());
		PoseRecorder::Finished finished;
		const bool taken = recorder.TakeFinished(finished);
		snprintf(detail, sizeof detail, "reached %d, waited %d, taken %d, reason %d, records %s", reached, waited, taken,
			static_cast<int>(finished.reason), KindsOf(back.records).c_str());
		check("pose recorder: a SteamVR quit waits for the file and records why it ended",
			reached && waited && taken && !finished.paused && finished.reason == EndReason::SteamVrQuit &&
				finished.closedCleanly && back.end == Reader::Next::EndOfFile && KindsOf(back.records) == "SE" &&
				back.records.back().end.reason == static_cast<uint8_t>(EndReason::SteamVrQuit) &&
				back.records.back().end.samples == 1 &&
				std::abs(back.records.back().end.unixMicroseconds - nowMicroseconds) < 3600ll * 1000000 &&
				!recorder.CanResume(folder.Dir()), detail);
	}

	// The program closes with a recording running and nobody stops it.
	{
		TempFolder folder;
		Feed feed;
		bool reached = false;
		{
			PoseRecorder recorder;
			recorder.Start(feed.hub, folder.Dir(), "test", { Headset() }, TestLimits());
			feed.Send(0);
			feed.Send(0);
			reached = WaitForSamples(recorder, 2);
		}
		const ReadBack back = ReadFile(folder.File());
		snprintf(detail, sizeof detail, "reached %d, records %s, end %d", reached, KindsOf(back.records).c_str(),
			static_cast<int>(back.end));
		check("pose recorder: a recorder destroyed while recording closes its file",
			reached && back.end == Reader::Next::EndOfFile && KindsOf(back.records) == "SSE" &&
				back.records.back().end.reason == static_cast<uint8_t>(EndReason::AppExit), detail);
	}
}

// ---------------------------------------------------------------------------
// The calibration beside the samples, and playback

// A quarter turn about the vertical, which takes (x, y, z) to (z, y, -x), and
// a shift along x.
Alignment QuarterTurn(bool applied, double shift)
{
	Alignment alignment;
	alignment.applied = applied;
	alignment.trackingSystem = "lighthouse";
	alignment.rotation[0] = std::sqrt(0.5);
	alignment.rotation[2] = std::sqrt(0.5);
	alignment.translation[0] = shift;
	return alignment;
}

void RecorderAlignment(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	recorder.SetAlignment(QuarterTurn(true, 10.0));
	bool reached = recorder.Start(feed.hub, folder.Dir(), "test writer", { Headset(), Tracker() }, TestLimits()) ==
		PoseRecorder::StartResult::Started;
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 1);
	// The same values again are no news; new ones are.
	recorder.SetAlignment(QuarterTurn(true, 10.0));
	const uint64_t before = recorder.GetProgress().bytes;
	recorder.SetAlignment(QuarterTurn(true, 12.5));
	reached = reached && WaitFor([&] { return recorder.GetProgress().bytes > before; });
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 2);
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	bool closed = Finish(recorder, finished);
	// The next stretch states it again, so that it reads by itself.
	reached = reached && recorder.Start(feed.hub, folder.Dir(), "test writer", { Headset(), Tracker() }, TestLimits()) ==
		PoseRecorder::StartResult::Resumed;
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 3);
	recorder.RequestPause();
	closed = Finish(recorder, finished) && closed;

	const ReadBack back = ReadBytes(FileContents(folder.File()));
	const std::string kinds = KindsOf(back.records);
	const bool shape = reached && closed && back.end == Reader::Next::EndOfFile && kinds == "ASASPRDDASP";
	const bool values = shape && back.records[0].alignment.SameValues(QuarterTurn(true, 10.0)) &&
		back.records[2].alignment.SameValues(QuarterTurn(true, 12.5)) &&
		back.records[8].alignment.SameValues(QuarterTurn(true, 12.5)) &&
		back.records[0].alignment.qpc == back.header.startQpc &&
		back.records[2].alignment.qpc >= back.records[0].alignment.qpc && SamplesAre(back, feed.sent);
	char detail[160];
	snprintf(detail, sizeof detail, "reached %d, closed %d, records %s, end %d %s", reached, closed, kinds.c_str(),
		static_cast<int>(back.end), back.error.c_str());
	check("pose recorder: the calibration in use is a record at each start and when it changes, and no sample changes",
		values, detail);
}

void RecorderModels(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	RecordedDevice headset = Headset(), tracker = Tracker();
	headset.renderModel = "generic_hmd";
	tracker.renderModel = "{htc}vr_tracker_vive_3_0";
	bool reached = recorder.Start(feed.hub, folder.Dir(), "test writer", { headset, tracker }, TestLimits()) ==
		PoseRecorder::StartResult::Started;
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 1);
	// The same again is no news; another model for a device is.
	recorder.SetDevices({ headset, tracker });
	const uint64_t before = recorder.GetProgress().bytes;
	tracker.renderModel = "{htc}vr_tracker_vive_1_0";
	recorder.SetDevices({ headset, tracker });
	reached = reached && WaitFor([&] { return recorder.GetProgress().bytes > before; });
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 2);
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);

	const ReadBack back = ReadBytes(FileContents(folder.File()));
	const std::string kinds = KindsOf(back.records);
	const bool shape = reached && closed && back.end == Reader::Next::EndOfFile && kinds == "MMSMSP";
	const auto names = [&](size_t i, uint32_t id, const char *name)
	{
		return back.records[i].model.deviceId == id && back.records[i].model.renderModel == name;
	};
	const bool values = shape && names(0, 0, "generic_hmd") && names(1, 3, "{htc}vr_tracker_vive_3_0") &&
		names(3, 3, "{htc}vr_tracker_vive_1_0") && back.header.devices.size() == 2 && SamplesAre(back, feed.sent);
	char detail[160];
	snprintf(detail, sizeof detail, "reached %d, closed %d, records %s, end %d %s", reached, closed, kinds.c_str(),
		static_cast<int>(back.end), back.error.c_str());
	check("pose recorder: a device's render model is a record at the start and when it changes", values, detail);
}

// A full recording gets a record where something was marked, with the moment
// and the device it is about. Nothing after the stop is in the file, a mark
// included.
void RecorderMarks(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, TestLimits()) ==
		PoseRecorder::StartResult::Started;
	feed.Send(0);
	reached = reached && WaitForSamples(recorder, 1);
	const uint64_t before = recorder.GetProgress().bytes;
	recorder.Mark(markname::TrackingLost, 3);
	reached = reached && WaitFor([&] { return recorder.GetProgress().bytes > before; });
	feed.Send(3);
	reached = reached && WaitForSamples(recorder, 2);
	recorder.RequestPause();
	recorder.Mark(markname::Player);
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);

	const ReadBack back = ReadBytes(FileContents(folder.File()));
	const std::string kinds = KindsOf(back.records);
	const bool marked = reached && closed && kinds == "SKSP" && back.records[1].mark.name == markname::TrackingLost &&
		back.records[1].mark.deviceId == 3 && back.records[1].mark.qpc >= back.header.startQpc &&
		back.records[1].mark.qpc <= back.records[3].pause.qpc &&
		back.records[1].mark.unixMicroseconds >= back.header.startUnixMicroseconds &&
		back.records[1].mark.unixMicroseconds <= back.records[3].pause.unixMicroseconds && !finished.clipMode &&
		SamplesAre(back, feed.sent);
	char detail[160];
	snprintf(detail, sizeof detail, "reached %d, closed %d, records %s", reached, closed, kinds.c_str());
	check("pose recorder: a mark is a record in a full recording, with its moment and its device", marked, detail);
}

PoseRecordingLimits ClipLimits(double before, double after)
{
	PoseRecordingLimits limits = TestLimits();
	limits.clips = true;
	limits.clipBeforeSeconds = before;
	limits.clipAfterSeconds = after;
	return limits;
}

// Until this many samples wait in memory for a clip.
bool WaitForKept(const PoseRecorder &recorder, uint64_t samples)
{
	return WaitFor([&] { return recorder.GetProgress().kept == samples; });
}

// "Automatic": nothing reaches the file until something is marked. A mark
// starts a clip with what waited in memory, and the clip states every device
// as it was in front of its first sample.
void RecorderClips(Check check)
{
	using Start = PoseRecorder::StartResult;
	TempFolder folder;
	Feed feed;
	char detail[400];

	// Yesterday's recording, ended.
	{
		PoseRecorder yesterday;
		yesterday.Start(feed.hub, folder.Dir(), "yesterday", { Headset() }, TestLimits());
		feed.Send(0);
		WaitForSamples(yesterday, 1);
		yesterday.StopAndWait(EndReason::AppExit, 20000);
	}
	const std::string old = FileContents(folder.File());
	PoseRecorder recorder;

	// Poses arrive and nothing is marked.
	{
		const auto started = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), Tracker() }, ClipLimits(60.0, 60.0));
		feed.Send(0);
		feed.Send(3);
		const bool kept = WaitForKept(recorder, 2);
		const PoseRecorder::Progress waiting = recorder.GetProgress();
		recorder.RequestPause();
		PoseRecorder::Finished finished;
		const bool closed = Finish(recorder, finished);
		snprintf(detail, sizeof detail, "start %d, kept %d, samples %llu, closed %d, saved %d, clips %llu, %zu file(s)",
			static_cast<int>(started), kept, static_cast<unsigned long long>(waiting.samples), closed, finished.saved,
			static_cast<unsigned long long>(finished.clips), folder.Files());
		check("pose recorder: a recording of clips with nothing marked saves nothing and replaces nothing",
			started == Start::Started && kept && waiting.samples == 0 && closed && !finished.saved && finished.clipMode &&
				finished.clips == 0 && FileContents(folder.File()) == old && folder.Files() == 1, detail);
	}

	// Four poses wait, the tracker's role changes between them, and a base
	// station moves: the clip holds all four, the mark and what comes after.
	const size_t first = feed.sent.size();
	RecordedDevice changed = Tracker();
	changed.controllerType = "vive_tracker_chest";
	{
		const auto started = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), Tracker() }, ClipLimits(60.0, 60.0));
		feed.Send(0);
		feed.Send(3);
		bool reached = started == Start::Started && WaitForKept(recorder, 2);
		recorder.SetDevices({ Headset(), changed });
		feed.Send(3);
		feed.Send(0);
		reached = reached && WaitForKept(recorder, 4);
		const bool nothingYet = FileContents(folder.File()) == old;
		recorder.Mark(markname::BaseStationMoved);
		reached = reached && WaitForSamples(recorder, 4);
		feed.Send(0);
		reached = reached && WaitForSamples(recorder, 5);
		const PoseRecorder::Progress open = recorder.GetProgress();
		recorder.RequestPause();
		PoseRecorder::Finished finished;
		const bool closed = Finish(recorder, finished);

		const ReadBack back = ReadFile(folder.File());
		const std::string kinds = KindsOf(back.records);
		const std::vector<Sample> clip(feed.sent.begin() + static_cast<std::ptrdiff_t>(first), feed.sent.end());
		bool holds = reached && closed && nothingYet && kinds == "PRDDSSDSSKSP" && back.end == Reader::Next::EndOfFile &&
			back.header.writer == "clips" && SamplesAre(back, clip);
		if (holds)
		{
			const Resume &resume = back.records[1].resume;
			const Mark &mark = back.records[9].mark;
			holds = back.records[0].pause.cause == static_cast<uint8_t>(PauseCause::Clip) &&
				resume.cause == static_cast<uint8_t>(PauseCause::Clip) && resume.trigger == markname::BaseStationMoved &&
				resume.qpc >= back.records[0].pause.qpc && resume.qpc <= mark.qpc &&
				resume.unixMicroseconds <= mark.unixMicroseconds && Describes(back.records[2].device, Headset()) &&
				Describes(back.records[3].device, Tracker()) && Describes(back.records[6].device, changed) &&
				mark.name == markname::BaseStationMoved && mark.deviceId == NoDevice &&
				back.records[11].pause.cause == static_cast<uint8_t>(PauseCause::Player) && open.clipOpen &&
				open.clips == 0 && open.kept == 0 && finished.paused && finished.saved && finished.clips == 1 &&
				finished.samples == 5 && finished.bytes == FileContents(folder.File()).size();
		}
		snprintf(detail, sizeof detail, "start %d, reached %d, closed %d, nothing before the mark %d, records %s, clips %llu",
			static_cast<int>(started), reached, closed, nothingYet, kinds.c_str(),
			static_cast<unsigned long long>(finished.clips));
		check("pose recorder: a mark starts a clip with what waited in memory, every device first, then the mark",
			holds, detail);
	}

	// The recording goes on, poses arrive, and the player stops it: the file
	// is as the clip left it.
	const std::string afterClip = FileContents(folder.File());
	{
		const auto started = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), changed }, ClipLimits(60.0, 60.0));
		feed.Send(0);
		feed.Send(3);
		const bool kept = WaitForKept(recorder, 2);
		recorder.RequestPause();
		PoseRecorder::Finished finished;
		const bool closed = Finish(recorder, finished);
		const bool untouched = FileContents(folder.File()) == afterClip;
		snprintf(detail, sizeof detail, "start %d, kept %d, closed %d, file untouched %d, saved %d, clean %d, can go on %d",
			static_cast<int>(started), kept, closed, untouched, finished.saved, finished.closedCleanly,
			recorder.CanResume(folder.Dir()));
		check("pose recorder: between clips nothing reaches the file, and the player's stop adds nothing to it",
			started == Start::Resumed && kept && closed && untouched && finished.saved && finished.paused &&
				finished.closedCleanly && finished.clips == 1 && finished.samples == 5 && recorder.CanResume(folder.Dir()) &&
				folder.Files() == 1, detail);
	}

	// The player saves what just happened: a clip that ends at its mark. What
	// arrives behind it waits again, and SteamVR's end closes the file.
	{
		const size_t saved = feed.sent.size();
		const auto started = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), changed }, ClipLimits(60.0, 60.0));
		feed.Send(3);
		feed.Send(0);
		feed.Send(3);
		bool reached = started == Start::Resumed && WaitForKept(recorder, 3);
		recorder.Mark(markname::Player, NoDevice, false);
		reached = reached && WaitFor([&] { return recorder.GetProgress().clips == 2; });
		feed.Send(0);
		reached = reached && WaitForKept(recorder, 1);
		const bool closed = recorder.StopAndWait(EndReason::SteamVrQuit, 20000);
		PoseRecorder::Finished finished;
		const bool taken = recorder.TakeFinished(finished);

		const std::string bytes = FileContents(folder.File());
		const ReadBack back = ReadBytes(bytes);
		const std::string kinds = KindsOf(back.records);
		std::vector<Sample> expected(feed.sent.begin() + static_cast<std::ptrdiff_t>(first),
			feed.sent.begin() + static_cast<std::ptrdiff_t>(first) + 5);
		expected.insert(expected.end(), feed.sent.begin() + static_cast<std::ptrdiff_t>(saved),
			feed.sent.begin() + static_cast<std::ptrdiff_t>(saved) + 3);
		bool holds = reached && closed && taken && kinds == "PRDDSSDSSKSPRDDSSSKPE" &&
			bytes.compare(0, afterClip.size(), afterClip) == 0 && back.end == Reader::Next::EndOfFile &&
			SamplesAre(back, expected);
		if (holds)
			holds = back.records[12].resume.cause == static_cast<uint8_t>(PauseCause::Clip) &&
				back.records[12].resume.trigger == markname::Player &&
				back.records[12].resume.qpc >= back.records[11].pause.qpc && Describes(back.records[13].device, Headset()) &&
				Describes(back.records[14].device, changed) &&
				back.records[14].device.label == back.header.devices[1].label &&
				back.records[18].mark.name == markname::Player &&
				back.records[19].pause.cause == static_cast<uint8_t>(PauseCause::Clip) &&
				back.records[20].end.reason == static_cast<uint8_t>(EndReason::SteamVrQuit) &&
				back.records[20].end.samples == 8 && !finished.paused && finished.clips == 2 && finished.samples == 8 &&
				!recorder.CanResume(folder.Dir());
		snprintf(detail, sizeof detail, "start %d, reached %d, closed %d, records %s, clips %llu, samples %llu",
			static_cast<int>(started), reached, closed, kinds.c_str(), static_cast<unsigned long long>(finished.clips),
			static_cast<unsigned long long>(finished.samples));
		check("pose recorder: the player's save is a clip that ends at its mark, and what follows it waits again",
			holds, detail);
	}
}

// A clip reaches back as far as the time it is given and no further: what is
// older left the memory and is in no file.
void RecorderClipsKeepLittle(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	bool reached = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), Tracker() }, ClipLimits(1.0, 60.0)) ==
		PoseRecorder::StartResult::Started;
	feed.Send(0);
	feed.Send(3);
	feed.Send(0);
	reached = reached && WaitForKept(recorder, 3);
	const bool dropped = WaitForKept(recorder, 0);
	const size_t recent = feed.sent.size();
	feed.Send(3);
	feed.Send(0);
	reached = reached && WaitForKept(recorder, 2);
	recorder.Mark(markname::UniverseJump);
	reached = reached && WaitForSamples(recorder, 2);
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);

	const ReadBack back = ReadFile(folder.File());
	const std::string kinds = KindsOf(back.records);
	const std::vector<Sample> expected(feed.sent.begin() + static_cast<std::ptrdiff_t>(recent), feed.sent.end());
	char detail[200];
	snprintf(detail, sizeof detail, "reached %d, old poses dropped %d, closed %d, records %s, samples %llu", reached,
		dropped, closed, kinds.c_str(), static_cast<unsigned long long>(finished.samples));
	check("pose recorder: a clip reaches back no further than the time before its mark",
		reached && dropped && closed && kinds == "PRDDSSKP" && SamplesAre(back, expected) && finished.samples == 2,
		detail);
}

PoseRecordingLimits KeepLimits(uint64_t cap, uint64_t headroom, uint64_t spacing)
{
	PoseRecordingLimits limits = TestLimits();
	limits.maxFileBytes = cap;
	limits.keepNewest = true;
	limits.trimHeadroomBytes = headroom;
	limits.trimSpacingBytes = spacing;
	return limits;
}

// Every sample the recorder wrote so far, those that went with a dropped
// oldest part included.
uint64_t Written(const PoseRecorder &recorder)
{
	const PoseRecorder::Progress progress = recorder.GetProgress();
	return progress.samples + progress.trimmedSamples;
}

// Whether the file's samples are the newest of `sent`, in order; `count`
// takes how many it holds.
bool NewestOf(const ReadBack &back, const std::vector<Sample> &sent, size_t &count)
{
	count = 0;
	for (const Record &record : back.records)
		count += record.kind == KindSample ? 1 : 0;
	return count <= sent.size() &&
		SamplesAre(back, std::vector<Sample>(sent.end() - static_cast<std::ptrdiff_t>(count), sent.end()));
}

// At the cap the oldest part of the file goes and the recording goes on. What
// is left is a whole recording by itself: it starts with a header that says
// of each device what the dropped part said, and holds the newest samples.
void RecorderKeepsNewest(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	const PoseRecordingLimits limits = KeepLimits(16 * 1024, 2048, 1024);
	const RecordedDevice headset = Headset();
	RecordedDevice tracker = Tracker();
	tracker.renderModel = "{htc}vr_tracker_vive_3_0";
	recorder.SetAlignment(QuarterTurn(true, 0.5));
	bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { headset, tracker }, limits) ==
		PoseRecorder::StartResult::Started;
	const auto send = [&](size_t count)
	{
		for (size_t i = 0; i < count; ++i)
			feed.Send(i % 2 ? 3 : 0);
		return WaitFor([&] { return Written(recorder) == feed.sent.size(); });
	};
	reached = reached && send(10);
	// The tracker gets another role early: the part of the file that says so
	// is dropped long before the end.
	RecordedDevice changed = tracker;
	changed.controllerType = "vive_tracker_chest";
	const uint64_t before = recorder.GetProgress().bytes;
	recorder.SetDevices({ headset, changed });
	reached = reached && WaitFor([&] { return recorder.GetProgress().bytes > before; });
	// Three and a half times the cap.
	uint64_t largest = 0;
	for (int batch = 0; batch < 29 && reached; ++batch)
	{
		reached = send(10);
		largest = (std::max)(largest, recorder.GetProgress().bytes);
	}
	const PoseRecorder::Progress progress = recorder.GetProgress();
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);

	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const std::string kinds = KindsOf(back.records);
	size_t inFile = 0;
	bool holds = reached && closed && back.headerRead && back.end == Reader::Next::EndOfFile &&
		bytes.size() <= limits.maxFileBytes && largest <= limits.maxFileBytes && NewestOf(back, feed.sent, inFile) &&
		inFile >= 20 && inFile < feed.sent.size() && kinds.size() == inFile + 3 && kinds.compare(0, 2, "MA") == 0 &&
		kinds.back() == 'P' && kinds.find_first_not_of('S', 2) == kinds.size() - 1;
	if (holds)
		holds = back.header.writer == "test" && back.header.devices.size() == 2 &&
			Describes(back.header.devices[0], headset) && Describes(back.header.devices[1], changed) &&
			back.records[0].model.deviceId == 3 && back.records[0].model.renderModel == tracker.renderModel &&
			back.records[1].alignment.applied && back.records[1].alignment.qpc == back.header.startQpc &&
			back.header.startQpc <= back.records.back().pause.qpc &&
			back.header.startUnixMicroseconds <= back.records.back().pause.unixMicroseconds &&
			finished.samples == inFile && finished.trims >= 1 && finished.trims == progress.trims &&
			progress.trimmedSamples + inFile == feed.sent.size() && finished.paused && finished.closedCleanly &&
			finished.bytes == bytes.size() && finished.seconds >= 0.0 && folder.Files() == 1 &&
			recorder.CanResume(folder.Dir());
	char detail[320];
	snprintf(detail, sizeof detail,
		"reached %d, closed %d, %zu of %zu samples in %zu bytes under a cap of %llu (largest %llu), dropped %llu time(s), "
		"records %.12s..., %zu file(s) [%s]", reached, closed, inFile, feed.sent.size(), bytes.size(),
		static_cast<unsigned long long>(limits.maxFileBytes), static_cast<unsigned long long>(largest),
		static_cast<unsigned long long>(finished.trims), kinds.c_str(), folder.Files(), finished.error.c_str());
	check("pose recorder: at the cap the oldest part goes, and what is left is a whole recording of the newest samples",
		holds, detail);
}

// The same with clips: the file that is left starts at a clip, and every clip
// in it is whole.
void RecorderKeepsNewestClips(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	PoseRecordingLimits limits = KeepLimits(8 * 1024, 1024, 512);
	limits.clips = true;
	limits.clipBeforeSeconds = 60.0;
	limits.clipAfterSeconds = 60.0;
	bool reached = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), Tracker() }, limits) ==
		PoseRecorder::StartResult::Started;
	for (int clip = 0; clip < 14 && reached; ++clip)
	{
		for (int i = 0; i < 6; ++i)
			feed.Send(i % 2 ? 3 : 0);
		reached = WaitForKept(recorder, 6);
		recorder.Mark(markname::Player, NoDevice, false);
		reached = reached && WaitFor([&]
		{
			const PoseRecorder::Progress progress = recorder.GetProgress();
			return Written(recorder) == feed.sent.size() && !progress.clipOpen && progress.kept == 0;
		});
	}
	const PoseRecorder::Progress progress = recorder.GetProgress();
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);

	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const std::string kinds = KindsOf(back.records);
	size_t inFile = 0;
	std::string expected;
	for (uint64_t clip = 0; clip < finished.clips; ++clip)
		expected += "RDDSSSSSSKP";
	bool causes = true;
	for (const Record &record : back.records)
		causes = causes && (record.kind != KindResume || record.resume.cause == static_cast<uint8_t>(PauseCause::Clip)) &&
			(record.kind != KindPause || record.pause.cause == static_cast<uint8_t>(PauseCause::Clip));
	const bool holds = reached && closed && back.end == Reader::Next::EndOfFile && bytes.size() <= limits.maxFileBytes &&
		NewestOf(back, feed.sent, inFile) && finished.clips >= 2 && finished.clips < 14 && kinds == expected && causes &&
		inFile == finished.clips * 6 && finished.samples == inFile && finished.trims >= 1 &&
		progress.clips == finished.clips && finished.closedCleanly && folder.Files() == 1 &&
		recorder.CanResume(folder.Dir());
	char detail[256];
	snprintf(detail, sizeof detail, "reached %d, closed %d, %llu of 14 clips and %zu samples in %zu bytes, dropped %llu time(s), "
		"records %.14s... [%s]", reached, closed, static_cast<unsigned long long>(finished.clips), inFile, bytes.size(),
		static_cast<unsigned long long>(finished.trims), kinds.c_str(), finished.error.c_str());
	check("pose recorder: a recording of clips that drops its oldest part starts at a clip and keeps whole clips",
		holds, detail);
}

// The player stops while the newer part is being copied. The copy is thrown
// away and the file it was made from is the recording, whole, with every
// sample up to the stop. (When the copy is done before the stop arrives, the
// file is the newer part: either way there is one whole file.)
void RecorderStopsWhileDropping(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	PoseRecordingLimits limits = KeepLimits(16 * 1024, 2048, 1024);
	// A byte at a time, so that the stop lands inside the copy.
	limits.trimSliceBytes = 1;
	bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, limits) ==
		PoseRecorder::StartResult::Started;
	bool began = false;
	while (reached && !began && feed.sent.size() < 300)
	{
		for (int i = 0; i < 10; ++i)
			feed.Send(i % 2 ? 3 : 0);
		reached = WaitFor([&] { return Written(recorder) == feed.sent.size(); });
		const PoseRecorder::Progress progress = recorder.GetProgress();
		began = progress.trimming || progress.trims > 0;
	}
	const bool dropping = recorder.GetProgress().trimming;
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);

	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const std::string kinds = KindsOf(back.records);
	size_t inFile = 0;
	const bool holds = reached && began && closed && back.end == Reader::Next::EndOfFile &&
		bytes.size() <= limits.maxFileBytes && NewestOf(back, feed.sent, inFile) && inFile > 0 &&
		finished.samples == inFile && (finished.trims > 0 || inFile == feed.sent.size()) && !kinds.empty() &&
		kinds.back() == 'P' && finished.paused && finished.closedCleanly && finished.bytes == bytes.size() &&
		folder.Files() == 1 && recorder.CanResume(folder.Dir());
	char detail[256];
	snprintf(detail, sizeof detail, "reached %d, a drop began %d, in progress at the stop %d, closed %d, %zu of %zu samples, "
		"dropped %llu time(s), %zu file(s) [%s]", reached, began, dropping, closed, inFile, feed.sent.size(),
		static_cast<unsigned long long>(finished.trims), folder.Files(), finished.error.c_str());
	check("pose recorder: a stop while the oldest part is dropped leaves one whole file", holds, detail);
}

// Another program's hold on the recording: an open handle that, like a script
// that reads the live file, does not let the file be replaced.
HANDLE HoldFile(const std::wstring &path)
{
	return CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL, nullptr);
}

// Windows refuses to put a file in the place of one that any program has open,
// and a virus scanner opens a file the moment it was written and closed: the
// moment the recorder replaces it. The recorder waits for the other program
// (PoseRecordingLimits::replaceWaitMs) and only then calls the step failed.
void RecorderWaitsForHeldFile(Check check)
{
	char detail[320];
	const auto sendTo = [](Feed &feed, size_t count)
	{
		for (size_t i = 0; i < count; ++i)
			feed.Send(i % 2 ? 3 : 0);
	};

	// The newer part cannot take the file's place while the file is held. When
	// the other program lets go, the oldest part is dropped as at any cap.
	{
		TempFolder folder;
		Feed feed;
		PoseRecorder recorder;
		PoseRecordingLimits limits = KeepLimits(16 * 1024, 2048, 1024);
		// Long enough that only the release below ends the wait.
		limits.replaceWaitMs = 60000;
		bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, limits) ==
			PoseRecorder::StartResult::Started;
		sendTo(feed, 10);
		reached = reached && WaitFor([&] { return Written(recorder) == feed.sent.size(); });
		const HANDLE held = HoldFile(folder.File());
		bool waited = false;
		while (reached && held != INVALID_HANDLE_VALUE && !waited && feed.sent.size() < 600)
		{
			sendTo(feed, 10);
			reached = WaitFor([&]
			{
				return recorder.GetProgress().replaceRetries > 0 || Written(recorder) == feed.sent.size();
			});
			waited = recorder.GetProgress().replaceRetries > 0;
		}
		const uint64_t droppedWhileHeld = recorder.GetProgress().trims;
		if (held != INVALID_HANDLE_VALUE)
			CloseHandle(held);
		const bool dropped = waited && WaitFor([&] { return recorder.GetProgress().trims >= 1; });
		sendTo(feed, 10);
		reached = reached && WaitFor([&] { return Written(recorder) == feed.sent.size(); });
		recorder.RequestPause();
		PoseRecorder::Finished finished;
		const bool closed = Finish(recorder, finished);

		const std::string bytes = FileContents(folder.File());
		const ReadBack back = ReadBytes(bytes);
		const std::string kinds = KindsOf(back.records);
		size_t inFile = 0;
		const bool holds = reached && waited && droppedWhileHeld == 0 && dropped && closed &&
			back.end == Reader::Next::EndOfFile && bytes.size() <= limits.maxFileBytes &&
			NewestOf(back, feed.sent, inFile) && inFile > 0 && inFile < feed.sent.size() && finished.samples == inFile &&
			finished.trims >= 1 && finished.replaceRetries >= 1 && finished.error.empty() && !kinds.empty() &&
			kinds.back() == 'P' && finished.paused && finished.closedCleanly && folder.Files() == 1 &&
			recorder.CanResume(folder.Dir());
		snprintf(detail, sizeof detail, "reached %d, held %d, waited %d, dropped while held %llu, dropped after %d, closed %d, "
			"%zu of %zu samples, dropped %llu time(s), tried again %llu time(s), %zu file(s) [%s]", reached,
			held != INVALID_HANDLE_VALUE, waited, static_cast<unsigned long long>(droppedWhileHeld), dropped, closed, inFile,
			feed.sent.size(), static_cast<unsigned long long>(finished.trims),
			static_cast<unsigned long long>(finished.replaceRetries), folder.Files(), finished.error.c_str());
		check("pose recorder: the oldest part is dropped once another program lets go of the file", holds, detail);
	}

	// The other program never lets go. The wait ends, the copy is thrown away,
	// and the file that was written all along is the recording: whole, with the
	// oldest samples, ended by the cap.
	{
		TempFolder folder;
		Feed feed;
		PoseRecorder recorder;
		PoseRecordingLimits limits = KeepLimits(16 * 1024, 2048, 1024);
		limits.replaceWaitMs = 20;
		bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, limits) ==
			PoseRecorder::StartResult::Started;
		sendTo(feed, 10);
		reached = reached && WaitFor([&] { return Written(recorder) == feed.sent.size(); });
		const HANDLE held = HoldFile(folder.File());
		bool ended = false;
		while (reached && held != INVALID_HANDLE_VALUE && !ended && feed.sent.size() < 600)
		{
			sendTo(feed, 10);
			reached = WaitFor([&]
			{
				return !recorder.GetProgress().recording || Written(recorder) == feed.sent.size();
			});
			ended = !recorder.GetProgress().recording;
		}
		PoseRecorder::Finished finished;
		const bool closed = ended && Finish(recorder, finished);
		if (held != INVALID_HANDLE_VALUE)
			CloseHandle(held);

		const std::string bytes = FileContents(folder.File());
		const ReadBack back = ReadBytes(bytes);
		const std::string kinds = KindsOf(back.records);
		size_t inFile = 0;
		for (const Record &record : back.records)
			inFile += record.kind == KindSample ? 1 : 0;
		const bool oldest = inFile >= 20 && inFile < feed.sent.size() &&
			SamplesAre(back, std::vector<Sample>(feed.sent.begin(), feed.sent.begin() + static_cast<std::ptrdiff_t>(inFile)));
		const bool holds = reached && ended && closed && back.end == Reader::Next::EndOfFile &&
			bytes.size() <= limits.maxFileBytes && oldest && finished.samples == inFile && finished.trims == 0 &&
			finished.replaceRetries >= 1 && finished.error.find("could not take its place") != std::string::npos &&
			!finished.paused && finished.reason == EndReason::SizeLimit && !kinds.empty() && kinds.back() == 'E' &&
			finished.closedCleanly && finished.bytes == bytes.size() && folder.Files() == 1;
		snprintf(detail, sizeof detail, "reached %d, held %d, ended %d, closed %d, %zu of %zu samples in %zu bytes, "
			"dropped %llu time(s), tried again %llu time(s), reason %d, %zu file(s) [%s]", reached,
			held != INVALID_HANDLE_VALUE, ended, closed, inFile, feed.sent.size(), bytes.size(),
			static_cast<unsigned long long>(finished.trims), static_cast<unsigned long long>(finished.replaceRetries),
			static_cast<int>(finished.reason), folder.Files(), finished.error.c_str());
		check("pose recorder: a file another program keeps open ends the wait, and the cap ends a whole recording",
			holds, detail);
	}

	// The same step at a new recording's first sample: the old recording is
	// held, and stays as it is until the new one can take its place.
	{
		TempFolder folder;
		Feed feed;
		{
			PoseRecorder yesterday;
			yesterday.Start(feed.hub, folder.Dir(), "yesterday", { Headset() }, TestLimits());
			feed.Send(0);
			WaitForSamples(yesterday, 1);
			yesterday.StopAndWait(EndReason::AppExit, 20000);
		}
		const std::string old = FileContents(folder.File());
		PoseRecorder recorder;
		PoseRecordingLimits limits = TestLimits();
		limits.replaceWaitMs = 60000;
		const HANDLE held = HoldFile(folder.File());
		const bool started = recorder.Start(feed.hub, folder.Dir(), "today", { Headset() }, limits) ==
			PoseRecorder::StartResult::Started;
		const size_t before = feed.sent.size();
		feed.Send(0);
		feed.Send(0);
		const bool waited = started && held != INVALID_HANDLE_VALUE &&
			WaitFor([&] { return recorder.GetProgress().replaceRetries > 0; });
		const bool keptMeanwhile = FileContents(folder.File()) == old;
		if (held != INVALID_HANDLE_VALUE)
			CloseHandle(held);
		const bool reached = WaitForSamples(recorder, 2);
		recorder.RequestPause();
		PoseRecorder::Finished finished;
		const bool closed = Finish(recorder, finished);
		const ReadBack back = ReadFile(folder.File());
		const std::vector<Sample> today(feed.sent.begin() + static_cast<std::ptrdiff_t>(before), feed.sent.end());
		snprintf(detail, sizeof detail, "started %d, waited %d, old kept meanwhile %d, reached %d, closed %d, writer %s, "
			"records %s, tried again %llu time(s), %zu file(s) [%s]", started, waited, keptMeanwhile, reached, closed,
			back.header.writer.c_str(), KindsOf(back.records).c_str(),
			static_cast<unsigned long long>(finished.replaceRetries), folder.Files(), finished.error.c_str());
		check("pose recorder: a new recording waits for another program to let go of the old one",
			started && waited && keptMeanwhile && reached && closed && finished.saved && finished.error.empty() &&
				finished.replaceRetries >= 1 && back.header.writer == "today" && KindsOf(back.records) == "SSP" &&
				SamplesAre(back, today) && folder.Files() == 1, detail);
	}
}

// The first sample record behind each place a reader forgets its devices (the
// header, a resume, a sync) is a whole one, never a block.
bool StretchesStartWhole(const ReadBack &back)
{
	bool fresh = true, whole = true;
	for (const Record &record : back.records)
	{
		if (record.kind == KindResume || record.kind == KindSync)
			fresh = true;
		else if (record.kind == KindSample)
			fresh = false;
		else if (record.kind == KindSamples)
			whole = whole && !fresh;
	}
	return whole;
}

// A compact recording: the same stream, gaps and boundaries, each sample
// within the format's steps. It goes on as it began, whatever is asked for at
// the resume, and so does a raw one.
void RecorderCompact(Check check)
{
	using Start = PoseRecorder::StartResult;
	char detail[320];
	{
		TempFolder folder;
		Feed feed;
		PoseRecorder recorder;
		PoseRecordingLimits limits = TestLimits();
		limits.compact = true;
		bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, limits) == Start::Started;
		const auto send = [&](size_t count)
		{
			for (size_t i = 0; i < count; ++i)
				feed.Send(i % 2 ? 3 : 0);
			return WaitForSamples(recorder, feed.sent.size());
		};
		reached = reached && send(40);
		feed.hub.AppendGapForTest(5);
		reached = reached && send(40);
		feed.hub.AppendSessionBoundaryForTest();
		reached = reached && send(40);
		recorder.RequestPause();
		PoseRecorder::Finished first;
		reached = reached && Finish(recorder, first);
		const std::string afterPause = FileContents(folder.File());
		// Raw is asked for now: the file is a compact one and stays one.
		const auto resumed = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, TestLimits());
		reached = reached && send(40);
		recorder.RequestPause();
		PoseRecorder::Finished second;
		reached = reached && Finish(recorder, second);

		const std::string bytes = FileContents(folder.File());
		const ReadBack back = ReadBytes(bytes);
		const std::string kinds = KindsOf(back.records);
		const std::string stored = StoredAre(back, feed.sent);
		size_t inBlocks = 0, gaps = 0, boundaries = 0, resumes = 0;
		for (const Record &record : back.records)
		{
			inBlocks += record.kind == KindSamples ? record.samples.size() : 0;
			gaps += record.kind == KindGap ? 1 : 0;
			boundaries += record.kind == KindBoundary ? 1 : 0;
			resumes += record.kind == KindResume ? 1 : 0;
		}
		LARGE_INTEGER frequency{};
		QueryPerformanceFrequency(&frequency);
		const bool holds = reached && resumed == Start::Resumed && back.headerRead && back.end == Reader::Next::EndOfFile &&
			back.header.version == FormatVersionCompact && back.header.mode == ModeCompact &&
			back.header.velocityScale == limits.velocityScale && back.header.angularScale == limits.angularScale &&
			back.header.qpcFrequency == frequency.QuadPart && stored.empty() && inBlocks >= 150 && gaps == 2 &&
			boundaries == 1 && resumes == 1 && StretchesStartWhole(back) && kinds.compare(0, 2, "SS") == 0 &&
			kinds.back() == 'P' && bytes.compare(0, afterPause.size(), afterPause) == 0 && first.compact && second.compact &&
			second.samples == feed.sent.size() && second.bytes == bytes.size() && second.closedCleanly &&
			bytes.size() * 3 < feed.sent.size() * (RecordPrefixBytes + SamplePayloadBytes);
		snprintf(detail, sizeof detail, "reached %d, resume %d, version %u, %zu of %zu samples in blocks, %zu bytes, "
			"records %.24s..., end %d %s %s", reached, static_cast<int>(resumed), back.header.version, inBlocks,
			feed.sent.size(), bytes.size(), kinds.c_str(), static_cast<int>(back.end), back.error.c_str(), stored.c_str());
		check("pose recorder: a compact recording holds the hub's stream within its steps and goes on compact", holds,
			detail);
	}
	{
		// The other way round: a raw recording goes on raw when compact is asked for.
		TempFolder folder;
		Feed feed;
		PoseRecorder recorder;
		bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, TestLimits()) == Start::Started;
		feed.Send(0);
		feed.Send(3);
		reached = reached && WaitForSamples(recorder, 2);
		recorder.RequestPause();
		PoseRecorder::Finished first;
		reached = reached && Finish(recorder, first);
		PoseRecordingLimits limits = TestLimits();
		limits.compact = true;
		const auto resumed = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, limits);
		for (int i = 0; i < 20; ++i)
			feed.Send(i % 2 ? 3 : 0);
		reached = reached && WaitForSamples(recorder, feed.sent.size());
		recorder.RequestPause();
		PoseRecorder::Finished second;
		reached = reached && Finish(recorder, second);
		const ReadBack back = ReadFile(folder.File());
		// A velocity scale no compact file has: the new recording is a raw one.
		Feed other;
		TempFolder elsewhere;
		PoseRecorder plain;
		limits.velocityScale = 7;
		bool wrote = plain.Start(other.hub, elsewhere.Dir(), "test", { Headset() }, limits) == Start::Started;
		other.Send(0);
		wrote = wrote && WaitForSamples(plain, 1);
		plain.RequestPause();
		PoseRecorder::Finished third;
		wrote = wrote && Finish(plain, third);
		const ReadBack fallback = ReadFile(elsewhere.File());
		snprintf(detail, sizeof detail, "reached %d, resume %d, version %u, records %.24s...; with a scale of 7: wrote %d, "
			"version %u", reached, static_cast<int>(resumed), back.header.version, KindsOf(back.records).c_str(), wrote,
			fallback.header.version);
		check("pose recorder: a raw recording goes on raw, and a compact one that cannot be made is a raw one",
			reached && resumed == Start::Resumed && back.end == Reader::Next::EndOfFile &&
				back.header.version == FormatVersion && SamplesAre(back, feed.sent) && !first.compact && !second.compact &&
				wrote && fallback.header.version == FormatVersion && SamplesAre(fallback, other.sent) && !third.compact,
			detail);
	}
}

// "Automatic" in a compact file: what waited in memory is made into blocks
// when the clip is written, behind the clip's resume record, so every clip
// starts with whole samples and reads by itself.
void RecorderCompactClips(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	PoseRecordingLimits limits = ClipLimits(60.0, 60.0);
	limits.compact = true;
	bool reached = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), Tracker() }, limits) ==
		PoseRecorder::StartResult::Started;
	// Thirty poses wait; a base station moves; ten more follow.
	for (int i = 0; i < 30; ++i)
		feed.Send(i % 2 ? 3 : 0);
	reached = reached && WaitForKept(recorder, 30);
	recorder.Mark(markname::BaseStationMoved);
	reached = reached && WaitForSamples(recorder, 30);
	for (int i = 0; i < 10; ++i)
		feed.Send(i % 2 ? 3 : 0);
	reached = reached && WaitForSamples(recorder, 40);
	recorder.RequestPause();
	PoseRecorder::Finished first;
	reached = reached && Finish(recorder, first);
	// The recording goes on: twenty wait, and the player saves them.
	const auto resumed = recorder.Start(feed.hub, folder.Dir(), "clips", { Headset(), Tracker() }, limits);
	for (int i = 0; i < 20; ++i)
		feed.Send(i % 2 ? 3 : 0);
	reached = reached && WaitForKept(recorder, 20);
	recorder.Mark(markname::Player, NoDevice, false);
	reached = reached && WaitFor([&] { return recorder.GetProgress().clips == 2; });
	recorder.RequestPause();
	PoseRecorder::Finished second;
	reached = reached && Finish(recorder, second);

	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const std::string kinds = KindsOf(back.records);
	const std::string stored = StoredAre(back, feed.sent);
	size_t inBlocks = 0, resumes = 0;
	for (const Record &record : back.records)
	{
		inBlocks += record.kind == KindSamples ? record.samples.size() : 0;
		resumes += record.kind == KindResume ? 1 : 0;
	}
	const bool holds = reached && resumed == PoseRecorder::StartResult::Resumed && back.end == Reader::Next::EndOfFile &&
		back.header.version == FormatVersionCompact && stored.empty() && resumes == 2 && StretchesStartWhole(back) &&
		kinds.compare(0, 6, "PRDDSS") == 0 && inBlocks == feed.sent.size() - 4 && second.clips == 2 && second.compact &&
		second.samples == feed.sent.size() && second.bytes == bytes.size();
	char detail[320];
	snprintf(detail, sizeof detail, "reached %d, resume %d, version %u, records %s, %zu of %zu samples in blocks, clips %llu, "
		"end %d %s %s", reached, static_cast<int>(resumed), back.header.version, kinds.c_str(), inBlocks, feed.sent.size(),
		static_cast<unsigned long long>(second.clips), static_cast<int>(back.end), back.error.c_str(), stored.c_str());
	check("pose recorder: a compact clip starts with whole samples and holds what waited in memory within its steps",
		holds, detail);
}

// The size cap in a compact file: the part that stays starts behind a sync, so
// the file that is left is a whole compact recording of the newest samples.
void RecorderCompactKeepsNewest(Check check)
{
	TempFolder folder;
	Feed feed;
	PoseRecorder recorder;
	// A place to cut every 2 KB, each of which costs a sync and two whole
	// samples: the file is still mostly blocks.
	PoseRecordingLimits limits = KeepLimits(16 * 1024, 2048, 2048);
	limits.compact = true;
	bool reached = recorder.Start(feed.hub, folder.Dir(), "test", { Headset(), Tracker() }, limits) ==
		PoseRecorder::StartResult::Started;
	uint64_t largest = 0;
	for (int batch = 0; batch < 80 && reached; ++batch)
	{
		for (int i = 0; i < 50; ++i)
			feed.Send(i % 2 ? 3 : 0);
		reached = WaitFor([&] { return Written(recorder) == feed.sent.size(); });
		largest = (std::max)(largest, recorder.GetProgress().bytes);
	}
	const PoseRecorder::Progress progress = recorder.GetProgress();
	recorder.RequestPause();
	PoseRecorder::Finished finished;
	const bool closed = Finish(recorder, finished);

	const std::string bytes = FileContents(folder.File());
	const ReadBack back = ReadBytes(bytes);
	const std::string kinds = KindsOf(back.records);
	const std::vector<Sample> stored = SamplesOf(back);
	std::string newest = stored.size() > feed.sent.size() ? "the file holds more samples than were sent" : "";
	if (newest.empty())
		newest = StoredAre(back, std::vector<Sample>(feed.sent.end() - static_cast<std::ptrdiff_t>(stored.size()),
			feed.sent.end()));
	const bool holds = reached && closed && back.headerRead && back.end == Reader::Next::EndOfFile &&
		back.header.version == FormatVersionCompact && bytes.size() <= limits.maxFileBytes &&
		largest <= limits.maxFileBytes && newest.empty() && stored.size() >= 20 && stored.size() < feed.sent.size() &&
		StretchesStartWhole(back) && finished.samples == stored.size() && finished.trims >= 1 &&
		progress.trimmedSamples + stored.size() == feed.sent.size() && finished.closedCleanly &&
		finished.bytes == bytes.size() && folder.Files() == 1 && recorder.CanResume(folder.Dir());
	char detail[320];
	snprintf(detail, sizeof detail, "reached %d, closed %d, %zu of %zu samples in %zu bytes under a cap of %llu (largest "
		"%llu), dropped %llu time(s), records %.16s..., end %d %s %s", reached, closed, stored.size(), feed.sent.size(),
		bytes.size(), static_cast<unsigned long long>(limits.maxFileBytes), static_cast<unsigned long long>(largest),
		static_cast<unsigned long long>(finished.trims), kinds.c_str(), static_cast<int>(back.end), back.error.c_str(),
		newest.c_str());
	check("pose recorder: a compact recording that drops its oldest part is a whole compact recording of the newest "
		"samples", holds, detail);
}

// What turns into a mark: a count of Nova Calibrator's that went up, and a
// device that had tracking for a while and lost it.
void TriggerWatchScenarios(Check check)
{
	const auto names = [](const std::vector<Trigger> &triggers)
	{
		std::string text;
		for (const Trigger &trigger : triggers)
			text += std::string(text.empty() ? "" : " ") + trigger.name +
				(trigger.deviceId == NoDevice ? "" : ":" + std::to_string(trigger.deviceId));
		return text;
	};
	TriggerWatch watch;
	std::vector<Trigger> triggers;
	TriggerCounts counts;
	counts.universeJumps = 3;
	counts.baseStationMoves = 1;
	std::vector<TriggerDevice> devices = { { 0, true }, { 3, true } };
	watch.Step(0.0, counts, devices, triggers);
	const std::string firstLook = names(triggers);
	// Two jumps in one step are one mark; a stop is marked when it begins.
	counts.universeJumps = 5;
	counts.calibrationStopped = true;
	watch.Step(0.1, counts, devices, triggers);
	const std::string jumped = names(triggers);
	triggers.clear();
	watch.Step(0.2, counts, devices, triggers);
	const std::string unchanged = names(triggers);
	counts.baseStationMoves = 2;
	counts.deviceSlides = 1;
	counts.trackingJumps = 1;
	counts.realignments = 1;
	counts.calibrationStopped = false;
	watch.Step(0.3, counts, devices, triggers);
	const std::string rest = names(triggers);
	const std::string detail = "first look \"" + firstLook + "\", then \"" + jumped + "\", \"" + unchanged + "\", \"" +
		rest + "\"";
	check("motion recorder: a count that went up is one mark, and the first look marks nothing",
		firstLook.empty() && jumped == "universe-jump calibration-stopped" && unchanged.empty() &&
			rest == "base-station-moved device-slid tracking-jumped calibration-realigned", detail.c_str());

	// The tracker loses tracking before it had it for long: no news, however
	// long it stays away.
	const auto step = [&](double time, bool tracked)
	{
		devices.back().tracked = tracked;
		watch.Step(time, counts, devices, triggers);
	};
	triggers.clear();
	step(0.4, false);
	step(0.95, false);
	const std::string early = names(triggers);
	// It tracks again for two seconds. A break of a fifth of a second is not
	// news, and does not start the two seconds again.
	step(1.0, true);
	step(3.1, true);
	step(3.2, false);
	step(3.3, false);
	step(3.4, true);
	step(3.5, false);
	step(3.9, false);
	const std::string brief = names(triggers);
	// Half a second without tracking is news, once.
	step(4.0, false);
	step(4.6, false);
	const std::string lost = names(triggers);
	triggers.clear();
	// Tracking that came back a moment ago and goes again is not.
	step(4.7, true);
	step(4.8, false);
	step(5.5, false);
	const std::string flicker = names(triggers);
	// A loss with no step inside it (a late frame) is marked when it ends.
	step(5.6, true);
	step(7.7, true);
	step(7.8, false);
	step(8.5, true);
	const std::string late = names(triggers);
	triggers.clear();
	// A device SteamVR no longer lists has no tracking.
	devices.erase(devices.begin());
	watch.Step(9.0, counts, devices, triggers);
	const std::string justGone = names(triggers);
	watch.Step(9.5, counts, devices, triggers);
	watch.Step(10.5, counts, devices, triggers);
	const std::string unlisted = names(triggers);
	// After a reset the next step is a first look again.
	triggers.clear();
	watch.Reset();
	counts.universeJumps = 9;
	watch.Step(11.0, counts, devices, triggers);
	const std::string afterReset = names(triggers);
	const std::string deviceDetail = "early \"" + early + "\", brief \"" + brief + "\", lost \"" + lost + "\", flicker \"" +
		flicker + "\", late \"" + late + "\", just gone \"" + justGone + "\", unlisted \"" + unlisted +
		"\", after a reset \"" + afterReset + "\"";
	check("motion recorder: a device that had tracking and stays without it is marked once, and a short break is not",
		early.empty() && brief.empty() && lost == "tracking-lost:3" && flicker.empty() && late == "tracking-lost:3" &&
			justGone.empty() && unlisted == "tracking-lost:0" && afterReset.empty(), deviceDetail.c_str());
}

// The reference recording of two clips, played back: each clip is a stretch
// that names what started it, and each mark a moment on its stretch's clock
// with the device it is about.
void PlaybackMarks(Check check)
{
	std::string bytes;
	for (const reference::File &file : reference::Files())
		if (file.name == "clips")
			bytes = file.bytes;
	const auto shared = std::make_shared<const std::string>(bytes);
	Playback playback;
	playback.Open([shared] {
		return std::unique_ptr<std::istream>(new std::istringstream(*shared, std::ios::in | std::ios::binary));
	}, bytes.size(), true);
	const Playback::Summary summary = playback.GetSummary();
	bool holds = summary.state == Playback::State::Ready && summary.stretches.size() == 2 && summary.marks.size() == 3 &&
		summary.samples == 55 && !summary.cutShort;
	if (holds)
	{
		const std::vector<Playback::Mark> &marks = summary.marks;
		const Playback::Stretch &one = summary.stretches[0], &two = summary.stretches[1];
		holds = one.trigger == markname::BaseStationMoved && two.trigger == markname::TrackingLost &&
			marks[0].name == markname::BaseStationMoved && !marks[0].hasDevice && marks[0].time > 0.0 &&
			marks[0].time <= one.seconds && marks[1].name == markname::TrackingLost && marks[1].hasDevice &&
			marks[1].device.deviceId == 3 && marks[1].device.controllerType == "vive_tracker_waist" &&
			marks[1].time > two.start && marks[1].time < marks[2].time && marks[2].name == markname::Player &&
			!marks[2].hasDevice && marks[2].time <= summary.seconds &&
			marks[1].unixMicroseconds > marks[0].unixMicroseconds;
	}
	char detail[200];
	snprintf(detail, sizeof detail, "state %d, %zu stretch(es), %zu mark(s), %llu samples, %.4f s",
		static_cast<int>(summary.state), summary.stretches.size(), summary.marks.size(),
		static_cast<unsigned long long>(summary.samples), summary.seconds);
	check("playback: a clip is a stretch that names what started it, and a mark is a moment on its clock", holds,
		detail);
}

// Two devices for three seconds: two seconds, a stop, and one second more on
// a clock that started again (the computer was restarted), with a loss half
// way through it. The tracker moves along x at one meter a second in a driver
// space one meter along z from its world; the headset drifts along z. The
// first stretch has a calibration for the tracker's system, the second none.
Sample PlaybackPose(uint32_t id, int64_t count, double t)
{
	Sample sample;
	sample.qpc = count;
	sample.deviceId = id;
	sample.trackingResult = 200;
	sample.poseValid = true;
	sample.connected = true;
	if (id == 0)
	{
		sample.position[1] = 1.75;
		sample.position[2] = 0.125 * t;
	}
	else
	{
		sample.position[0] = t;
		sample.position[1] = 1.0;
		sample.worldFromDriverTranslation[2] = 1.0;
	}
	return sample;
}

std::string PlaybackRecording()
{
	using namespace reference;
	std::string bytes;
	AppendHeader(bytes, FileHeader());
	Alignment alignment = QuarterTurn(true, 10.0);
	alignment.qpc = StartCount;
	AppendAlignment(bytes, alignment);
	AppendModel(bytes, { 3, "{htc}vr_tracker_vive_3_0" });
	for (int i = 0; i <= 200; ++i)
		for (uint32_t id : { 0u, 3u })
			AppendSample(bytes, PlaybackPose(id, StartCount + i * (ClockRate / 100), i / 100.0));
	AppendPause(bytes, { 1, StartCount + 2 * ClockRate + 5, StartTime + 2000000 });
	AppendResume(bytes, { 1, 77, StartTime + 3600ll * 1000000, std::string() });
	alignment = QuarterTurn(false, 10.0);
	alignment.qpc = 77;
	AppendAlignment(bytes, alignment);
	for (int i = 0; i <= 100; ++i)
	{
		if (i == 50)
			AppendGap(bytes, { 77 + i * (ClockRate / 100), 9, 9 });
		for (uint32_t id : { 0u, 3u })
			AppendSample(bytes, PlaybackPose(id, 77 + i * (ClockRate / 100), i / 100.0));
	}
	AppendEnd(bytes, { 4, 77 + ClockRate + 5, StartTime + 3601ll * 1000000, 604 });
	return bytes;
}

Playback::Source SourceOf(const std::string &bytes)
{
	const auto shared = std::make_shared<const std::string>(bytes);
	return [shared] {
		return std::unique_ptr<std::istream>(new std::istringstream(*shared, std::ios::in | std::ios::binary));
	};
}

const Playback::DevicePose *PoseOf(const std::vector<Playback::DevicePose> &poses, uint32_t id)
{
	for (const Playback::DevicePose &pose : poses)
		if (pose.info.deviceId == id)
			return &pose;
	return nullptr;
}

bool Near(const Playback::DevicePose *pose, double x, double y, double z)
{
	return pose && pose->tracked && std::abs(pose->position[0] - x) < 1e-9 && std::abs(pose->position[1] - y) < 1e-9 &&
		std::abs(pose->position[2] - z) < 1e-9;
}

void PlaybackScenarios(Check check)
{
	const std::string bytes = PlaybackRecording();
	Playback playback;
	std::vector<Playback::DevicePose> poses;
	const bool nothingYet = !playback.Frame(0.0, poses) && playback.GetSummary().state == Playback::State::Empty;
	playback.Open(SourceOf(bytes), bytes.size(), true);
	const Playback::Summary summary = playback.GetSummary();
	char detail[256];
	snprintf(detail, sizeof detail, "state %d, %.6f s, %zu stretches, %zu losses, %llu samples, %zu devices, cut short %d %s",
		static_cast<int>(summary.state), summary.seconds, summary.stretches.size(), summary.losses.size(),
		static_cast<unsigned long long>(summary.samples), summary.devices, summary.cutShort, summary.error.c_str());
	const bool timeline = nothingYet && summary.state == Playback::State::Ready && std::abs(summary.seconds - 3.0) < 1e-9 &&
		summary.stretches.size() == 2 && std::abs(summary.stretches[0].seconds - 2.0) < 1e-9 &&
		std::abs(summary.stretches[1].start - 2.0) < 1e-9 && std::abs(summary.stretches[1].seconds - 1.0) < 1e-9 &&
		summary.stretches[0].startUnixMicroseconds == reference::StartTime &&
		summary.stretches[1].startUnixMicroseconds == reference::StartTime + 3600ll * 1000000 &&
		summary.losses.size() == 1 && summary.losses[0].samples == 9 && summary.lost == 9 &&
		summary.losses[0].time > 2.48 && summary.losses[0].time <= 2.5 + 1e-9 && summary.samples == 604 &&
		summary.devices == 2 && !summary.cutShort && summary.bytes == bytes.size();
	check("playback: the timeline is the recorder's stretches end to end, each on its own clock, with its losses",
		timeline, detail);

	// In the first stretch the tracker has the calibration: (1, 1, 1) turned
	// and shifted. In the second it is where the driver put it.
	bool placed = playback.Frame(1.0, poses) && poses.size() == 2 && Near(PoseOf(poses, 3), 11.0, 1.0, -1.0) &&
		Near(PoseOf(poses, 0), 0.0, 1.75, 0.125);
	// The tracker's own rotation is none, so it is turned as the calibration
	// turns its system: a quarter turn about the vertical.
	const double *turned = placed ? PoseOf(poses, 3)->rotation : nullptr;
	placed = placed && std::abs(turned[0] - std::sqrt(0.5)) < 1e-9 && std::abs(turned[2] - std::sqrt(0.5)) < 1e-9 &&
		std::abs(turned[1]) < 1e-9 && std::abs(turned[3]) < 1e-9 && PoseOf(poses, 0)->rotation[0] == 1.0 &&
		PoseOf(poses, 3)->renderModel == "{htc}vr_tracker_vive_3_0" && PoseOf(poses, 0)->renderModel.empty();
	const bool described = placed && PoseOf(poses, 3)->info.trackingSystem == "lighthouse" &&
		PoseOf(poses, 3)->info.controllerType == "vive_tracker_waist" && PoseOf(poses, 0)->info.deviceClass == 1;
	placed = placed && playback.Frame(2.5, poses) && Near(PoseOf(poses, 3), 0.5, 1.0, 1.0) &&
		Near(PoseOf(poses, 0), 0.0, 1.75, 0.0625) && PoseOf(poses, 3)->rotation[0] == 1.0 &&
		PoseOf(poses, 3)->renderModel == "{htc}vr_tracker_vive_3_0";
	// Past the end is the end.
	placed = placed && playback.Frame(99.0, poses) && Near(PoseOf(poses, 3), 1.0, 1.0, 1.0);
	check("playback: a device is where its last sample put it, with the calibration the recording names for its system",
		placed && described, placed ? "the device table" : "a position");

	// Back, and forward again: the same answers as the first time.
	bool jumps = playback.Frame(0.5, poses) && Near(PoseOf(poses, 3), 10.0 + 1.0, 1.0, -0.5);
	jumps = jumps && playback.Frame(1.9, poses) && Near(PoseOf(poses, 3), 11.0, 1.0, -1.9) &&
		!PoseOf(poses, 3)->trail.empty();
	jumps = jumps && playback.Frame(2.7, poses) && Near(PoseOf(poses, 3), 0.7, 1.0, 1.0);
	jumps = jumps && playback.Frame(1.9, poses) && Near(PoseOf(poses, 3), 11.0, 1.0, -1.9) &&
		!PoseOf(poses, 3)->trail.empty();
	check("playback: a jump back or ahead gives what playing through gave", jumps, "positions after four jumps");

	// Just into the second stretch, and just past its loss: the path behind the
	// tracker holds nothing from the other side.
	bool cut = playback.Frame(2.05, poses) && PoseOf(poses, 3) != nullptr;
	size_t afterPause = 0, afterLoss = 0;
	if (cut)
	{
		afterPause = PoseOf(poses, 3)->trail.size();
		for (const auto &point : PoseOf(poses, 3)->trail)
			cut = cut && point[0] < 5.0f;
	}
	cut = cut && afterPause >= 1 && afterPause <= 7 && playback.Frame(2.52, poses) && PoseOf(poses, 3) != nullptr;
	if (cut)
		afterLoss = PoseOf(poses, 3)->trail.size();
	cut = cut && afterLoss >= 1 && afterLoss <= 4;
	snprintf(detail, sizeof detail, "%zu points after the stop, %zu after the loss", afterPause, afterLoss);
	check("playback: a path never joins the two sides of a stop or a loss", cut, detail);

	// A file the recorder did not close plays up to where it stops; what is not
	// a recording, or cannot be opened, is refused with a reason.
	Playback shortened, garbage, missing;
	const std::string partial = bytes.substr(0, bytes.size() - 230);
	shortened.Open(SourceOf(partial), partial.size(), true);
	garbage.Open(SourceOf("not a recording, whatever it says"), 33, true);
	missing.Open([] { return std::unique_ptr<std::istream>(); }, 0, true);
	const Playback::Summary a = shortened.GetSummary(), b = garbage.GetSummary(), c = missing.GetSummary();
	const bool told = a.state == Playback::State::Ready && a.cutShort && a.seconds > 2.9 && a.seconds < 3.0 &&
		shortened.Frame(2.0, poses) && poses.size() == 2 && b.state == Playback::State::Failed && !b.error.empty() &&
		!garbage.Frame(0.0, poses) && c.state == Playback::State::Failed && !c.error.empty();
	snprintf(detail, sizeof detail, "cut: state %d, cut short %d, %.3f s; garbage: %d \"%s\"; missing: %d \"%s\"",
		static_cast<int>(a.state), a.cutShort, a.seconds, static_cast<int>(b.state), b.error.c_str(),
		static_cast<int>(c.state), c.error.c_str());
	check("playback: a file that was cut short plays to where it stops, and what is not a recording is refused", told,
		detail);

	// A model's name goes from the file to SteamVR, so it must be a name.
	using questcal::sceneview::PlainModelName;
	const bool names = PlainModelName("generic_hmd") && PlainModelName("{htc}vr_tracker_vive_3_0") &&
		PlainModelName("oculus_quest2_controller_left") && PlainModelName("vr_controller_vive_1_5") &&
		PlainModelName("{indexcontroller}valve_controller_knu_1_0_left") && !PlainModelName("") &&
		!PlainModelName("C:\\Windows\\model.obj") && !PlainModelName("..\\up") && !PlainModelName("a/b") &&
		!PlainModelName("{htc}") && !PlainModelName("{}x") && !PlainModelName("{a/b}x") && !PlainModelName("a..b") &&
		!PlainModelName(".hidden") && !PlainModelName("with space") && !PlainModelName(std::string(129, 'a')) &&
		!PlainModelName("{htc}\\\\server\\share");
	check("playback: only a plain render model name from a recording is given to SteamVR", names,
		"names with a path, a drive, a server or two dots are refused");
}

// A compact recording plays as its raw twin does: the same timeline, and each
// device at the same place within the format's steps, also after a jump to a
// moment whose samples are in the middle of a stretch.
void PlaybackCompact(Check check)
{
	const std::string records = RecordsOf(MotionStream(6.0));
	const std::string rawBytes = RawFileOf(records), compactBytes = CompactFileOf(records, 10000);
	Playback raw, compacted;
	raw.Open(SourceOf(rawBytes), rawBytes.size(), true);
	compacted.Open(SourceOf(compactBytes), compactBytes.size(), true);
	const Playback::Summary a = raw.GetSummary(), b = compacted.GetSummary();
	bool timeline = a.state == Playback::State::Ready && b.state == Playback::State::Ready && a.samples == b.samples &&
		a.samples > 5000 && a.seconds == b.seconds && a.seconds > 5.5 && a.stretches.size() == 2 &&
		b.stretches.size() == 2 && a.lost == b.lost && a.losses.size() == b.losses.size() && a.marks.size() == 1 &&
		b.marks.size() == 1 && a.devices == b.devices && !a.cutShort && !b.cutShort && b.error.empty();
	for (size_t i = 0; timeline && i < a.stretches.size(); ++i)
		timeline = a.stretches[i].start == b.stretches[i].start && a.stretches[i].seconds == b.stretches[i].seconds;
	// Forward, far back, far ahead, and a little back: each a new place to
	// start reading for the compact file.
	double furthest = 0.0;
	size_t compared = 0;
	bool same = true;
	std::vector<Playback::DevicePose> one, two;
	for (double moment : { 0.3, 1.1, 5.2, 0.7, 4.9, 2.6, 2.55, 5.9, 3.3 })
	{
		// A long jump is given a few frames to arrive, as the page gives it.
		for (int frame = 0; frame < 3; ++frame)
			same = raw.Frame(moment, one) && compacted.Frame(moment, two) && same;
		same = same && one.size() == two.size() && one.size() == 4;
		// By id: the order is that of each reading's first samples.
		for (uint32_t id : { 0u, 1u, 2u, 4u })
		{
			const Playback::DevicePose *left = PoseOf(one, id), *right = PoseOf(two, id);
			same = same && left && right && left->tracked && right->tracked && !left->trail.empty() && !right->trail.empty();
			if (!same)
				break;
			for (int axis = 0; axis < 3; ++axis)
				furthest = (std::max)(furthest, std::abs(left->position[axis] - right->position[axis]));
			++compared;
		}
	}
	char detail[256];
	snprintf(detail, sizeof detail, "raw: %llu samples, %.6f s; compact: %llu samples, %.6f s, state %d %s; %zu poses compared, "
		"furthest apart %.3g m, same devices and paths %d", static_cast<unsigned long long>(a.samples), a.seconds,
		static_cast<unsigned long long>(b.samples), b.seconds, static_cast<int>(b.state), b.error.c_str(), compared,
		furthest, same);
	check("playback: a compact recording plays as the raw one does, also after a jump", timeline && same && compared == 36 &&
		furthest < 2e-6, detail);
}

// Every sentence the module shows (Overlay/MotionRecorderText.h) has its
// translation in lang/ja.txt and lang/it.txt, the built messages by their
// patterns.
void TextIsTranslated(Check check)
{
	using questcal::i18n::HasTranslation;
	using questcal::i18n::Language;
	namespace text = questcal::motionrecorder::text;
	std::vector<std::string> shown(std::begin(text::Whole), std::end(text::Whole));
	shown.insert(shown.end(), std::begin(text::Disclaimer), std::end(text::Disclaimer));
	shown.insert(shown.end(), std::begin(text::DeviceNames), std::end(text::DeviceNames));
	shown.insert(shown.end(), std::begin(text::MarkLines), std::end(text::MarkLines));
	char built[160];
	snprintf(built, sizeof built, text::OnForPattern, 12);
	shown.emplace_back(built);
	snprintf(built, sizeof built, text::StatusPattern, 12, 1.25);
	shown.emplace_back(built);
	snprintf(built, sizeof built, text::StatusPattern, 0, 0.0);
	shown.emplace_back(built);
	snprintf(built, sizeof built, text::RecordedPattern, "2026-11-09 22:41");
	shown.emplace_back(built);
	snprintf(built, sizeof built, text::DevicesPattern, 1, "", 0.56);
	shown.emplace_back(built);
	snprintf(built, sizeof built, text::DevicesPattern, 9, "s", 12.0);
	shown.emplace_back(built);
	const std::string folder = "%LOCALAPPDATA%\\NovaCalibrator\\recordings";
	shown.push_back(text::SavedTo + folder + "\\NovaCalibrator-recording.novapose");
	shown.push_back(text::CannotCreateIn + folder + ".");
	std::string missing;
	for (const std::string &message : shown)
		for (Language language : { Language::Japanese, Language::Italian })
			if (!HasTranslation(language, message))
				missing += std::string(missing.empty() ? "" : "; ") + (language == Language::Japanese ? "ja: " : "it: ") + message;
	const std::string detail = std::to_string(shown.size()) + " strings" + (missing.empty() ? "" : ", untranslated: " + missing);
	check("motion recorder: every sentence it shows has a Japanese and an Italian translation", missing.empty(),
		detail.c_str());
}

} // namespace

void RunPoseRecordingScenarios(Check check)
{
	FormatRoundTrip(check);
	CutAnywhere(check);
	UnknownKinds(check);
	Refusals(check);
	ReferenceRecordings(check);
	ReaderProperties(check);
	CompactRoundTrip(check);
	CompactStartsAnywhere(check);
	CompactCutAnywhere(check);
	CompactRefusals(check);
	RecorderKeepsTheStream(check);
	RecorderPauseAndResume(check);
	RecorderOneRecording(check);
	RecorderSizeCap(check);
	RecorderDiskFloor(check);
	RecorderStalledDisk(check);
	RecorderExits(check);
	RecorderAlignment(check);
	RecorderModels(check);
	RecorderMarks(check);
	RecorderClips(check);
	RecorderClipsKeepLittle(check);
	RecorderKeepsNewest(check);
	RecorderKeepsNewestClips(check);
	RecorderStopsWhileDropping(check);
	RecorderWaitsForHeldFile(check);
	RecorderCompact(check);
	RecorderCompactClips(check);
	RecorderCompactKeepsNewest(check);
	TriggerWatchScenarios(check);
	PlaybackScenarios(check);
	PlaybackMarks(check);
	PlaybackCompact(check);
	TextIsTranslated(check);
}
