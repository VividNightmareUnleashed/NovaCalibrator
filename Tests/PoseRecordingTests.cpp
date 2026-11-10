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
			: record.kind == KindResume ? 'R' : record.kind == KindAlignment ? 'A' : '?';
	return kinds;
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
	header("a newer format version", [](Header &h) { h.version = FormatVersion + 1; });
	header("another mode", [](Header &h) { h.mode = 0; });
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
		newer = listingOf(4), aligned = listingOf(5);
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
		has(aligned, "alignment after sample 20: system \"lighthouse\", applied 0, ") && has(aligned, ", scale 1.25\n");
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
	const bool described = placed && PoseOf(poses, 3)->info.trackingSystem == "lighthouse" &&
		PoseOf(poses, 3)->info.controllerType == "vive_tracker_waist" && PoseOf(poses, 0)->info.deviceClass == 1;
	placed = placed && playback.Frame(2.5, poses) && Near(PoseOf(poses, 3), 0.5, 1.0, 1.0) &&
		Near(PoseOf(poses, 0), 0.0, 1.75, 0.0625);
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
	shown.insert(shown.end(), std::begin(text::Views), std::end(text::Views));
	shown.insert(shown.end(), std::begin(text::DeviceNames), std::end(text::DeviceNames));
	char built[160];
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
	RecorderKeepsTheStream(check);
	RecorderPauseAndResume(check);
	RecorderOneRecording(check);
	RecorderSizeCap(check);
	RecorderDiskFloor(check);
	RecorderStalledDisk(check);
	RecorderExits(check);
	RecorderAlignment(check);
	PlaybackScenarios(check);
	TextIsTranslated(check);
}
