#pragma once

// What the reader must keep of any bytes at all, since a recording read back
// is untrusted input. Two drivers run it: the harness, over the reference
// recordings and seeded mutations of them on every run, and the "recording"
// target of Tests/Fuzz/FuzzTargets.h under libFuzzer and AddressSanitizer
// (tools/fuzz.ps1), for the long search.
// Refusing an input is always allowed; what an input may not do is crash, or
// be read as something other than the bytes that are there.

#include "../Overlay/PoseRecordingFormat.h"
#include "../Overlay/RecordingPlayback.h"
#include "ReferenceRecordings.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace questcal::recording
{

// What a file read to where its reader stops holds: its samples in order,
// whole ones and those of blocks alike, and how the reading ended.
struct SamplesRead
{
	bool header = false;
	std::vector<Sample> samples;
	Reader::Next end = Reader::Next::Invalid;
};

inline SamplesRead ReadSamples(const std::string &bytes)
{
	SamplesRead read;
	std::istringstream in(bytes, std::ios::in | std::ios::binary);
	Reader reader(in);
	Header header;
	read.header = reader.ReadHeader(header);
	if (!read.header)
		return read;
	Record record;
	for (read.end = reader.Read(record); read.end == Reader::Next::Record; read.end = reader.Read(record))
	{
		if (record.kind == KindSample)
			read.samples.push_back(record.sample);
		else if (record.kind == KindSamples)
			read.samples.insert(read.samples.end(), record.samples.begin(), record.samples.end());
	}
	return read;
}

// A recording of either version, as far as its reader takes it, written as a
// compact one: every sample through the writer, every other record as it is.
// "" when the file has no header a reader takes.
inline std::string WriteCompact(const std::string &bytes, const compact::Parameters &parameters)
{
	std::istringstream in(bytes, std::ios::in | std::ios::binary);
	Reader reader(in);
	Header header;
	if (!reader.ReadHeader(header))
		return std::string();
	const bool wasCompact = header.version == FormatVersionCompact;
	header.version = FormatVersionCompact;
	header.mode = ModeCompact;
	header.velocityScale = parameters.velocityScale;
	header.angularScale = parameters.angularScale;
	std::string out;
	AppendHeader(out, header);
	CompactWriter writer(parameters);
	Record record;
	while (reader.Read(record) == Reader::Next::Record)
	{
		if (record.kind == KindSample)
			writer.Append(out, record.sample);
		else if (record.kind == KindSamples || record.kind == KindSync)
		{
			// A block's samples; the writer puts its own syncs. In a raw file
			// the two are kinds that mean something else here: left out.
			if (wasCompact)
				for (const Sample &sample : record.samples)
					writer.Append(out, sample);
		}
		else
		{
			writer.Flush(out);
			AppendRecord(out, record);
			if (record.kind == KindResume)
				writer.Restart();
		}
	}
	writer.Flush(out);
	return out;
}

// What a compact file may change in a sample, and no more: the pose time
// offset by half a clock count, and each of the thirteen pose numbers by half
// a step. Nothing in a sample of single-precision numbers, and nothing in one
// that no block can hold. "" when `stored` is `given` within that.
inline std::string StoredWithinSteps(const Sample &given, const Sample &stored, const compact::Parameters &parameters)
{
	std::string before, after;
	AppendSample(before, given);
	AppendSample(after, stored);
	if (before == after)
		return "";
	if (given.qpc != stored.qpc || given.deviceId != stored.deviceId || given.trackingResult != stored.trackingResult ||
		given.poseValid != stored.poseValid || given.connected != stored.connected ||
		std::memcmp(given.worldFromDriverRotation, stored.worldFromDriverRotation, sizeof given.worldFromDriverRotation) != 0 ||
		std::memcmp(given.worldFromDriverTranslation, stored.worldFromDriverTranslation,
			sizeof given.worldFromDriverTranslation) != 0)
		return "a compact file changes a sample's count, device, state or driver frame";
	if (!compact::Usable(given, parameters))
		return "a compact file changes a sample that no block can hold";
	double was[compact::PoseNumbers], now[compact::PoseNumbers];
	compact::PoseOf(given, was);
	compact::PoseOf(stored, now);
	bool single = true;
	for (double value : was)
	{
		uint32_t bits = 0;
		single = single && compact::SingleBits(value, bits);
	}
	if (single)
		return "a compact file changes a sample of single-precision numbers";
	// Half a step, and the last digit of a double of that size.
	const auto within = [](double a, double b, double scale)
	{
		return std::fabs(a - b) <= 0.5 / scale + std::fabs(a) * 1e-15;
	};
	if (!within(given.poseTimeOffset, stored.poseTimeOffset, static_cast<double>(parameters.clockRate)))
		return "a compact file moves a pose time offset by more than half a clock count";
	for (int i = 0; i < compact::PoseNumbers; ++i)
		if (!within(was[i], now[i], compact::ScaleOf(parameters, i)))
			return "a compact file moves a pose number by more than half a step";
	return "";
}

// The compact writer on whatever a reader returned for `bytes`: what it
// writes reads back whole, with every sample within the format's steps of the
// one it was given, and written once more it comes back bit for bit.
inline std::string CheckCompactWriting(const std::string &bytes, const Header &header)
{
	compact::Parameters parameters{ header.qpcFrequency, 10000, 10000 };
	if (header.version == FormatVersionCompact)
		parameters = { header.qpcFrequency, header.velocityScale, header.angularScale };
	// A clock no compact file has: the recorder writes a raw file there.
	if (!compact::Valid(parameters))
		return "";
	const SamplesRead given = ReadSamples(bytes);
	const std::string once = WriteCompact(bytes, parameters);
	const SamplesRead stored = ReadSamples(once);
	if (!stored.header || stored.end != Reader::Next::EndOfFile || stored.samples.size() != given.samples.size())
		return "a compact file the writer made does not read back whole";
	for (size_t i = 0; i < given.samples.size(); ++i)
	{
		const std::string why = StoredWithinSteps(given.samples[i], stored.samples[i], parameters);
		if (!why.empty())
			return why;
	}
	const SamplesRead again = ReadSamples(WriteCompact(once, parameters));
	if (again.end != Reader::Next::EndOfFile || again.samples.size() != stored.samples.size())
		return "a compact file written again does not read back whole";
	for (size_t i = 0; i < stored.samples.size(); ++i)
	{
		std::string first, second;
		AppendSample(first, stored.samples[i]);
		AppendSample(second, again.samples[i]);
		if (first != second)
			return "samples read from a compact file change when they are written again";
	}
	return "";
}

// Returns "" when every property holds, or the property that failed: whatever
// the reader returns before it stops is the front of the input, byte for byte,
// and how it stops fits the bytes that are left.
inline std::string CheckRecording(const uint8_t *data, size_t size)
{
	std::istringstream in(std::string(reinterpret_cast<const char *>(data), size), std::ios::in | std::ios::binary);
	Reader reader(in);
	Header header;
	if (!reader.ReadHeader(header))
		return reader.Error().empty() ? "a recording is refused without a reason" : "";
	const bool isCompact = header.version == FormatVersionCompact;
	if ((isCompact ? header.mode != ModeCompact ||
				!compact::Valid({ header.qpcFrequency, header.velocityScale, header.angularScale })
			: header.version != FormatVersion || header.mode != ModeRaw) ||
		header.qpcFrequency <= 0 || header.devices.size() > MaxHeaderDevices)
		return "an accepted header is one this format does not write";

	std::string again;
	AppendHeader(again, header);
	if (again.size() != reader.BytesRead())
		return "the header's byte count is not the bytes of the header";
	Record record;
	Reader::Next next = reader.Read(record);
	for (; next == Reader::Next::Record; next = reader.Read(record))
	{
		if (record.kind == 0 || record.unknown.size() > 0xFFFF)
			return "a record is returned that no file can hold";
		if (record.kind == KindSamples && isCompact && record.samples.empty())
			return "a block is returned that holds no sample";
		if (!isCompact && !record.samples.empty())
			return "a raw file is read as having a block";
		AppendRecord(again, record);
		if (again.size() != reader.BytesRead())
			return "the reader's byte count is not the bytes of what it returned";
	}
	if (again.size() > size || std::memcmp(again.data(), data, again.size()) != 0)
		return "accepted records do not write back as the bytes they were read from";
	if (reader.Read(record) != next)
		return "the reader goes on after it stopped";

	const size_t rest = size - again.size();
	const size_t declared = rest < RecordPrefixBytes ? 0 : RecordPrefixBytes +
		(static_cast<size_t>(data[again.size() + 1]) | (static_cast<size_t>(data[again.size() + 2]) << 8));
	switch (next)
	{
	case Reader::Next::EndOfFile:
		if (rest != 0)
			return "the end of the file is reported with bytes left";
		break;
	case Reader::Next::Truncated:
		if (rest == 0 || (rest >= RecordPrefixBytes && rest >= declared))
			return "a whole record is reported as cut short";
		break;
	default:
		if (reader.Error().empty())
			return "a record is refused without a reason";
		// Kind 0 is refused by its first byte; any other record is read whole
		// before it is judged.
		if (rest < RecordPrefixBytes || (data[again.size()] != 0 && rest < declared))
			return "a record that is not all there is refused, not reported as cut short";
		break;
	}

	// The playback reads the same bytes. With a header the reader takes, it
	// has a timeline that holds together and puts every device at numbers.
	const auto bytes = std::make_shared<const std::string>(reinterpret_cast<const char *>(data), size);
	Playback playback;
	playback.Open([bytes] {
		return std::unique_ptr<std::istream>(new std::istringstream(*bytes, std::ios::in | std::ios::binary));
	}, size, true);
	const Playback::Summary summary = playback.GetSummary();
	if (summary.state != Playback::State::Ready)
		return "the playback refuses a recording whose header the reader takes";
	double at = 0.0;
	for (const Playback::Stretch &stretch : summary.stretches)
	{
		if (stretch.start != at || !(stretch.seconds >= 0.0) || !std::isfinite(stretch.seconds))
			return "the playback's stretches are not end to end";
		at = stretch.start + stretch.seconds;
	}
	if (at != summary.seconds)
		return "the playback's length is not the sum of its stretches";
	for (const Playback::Loss &loss : summary.losses)
	{
		const bool inside = loss.time >= 0.0 && loss.time <= summary.seconds;
		if (!inside)
			return "the playback puts a loss outside the recording";
	}
	for (const Playback::Mark &mark : summary.marks)
	{
		const bool inside = mark.time >= 0.0 && mark.time <= summary.seconds;
		if (!inside)
			return "the playback puts a mark outside the recording";
	}
	std::vector<Playback::DevicePose> poses;
	for (double moment : { 0.0, summary.seconds * 0.5, summary.seconds, summary.seconds * 0.25 })
	{
		if (!playback.Frame(moment, poses))
			return "the playback has no frame for a recording it read";
		for (const Playback::DevicePose &pose : poses)
			if (!std::isfinite(pose.position[0]) || !std::isfinite(pose.position[1]) || !std::isfinite(pose.position[2]))
				return "the playback puts a device at a position that is not a number";
	}

	// And the other way: what the reader returned, written as a compact file.
	return CheckCompactWriting(std::string(reinterpret_cast<const char *>(data), size), header);
}

// Inputs the reader accepts, for the mutations and the fuzzer to start from:
// the reference recordings, and a header alone of each version.
inline std::vector<std::string> PropertySeeds()
{
	std::vector<std::string> seeds;
	for (const reference::File &file : reference::Files())
		seeds.push_back(file.bytes);
	Header header = reference::FileHeader();
	std::string alone;
	AppendHeader(alone, header);
	seeds.push_back(alone);
	header.version = FormatVersionCompact;
	header.mode = ModeCompact;
	header.velocityScale = header.angularScale = reference::CompactVelocityScale;
	alone.clear();
	AppendHeader(alone, header);
	seeds.push_back(alone);
	return seeds;
}

// The model a block is read against in the "recording-block" property: a
// reader behind the first three samples of the compact reference recording,
// which are whole and of three devices.
inline compact::Parameters BlockParameters()
{
	return { reference::ClockRate, reference::CompactVelocityScale, reference::CompactVelocityScale };
}

inline std::vector<Sample> BlockWholeSamples()
{
	std::vector<Sample> whole;
	whole.reserve(3);
	const std::vector<reference::Timed> stream = reference::Stream();
	for (size_t i = 0; i < 3; ++i)
		whole.push_back(reference::CompactPose(stream[i].id, stream[i].index, stream[i].count));
	return whole;
}

// A block's payload alone, as bytes from anywhere: refused, or read as samples
// that the writer stores again and a reader gets back bit for bit. This is
// the part of a compact file that is bits, where a mutated whole file seldom
// gets past its first sample.
inline std::string CheckRecordingBlock(const uint8_t *data, size_t size)
{
	const compact::Parameters parameters = BlockParameters();
	const std::vector<Sample> whole = BlockWholeSamples();
	compact::Model model;
	model.parameters = parameters;
	for (const Sample &sample : whole)
		model.Whole(sample);
	std::vector<Sample> samples;
	if (!compact::ReadBlock(model, reinterpret_cast<const char *>(data), size, samples))
		return "";
	if (samples.empty() || samples.size() > 0xFFFF)
		return "a block is read as a number of samples no block holds";

	Header header = reference::FileHeader();
	header.version = FormatVersionCompact;
	header.mode = ModeCompact;
	header.velocityScale = parameters.velocityScale;
	header.angularScale = parameters.angularScale;
	std::string file;
	AppendHeader(file, header);
	CompactWriter writer(parameters);
	for (const Sample &sample : whole)
		writer.Append(file, sample);
	for (const Sample &sample : samples)
		writer.Append(file, sample);
	writer.Flush(file);
	const SamplesRead back = ReadSamples(file);
	if (back.end != Reader::Next::EndOfFile || back.samples.size() != whole.size() + samples.size())
		return "samples read from a block do not read back whole when they are written again";
	for (size_t i = 0; i < samples.size(); ++i)
	{
		std::string first, second;
		AppendSample(first, samples[i]);
		AppendSample(second, back.samples[whole.size() + i]);
		if (first != second)
			return "samples read from a block change when they are written again";
	}
	return "";
}

// Blocks a reader takes behind those three samples: the reference recording's
// first block, and a block of one sample.
inline std::vector<std::string> BlockSeeds()
{
	std::vector<std::string> seeds;
	std::istringstream in(reference::CompactRecording(true), std::ios::in | std::ios::binary);
	Reader reader(in);
	Header header;
	Record record;
	if (reader.ReadHeader(header))
		while (seeds.empty() && reader.Read(record) == Reader::Next::Record)
			if (record.kind == KindSamples)
				seeds.push_back(record.block);
	compact::Model model;
	model.parameters = BlockParameters();
	const std::vector<Sample> whole = BlockWholeSamples();
	for (const Sample &sample : whole)
		model.Whole(sample);
	compact::BitWriter bits;
	bits.Bits(1, compact::BitLength(model.devices.size() - 1));
	compact::Writing coder{ bits };
	Sample next = reference::CompactPose(whole[1].deviceId, 1, whole[1].qpc + 25000);
	compact::CodeSample(coder, model, model.devices[1], next);
	std::string one;
	PutU16(one, 1);
	bits.TakeInto(one);
	seeds.push_back(one);
	return seeds;
}

// A seed with a few edits: flipped bits, bytes put in or taken out, a piece of
// another seed, an edge value on a field's place.
inline std::string Mutate(std::string bytes, const std::vector<std::string> &seeds, std::mt19937 &rng)
{
	static const uint64_t edges[] = { 0, 1, 2, 63, 64, 65, 185, 186, 187, 255, 256, 0xFFFF, 0x7FFFFFFF, 0xFFFFFFFFull,
		0x7FF8000000000000ull, 0xFFF0000000000000ull, 0x8000000000000000ull, 0xFFFFFFFFFFFFFFFFull };
	const int edits = 1 + static_cast<int>(rng() % 6);
	for (int edit = 0; edit < edits; ++edit)
	{
		const size_t at = bytes.empty() ? 0 : rng() % bytes.size();
		switch (rng() % 7)
		{
		case 0:
			if (!bytes.empty())
				bytes[at] = static_cast<char>(bytes[at] ^ static_cast<char>(1u << (rng() % 8)));
			break;
		case 1:
			if (!bytes.empty())
				bytes[at] = static_cast<char>(rng() % 256);
			break;
		case 2:
			bytes.insert(at, 1, static_cast<char>(rng() % 256));
			break;
		case 3:
			if (!bytes.empty())
				bytes.erase(at, 1 + rng() % 16);
			break;
		case 4:
		{
			const std::string &other = seeds[rng() % seeds.size()];
			const size_t from = other.empty() ? 0 : rng() % other.size();
			bytes = bytes.substr(0, at) + other.substr(from);
			break;
		}
		case 5:
			// The front of the file, where the header's fields are.
			if (!bytes.empty())
				bytes[rng() % (bytes.size() < 64 ? bytes.size() : 64)] = static_cast<char>(rng() % 256);
			break;
		default:
		{
			const uint64_t value = edges[rng() % (sizeof edges / sizeof edges[0])];
			const size_t width = static_cast<size_t>(1) << (rng() % 4);
			if (at + width <= bytes.size())
				std::memcpy(&bytes[at], &value, width);
			break;
		}
		}
	}
	return bytes;
}

} // namespace questcal::recording
