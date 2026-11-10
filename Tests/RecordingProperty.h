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
	if (header.version != FormatVersion || header.mode != ModeRaw || header.qpcFrequency <= 0 ||
		header.devices.size() > MaxHeaderDevices)
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
	std::vector<Playback::DevicePose> poses;
	for (double moment : { 0.0, summary.seconds * 0.5, summary.seconds, summary.seconds * 0.25 })
	{
		if (!playback.Frame(moment, poses))
			return "the playback has no frame for a recording it read";
		for (const Playback::DevicePose &pose : poses)
			if (!std::isfinite(pose.position[0]) || !std::isfinite(pose.position[1]) || !std::isfinite(pose.position[2]))
				return "the playback puts a device at a position that is not a number";
	}
	return "";
}

// Inputs the reader accepts, for the mutations and the fuzzer to start from:
// the reference recordings, and a header alone.
inline std::vector<std::string> PropertySeeds()
{
	std::vector<std::string> seeds;
	for (const reference::File &file : reference::Files())
		seeds.push_back(file.bytes);
	std::string header;
	AppendHeader(header, reference::FileHeader());
	seeds.push_back(header);
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
