#pragma once

// A recording as plain text: what a correct reader finds in it, in a form two
// readers written apart can be compared by. Tests/recordings/*.txt are this
// text for Tests/recordings/*.novapose, and tools/read-recording.py --listing
// prints the same lines; docs/recording-format.md describes them.
//
// Like PoseRecordingFormat.h this needs only the standard library.

#include "PoseRecordingFormat.h"

#include <cstdio>
#include <map>
#include <string>

namespace questcal::recording
{

namespace detail
{

inline std::string Number(long long value)
{
	return std::to_string(value);
}

inline std::string Unsigned(unsigned long long value)
{
	return std::to_string(value);
}

// A double as printf's %.17g prints it, which reads back as the same double.
inline std::string Real(double value)
{
	char text[40];
	snprintf(text, sizeof text, "%.17g", value);
	return text;
}

inline std::string DeviceLine(const DeviceInfo &device)
{
	return "device " + Unsigned(device.deviceId) + ": class " + Unsigned(device.deviceClass) + ", role " +
		Number(device.controllerRole) + ", label \"" + device.label + "\", model \"" + device.model +
		"\", system \"" + device.trackingSystem + "\", type \"" + device.controllerType + "\"";
}

inline std::string PoseText(const Sample &sample)
{
	return "count " + Number(sample.qpc) + " position " + Real(sample.position[0]) + " " + Real(sample.position[1]) +
		" " + Real(sample.position[2]) + " rotation " + Real(sample.rotation[0]) + " " + Real(sample.rotation[1]) +
		" " + Real(sample.rotation[2]) + " " + Real(sample.rotation[3]);
}

// CRC-32 (the one of zlib and PNG) over every sample, each as the 186 bytes a
// version 1 file holds it in: one number that two readers agree on only when
// they agree on every bit of every sample.
struct SampleDigest
{
	uint32_t crc = 0xFFFFFFFFu;
	std::string bytes;

	void Add(const Sample &sample)
	{
		bytes.clear();
		AppendSample(bytes, sample);
		for (size_t i = RecordPrefixBytes; i < bytes.size(); ++i)
		{
			crc ^= static_cast<unsigned char>(bytes[i]);
			for (int bit = 0; bit < 8; ++bit)
				crc = (crc >> 1) ^ ((crc & 1u) != 0 ? 0xEDB88320u : 0u);
		}
	}

	std::string Text() const
	{
		char text[16];
		snprintf(text, sizeof text, "%08x", static_cast<unsigned>(crc ^ 0xFFFFFFFFu));
		return text;
	}
};

} // namespace detail

// The listing of the recording `in` holds. A file that is not a recording, or
// stops being one, still gets a listing: its last line says where and how the
// file ended.
inline std::string Listing(std::istream &in)
{
	using namespace detail;
	std::string out = "novapose listing 2\n";
	Reader reader(in);
	Header header;
	if (!reader.ReadHeader(header))
		return out + "file: not a recording\n";
	const bool compact = header.version == FormatVersionCompact;
	out += "format: version " + Unsigned(header.version) + ", mode " + Unsigned(header.mode) + "\n";
	if (compact)
		out += "velocity steps: " + Unsigned(header.velocityScale) + " in a meter a second, " +
			Unsigned(header.angularScale) + " in a radian a second\n";
	out += "writer: " + header.writer + "\n";
	out += "clock: " + Number(header.qpcFrequency) + " ticks per second, start count " + Number(header.startQpc) +
		", start time " + Number(header.startUnixMicroseconds) + " us\n";
	out += "header devices: " + Unsigned(header.devices.size()) + "\n";
	for (const DeviceInfo &device : header.devices)
		out += DeviceLine(device) + "\n";

	struct PerDevice
	{
		unsigned long long samples = 0, valid = 0;
		Sample first, last;
	};
	std::map<uint32_t, PerDevice> devices;
	unsigned long long samples = 0, inBlocks = 0;
	SampleDigest digest;
	const auto one = [&](const Sample &sample)
	{
		PerDevice &device = devices[sample.deviceId];
		if (device.samples++ == 0)
			device.first = sample;
		device.valid += sample.poseValid ? 1 : 0;
		device.last = sample;
		digest.Add(sample);
		++samples;
	};
	Record record;
	Reader::Next next = reader.Read(record);
	for (; next == Reader::Next::Record; next = reader.Read(record))
	{
		const std::string after = " after sample " + Unsigned(samples) + ": ";
		const auto unknown = [&]
		{
			out += "unknown record" + after + "kind " + Unsigned(record.kind) + ", " + Unsigned(record.unknown.size()) +
				" bytes\n";
		};
		switch (record.kind)
		{
		case KindSample:
			one(record.sample);
			break;
		case KindSamples:
			// A block of a compact file; in a raw file a kind it does not know.
			if (record.samples.empty())
				unknown();
			for (const Sample &sample : record.samples)
				one(sample);
			inBlocks += record.samples.size();
			break;
		case KindSync:
			if (compact)
				out += "sync after sample " + Unsigned(samples) + "\n";
			else
				unknown();
			break;
		case KindGap:
			out += "gap" + after + "lost " + Unsigned(record.gap.lost) + ", hole " + Unsigned(record.gap.holeSize) +
				", count " + Number(record.gap.qpc) + "\n";
			break;
		case KindBoundary:
			out += "boundary" + after + "boundaries " + Unsigned(record.boundary.streamBoundaries) + ", count " +
				Number(record.boundary.qpc) + "\n";
			break;
		case KindDevice:
			out += "device record" + after + DeviceLine(record.device) + "\n";
			break;
		case KindEnd:
			out += "end" + after + "reason " + Unsigned(record.end.reason) + ", count " + Number(record.end.qpc) +
				", time " + Number(record.end.unixMicroseconds) + " us, samples " + Unsigned(record.end.samples) + "\n";
			break;
		case KindPause:
			out += "pause" + after + "cause " + Unsigned(record.pause.cause) + ", count " + Number(record.pause.qpc) +
				", time " + Number(record.pause.unixMicroseconds) + " us\n";
			break;
		case KindResume:
			out += "resume" + after + "cause " + Unsigned(record.resume.cause) + ", trigger \"" + record.resume.trigger +
				"\", count " + Number(record.resume.qpc) + ", time " + Number(record.resume.unixMicroseconds) + " us\n";
			break;
		case KindAlignment:
			out += "alignment" + after + "system \"" + record.alignment.trackingSystem + "\", applied " +
				Unsigned(record.alignment.applied ? 1 : 0) + ", count " + Number(record.alignment.qpc) + ", rotation " +
				Real(record.alignment.rotation[0]) + " " + Real(record.alignment.rotation[1]) + " " +
				Real(record.alignment.rotation[2]) + " " + Real(record.alignment.rotation[3]) + ", translation " +
				Real(record.alignment.translation[0]) + " " + Real(record.alignment.translation[1]) + " " +
				Real(record.alignment.translation[2]) + ", scale " + Real(record.alignment.scale) + "\n";
			break;
		case KindModel:
			out += "model" + after + "device " + Unsigned(record.model.deviceId) + ", render model \"" +
				record.model.renderModel + "\"\n";
			break;
		case KindMark:
			out += "mark" + after + "name \"" + record.mark.name + "\", device " + Unsigned(record.mark.deviceId) +
				", count " + Number(record.mark.qpc) + ", time " + Number(record.mark.unixMicroseconds) + " us\n";
			break;
		default:
			unknown();
			break;
		}
	}
	out += "samples: " + Unsigned(samples) + "\n";
	if (compact)
		out += "samples in blocks: " + Unsigned(inBlocks) + "\n";
	out += "sample digest: " + digest.Text() + "\n";
	for (const auto &entry : devices)
		out += "samples of device " + Unsigned(entry.first) + ": " + Unsigned(entry.second.samples) + ", valid " +
			Unsigned(entry.second.valid) + ", first " + PoseText(entry.second.first) + ", last " +
			PoseText(entry.second.last) + "\n";
	if (next == Reader::Next::EndOfFile)
		out += "file: complete\n";
	else if (next == Reader::Next::Truncated)
		out += "file: cut short inside a record after " + Unsigned(reader.BytesRead()) + " bytes\n";
	else
		out += "file: not a recording after " + Unsigned(reader.BytesRead()) + " bytes\n";
	return out;
}

} // namespace questcal::recording
