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

} // namespace detail

// The listing of the recording `in` holds. A file that is not a recording, or
// stops being one, still gets a listing: its last line says where and how the
// file ended.
inline std::string Listing(std::istream &in)
{
	using namespace detail;
	std::string out = "novapose listing 1\n";
	Reader reader(in);
	Header header;
	if (!reader.ReadHeader(header))
		return out + "file: not a recording\n";
	out += "format: version " + Unsigned(header.version) + ", mode " + Unsigned(header.mode) + "\n";
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
	unsigned long long samples = 0;
	Record record;
	Reader::Next next = reader.Read(record);
	for (; next == Reader::Next::Record; next = reader.Read(record))
	{
		const std::string after = " after sample " + Unsigned(samples) + ": ";
		switch (record.kind)
		{
		case KindSample:
		{
			PerDevice &device = devices[record.sample.deviceId];
			if (device.samples++ == 0)
				device.first = record.sample;
			device.valid += record.sample.poseValid ? 1 : 0;
			device.last = record.sample;
			++samples;
			break;
		}
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
		default:
			out += "unknown record" + after + "kind " + Unsigned(record.kind) + ", " +
				Unsigned(record.unknown.size()) + " bytes\n";
			break;
		}
	}
	out += "samples: " + Unsigned(samples) + "\n";
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
