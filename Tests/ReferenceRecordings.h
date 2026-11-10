#pragma once

// The reference recordings: small files made from fixed input, kept in
// Tests/recordings beside the text a correct reader reports for each
// (PoseRecordingListing.h). Another reader, in another language or another
// repository, is right when it reports the same. The harness checks that this
// generator still makes those exact bytes, so the format cannot change without
// the files, the document and the version changing with it.
//
// Every value is made with integer arithmetic and binary fractions, so the
// bytes do not depend on a compiler's math library or its optimizer.

#include "../Overlay/PoseRecordingFormat.h"

#include <algorithm>
#include <string>
#include <vector>

namespace questcal::recording::reference
{

struct File
{
	std::string name;    // Tests/recordings/<name>.novapose and <name>.txt
	std::string bytes;
};

constexpr int64_t ClockRate = 10000000;
constexpr int64_t StartCount = 5000000000;
constexpr int64_t StartTime = 1794230000000000;   // microseconds since 1970 UTC

inline DeviceInfo Device(uint32_t id, uint32_t deviceClass, int32_t role, const char *label, const char *model,
	const char *system, const char *type)
{
	DeviceInfo device;
	device.deviceId = id;
	device.deviceClass = deviceClass;
	device.controllerRole = role;
	device.label = label;
	device.model = model;
	device.trackingSystem = system;
	device.controllerType = type;
	return device;
}

inline std::vector<DeviceInfo> Devices()
{
	return {
		Device(0, 1, 0, "a1b2c3d4", "Headset model", "oculus", ""),
		Device(3, 3, 0, "0f0e0d0c", "Tracker model", "lighthouse", "vive_tracker_waist"),
		Device(5, 3, 0, "9a8b7c6d", "Tracker model", "lighthouse", "vive_tracker_left_foot"),
	};
}

inline Header FileHeader()
{
	Header header;
	header.qpcFrequency = ClockRate;
	header.startQpc = StartCount;
	header.startUnixMicroseconds = StartTime;
	header.writer = "novapose reference generator";
	header.devices = Devices();
	return header;
}

// The `index`-th pose of device `id`, captured at `count`.
inline Sample Pose(uint32_t id, int64_t index, int64_t count)
{
	static const double turns[4][4] = {
		{ 1.0, 0.0, 0.0, 0.0 }, { 0.5, 0.5, 0.5, 0.5 }, { 0.0, 1.0, 0.0, 0.0 }, { 0.5, -0.5, 0.5, -0.5 } };
	const bool headset = id == 0;
	Sample sample;
	sample.qpc = count;
	sample.deviceId = id;
	// Every 50th pose of device 5 is one the driver calls out of range.
	const bool lost = id == 5 && index % 50 == 49;
	sample.trackingResult = lost ? 101 : 200;
	sample.poseValid = !lost;
	sample.connected = true;
	sample.poseTimeOffset = headset ? 0.0 : -0.00390625;
	if (!headset)
	{
		const double fromDriver[4] = { 0.5, 0.5, -0.5, 0.5 };
		const double offset[3] = { 0.125, -1.5, 2.25 };
		std::copy(fromDriver, fromDriver + 4, sample.worldFromDriverRotation);
		std::copy(offset, offset + 3, sample.worldFromDriverTranslation);
	}
	std::copy(turns[index % 4], turns[index % 4] + 4, sample.rotation);
	sample.position[0] = static_cast<double>(id) * 0.25 + static_cast<double>(index) / 1024.0;
	sample.position[1] = 1.0 + static_cast<double>(index % 16) / 4096.0;
	sample.position[2] = -0.5 - static_cast<double>(id) / 8.0;
	sample.velocity[0] = 0.25;
	sample.velocity[2] = -static_cast<double>(index % 8) / 64.0;
	sample.angularVelocity[1] = static_cast<double>(index % 4) / 2.0;
	return sample;
}

// A quarter of a second of a headset at 72 Hz and two trackers at 400 Hz, in
// the order of their capture counts: 218 poses.
struct Timed
{
	int64_t count;
	uint32_t id;
	int64_t index;
};

inline std::vector<Timed> Stream(int64_t firstCount = StartCount + 1000)
{
	std::vector<Timed> stream;
	stream.reserve(218);
	for (int64_t i = 0; i < 18; ++i)
		stream.push_back({ firstCount + i * (ClockRate / 72), 0, i });
	for (uint32_t id : { 3u, 5u })
		for (int64_t i = 0; i < 100; ++i)
			stream.push_back({ firstCount + 400 + i * (ClockRate / 400), id, i });
	std::sort(stream.begin(), stream.end(), [](const Timed &a, const Timed &b)
	{
		return a.count != b.count ? a.count < b.count : a.id < b.id;
	});
	return stream;
}

inline int64_t TimeAt(int64_t count)
{
	return StartTime + (count - StartCount) / (ClockRate / 1000000);
}

inline std::vector<File> Files()
{
	const std::vector<Timed> stream = Stream();
	const uint8_t closed = static_cast<uint8_t>(EndReason::AppExit);
	const uint8_t player = static_cast<uint8_t>(PauseCause::Player);
	const auto end = [&](std::string &bytes, uint64_t total)
	{
		const int64_t count = stream.back().count + 1000;
		AppendEnd(bytes, { closed, count, TimeAt(count), total });
	};
	std::vector<File> files;

	// Nothing but a header, the poses and an end.
	std::string clean;
	AppendHeader(clean, FileHeader());
	for (const Timed &pose : stream)
		AppendSample(clean, Pose(pose.id, pose.index, pose.count));
	std::string cut = clean;
	end(clean, stream.size());
	files.push_back({ "clean", clean });

	// 40 poses lost in the stream, then a session boundary (which counts as
	// one), then a device the header does not list: first with an entry that
	// only gives its id, later with what is known of it.
	std::string holes;
	AppendHeader(holes, FileHeader());
	uint64_t samples = 0;
	for (size_t i = 0; i < stream.size(); ++i)
	{
		const Timed &pose = stream[i];
		if (i == 80)
			AppendGap(holes, { pose.count - 100, 40, 40 });
		if (i == 150)
		{
			AppendBoundary(holes, { pose.count - 100, 1 });
			AppendGap(holes, { pose.count - 100, 1, 1 });
		}
		if (i == 180)
		{
			DeviceInfo unknown;
			unknown.deviceId = 7;
			AppendDevice(holes, unknown);
		}
		if (i == 200)
			AppendDevice(holes, Device(7, 2, 2, "55aa55aa", "Controller model", "lighthouse", "knuckles"));
		AppendSample(holes, Pose(pose.id, pose.index, pose.count));
		++samples;
		if (i >= 180 && i % 4 == 0)
		{
			AppendSample(holes, Pose(7, static_cast<int64_t>(i - 180) / 4, pose.count + 50));
			++samples;
		}
	}
	end(holes, samples);
	files.push_back({ "gap-and-boundary", holes });

	// The player turns recording off after 100 poses and on again thirty
	// seconds later: a pause record, a resume record, every device again, and
	// the stream as it then is.
	std::string paused;
	AppendHeader(paused, FileHeader());
	for (size_t i = 0; i < 100; ++i)
		AppendSample(paused, Pose(stream[i].id, stream[i].index, stream[i].count));
	const int64_t pauseCount = stream[99].count + 300;
	const int64_t resumeCount = pauseCount + 30 * ClockRate;
	AppendPause(paused, { player, pauseCount, TimeAt(pauseCount) });
	AppendResume(paused, { player, resumeCount, TimeAt(resumeCount), std::string() });
	for (const DeviceInfo &device : Devices())
		AppendDevice(paused, device);
	const std::vector<Timed> later = Stream(resumeCount + 1000);
	for (size_t i = 0; i < 60; ++i)
		AppendSample(paused, Pose(later[i].id, later[i].index + 1000, later[i].count));
	AppendEnd(paused, { closed, later[59].count + 1000, TimeAt(later[59].count + 1000), 160 });
	files.push_back({ "pause-and-resume", paused });

	// The recorder was ended without warning: the file stops 77 bytes into its
	// last pose and has no end record.
	cut.resize(cut.size() - (RecordPrefixBytes + SamplePayloadBytes) + 77);
	files.push_back({ "cut-short", cut });

	// What a later writer may add: a record kind this version does not know,
	// and a pause and resume whose cause is a clip with a trigger's name.
	std::string newer;
	AppendHeader(newer, FileHeader());
	for (size_t i = 0; i < 40; ++i)
	{
		if (i == 10)
		{
			PutRecordPrefix(newer, 77, 5);
			newer += "hello";
		}
		if (i == 20)
		{
			const uint8_t clip = static_cast<uint8_t>(PauseCause::Clip);
			AppendPause(newer, { clip, stream[i].count - 200, TimeAt(stream[i].count - 200) });
			AppendResume(newer, { clip, stream[i].count - 100, TimeAt(stream[i].count - 100), "example trigger" });
		}
		AppendSample(newer, Pose(stream[i].id, stream[i].index, stream[i].count));
	}
	AppendEnd(newer, { closed, stream[39].count + 1000, TimeAt(stream[39].count + 1000), 40 });
	files.push_back({ "newer-records", newer });

	// The calibration in use for the trackers' system, stated at the start and
	// again when it changes: a third of a turn and a shift, then none applied.
	// With it, the render model of one tracker.
	std::string aligned;
	AppendHeader(aligned, FileHeader());
	Alignment alignment;
	alignment.applied = true;
	alignment.qpc = StartCount;
	alignment.rotation[0] = 0.5;
	alignment.rotation[1] = 0.5;
	alignment.rotation[2] = -0.5;
	alignment.rotation[3] = 0.5;
	alignment.translation[0] = 1.5;
	alignment.translation[1] = -0.25;
	alignment.translation[2] = 2.0;
	alignment.trackingSystem = "lighthouse";
	AppendAlignment(aligned, alignment);
	AppendModel(aligned, { 3, "{htc}vr_tracker_vive_3_0" });
	for (size_t i = 0; i < 30; ++i)
	{
		if (i == 20)
		{
			alignment.applied = false;
			alignment.qpc = stream[i].count - 100;
			alignment.scale = 1.25;
			AppendAlignment(aligned, alignment);
		}
		AppendSample(aligned, Pose(stream[i].id, stream[i].index, stream[i].count));
	}
	AppendEnd(aligned, { closed, stream[29].count + 1000, TimeAt(stream[29].count + 1000), 30 });
	files.push_back({ "alignment", aligned });
	return files;
}

} // namespace questcal::recording::reference
