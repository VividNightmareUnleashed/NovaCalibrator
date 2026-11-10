#pragma once

// What makes the Motion Recorder mark a moment (and, in "Automatic", start a
// clip): the tracking problems Nova Calibrator already counts, and a device
// that had tracking and lost it. The watch only compares what it is shown
// from one step to the next, so it needs nothing but the standard library and
// the format's names.

#include "PoseRecordingFormat.h"

#include <cstdint>
#include <vector>

namespace questcal::recording
{

// Nova Calibrator's own counts of what its monitors saw, as they stand now.
struct TriggerCounts
{
	uint32_t universeJumps = 0;       // the headset's tracking space jumped
	uint32_t baseStationMoves = 0;    // SteamVR moved the lighthouse frame
	uint32_t deviceSlides = 0;        // a device that was still slid
	uint32_t trackingJumps = 0;       // a device came out of a loss somewhere else
	uint32_t realignments = 0;        // continuous calibration aligned again
	bool calibrationStopped = false;  // continuous calibration holds off for a large deviation
};

struct TriggerDevice
{
	uint32_t id = NoDevice;
	bool tracked = false;
};

struct Trigger
{
	const char *name = "";
	uint32_t deviceId = NoDevice;
};

class TriggerWatch
{
public:
	// A device must have tracked this long before its loss is news, so one
	// that goes in and out of view says so once, not at every flicker.
	static constexpr double SettledSeconds = 2.0;
	static constexpr size_t MaxDevices = 128;

	// The next step is a first look: it reports nothing.
	void Reset()
	{
		primed_ = false;
		seen_.clear();
	}

	// Adds to `triggers` what happened since the last step. `devices` lists
	// each device that can track; one that is no longer listed has no tracking.
	void Step(double time, const TriggerCounts &counts, const std::vector<TriggerDevice> &devices,
		std::vector<Trigger> &triggers)
	{
		if (primed_)
		{
			// One mark however far a count went up in one step.
			if (counts.universeJumps > last_.universeJumps)
				triggers.push_back({ markname::UniverseJump, NoDevice });
			if (counts.baseStationMoves > last_.baseStationMoves)
				triggers.push_back({ markname::BaseStationMoved, NoDevice });
			if (counts.deviceSlides > last_.deviceSlides)
				triggers.push_back({ markname::DeviceSlid, NoDevice });
			if (counts.trackingJumps > last_.trackingJumps)
				triggers.push_back({ markname::TrackingJumped, NoDevice });
			if (counts.realignments > last_.realignments)
				triggers.push_back({ markname::CalibrationRealigned, NoDevice });
			if (counts.calibrationStopped && !last_.calibrationStopped)
				triggers.push_back({ markname::CalibrationStopped, NoDevice });
		}
		last_ = counts;

		for (Seen &seen : seen_)
			seen.listed = false;
		for (const TriggerDevice &device : devices)
		{
			Seen *seen = nullptr;
			for (Seen &candidate : seen_)
				if (candidate.id == device.id)
					seen = &candidate;
			if (!seen)
			{
				if (seen_.size() >= MaxDevices)
					continue;
				seen_.push_back({ device.id, device.tracked, false, time, true });
				continue;
			}
			seen->listed = true;
			Note(*seen, device.tracked, time, triggers);
		}
		for (Seen &seen : seen_)
			if (!seen.listed)
				Note(seen, false, time, triggers);
		primed_ = true;
	}

private:
	struct Seen
	{
		uint32_t id = NoDevice;
		bool tracked = false;
		bool settled = false;   // tracked for SettledSeconds without a break
		double since = 0.0;     // when `tracked` last changed
		bool listed = false;
	};

	void Note(Seen &seen, bool tracked, double time, std::vector<Trigger> &triggers) const
	{
		if (tracked == seen.tracked)
		{
			if (tracked && time - seen.since >= SettledSeconds)
				seen.settled = true;
			return;
		}
		if (!tracked && seen.settled && primed_)
			triggers.push_back({ markname::TrackingLost, seen.id });
		seen.tracked = tracked;
		seen.settled = false;
		seen.since = time;
	}

	bool primed_ = false;
	TriggerCounts last_;
	std::vector<Seen> seen_;
};

} // namespace questcal::recording
