#pragma once

// What makes the Motion Recorder mark a moment (and, in "Automatic", start a
// clip): the tracking problems Nova Calibrator already counts, and a device
// that had tracking and stays without it. The watch only compares what it is shown
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
	// A device must have tracked this long before its loss is news, and must
	// then stay without tracking this long. A lighthouse device behind a body
	// or with a weak radio link loses tracking for a few hundredths of a second
	// again and again (a recorded session had one such break every three
	// seconds), and the pose carries on through them: a mark at each would make
	// a recording of clips a full recording.
	static constexpr double SettledSeconds = 2.0;
	static constexpr double LostSeconds = 0.5;
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
				Seen first;
				first.id = device.id;
				first.tracked = device.tracked;
				first.since = first.stretch = time;
				first.listed = true;
				seen_.push_back(first);
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
		bool settled = false;   // tracked for SettledSeconds, breaks shorter than LostSeconds aside
		bool told = false;      // the loss it is in has had its mark, or was not news
		double since = 0.0;     // when `tracked` last changed
		double stretch = 0.0;   // when the tracking it has now began; a short break does not end it
		bool listed = false;
	};

	void Note(Seen &seen, bool tracked, double time, std::vector<Trigger> &triggers) const
	{
		if (tracked)
		{
			if (!seen.tracked)
			{
				// Back. A loss this long is news even when no step fell inside
				// it (the recorder steps once a frame, and a frame can be late).
				if (time - seen.since >= LostSeconds)
				{
					Tell(seen, triggers);
					seen.settled = false;
					seen.stretch = time;
				}
				seen.tracked = true;
				seen.since = time;
			}
			if (time - seen.stretch >= SettledSeconds)
				seen.settled = true;
			return;
		}
		if (seen.tracked)
		{
			seen.tracked = false;
			seen.told = false;
			seen.since = time;
		}
		if (time - seen.since >= LostSeconds)
		{
			Tell(seen, triggers);
			seen.settled = false;
		}
	}

	// One mark for a loss, and only of tracking that had settled.
	void Tell(Seen &seen, std::vector<Trigger> &triggers) const
	{
		if (seen.settled && !seen.told && primed_)
			triggers.push_back({ markname::TrackingLost, seen.id });
		seen.told = true;
	}

	bool primed_ = false;
	TriggerCounts last_;
	std::vector<Seen> seen_;
};

} // namespace questcal::recording
