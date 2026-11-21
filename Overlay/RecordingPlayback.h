#pragma once

// A pose recording (PoseRecordingFormat.h) read back for playback: where each
// device was at a moment of the recording, with the calibration the recording
// names applied, and what a timeline shows of it.
//
// The file can hold hours, so it is never loaded. Open reads it once on a
// thread of its own and keeps where its records start; Frame then reads the
// short span around the moment asked for. Like the format header, this needs
// only the C++17 standard library.
//
// Time here is recorded time, in seconds: the stretches in which the recorder
// was on, laid end to end. The time between two stretches is not on it.
//
// A recording read back is untrusted input: what does not fit the format ends
// the reading, and nothing here trusts a count or a time a record states.

#include "PoseRecordingFormat.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <istream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace questcal::recording
{

class Playback
{
public:
	// Opens the recording's bytes anew, each time for one reader. Null when
	// they cannot be opened.
	using Source = std::function<std::unique_ptr<std::istream>()>;

	// How long a device's path stays behind it.
	static constexpr double TrailSeconds = 1.2;

	enum class State
	{
		Empty,     // nothing is open
		Reading,   // the first pass over the file is in progress
		Ready,
		Failed,    // not a recording, or it cannot be opened; `error` says which
	};

	// One run of the recorder, from a start or a resume to a pause or the end.
	struct Stretch
	{
		double start = 0.0;                  // recorded time at its first sample
		double seconds = 0.0;
		int64_t startUnixMicroseconds = 0;   // wall clock, from the header or the resume record
		std::string trigger;                 // what started a clip; empty otherwise
	};

	// Samples the recorder could not fetch in time.
	struct Loss
	{
		double time = 0.0;
		uint64_t samples = 0;
	};

	// A moment the recorder marked: a tracking problem Nova Calibrator saw, or
	// the player's own "save".
	struct Mark
	{
		double time = 0.0;
		int64_t unixMicroseconds = 0;
		std::string name;      // recording::markname, or what a newer writer calls it
		bool hasDevice = false;
		DeviceInfo device;     // the device it is about, as the file described it then
	};

	struct Summary
	{
		State state = State::Empty;
		double progress = 0.0;   // of the first pass, 0 to 1
		std::string error;       // for the log
		double seconds = 0.0;
		int64_t startUnixMicroseconds = 0;
		uint64_t bytes = 0;
		uint64_t samples = 0;
		uint64_t lost = 0;
		size_t devices = 0;      // ids that have a sample
		// The file does not end with an end or a pause record: the recorder was
		// cut short, or the file stops being a recording part way.
		bool cutShort = false;
		std::vector<Stretch> stretches;
		std::vector<Loss> losses;
		std::vector<Mark> marks;
	};

	struct DevicePose
	{
		DeviceInfo info;
		// The last sample before the moment was valid and connected.
		bool tracked = false;
		// Tracked, but its driver did not report normal tracking: the position
		// is what the driver could still work out.
		bool limited = false;
		// Meters in the recorded standing space: the driver's pose
		// put in its world, then the recording's alignment for its system.
		double position[3] = { 0.0, 0.0, 0.0 };
		// Its orientation in that space, w x y z.
		double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };
		// SteamVR's name of the model that shows it; empty when the recording
		// names none.
		std::string renderModel;
		// Where it was over the last TrailSeconds, oldest first.
		std::vector<std::array<float, 3>> trail;
	};

	Playback();
	~Playback();
	Playback(const Playback &) = delete;
	Playback &operator=(const Playback &) = delete;

	// Replaces what was open and starts the first pass on its own thread, or,
	// with `wait`, does it before returning. `bytes` is the file's size, for
	// the progress.
	void Open(const Source &source, uint64_t bytes, bool wait = false);
	void Close();

	// For any thread.
	Summary GetSummary() const;

	// Every device that has a sample at or before `time`, which is clamped to
	// the recording. False until the state is Ready. For one thread; a call
	// reads at most a bounded part of the file, so after a long jump the
	// answer can be a moment behind for a frame or two.
	bool Frame(double time, std::vector<DevicePose> &devices);

private:
	struct Index;
	struct Cursor;
	static void Read(const std::shared_ptr<Index> &index, const Source &source, uint64_t bytes);
	void Seek(const Index &index, double time);

	mutable std::mutex mutex_;
	std::shared_ptr<Index> index_;
	std::thread thread_;
	Source source_;
	std::unique_ptr<Cursor> cursor_;
};

} // namespace questcal::recording
