#pragma once

#include "PoseRecordingFormat.h"
#include "PoseStreamHub.h"

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace questcal
{

// Free bytes on the drive that holds `directory`, as the account may use them.
// False when Windows cannot say.
bool FreeDiskBytes(const std::wstring &directory, uint64_t &freeBytes);

// What bounds a recording. Eight lighthouse devices and a headset write about
// 2.4 GB an hour (3,450 samples a second, 189 bytes each).
struct PoseRecordingLimits
{
	static constexpr uint64_t DefaultMinFreeDiskBytes = 4ull << 30;

	// The file never grows past this. What happens at the cap is decided in
	// one place, `full` in PoseRecorder::Run: today the recording ends at the
	// last record that fits.
	uint64_t maxFileBytes = 5ull << 30;
	// A recording does not start or go on, and one in progress ends, while the
	// drive has less than this free.
	uint64_t minFreeDiskBytes = DefaultMinFreeDiskBytes;
	// How the free space is read, and how often while recording.
	bool (*freeDiskBytes)(const std::wstring &directory, uint64_t &freeBytes) = &FreeDiskBytes;
	unsigned diskCheckIntervalMs = 1000;
};

// What the program knows of a device. The serial number only tells devices
// apart inside the recorder: the file gets a label made up for this recording
// in its place, the same one each time the device is described.
struct RecordedDevice
{
	uint32_t deviceId = 0xFFFFFFFF;
	uint32_t deviceClass = 0;
	int32_t controllerRole = 0;
	std::string serial;
	std::string model;
	std::string trackingSystem;
	std::string controllerType;
	// SteamVR's name of the model that shows the device; empty when it gives none.
	std::string renderModel;
};

// Writes every sample the pose stream hub publishes to the one recording file
// (PoseRecordingFormat.h) while it is recording. The hub is read as any
// consumer reads it, and the file is written from the recorder's own thread,
// so a slow disk delays neither the hub nor the caller: samples the thread
// cannot fetch before the hub's history overwrites them are recorded as a gap.
//
// There is one recording, `directory`\NovaCalibrator-recording.novapose. A
// pause closes the file with a pause record; the next start goes on in the
// same file behind a resume record, if the file is still as the recorder left
// it. Anything else begins a new recording, which replaces the old file when
// its first sample is written and not before. Nothing published while the
// recorder is paused or stopped ever reaches a file.
//
// Every member is for one thread (the UI's).
class PoseRecorder
{
public:
	enum class StartResult
	{
		Started,        // a new recording
		Resumed,        // the paused recording goes on
		Busy,           // the last stretch has not closed the file yet
		NoFolder,       // the folder cannot be made, or its free space read
		DiskSpaceLow,   // under PoseRecordingLimits::minFreeDiskBytes
		CannotCreate,   // the file cannot be created or opened, or its start written
	};

	// The whole recording so far, its paused stretches left out.
	struct Progress
	{
		bool recording = false;   // the thread has the file open
		uint64_t bytes = 0;
		uint64_t samples = 0;
		uint64_t lost = 0;        // the sum of every gap record
		double seconds = 0.0;
	};

	// How a stretch of recording ended, once its file is closed.
	struct Finished
	{
		std::wstring path;
		// True: the player's pause, and the next start can go on in this file.
		// False: `reason` ended the recording.
		bool paused = false;
		recording::EndReason reason = recording::EndReason::AppExit;
		// False when no sample arrived in a new recording: no file was kept and
		// the previous recording, if there was one, is untouched.
		bool saved = false;
		uint64_t bytes = 0;
		uint64_t samples = 0;
		uint64_t lost = 0;
		double seconds = 0.0;
		// The file ends with its pause or end record. False after a failed write.
		bool closedCleanly = false;
		std::string error;   // for the log; empty when nothing failed
	};

	PoseRecorder();
	~PoseRecorder();
	PoseRecorder(const PoseRecorder &) = delete;
	PoseRecorder &operator=(const PoseRecorder &) = delete;

	// The one recording's file in `directory`.
	static std::wstring FilePath(const std::wstring &directory);

	// Starts recording from the hub's next sample: into the paused recording
	// when CanResume, into a new one otherwise. `writer` names the program in a
	// new file's header. `detail`, when given, receives the reason a start was
	// refused, for the log.
	StartResult Start(PoseStreamHub &hub, const std::wstring &directory, const std::string &writer,
		const std::vector<RecordedDevice> &devices, const PoseRecordingLimits &limits = PoseRecordingLimits(),
		std::string *detail = nullptr);

	// Whether a start in `directory` would go on in the paused recording: there
	// is one, and its file is the size this recorder left it.
	bool CanResume(const std::wstring &directory) const;
	// The paused recording is no longer this recorder's to go on with (its file
	// was deleted, say): the next start is a new recording.
	void Forget();

	// What the program knows about the devices now. A device that is new or
	// changed since the last call (or the start) gets a device record.
	void SetDevices(const std::vector<RecordedDevice> &devices);
	// The calibration in use for one tracking system's devices. A value that
	// is new, and the known one at each start, gets an alignment record; the
	// samples are never changed by it. Its count is set when it is written.
	void SetAlignment(const recording::Alignment &alignment);

	// Stops recording at once: no sample published after this returns reaches
	// the file. The thread then writes a pause record and closes the file, and
	// TakeFinished reports it.
	void RequestPause();
	// The same, for good: the thread writes an end record with `reason`.
	void RequestStop(recording::EndReason reason);
	// RequestStop, then waits until the file is closed. False when it was not
	// closed in time: the thread is left to finish by itself and no longer
	// touches the hub.
	bool StopAndWait(recording::EndReason reason, unsigned timeoutMs);

	Progress GetProgress() const;

	// True once for each stretch, after its file is closed.
	bool TakeFinished(Finished &finished);

private:
	struct Session;
	// The thread's entry. std::thread keeps its own copy of the pointer for as
	// long as the thread runs.
	static void Run(const std::shared_ptr<Session> &shared);
	void Request(bool pause, recording::EndReason reason);
	// The closed stretch's result, folded into what the next start goes on from.
	void Harvest(Finished &finished);
	recording::DeviceInfo Labelled(const RecordedDevice &device);

	std::shared_ptr<Session> session_;
	std::thread thread_;
	std::vector<recording::DeviceInfo> devices_;
	// Serial (or id, for a device without one) to this recording's label.
	std::map<std::string, std::string> labels_;
	// Each device's render model as the file last stated it, by id.
	std::map<uint32_t, std::string> models_;
	recording::Alignment alignment_;
	bool hasAlignment_ = false;
	PoseStreamHub *hub_ = nullptr;
	int consumer_ = -1;

	// The recording a pause left, for the next start to go on with.
	struct Paused
	{
		bool valid = false;
		std::wstring path;
		uint64_t bytes = 0;
		uint64_t samples = 0;
		uint64_t lost = 0;
		double seconds = 0.0;
	} paused_;
};

} // namespace questcal
