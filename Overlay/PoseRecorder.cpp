#include "stdafx.h"
#include "PoseRecorder.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <random>

namespace questcal
{

namespace
{

// The thread looks for new samples this often.
constexpr auto PollInterval = std::chrono::milliseconds(20);
// Bytes encoded before they are handed to Windows: a backlog after a stall is
// written in pieces of this size, not as one buffer of its own size.
constexpr size_t WriteChunkBytes = 256 * 1024;
// Kept free under the size cap for the record that closes a stretch.
constexpr size_t ClosingRecordBytes = recording::RecordPrefixBytes + recording::EndPayloadBytes;
constexpr size_t SampleRecordBytes = recording::RecordPrefixBytes + recording::SamplePayloadBytes;
constexpr size_t DeviceRecordBytes = recording::RecordPrefixBytes + recording::MaxDevicePayloadBytes;
constexpr size_t ModelRecordBytes = recording::RecordPrefixBytes + 4 + 1 + recording::MaxStringBytes;
constexpr size_t AlignmentRecordBytes = recording::RecordPrefixBytes + 73 + 1 + recording::MaxStringBytes;
constexpr size_t GapRecordBytes = recording::RecordPrefixBytes + recording::GapPayloadBytes;
constexpr size_t BoundaryRecordBytes = recording::RecordPrefixBytes + recording::BoundaryPayloadBytes;
constexpr size_t MarkRecordBytes = recording::RecordPrefixBytes + recording::MaxMarkPayloadBytes;
// Marks that wait for the thread. More in one poll interval are not news.
constexpr size_t MaxPendingMarks = 64;
constexpr size_t MaxKnownAlignments = 16;
// A single Drain stops at each gap; this many reach the head of any backlog
// the hub's history holds, as PoseStreamHub::DrainThroughGaps bounds it.
constexpr int DrainPasses = 64;

int64_t QpcNow()
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	return now.QuadPart;
}

int64_t UnixMicrosecondsNow()
{
	FILETIME time{};
	GetSystemTimePreciseAsFileTime(&time);
	const uint64_t ticks = (static_cast<uint64_t>(time.dwHighDateTime) << 32) | time.dwLowDateTime;
	// FILETIME counts 100 ns from 1601; 1970 is 11,644,473,600 s later.
	return static_cast<int64_t>(ticks / 10) - 11644473600000000ll;
}

// A file's size, or false when it cannot be read (it is gone, say).
bool FileSize(const std::wstring &path, uint64_t &bytes)
{
	WIN32_FILE_ATTRIBUTE_DATA data{};
	if (!GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &data) ||
		(data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
		return false;
	bytes = (static_cast<uint64_t>(data.nFileSizeHigh) << 32) | data.nFileSizeLow;
	return true;
}

// Appends go to the end whatever the file pointer says. Readable while it is
// written, so a reader can look at a live file.
HANDLE OpenForAppend(const std::wstring &path, DWORD disposition)
{
	return CreateFileW(path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, disposition,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
}

// Puts the file `from` in the place of `to`. Windows refuses that while any
// program has `to` open, and a virus scanner or an indexer opens a file the
// moment it was written and closed, which is the moment this is called. So a
// refusal of that kind is not a failure yet: the step is tried again, at
// pauses that grow to 50 ms, until `waitMs` have gone by. Returns Windows'
// error of the last try, ERROR_SUCCESS when the file took its place; `retries`
// counts the tries another program's hold refused.
DWORD TakePlaceOf(const std::wstring &from, const std::wstring &to, unsigned waitMs, std::atomic<uint64_t> &retries)
{
	const ULONGLONG started = GetTickCount64();
	for (DWORD pause = 1;; pause = (std::min<DWORD>)(pause * 2, 50))
	{
		if (MoveFileExW(from.c_str(), to.c_str(), MOVEFILE_REPLACE_EXISTING))
			return ERROR_SUCCESS;
		const DWORD error = GetLastError();
		const bool held = error == ERROR_ACCESS_DENIED || error == ERROR_SHARING_VIOLATION ||
			error == ERROR_LOCK_VIOLATION;
		if (!held || GetTickCount64() - started >= waitMs)
			return error != ERROR_SUCCESS ? error : ERROR_GEN_FAILURE;
		retries.fetch_add(1, std::memory_order_release);
		Sleep(pause);
	}
}

recording::Sample ToSample(const protocol::DevicePoseSample &pose)
{
	recording::Sample sample;
	sample.qpc = pose.sampleTimeQpc;
	sample.deviceId = pose.deviceId;
	sample.trackingResult = pose.trackingResult;
	sample.poseValid = pose.poseIsValid;
	sample.connected = pose.deviceIsConnected;
	sample.poseTimeOffset = pose.poseTimeOffset;
	const auto quaternion = [](const vr::HmdQuaternion_t &q, double (&to)[4])
	{
		to[0] = q.w;
		to[1] = q.x;
		to[2] = q.y;
		to[3] = q.z;
	};
	quaternion(pose.worldFromDriverRotation, sample.worldFromDriverRotation);
	quaternion(pose.rotation, sample.rotation);
	for (int i = 0; i < 3; ++i)
	{
		sample.worldFromDriverTranslation[i] = pose.worldFromDriverTranslation[i];
		sample.position[i] = pose.position[i];
		sample.velocity[i] = pose.velocity[i];
		sample.angularVelocity[i] = pose.angularVelocity[i];
	}
	return sample;
}

// A header for a file of this kind, without its start and its devices.
recording::Header HeaderFor(bool compact, const recording::compact::Parameters &parameters)
{
	recording::Header header;
	header.qpcFrequency = parameters.clockRate;
	if (compact)
	{
		header.version = recording::FormatVersionCompact;
		header.mode = recording::ModeCompact;
		header.velocityScale = parameters.velocityScale;
		header.angularScale = parameters.angularScale;
	}
	return header;
}

struct PendingMark
{
	recording::Mark record;
	bool keepAfter = true;
};

// What one pass of the thread brought, kept in memory between clips: its
// records as the file would hold them, and what they say of the devices.
struct KeptPiece
{
	int64_t qpc = 0;   // when it arrived
	std::string bytes;
	std::vector<recording::DeviceInfo> devices;
	std::vector<recording::Model> models;
	std::vector<recording::Alignment> alignments;
	uint64_t samples = 0;
	uint64_t lost = 0;
};

// What a clip must state first for the records behind it to read by
// themselves: every device, its model and the calibration, as they were in
// front of the oldest record kept.
struct KnownState
{
	std::vector<recording::DeviceInfo> devices;
	std::map<uint32_t, std::string> models;
	std::vector<recording::Alignment> alignments;

	void Device(const recording::DeviceInfo &device)
	{
		for (recording::DeviceInfo &known : devices)
			if (known.deviceId == device.deviceId)
			{
				known = device;
				return;
			}
		devices.push_back(device);
	}

	void Shape(const recording::Model &model)
	{
		models[model.deviceId] = model.renderModel;
	}

	void Align(const recording::Alignment &alignment)
	{
		for (recording::Alignment &known : alignments)
			if (known.trackingSystem == alignment.trackingSystem)
			{
				known = alignment;
				return;
			}
		if (alignments.size() < MaxKnownAlignments)
			alignments.push_back(alignment);
	}

	void Fold(const KeptPiece &piece)
	{
		for (const recording::DeviceInfo &device : piece.devices)
			Device(device);
		for (const recording::Model &model : piece.models)
			Shape(model);
		for (const recording::Alignment &alignment : piece.alignments)
			Align(alignment);
	}
};

// A place the file can be cut at: a record's edge, what a file that starts
// there must state first, and what the recording held in front of it.
struct CutPoint
{
	uint64_t offset = 0;
	int64_t qpc = 0;
	int64_t unixMicroseconds = 0;
	KnownState state;
	uint64_t samplesBefore = 0;
	uint64_t lostBefore = 0;
	uint64_t clipsBefore = 0;
	double secondsBefore = 0.0;
};

} // namespace

struct PoseRecorder::CutPoints
{
	std::vector<CutPoint> points;
};

bool FreeDiskBytes(const std::wstring &directory, uint64_t &freeBytes)
{
	ULARGE_INTEGER available{};
	if (!GetDiskFreeSpaceExW(directory.c_str(), &available, nullptr, nullptr))
		return false;
	freeBytes = available.QuadPart;
	return true;
}

// What the recorder and its thread share for one stretch of recording. The
// thread holds its own reference, so a thread left behind by StopAndWait's
// timeout outlives the recorder safely.
struct PoseRecorder::Session
{
	PoseStreamHub *hub = nullptr;
	int consumer = -1;
	HANDLE file = INVALID_HANDLE_VALUE;
	std::wstring directory;
	// The recording's file, and the name a new recording is written under
	// until its first sample: `replaced` turns true when it takes the file's
	// place. A resumed recording is in place from the start.
	std::wstring finalPath;
	std::wstring newPath;
	bool replaced = false;
	PoseRecordingLimits limits;
	// For the header of the file that is left when the oldest part is dropped.
	std::string writer;
	std::shared_ptr<CutPoints> cuts;
	int64_t qpcFrequency = 1;
	int64_t startQpc = 0;
	// The file is a compact one, with these scales (format version 2).
	bool compact = false;
	recording::compact::Parameters parameters;
	// The recording's counts and length before this stretch.
	uint64_t earlierSamples = 0;
	double earlierSeconds = 0.0;
	// The hub's boundary count as of the last boundary record (or the start).
	uint64_t boundariesNoted = 0;
	// Ids that have a device entry in this stretch of the file.
	std::array<bool, vr::k_unMaxTrackedDeviceCount> described{};

	// Guards the fields below, and every hub call the thread makes: a request
	// to stop, made under it, is therefore in front of any later drain.
	std::mutex mutex;
	std::condition_variable wake;
	bool stop = false;
	bool stopIsPause = false;
	recording::EndReason stopReason = recording::EndReason::AppExit;
	int64_t stopQpc = 0;
	int64_t stopUnixMicroseconds = 0;
	std::vector<recording::DeviceInfo> pendingDevices;
	std::vector<recording::Alignment> pendingAlignments;
	std::vector<recording::Model> pendingModels;
	std::vector<PendingMark> pendingMarks;

	// The devices, models and calibration as the start knew them, for the
	// thread's first clip.
	KnownState startState;

	std::atomic<uint64_t> bytes{ 0 };     // the file's size
	std::atomic<uint64_t> samples{ 0 };   // the whole recording's
	std::atomic<uint64_t> lost{ 0 };
	// "Automatic": the clips closed so far in the whole recording, the counts
	// this stretch's closed clips lasted, and when the open one began (0: none).
	std::atomic<uint64_t> clips{ 0 };
	std::atomic<int64_t> clipTicks{ 0 };
	std::atomic<int64_t> clipOpenSince{ 0 };
	std::atomic<uint64_t> kept{ 0 };      // samples that wait in memory for a clip
	// The oldest part dropped at the cap: how often in this stretch, the
	// samples and the seconds that went with it, and whether one is under way.
	std::atomic<uint64_t> trims{ 0 };
	std::atomic<uint64_t> trimmedSamples{ 0 };
	std::atomic<double> droppedSeconds{ 0.0 };
	std::atomic<bool> trimming{ false };
	// Tries to put a file in another's place that another program's hold on
	// the file refused (TakePlaceOf), in this stretch.
	std::atomic<uint64_t> replaceRetries{ 0 };

	// Written by the thread, then `done` is set; read only after it.
	bool paused = false;
	recording::EndReason reason = recording::EndReason::AppExit;
	bool saved = false;
	bool closedCleanly = false;
	double seconds = 0.0;
	std::string error;
	std::atomic<bool> done{ false };
};

PoseRecorder::PoseRecorder() = default;

PoseRecorder::~PoseRecorder()
{
	StopAndWait(recording::EndReason::AppExit, 5000);
	// Finished, or detached by the timeout.
	if (thread_.joinable())
		thread_.join();
}

std::wstring PoseRecorder::FilePath(const std::wstring &directory)
{
	return directory + L"\\NovaCalibrator-recording.novapose";
}

bool PoseRecorder::CanResume(const std::wstring &directory) const
{
	uint64_t bytes = 0;
	return paused_.valid && paused_.path == FilePath(directory) && FileSize(paused_.path, bytes) &&
		bytes == paused_.bytes;
}

void PoseRecorder::Forget()
{
	paused_ = Paused();
}

// A label of eight hex digits, made up once for each device in a recording.
recording::DeviceInfo PoseRecorder::Labelled(const RecordedDevice &device)
{
	const std::string key = device.serial.empty() ? "id:" + std::to_string(device.deviceId) : "serial:" + device.serial;
	auto known = labels_.find(key);
	if (known == labels_.end())
	{
		std::random_device random;
		std::string label;
		bool taken = true;
		while (taken)
		{
			char text[16];
			snprintf(text, sizeof text, "%08x", static_cast<unsigned>(random()));
			label = text;
			taken = false;
			for (const auto &entry : labels_)
				taken = taken || entry.second == label;
		}
		known = labels_.emplace(key, label).first;
	}
	recording::DeviceInfo info;
	info.deviceId = device.deviceId;
	info.deviceClass = device.deviceClass;
	info.controllerRole = device.controllerRole;
	info.label = known->second;
	info.model = device.model;
	info.trackingSystem = device.trackingSystem;
	info.controllerType = device.controllerType;
	return info;
}

PoseRecorder::StartResult PoseRecorder::Start(PoseStreamHub &hub, const std::wstring &directory,
	const std::string &writer, const std::vector<RecordedDevice> &devices, const PoseRecordingLimits &limits,
	std::string *detail)
{
	const auto refuse = [detail](StartResult result, const std::string &why)
	{
		if (detail)
			*detail = why;
		return result;
	};
	if (session_)
	{
		if (!session_->done.load(std::memory_order_acquire))
			return refuse(StartResult::Busy, "the last stretch has not closed the file yet");
		Finished unread;
		Harvest(unread);
	}

	if (!CreateDirectoryW(directory.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS)
		return refuse(StartResult::NoFolder, "CreateDirectory failed with error " + std::to_string(GetLastError()));
	uint64_t freeBytes = 0;
	if (!limits.freeDiskBytes(directory, freeBytes))
		return refuse(StartResult::NoFolder, "the drive's free space could not be read");
	if (freeBytes < limits.minFreeDiskBytes)
		return refuse(StartResult::DiskSpaceLow, std::to_string(freeBytes >> 20) + " MiB free, under the floor of " +
			std::to_string(limits.minFreeDiskBytes >> 20) + " MiB");

	const bool resume = CanResume(directory);
	const std::wstring finalPath = FilePath(directory);
	const std::wstring newPath = finalPath + L".new";
	// What a drop of the oldest part left half made, if the program ended in it.
	DeleteFileW((finalPath + L".keep").c_str());
	// A new recording is written beside the old one until it has a sample.
	const std::wstring &path = resume ? finalPath : newPath;
	const HANDLE file = OpenForAppend(path, resume ? OPEN_EXISTING : CREATE_ALWAYS);
	if (file == INVALID_HANDLE_VALUE)
		return refuse(StartResult::CannotCreate, "CreateFile failed with error " + std::to_string(GetLastError()));
	const auto abandon = [&](const std::string &why)
	{
		CloseHandle(file);
		// A resumed file keeps what it had: the bytes this start added, if any,
		// make it a file the next start does not go on with.
		if (!resume)
			DeleteFileW(path.c_str());
		session_.reset();
		return refuse(StartResult::CannotCreate, why);
	};

	try
	{
		if (!resume)
		{
			labels_.clear();
			paused_ = Paused();
		}
		session_ = std::make_shared<Session>();
		Session &session = *session_;
		session.file = file;
		session.directory = directory;
		session.finalPath = finalPath;
		session.newPath = newPath;
		session.replaced = resume;
		session.limits = limits;
		session.writer = writer;
		if (!resume || !cuts_)
			cuts_ = std::make_shared<CutPoints>();
		session.cuts = cuts_;
		LARGE_INTEGER frequency{};
		QueryPerformanceFrequency(&frequency);
		session.qpcFrequency = frequency.QuadPart;
		// A recording that goes on is stored as its file began. A new one is
		// compact when that was asked for and this clock is one a compact file
		// can have.
		session.parameters = { session.qpcFrequency, resume ? paused_.velocityScale : limits.velocityScale,
			resume ? paused_.angularScale : limits.angularScale };
		session.compact = resume ? paused_.compact : limits.compact && recording::compact::Valid(session.parameters);
		session.earlierSamples = paused_.samples;
		session.earlierSeconds = paused_.seconds;
		session.samples.store(paused_.samples, std::memory_order_release);
		session.lost.store(paused_.lost, std::memory_order_release);
		session.clips.store(paused_.clips, std::memory_order_release);

		// One entry per id.
		devices_.clear();
		models_.clear();
		for (const RecordedDevice &device : devices)
		{
			bool repeated = false;
			for (const recording::DeviceInfo &known : devices_)
				repeated = repeated || known.deviceId == device.deviceId;
			if (!repeated)
			{
				devices_.push_back(Labelled(device));
				models_[device.deviceId] = device.renderModel;
			}
		}

		if (hub_ != &hub)
		{
			hub_ = &hub;
			consumer_ = hub.CreateConsumer();
		}
		session.hub = &hub;
		session.consumer = consumer_;
		// The count first: a boundary that lands before the discard is then
		// neither in the cursor's hole nor taken for one already recorded.
		session.boundariesNoted = hub.StreamBoundaries();
		// Everything published before now, the whole of a pause included, is
		// behind the cursor and never reaches the file.
		hub.DiscardBacklog(consumer_);
		session.startQpc = QpcNow();

		for (const recording::DeviceInfo &device : devices_)
		{
			session.startState.Device(device);
			if (!models_[device.deviceId].empty())
				session.startState.Shape({ device.deviceId, models_[device.deviceId] });
		}
		if (hasAlignment_)
			session.startState.Align(alignment_);

		// What the stretch starts with: a new file's header, which holds as many
		// devices as fit, or a resume record and every device again, so that the
		// stretch reads by itself. A stretch of clips that goes on in a file adds
		// nothing yet: its first clip states all of it.
		std::string bytes;
		size_t inHeader = 0;
		const bool silent = resume && limits.clips;
		if (silent)
		{
			for (const recording::DeviceInfo &device : devices_)
				if (device.deviceId < session.described.size())
					session.described[device.deviceId] = true;
		}
		else if (resume)
			recording::AppendResume(bytes, { static_cast<uint8_t>(recording::PauseCause::Player), session.startQpc,
				UnixMicrosecondsNow(), std::string() });
		else
		{
			recording::Header header = HeaderFor(session.compact, session.parameters);
			header.startQpc = session.startQpc;
			header.startUnixMicroseconds = UnixMicrosecondsNow();
			header.writer = writer;
			inHeader = (std::min)(devices_.size(), recording::MaxHeaderDevices);
			header.devices.assign(devices_.begin(), devices_.begin() + static_cast<std::ptrdiff_t>(inHeader));
			recording::AppendHeader(bytes, header);
		}
		for (size_t i = 0; i < devices_.size() && !silent; ++i)
		{
			if (i >= inHeader)
				recording::AppendDevice(bytes, devices_[i]);
			if (devices_[i].deviceId < session.described.size())
				session.described[devices_[i].deviceId] = true;
		}
		// The shape of each device that has one, then the calibration in use, so
		// that this stretch too reads by itself.
		for (const recording::DeviceInfo &device : devices_)
			if (!silent && !models_[device.deviceId].empty())
				recording::AppendModel(bytes, { device.deviceId, models_[device.deviceId] });
		if (hasAlignment_ && !silent)
		{
			alignment_.qpc = session.startQpc;
			recording::AppendAlignment(bytes, alignment_);
		}
		// A new recording of clips holds nothing until its first clip, and says so.
		if (limits.clips && !resume)
			recording::AppendPause(bytes, { static_cast<uint8_t>(recording::PauseCause::Clip), session.startQpc,
				UnixMicrosecondsNow() });
		DWORD written = 0;
		if (!bytes.empty() && (!WriteFile(file, bytes.data(), static_cast<DWORD>(bytes.size()), &written, nullptr) ||
			written != bytes.size()))
			return abandon("writing the start of the stretch failed with error " + std::to_string(GetLastError()));
		session.bytes.store((resume ? paused_.bytes : 0) + bytes.size(), std::memory_order_release);

		thread_ = std::thread(&PoseRecorder::Run, session_);
	}
	catch (const std::exception &e)
	{
		return abandon(e.what());
	}
	return resume ? StartResult::Resumed : StartResult::Started;
}

void PoseRecorder::SetDevices(const std::vector<RecordedDevice> &devices)
{
	if (!session_ || session_->done.load(std::memory_order_acquire))
		return;
	std::vector<recording::DeviceInfo> changed;
	std::vector<recording::Model> shapes;
	for (const RecordedDevice &described : devices)
	{
		const recording::DeviceInfo device = Labelled(described);
		recording::DeviceInfo *known = nullptr;
		for (recording::DeviceInfo &candidate : devices_)
			if (candidate.deviceId == device.deviceId)
				known = &candidate;
		const bool same = known && *known == device;
		std::string &model = models_[device.deviceId];
		if (same && model == described.renderModel)
			continue;
		// A device's model is stated with its entry, and again when it changes.
		model = described.renderModel;
		if (!same)
		{
			if (known)
				*known = device;
			else
				devices_.push_back(device);
			changed.push_back(device);
		}
		if (!model.empty())
			shapes.push_back({ device.deviceId, model });
	}
	if (changed.empty() && shapes.empty())
		return;
	std::lock_guard<std::mutex> lock(session_->mutex);
	session_->pendingDevices.insert(session_->pendingDevices.end(), changed.begin(), changed.end());
	session_->pendingModels.insert(session_->pendingModels.end(), shapes.begin(), shapes.end());
}

void PoseRecorder::SetAlignment(const recording::Alignment &alignment)
{
	if (hasAlignment_ && alignment_.SameValues(alignment))
		return;
	hasAlignment_ = true;
	alignment_ = alignment;
	if (!session_ || session_->done.load(std::memory_order_acquire))
		return;
	std::lock_guard<std::mutex> lock(session_->mutex);
	session_->pendingAlignments.push_back(alignment);
}

void PoseRecorder::Mark(const std::string &name, uint32_t deviceId, bool keepAfter)
{
	if (!session_ || session_->done.load(std::memory_order_acquire))
		return;
	PendingMark mark;
	mark.record.qpc = QpcNow();
	mark.record.unixMicroseconds = UnixMicrosecondsNow();
	mark.record.deviceId = deviceId;
	mark.record.name = name;
	mark.keepAfter = keepAfter;
	std::lock_guard<std::mutex> lock(session_->mutex);
	// Nothing after a stop reaches the file, a mark included.
	if (!session_->stop && session_->pendingMarks.size() < MaxPendingMarks)
		session_->pendingMarks.push_back(std::move(mark));
}

void PoseRecorder::Request(bool pause, recording::EndReason reason)
{
	if (!session_)
		return;
	{
		// The thread drains the hub under this mutex, so once it is taken here
		// no drain is in progress and none starts again.
		std::lock_guard<std::mutex> lock(session_->mutex);
		// The first request is the one recorded.
		if (!session_->stop)
		{
			session_->stop = true;
			session_->stopIsPause = pause;
			session_->stopReason = reason;
			session_->stopQpc = QpcNow();
			session_->stopUnixMicroseconds = UnixMicrosecondsNow();
		}
	}
	session_->wake.notify_all();
}

void PoseRecorder::RequestPause()
{
	Request(true, recording::EndReason::AppExit);
}

void PoseRecorder::RequestStop(recording::EndReason reason)
{
	Request(false, reason);
}

bool PoseRecorder::StopAndWait(recording::EndReason reason, unsigned timeoutMs)
{
	if (!session_)
		return true;
	RequestStop(reason);
	const ULONGLONG started = GetTickCount64();
	while (!session_->done.load(std::memory_order_acquire))
	{
		if (GetTickCount64() - started >= timeoutMs)
		{
			// A write that does not return. The thread closes the file when it
			// does, with its own reference to the session and no hub call.
			if (thread_.joinable())
				thread_.detach();
			session_.reset();
			paused_ = Paused();
			return false;
		}
		Sleep(1);
	}
	return true;
}

PoseRecorder::Progress PoseRecorder::GetProgress() const
{
	Progress progress;
	if (!session_)
	{
		progress.bytes = paused_.bytes;
		progress.samples = paused_.samples;
		progress.lost = paused_.lost;
		progress.seconds = paused_.seconds;
		progress.clips = paused_.clips;
		return progress;
	}
	const Session &session = *session_;
	progress.recording = !session.done.load(std::memory_order_acquire);
	progress.bytes = session.bytes.load(std::memory_order_acquire);
	progress.samples = session.samples.load(std::memory_order_acquire);
	progress.lost = session.lost.load(std::memory_order_acquire);
	progress.clips = session.clips.load(std::memory_order_acquire);
	const int64_t openSince = session.clipOpenSince.load(std::memory_order_acquire);
	progress.clipOpen = progress.recording && openSince != 0;
	progress.kept = progress.recording ? session.kept.load(std::memory_order_acquire) : 0;
	progress.trims = session.trims.load(std::memory_order_acquire);
	progress.trimmedSamples = session.trimmedSamples.load(std::memory_order_acquire);
	progress.trimming = progress.recording && session.trimming.load(std::memory_order_acquire);
	progress.replaceRetries = session.replaceRetries.load(std::memory_order_acquire);
	const double dropped = session.droppedSeconds.load(std::memory_order_acquire);
	if (!progress.recording)
		progress.seconds = session.seconds;
	else if (session.limits.clips)
		progress.seconds = session.earlierSeconds - dropped +
			static_cast<double>(session.clipTicks.load(std::memory_order_acquire) +
				(openSince != 0 ? (std::max)(QpcNow() - openSince, static_cast<int64_t>(0)) : 0)) /
				static_cast<double>(session.qpcFrequency);
	else
		progress.seconds = session.earlierSeconds - dropped +
			static_cast<double>(QpcNow() - session.startQpc) / static_cast<double>(session.qpcFrequency);
	return progress;
}

void PoseRecorder::Harvest(Finished &finished)
{
	if (thread_.joinable())
		thread_.join();
	const Session &session = *session_;
	finished.path = session.finalPath;
	finished.paused = session.paused;
	finished.reason = session.reason;
	finished.saved = session.saved;
	finished.bytes = session.bytes.load(std::memory_order_acquire);
	finished.samples = session.samples.load(std::memory_order_acquire);
	finished.lost = session.lost.load(std::memory_order_acquire);
	finished.seconds = session.seconds;
	finished.clipMode = session.limits.clips;
	finished.clips = session.clips.load(std::memory_order_acquire);
	finished.compact = session.compact;
	finished.trims = session.trims.load(std::memory_order_acquire);
	finished.replaceRetries = session.replaceRetries.load(std::memory_order_acquire);
	finished.closedCleanly = session.closedCleanly;
	finished.error = session.error;
	paused_ = Paused();
	if (finished.paused && finished.saved && finished.closedCleanly)
		paused_ = { true, finished.path, finished.bytes, finished.samples, finished.lost, finished.seconds,
			finished.clips, session.compact, session.parameters.velocityScale, session.parameters.angularScale };
	session_.reset();
}

bool PoseRecorder::TakeFinished(Finished &finished)
{
	if (!session_ || !session_->done.load(std::memory_order_acquire))
		return false;
	Harvest(finished);
	return true;
}

void PoseRecorder::Run(const std::shared_ptr<Session> &shared)
{
	Session &session = *shared;
	const bool clips = session.limits.clips;
	bool pause = false;
	recording::EndReason reason = recording::EndReason::AppExit;
	int64_t closeQpc = 0, closeUnixMicroseconds = 0;
	std::string error;
	bool writeFailed = false;
	uint64_t samples = session.earlierSamples;
	std::string out;
	// "Automatic": between clips nothing is written, and what arrives waits in
	// memory for the next mark. `armed` is that state; otherwise a clip is open
	// (or, in a full recording, the file is) and every record goes to the file.
	bool armed = clips;
	int64_t clipStartQpc = 0, clipEndQpc = 0;
	// The counts this stretch's closed clips lasted.
	int64_t clipTicks = 0;
	// A compact file's samples go through this: most into a block that stays
	// open until the pass ends or another record comes. It starts as a reader
	// does behind the header or the resume record this stretch began with.
	const bool compact = session.compact;
	recording::CompactWriter encoder(session.parameters);
	// What one more sample can add to the file at most.
	const size_t sampleBytes = compact ? recording::compact::MaxSampleBytes + 16 : SampleRecordBytes;

	const auto fail = [&](const std::string &why)
	{
		error = why;
		writeFailed = true;
		pause = false;
		reason = recording::EndReason::WriteError;
		return false;
	};
	// Hands `out` to Windows and empties it.
	const auto write = [&]()
	{
		size_t done = 0;
		while (done < out.size())
		{
			const DWORD piece = static_cast<DWORD>((std::min)(out.size() - done, static_cast<size_t>(1) << 20));
			DWORD wrote = 0;
			if (!WriteFile(session.file, out.data() + done, piece, &wrote, nullptr) || wrote == 0)
				return fail("WriteFile failed with error " + std::to_string(GetLastError()));
			done += wrote;
			session.bytes.fetch_add(wrote, std::memory_order_release);
		}
		out.clear();
		session.samples.store(samples, std::memory_order_release);
		return true;
	};
	// A new recording takes the old one's place when its first sample is about
	// to be written: until then a recording with nothing in it has replaced
	// nothing.
	const auto takePlace = [&]()
	{
		if (session.replaced)
			return true;
		if (!write())
			return false;
		CloseHandle(session.file);
		session.file = INVALID_HANDLE_VALUE;
		const DWORD moveError = TakePlaceOf(session.newPath, session.finalPath, session.limits.replaceWaitMs,
			session.replaceRetries);
		if (moveError != ERROR_SUCCESS)
			return fail("the previous recording could not be replaced (error " + std::to_string(moveError) + ")");
		session.replaced = true;
		session.file = OpenForAppend(session.finalPath, OPEN_EXISTING);
		if (session.file == INVALID_HANDLE_VALUE)
			return fail("the recording could not be opened again (error " + std::to_string(GetLastError()) + ")");
		return true;
	};
	// What the file holds of the recording at `now`, in seconds.
	double droppedSeconds = 0.0;
	const auto heldSeconds = [&](int64_t now)
	{
		const int64_t ticks = !clips ? now - session.startQpc
			: clipTicks + (armed ? 0 : (std::max)(now - clipStartQpc, static_cast<int64_t>(0)));
		return session.earlierSeconds - droppedSeconds +
			static_cast<double>(ticks) / static_cast<double>(session.qpcFrequency);
	};

	// The size cap with `keepNewest`: the oldest part of the file is dropped.
	// A file written front to back cannot lose its front in place, so the part
	// that stays is copied into a new file behind a header of its own, and the
	// new file takes the old one's place in one step. Until that step the old
	// file is whole and still written to, so a crash or a stop at any moment
	// costs nothing: the copy is thrown away.
	const bool keepNewest = session.limits.keepNewest;
	const uint64_t cap = session.limits.maxFileBytes;
	const uint64_t trimStart = cap - (std::min)(cap,
		session.limits.trimHeadroomBytes != 0 ? session.limits.trimHeadroomBytes : cap / 16);
	const uint64_t trimSpacing = (std::max)(
		session.limits.trimSpacingBytes != 0 ? session.limits.trimSpacingBytes : cap / 64, static_cast<uint64_t>(1));
	const std::wstring keepPath = session.finalPath + L".keep";
	std::vector<CutPoint> &cuts = session.cuts->points;
	struct
	{
		bool active = false;
		HANDLE in = INVALID_HANDLE_VALUE;
		HANDLE out = INVALID_HANDLE_VALUE;
		size_t index = 0;       // the cut point the new file starts at
		uint64_t at = 0;        // how far the old file is copied
		uint64_t prefix = 0;    // what the new file states in front of the copy
		uint64_t written = 0;   // the new file's size so far
	} trim;
	// A drop that could not be done is not tried again in this stretch: the
	// cap then ends the recording.
	bool trimFailed = false;
	std::vector<char> slice;

	const auto abandonTrim = [&]()
	{
		if (trim.in != INVALID_HANDLE_VALUE)
			CloseHandle(trim.in);
		if (trim.out != INVALID_HANDLE_VALUE)
			CloseHandle(trim.out);
		trim.in = INVALID_HANDLE_VALUE;
		trim.out = INVALID_HANDLE_VALUE;
		if (trim.active)
			DeleteFileW(keepPath.c_str());
		trim.active = false;
		session.trimming.store(false, std::memory_order_release);
	};
	const auto failTrim = [&](const std::string &why)
	{
		abandonTrim();
		trimFailed = true;
		if (error.empty())
			error = why;
		return false;
	};
	// Starts a drop at the first cut point that leaves no more than
	// keepFraction of the cap. False when no cut point frees enough.
	const auto beginTrim = [&]()
	{
		if (cuts.empty())
			return false;
		const uint64_t bytes = session.bytes.load(std::memory_order_relaxed);
		const uint64_t keep = static_cast<uint64_t>(
			static_cast<double>(cap) * (std::min)(1.0, (std::max)(0.0, session.limits.keepFraction)));
		size_t index = cuts.size() - 1;
		for (size_t i = 0; i < cuts.size(); ++i)
			if (cuts[i].offset + keep >= bytes)
			{
				index = i;
				break;
			}
		const CutPoint &point = cuts[index];
		// A header that begins at the cut, then what the file had said by then of
		// each device, its model and the calibration.
		std::string prefix;
		recording::Header header = HeaderFor(session.compact, session.parameters);
		header.startQpc = point.qpc;
		header.startUnixMicroseconds = point.unixMicroseconds;
		header.writer = session.writer;
		const size_t inHeader = (std::min)(point.state.devices.size(), recording::MaxHeaderDevices);
		header.devices.assign(point.state.devices.begin(),
			point.state.devices.begin() + static_cast<std::ptrdiff_t>(inHeader));
		recording::AppendHeader(prefix, header);
		for (size_t i = inHeader; i < point.state.devices.size(); ++i)
			recording::AppendDevice(prefix, point.state.devices[i]);
		for (const auto &model : point.state.models)
			if (!model.second.empty())
				recording::AppendModel(prefix, { model.first, model.second });
		for (recording::Alignment alignment : point.state.alignments)
		{
			alignment.qpc = point.qpc;
			recording::AppendAlignment(prefix, alignment);
		}
		// Not worth a copy unless what is left is under where dropping starts.
		if (point.offset > bytes || point.offset <= prefix.size() || prefix.size() + (bytes - point.offset) >= trimStart)
			return false;

		trim.in = CreateFileW(session.finalPath.c_str(), GENERIC_READ,
			FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
		trim.out = CreateFileW(keepPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
			FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
		trim.active = true;
		LARGE_INTEGER from{};
		from.QuadPart = static_cast<LONGLONG>(point.offset);
		DWORD wrote = 0;
		if (trim.in == INVALID_HANDLE_VALUE || trim.out == INVALID_HANDLE_VALUE ||
			!SetFilePointerEx(trim.in, from, nullptr, FILE_BEGIN) ||
			!WriteFile(trim.out, prefix.data(), static_cast<DWORD>(prefix.size()), &wrote, nullptr) ||
			wrote != prefix.size())
			return failTrim("the recording's newer part could not be copied (error " +
				std::to_string(GetLastError()) + ")");
		trim.index = index;
		trim.at = point.offset;
		trim.prefix = prefix.size();
		trim.written = prefix.size();
		session.trimming.store(true, std::memory_order_release);
		return true;
	};
	// The copy reached the old file's end: the new file takes its place, and
	// every count becomes the new file's own. False when it did not.
	const auto finishTrim = [&]()
	{
		const CutPoint point = cuts[trim.index];
		const bool flushed = FlushFileBuffers(trim.out) != 0;
		CloseHandle(trim.in);
		CloseHandle(trim.out);
		trim.in = INVALID_HANDLE_VALUE;
		trim.out = INVALID_HANDLE_VALUE;
		if (!flushed)
			return failTrim("the recording's newer part could not be written out (error " +
				std::to_string(GetLastError()) + ")");
		CloseHandle(session.file);
		session.file = INVALID_HANDLE_VALUE;
		const DWORD moveError = TakePlaceOf(keepPath, session.finalPath, session.limits.replaceWaitMs,
			session.replaceRetries);
		const bool moved = moveError == ERROR_SUCCESS;
		session.file = OpenForAppend(session.finalPath, OPEN_EXISTING);
		if (session.file == INVALID_HANDLE_VALUE)
		{
			abandonTrim();
			return fail("the recording could not be opened again (error " + std::to_string(GetLastError()) + ")");
		}
		if (!moved)
			return failTrim("the recording's newer part could not take its place (error " +
				std::to_string(moveError) + ")");
		trim.active = false;
		session.trimming.store(false, std::memory_order_release);

		const uint64_t removed = point.offset - trim.prefix;
		session.bytes.store(trim.written, std::memory_order_release);
		samples -= (std::min)(samples, point.samplesBefore);
		session.samples.store(samples, std::memory_order_release);
		session.lost.fetch_sub((std::min)(session.lost.load(std::memory_order_relaxed), point.lostBefore),
			std::memory_order_release);
		session.clips.fetch_sub((std::min)(session.clips.load(std::memory_order_relaxed), point.clipsBefore),
			std::memory_order_release);
		droppedSeconds += point.secondsBefore;
		session.droppedSeconds.store(droppedSeconds, std::memory_order_release);
		session.trimmedSamples.fetch_add(point.samplesBefore, std::memory_order_release);
		session.trims.fetch_add(1, std::memory_order_release);
		cuts.erase(cuts.begin(), cuts.begin() + static_cast<std::ptrdiff_t>(trim.index) + 1);
		for (CutPoint &later : cuts)
		{
			later.offset -= removed;
			later.samplesBefore -= point.samplesBefore;
			later.lostBefore -= point.lostBefore;
			later.clipsBefore -= point.clipsBefore;
			later.secondsBefore -= point.secondsBefore;
		}
		return true;
	};
	// Copies the next piece of the old file, or with `toEnd` all that is left,
	// and swaps the files when the copy has reached the old file's end. `out`
	// must be empty. False only when a write to the recording itself failed.
	const auto stepTrim = [&](bool toEnd)
	{
		if (slice.empty())
			slice.resize((std::max)(session.limits.trimSliceBytes, static_cast<size_t>(1)));
		for (;;)
		{
			const uint64_t end = session.bytes.load(std::memory_order_relaxed);
			if (trim.at >= end)
				return finishTrim() || !writeFailed;
			const DWORD want = static_cast<DWORD>((std::min)(static_cast<uint64_t>(slice.size()), end - trim.at));
			DWORD got = 0, wrote = 0;
			if (!ReadFile(trim.in, slice.data(), want, &got, nullptr) || got == 0 ||
				!WriteFile(trim.out, slice.data(), got, &wrote, nullptr) || wrote != got)
			{
				failTrim("the recording's newer part could not be copied (error " + std::to_string(GetLastError()) + ")");
				return true;
			}
			trim.at += got;
			trim.written += got;
			if (!toEnd && trim.at < end)
				return true;
		}
	};

	// Whether a record of this size and the closing record still fit under the
	// cap. A clip's pause record and the end record behind it both close. The
	// size cap's policy is in this one place: with `keepNewest`, a record that
	// does not fit has the oldest part dropped for it first.
	const size_t closingBytes = ClosingRecordBytes * (clips ? 2 : 1);
	const auto underCap = [&](size_t recordBytes)
	{
		return session.bytes.load(std::memory_order_relaxed) + out.size() + encoder.Pending() + recordBytes +
			closingBytes <= cap;
	};
	const auto fits = [&](size_t recordBytes)
	{
		if (underCap(recordBytes))
			return true;
		// The cap is here before a drop was done (the disk was slow, or nothing
		// could be cut yet): the drop is done now, in one go, while the hub
		// holds what arrives.
		if (!keepNewest || trimFailed || writeFailed || !session.replaced)
			return false;
		if (!write())
			return false;
		if (!trim.active && !beginTrim())
			return false;
		return stepTrim(true) && underCap(recordBytes);
	};
	const auto ticksOf = [&](double seconds)
	{
		return static_cast<int64_t>((std::max)(0.0, seconds) * static_cast<double>(session.qpcFrequency));
	};

	// An exception escaping a thread entry ends the process without unwinding
	// (PoseStreamHub::DrainLoop). The buffers here grow with a backlog, so
	// bad_alloc is reachable: it ends the recording, not the program.
	try
	{
		std::vector<protocol::DevicePoseSample> batch;
		ULONGLONG lastDiskCheck = GetTickCount64();
		bool ended = false;
		// Nothing more fits (`fits` above): the recording ends, and what it
		// holds stays.
		const auto full = [&]()
		{
			if (!writeFailed)
			{
				pause = false;
				reason = recording::EndReason::SizeLimit;
			}
			ended = true;
		};
		// What waits for the next clip, oldest first, and what was known in
		// front of it.
		KnownState known = session.startState;
		std::deque<KeptPiece> kept;
		size_t keptBytes = 0;
		uint64_t keptSamples = 0;
		const int64_t beforeTicks = ticksOf(session.limits.clipBeforeSeconds);
		const int64_t afterTicks = ticksOf(session.limits.clipAfterSeconds);

		while (!ended)
		{
			bool waitForMore = true;
			for (int pass = 0; pass < DrainPasses && !ended; ++pass)
			{
				std::vector<recording::DeviceInfo> changed;
				std::vector<recording::Alignment> aligned;
				std::vector<recording::Model> shapes;
				std::vector<PendingMark> marks;
				PoseStreamHub::Hole hole;
				uint64_t dropped = 0, boundaries = 0;
				bool newBoundary = false;
				{
					std::lock_guard<std::mutex> lock(session.mutex);
					if (session.stop)
					{
						pause = session.stopIsPause;
						reason = session.stopReason;
						closeQpc = session.stopQpc;
						closeUnixMicroseconds = session.stopUnixMicroseconds;
						ended = true;
						break;
					}
					changed.swap(session.pendingDevices);
					aligned.swap(session.pendingAlignments);
					shapes.swap(session.pendingModels);
					marks.swap(session.pendingMarks);
					dropped = session.hub->Drain(session.consumer, batch, &hole);
					// Once for each boundary: an open hole reports its boundary
					// with every drain until a sample closes it.
					if (hole.sessionBoundary)
					{
						boundaries = session.hub->StreamBoundaries();
						newBoundary = boundaries != session.boundariesNoted;
						session.boundariesNoted = boundaries;
					}
				}
				const int64_t now = QpcNow();
				// Where this pass's records go: the file, or the memory between
				// clips, which no cap bounds here (it is bounded when it is kept).
				const bool keeping = armed;
				KeptPiece piece;
				piece.qpc = now;
				std::string &to = keeping ? piece.bytes : out;
				// The room a record needs, and the open block closed in front of
				// it: a record that is no sample never goes inside one.
				const auto room = [&](size_t recordBytes)
				{
					if (keeping)
						return true;
					if (!fits(recordBytes))
						return false;
					encoder.Flush(out);
					return true;
				};
				const auto describe = [&](const recording::DeviceInfo &device)
				{
					if (!room(DeviceRecordBytes))
						return full();
					recording::AppendDevice(to, device);
					if (device.deviceId < session.described.size())
						session.described[device.deviceId] = true;
					if (keeping)
						piece.devices.push_back(device);
					else
						known.Device(device);
				};

				for (size_t i = 0; i < changed.size() && !ended; ++i)
					describe(changed[i]);
				for (size_t i = 0; i < shapes.size() && !ended; ++i)
				{
					if (!room(ModelRecordBytes))
					{
						full();
						break;
					}
					recording::AppendModel(to, shapes[i]);
					if (keeping)
						piece.models.push_back(shapes[i]);
					else
						known.Shape(shapes[i]);
				}
				for (size_t i = 0; i < aligned.size() && !ended; ++i)
				{
					if (!room(AlignmentRecordBytes))
					{
						full();
						break;
					}
					aligned[i].qpc = now;
					recording::AppendAlignment(to, aligned[i]);
					if (keeping)
						piece.alignments.push_back(aligned[i]);
					else
						known.Align(aligned[i]);
				}
				const bool arrived = dropped > 0 || !batch.empty();
				if (arrived)
					waitForMore = false;
				if (newBoundary && arrived && !ended)
				{
					if (!room(BoundaryRecordBytes))
						full();
					else
						recording::AppendBoundary(to, { now, boundaries });
				}
				if (dropped > 0 && !ended)
				{
					if (!room(GapRecordBytes))
						full();
					else
					{
						recording::AppendGap(to, { now, dropped, hole.size });
						if (keeping)
							piece.lost += dropped;
						else
							session.lost.fetch_add(dropped, std::memory_order_release);
					}
				}
				for (size_t i = 0; i < batch.size() && !ended; ++i)
				{
					const protocol::DevicePoseSample &pose = batch[i];
					// A device nothing has described yet still gets its entry in
					// front of its first sample.
					if (pose.deviceId < session.described.size() && !session.described[pose.deviceId])
					{
						recording::DeviceInfo unknown;
						unknown.deviceId = pose.deviceId;
						describe(unknown);
						if (ended)
							break;
					}
					if (!keeping && !fits(sampleBytes))
					{
						full();
						break;
					}
					if (!keeping && !takePlace())
					{
						ended = true;
						break;
					}
					// What waits in memory for a clip waits as it came: a clip's
					// blocks are made when the clip is written.
					if (keeping)
					{
						recording::AppendSample(to, ToSample(pose));
						++piece.samples;
					}
					else
					{
						if (compact)
							encoder.Append(out, ToSample(pose));
						else
							recording::AppendSample(out, ToSample(pose));
						++samples;
						if (out.size() >= WriteChunkBytes && !write())
							ended = true;
					}
				}
				if (keeping && !ended)
				{
					if (!piece.bytes.empty())
					{
						keptBytes += piece.bytes.size();
						keptSamples += piece.samples;
						kept.push_back(std::move(piece));
					}
					// What is older than a clip reaches back goes, and so does
					// what is more than the memory allowed for it. What it said of
					// the devices stays known.
					while (!kept.empty() &&
						(kept.front().qpc < now - beforeTicks || keptBytes > session.limits.clipKeptBytes))
					{
						known.Fold(kept.front());
						keptBytes -= kept.front().bytes.size();
						keptSamples -= kept.front().samples;
						kept.pop_front();
					}
					session.kept.store(keptSamples, std::memory_order_release);
				}

				for (size_t i = 0; i < marks.size() && !ended; ++i)
				{
					const PendingMark &mark = marks[i];
					const int64_t until = mark.record.qpc + (mark.keepAfter ? afterTicks : 0);
					if (!armed)
					{
						// A full recording, or a clip that is open: a record where
						// it happened, and the clip goes on for it.
						if (!fits(MarkRecordBytes))
							full();
						else
						{
							encoder.Flush(out);
							recording::AppendMark(out, mark.record);
							clipEndQpc = (std::max)(clipEndQpc, until);
						}
						continue;
					}
					// A clip needs something to hold. With nothing kept, a new
					// recording would replace the old one without a sample, and
					// the player's "save" has nothing to save.
					if (keptSamples == 0 && (!session.replaced || !mark.keepAfter))
						continue;
					// The clip reaches back to the oldest thing kept.
					const int64_t from = kept.empty() ? mark.record.qpc : (std::min)(kept.front().qpc, mark.record.qpc);
					const int64_t fromUnixMicroseconds = mark.record.unixMicroseconds -
						(mark.record.qpc - from) * 1000000 / session.qpcFrequency;
					std::string opening;
					recording::AppendResume(opening, { static_cast<uint8_t>(recording::PauseCause::Clip), from,
						fromUnixMicroseconds, mark.record.name });
					for (const recording::DeviceInfo &device : known.devices)
						recording::AppendDevice(opening, device);
					for (const auto &model : known.models)
						if (!model.second.empty())
							recording::AppendModel(opening, { model.first, model.second });
					for (recording::Alignment alignment : known.alignments)
					{
						alignment.qpc = from;
						recording::AppendAlignment(opening, alignment);
					}
					// A compact file gets what was kept as blocks, which start anew
					// behind the resume record as a reader does there. Their size is
					// known only once they are made.
					std::string blocks;
					if (compact)
					{
						encoder.Restart();
						for (const KeptPiece &old : kept)
							recording::AppendCompact(blocks, encoder, old.bytes);
						encoder.Flush(blocks);
					}
					if (!fits(opening.size() + (compact ? blocks.size() : keptBytes) + MarkRecordBytes))
					{
						full();
						break;
					}
					if (!takePlace())
					{
						ended = true;
						break;
					}
					out += opening;
					out += blocks;
					for (const KeptPiece &old : kept)
					{
						if (!compact)
							out += old.bytes;
						samples += old.samples;
						session.lost.fetch_add(old.lost, std::memory_order_release);
						known.Fold(old);
						if (out.size() >= WriteChunkBytes && !write())
						{
							ended = true;
							break;
						}
					}
					kept.clear();
					keptBytes = 0;
					keptSamples = 0;
					session.kept.store(0, std::memory_order_release);
					if (ended)
						break;
					recording::AppendMark(out, mark.record);
					armed = false;
					clipStartQpc = from;
					clipEndQpc = until;
					session.clipOpenSince.store(from, std::memory_order_release);
				}
				// The pass's block closes with the pass, so that the file ends on
				// a record's edge whenever nothing is being written.
				if (!ended)
					encoder.Flush(out);
				if (!ended && !out.empty() && !write())
					ended = true;

				// A clip ends when nothing was marked for clipAfterSeconds. What
				// arrives from here on waits for the next one.
				if (clips && !armed && !ended && now >= clipEndQpc)
				{
					const int64_t at = QpcNow();
					recording::AppendPause(out, { static_cast<uint8_t>(recording::PauseCause::Clip), at,
						UnixMicrosecondsNow() });
					if (!write())
						ended = true;
					else
					{
						armed = true;
						clipTicks += (std::max)(at - clipStartQpc, static_cast<int64_t>(0));
						session.clipTicks.store(clipTicks, std::memory_order_release);
						session.clipOpenSince.store(0, std::memory_order_release);
						session.clips.fetch_add(1, std::memory_order_release);
					}
				}
				// A place the file can later be cut at: what was written ends on
				// a record's edge here, and `known` is what a file that starts
				// here must state (between clips the next clip states the rest).
				if (keepNewest && session.replaced && !ended && out.empty() &&
					session.bytes.load(std::memory_order_relaxed) >= (cuts.empty() ? 0 : cuts.back().offset) + trimSpacing)
				{
					// A compact file that starts here must start with whole samples:
					// a sync record goes in front of the place. Between clips the
					// next clip's resume record does the same.
					if (compact && !armed)
					{
						encoder.Sync(out);
						if (!out.empty() && !write())
							ended = true;
					}
					const uint64_t bytes = session.bytes.load(std::memory_order_relaxed);
					if (!ended)
					{
						CutPoint point;
						point.offset = bytes;
						point.qpc = QpcNow();
						point.unixMicroseconds = UnixMicrosecondsNow();
						point.state = known;
						point.samplesBefore = samples;
						point.lostBefore = session.lost.load(std::memory_order_relaxed);
						point.clipsBefore = session.clips.load(std::memory_order_relaxed);
						point.secondsBefore = heldSeconds(point.qpc);
						cuts.push_back(std::move(point));
					}
				}
				if (!arrived)
					break;
			}

			// Near the cap the oldest part is dropped, a piece at a time between
			// two looks at the hub, so that recording goes on meanwhile.
			if (keepNewest && session.replaced && !ended && !trimFailed)
			{
				if (!trim.active && session.bytes.load(std::memory_order_relaxed) >= trimStart)
					beginTrim();
				if (trim.active)
				{
					waitForMore = false;
					if (!stepTrim(false))
						ended = true;
				}
			}

			const ULONGLONG tick = GetTickCount64();
			if (!ended && tick - lastDiskCheck >= session.limits.diskCheckIntervalMs)
			{
				lastDiskCheck = tick;
				uint64_t freeBytes = 0;
				// A read that fails proves nothing; the size cap still holds.
				if (session.limits.freeDiskBytes(session.directory, freeBytes) &&
					freeBytes < session.limits.minFreeDiskBytes)
				{
					pause = false;
					reason = recording::EndReason::DiskSpaceLow;
					ended = true;
				}
			}
			if (ended)
				break;
			if (waitForMore)
			{
				std::unique_lock<std::mutex> lock(session.mutex);
				session.wake.wait_for(lock, PollInterval, [&session] { return session.stop; });
			}
		}
	}
	catch (const std::exception &e)
	{
		fail(e.what());
	}
	catch (...)
	{
		fail("unknown exception");
	}

	// The record that closes the stretch, behind whatever was encoded when a
	// limit ended the loop. A recording that never got a sample is not kept.
	// What waited in memory for a clip is dropped here: it was never saved.
	// A drop of the oldest part that is under way is given up: the file it was
	// copying from is the recording, and it is whole.
	abandonTrim();
	bool closed = false;
	if (!writeFailed && session.replaced)
	{
		try
		{
			if (closeQpc == 0)
			{
				closeQpc = QpcNow();
				closeUnixMicroseconds = UnixMicrosecondsNow();
			}
			// What a limit left in an open block still belongs to the file.
			encoder.Flush(out);
			// Between clips the file already ends with a pause record (the last
			// clip's, or the one this stretch went on from): the player's stop
			// has nothing to add to it.
			if (!(pause && armed))
			{
				if (pause)
					recording::AppendPause(out, { static_cast<uint8_t>(recording::PauseCause::Player), closeQpc,
						closeUnixMicroseconds });
				else
					recording::AppendEnd(out, { static_cast<uint8_t>(reason), closeQpc, closeUnixMicroseconds, samples });
			}
			closed = write();
			if (clips && !armed)
			{
				clipTicks += (std::max)(closeQpc - clipStartQpc, static_cast<int64_t>(0));
				session.clips.fetch_add(1, std::memory_order_release);
			}
		}
		catch (const std::exception &e)
		{
			fail(e.what());
		}
	}
	if (session.file != INVALID_HANDLE_VALUE)
	{
		if (closed && !FlushFileBuffers(session.file))
			error = "FlushFileBuffers failed with error " + std::to_string(GetLastError());
		CloseHandle(session.file);
		session.file = INVALID_HANDLE_VALUE;
	}
	if (!session.replaced)
		DeleteFileW(session.newPath.c_str());

	session.samples.store(samples, std::memory_order_release);
	session.clipOpenSince.store(0, std::memory_order_release);
	session.paused = pause;
	session.reason = reason;
	session.saved = session.replaced;
	session.closedCleanly = closed;
	// What the file holds of this stretch: its clips, or the time to the moment
	// recording stopped (not to the moment the file closed).
	const int64_t stretchTicks = clips ? clipTicks : (closeQpc != 0 ? closeQpc : QpcNow()) - session.startQpc;
	session.seconds = session.earlierSeconds - droppedSeconds +
		static_cast<double>(stretchTicks) / static_cast<double>(session.qpcFrequency);
	session.error = error;
	session.done.store(true, std::memory_order_release);
}

} // namespace questcal
