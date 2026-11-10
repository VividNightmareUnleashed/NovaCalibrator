#include "stdafx.h"
#include "PoseRecorder.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
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
constexpr size_t AlignmentRecordBytes = recording::RecordPrefixBytes + 73 + 1 + recording::MaxStringBytes;
constexpr size_t GapRecordBytes = recording::RecordPrefixBytes + recording::GapPayloadBytes;
constexpr size_t BoundaryRecordBytes = recording::RecordPrefixBytes + recording::BoundaryPayloadBytes;
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

} // namespace

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
	int64_t qpcFrequency = 1;
	int64_t startQpc = 0;
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

	std::atomic<uint64_t> bytes{ 0 };     // the file's size
	std::atomic<uint64_t> samples{ 0 };   // the whole recording's
	std::atomic<uint64_t> lost{ 0 };

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
		LARGE_INTEGER frequency{};
		QueryPerformanceFrequency(&frequency);
		session.qpcFrequency = frequency.QuadPart;
		session.earlierSamples = paused_.samples;
		session.earlierSeconds = paused_.seconds;
		session.samples.store(paused_.samples, std::memory_order_release);
		session.lost.store(paused_.lost, std::memory_order_release);

		// One entry per id.
		devices_.clear();
		for (const RecordedDevice &device : devices)
		{
			bool repeated = false;
			for (const recording::DeviceInfo &known : devices_)
				repeated = repeated || known.deviceId == device.deviceId;
			if (!repeated)
				devices_.push_back(Labelled(device));
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

		// What the stretch starts with: a new file's header, which holds as many
		// devices as fit, or a resume record and every device again, so that the
		// stretch reads by itself.
		std::string bytes;
		size_t inHeader = 0;
		if (resume)
			recording::AppendResume(bytes, { static_cast<uint8_t>(recording::PauseCause::Player), session.startQpc,
				UnixMicrosecondsNow(), std::string() });
		else
		{
			recording::Header header;
			header.qpcFrequency = session.qpcFrequency;
			header.startQpc = session.startQpc;
			header.startUnixMicroseconds = UnixMicrosecondsNow();
			header.writer = writer;
			inHeader = (std::min)(devices_.size(), recording::MaxHeaderDevices);
			header.devices.assign(devices_.begin(), devices_.begin() + static_cast<std::ptrdiff_t>(inHeader));
			recording::AppendHeader(bytes, header);
		}
		for (size_t i = 0; i < devices_.size(); ++i)
		{
			if (i >= inHeader)
				recording::AppendDevice(bytes, devices_[i]);
			if (devices_[i].deviceId < session.described.size())
				session.described[devices_[i].deviceId] = true;
		}
		// The calibration in use, so that this stretch too reads by itself.
		if (hasAlignment_)
		{
			alignment_.qpc = session.startQpc;
			recording::AppendAlignment(bytes, alignment_);
		}
		DWORD written = 0;
		if (!WriteFile(file, bytes.data(), static_cast<DWORD>(bytes.size()), &written, nullptr) ||
			written != bytes.size())
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
	for (const RecordedDevice &described : devices)
	{
		const recording::DeviceInfo device = Labelled(described);
		recording::DeviceInfo *known = nullptr;
		for (recording::DeviceInfo &candidate : devices_)
			if (candidate.deviceId == device.deviceId)
				known = &candidate;
		if (known && *known == device)
			continue;
		if (known)
			*known = device;
		else
			devices_.push_back(device);
		changed.push_back(device);
	}
	if (changed.empty())
		return;
	std::lock_guard<std::mutex> lock(session_->mutex);
	session_->pendingDevices.insert(session_->pendingDevices.end(), changed.begin(), changed.end());
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
		return progress;
	}
	const Session &session = *session_;
	progress.recording = !session.done.load(std::memory_order_acquire);
	progress.bytes = session.bytes.load(std::memory_order_acquire);
	progress.samples = session.samples.load(std::memory_order_acquire);
	progress.lost = session.lost.load(std::memory_order_acquire);
	progress.seconds = progress.recording
		? session.earlierSeconds +
			static_cast<double>(QpcNow() - session.startQpc) / static_cast<double>(session.qpcFrequency)
		: session.seconds;
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
	finished.closedCleanly = session.closedCleanly;
	finished.error = session.error;
	paused_ = Paused();
	if (finished.paused && finished.saved && finished.closedCleanly)
		paused_ = { true, finished.path, finished.bytes, finished.samples, finished.lost, finished.seconds };
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
	bool pause = false;
	recording::EndReason reason = recording::EndReason::AppExit;
	int64_t closeQpc = 0, closeUnixMicroseconds = 0;
	std::string error;
	bool writeFailed = false;
	uint64_t samples = session.earlierSamples;
	std::string out;

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
		if (!MoveFileExW(session.newPath.c_str(), session.finalPath.c_str(), MOVEFILE_REPLACE_EXISTING))
			return fail("the previous recording could not be replaced (error " + std::to_string(GetLastError()) + ")");
		session.replaced = true;
		session.file = OpenForAppend(session.finalPath, OPEN_EXISTING);
		if (session.file == INVALID_HANDLE_VALUE)
			return fail("the recording could not be opened again (error " + std::to_string(GetLastError()) + ")");
		return true;
	};
	// Whether a record of this size and the closing record still fit under the cap.
	const auto fits = [&](size_t recordBytes)
	{
		return session.bytes.load(std::memory_order_relaxed) + out.size() + recordBytes + ClosingRecordBytes <=
			session.limits.maxFileBytes;
	};

	// An exception escaping a thread entry ends the process without unwinding
	// (PoseStreamHub::DrainLoop). The buffers here grow with a backlog, so
	// bad_alloc is reachable: it ends the recording, not the program.
	try
	{
		std::vector<protocol::DevicePoseSample> batch;
		ULONGLONG lastDiskCheck = GetTickCount64();
		bool ended = false;
		// The size cap's policy, in this one place: the recording ends, and what
		// it holds stays. (Keeping the newest instead means copying the file's
		// newer part into a new file; docs/recording-format.md, "The size cap".)
		const auto full = [&]()
		{
			pause = false;
			reason = recording::EndReason::SizeLimit;
			ended = true;
		};
		const auto describe = [&](const recording::DeviceInfo &device)
		{
			if (!fits(DeviceRecordBytes))
				return full();
			recording::AppendDevice(out, device);
			if (device.deviceId < session.described.size())
				session.described[device.deviceId] = true;
		};

		while (!ended)
		{
			bool waitForMore = true;
			for (int pass = 0; pass < DrainPasses && !ended; ++pass)
			{
				std::vector<recording::DeviceInfo> changed;
				std::vector<recording::Alignment> aligned;
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
				for (size_t i = 0; i < changed.size() && !ended; ++i)
					describe(changed[i]);
				for (size_t i = 0; i < aligned.size() && !ended; ++i)
				{
					if (!fits(AlignmentRecordBytes))
						full();
					else
					{
						aligned[i].qpc = QpcNow();
						recording::AppendAlignment(out, aligned[i]);
					}
				}
				if (dropped == 0 && batch.empty())
					break;
				waitForMore = false;
				const int64_t now = QpcNow();
				if (newBoundary && !ended)
				{
					if (!fits(BoundaryRecordBytes))
						full();
					else
						recording::AppendBoundary(out, { now, boundaries });
				}
				if (dropped > 0 && !ended)
				{
					if (!fits(GapRecordBytes))
						full();
					else
					{
						recording::AppendGap(out, { now, dropped, hole.size });
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
					if (!fits(SampleRecordBytes))
					{
						full();
						break;
					}
					if (!takePlace())
					{
						ended = true;
						break;
					}
					recording::AppendSample(out, ToSample(pose));
					++samples;
					if (out.size() >= WriteChunkBytes && !write())
						ended = true;
				}
				if (!ended && !out.empty() && !write())
					ended = true;
			}
			// Device records from a pass that brought no samples.
			if (!ended && !out.empty() && !write())
				ended = true;

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
			if (pause)
				recording::AppendPause(out, { static_cast<uint8_t>(recording::PauseCause::Player), closeQpc,
					closeUnixMicroseconds });
			else
				recording::AppendEnd(out, { static_cast<uint8_t>(reason), closeQpc, closeUnixMicroseconds, samples });
			closed = write();
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
	session.paused = pause;
	session.reason = reason;
	session.saved = session.replaced;
	session.closedCleanly = closed;
	// To the moment recording stopped, not to the moment the file closed.
	session.seconds = session.earlierSeconds +
		static_cast<double>((closeQpc != 0 ? closeQpc : QpcNow()) - session.startQpc) /
			static_cast<double>(session.qpcFrequency);
	session.error = error;
	session.done.store(true, std::memory_order_release);
}

} // namespace questcal
