#include "stdafx.h"
#include "TrackingTrace.h"
#include "TrackingTraceWriter.h"
#include "../common/PoseRingReader.h"
#include <memory>

namespace questcal::stagetrace
{
namespace
{
int64_t Clock()
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	return now.QuadPart;
}
struct Handle
{
	HANDLE h;
	~Handle() { if (h) CloseHandle(h); }
};
// The actual interval is stored for every runtime read. A late poll never
// causes a burst of queries with made-up times.
void WaitUntil(HANDLE timer, int64_t target, int64_t frequency)
{
	const auto ticks = target - Clock();
	if (ticks <= 0) return;
	LARGE_INTEGER due;
	due.QuadPart = -(std::max)(1ll, ticks * 10000000 / frequency);
	if (timer && SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE))
		WaitForSingleObject(timer, 15);
	else
		Sleep(1);
}
void Describe(Writer &out, vr::IVRSystem &system, uint32_t id)
{
	char name[256]{};
	vr::ETrackedPropertyError error;
	system.GetStringTrackedDeviceProperty(id, vr::Prop_ControllerType_String, name, sizeof(name), &error);
	if (error != vr::TrackedProp_Success) name[0] = 0;
	name[sizeof(name) - 1] = 0;
	auto role = system.GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &error);
	if (error != vr::TrackedProp_Success) role = 0;
	out.Describe(id, system.GetTrackedDeviceClass(id), role, name);
}
}

void Capture::Start(vr::IVRSystem *system, const std::wstring &directory)
{
	if (Status() == Result::Working) return;
	Close();
	if (!system || directory.empty())
	{
		result.store(Result::Failed);
		return;
	}
	stopAt.store(0);
	result.store(Result::Working);
	try
	{
		worker = std::thread([this, system, directory] {
			try { result.store(Run(*system, directory), std::memory_order_release); }
			catch (...) { result.store(Result::Failed, std::memory_order_release); }
		});
	}
	catch (...) { result.store(Result::Failed); }
}
void Capture::Stop()
{
	int64_t unset = 0;
	stopAt.compare_exchange_strong(unset, Clock());
}
void Capture::Close()
{
	Stop();
	if (worker.joinable()) worker.join();
}

Result Capture::Run(vr::IVRSystem &system, const std::wstring &directory)
{
	Handle request{ CreateEventA(nullptr, TRUE, FALSE, Request) };
	if (!request.h || GetLastError() == ERROR_ALREADY_EXISTS) return Result::Failed;
	ULARGE_INTEGER free{};
	if (!GetDiskFreeSpaceExW(directory.c_str(), &free, nullptr, nullptr) || free.QuadPart < (256ull << 20))
		return Result::Failed;
	LARGE_INTEGER hz;
	QueryPerformanceFrequency(&hz);
	const int64_t start = Clock();
	FILETIME utc;
	GetSystemTimePreciseAsFileTime(&utc);
	const uint64_t unixUs = (((uint64_t{utc.dwHighDateTime} << 32) | utc.dwLowDateTime) - 116444736000000000ull) / 10;
	SYSTEMTIME date;
	GetLocalTime(&date);
	wchar_t leaf[128];
	swprintf_s(leaf, L"\\%ls%04u%02u%02u-%02u%02u%02u-%lld%ls", FilePrefix,
		date.wYear, date.wMonth, date.wDay, date.wHour, date.wMinute, date.wSecond, start, FileSuffix);
	RemoveOldCaptures(directory, KeptCaptures - 1);
	const std::wstring path = directory + leaf;
	FILE *file = nullptr;
	if (_wfopen_s(&file, path.c_str(), L"wbx") || !file) return Result::Failed;
	// A capture that did not reach its closing record leaves no file. Declared
	// before the file's owner, so the file is closed when this removes it.
	struct Unsaved
	{
		const std::wstring &path;
		bool saved = false;
		~Unsaved() { if (!saved) DeleteFileW(path.c_str()); }
	} unsaved{ path };
	std::unique_ptr<FILE, decltype(&fclose)> owner(file, &fclose);
	Writer out(file, hz.QuadPart, start, unixUs);
	if (!out.good || !SetEvent(request.h)) return Result::Failed;
	protocol::PoseRingReader reader;
	Handle timer{ CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS) };
	const int64_t period = (std::max)(1ll, hz.QuadPart / 120);
	const int64_t deadline = start + 30 * hz.QuadPart;
	uint64_t connected = 0, driverCount = 0, runtimeCount = 0, epoch = 0;
	std::vector<protocol::DevicePoseSample> batch;
	batch.reserve(1024);
	int64_t nextOpen = start, nextPoll = start;
	bool first = true;
	Result reason = Result::Complete;
	for (;;)
	{
		const auto stop = stopAt.load();
		const auto cutoff = stop ? (std::min)(stop, deadline) : deadline;
		const auto now = Clock();
		if (!reader.IsOpen() && now >= nextOpen)
		{
			if (reader.Open(Ring)) epoch = reader.SessionEpoch();
			nextOpen = now + hz.QuadPart / 20;
		}
		if (reader.IsOpen())
		{
			batch.clear();
			uint64_t overflow = 0;
			const auto status = reader.Drain([&](const protocol::DevicePoseSample &s) {
				if (s.sampleTimeQpc < start || s.sampleTimeQpc > cutoff) return;
				if (batch.size() < protocol::PoseRing::Capacity) batch.push_back(s);
				else ++overflow;
			}, [&](uint64_t lost) {
				if (batch.size() >= protocol::PoseRing::Capacity) { overflow += lost; return; }
				auto &gap = batch.emplace_back();
				gap.deviceId = UINT32_MAX; // unused stage tag 7
				gap.sampleTimeQpc = static_cast<int64_t>(lost);
			});
			for (const auto &s : batch)
			{
				if (s.deviceId == UINT32_MAX) out.Gap(now, static_cast<uint64_t>(s.sampleTimeQpc));
				else
				{
					const auto stopped = stopAt.load();
					if (stopped && s.sampleTimeQpc > stopped) continue;
					out.Driver(s);
					if (Kind(s.deviceId) == Stage::Submitted) ++driverCount;
				}
				if (!out.good) break;
			}
			if (overflow) out.Gap(now, overflow);
			if (status != protocol::PoseRingReader::DrainStatus::Drained || reader.SessionEpoch() != epoch)
			{ reason = Result::DriverEnded; break; }
		}
		if (!out.good) { reason = Result::Failed; break; }
		if (out.written >= (128ull << 20)) { reason = Result::SizeLimit; break; }
		if (now >= cutoff) { reason = stop ? Result::Stopped : Result::Complete; break; }
		if (!driverCount && now - start > 5 * hz.QuadPart) { reason = Result::DriverMissing; break; }
		vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
		const auto begin = Clock();
		system.GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);
		const auto end = Clock();
		const auto stopped = stopAt.load();
		if (end <= deadline && (!stopped || end <= stopped))
		{
			uint64_t current = 0;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				if (!poses[id].bDeviceIsConnected && !poses[id].bPoseIsValid) continue;
				const uint64_t bit = uint64_t{1} << id;
				current |= bit;
				if (!(connected & bit)) Describe(out, system, id);
			}
			out.Runtime(begin, end, connected | current, poses);
			if (current) ++runtimeCount;
			connected = current;
			const auto origin = system.GetRawZeroPoseToStandingAbsoluteTrackingPose();
			out.Origin(Clock(), origin, first);
			first = false;
		}
		nextPoll = (std::max)(nextPoll + period, end);
		WaitUntil(timer.h, nextPoll, hz.QuadPart);
	}
	ResetEvent(request.h);
	if ((reason == Result::Complete || reason == Result::Stopped) && (!driverCount || !runtimeCount))
		reason = !driverCount ? Result::DriverMissing : Result::Failed;
	const auto stopped = stopAt.load();
	out.End((std::min)(Clock(), stopped ? (std::min)(stopped, deadline) : deadline), static_cast<uint8_t>(reason));
	unsaved.saved = fclose(owner.release()) == 0 && out.good;
	return unsaved.saved ? reason : Result::Failed;
}
}
