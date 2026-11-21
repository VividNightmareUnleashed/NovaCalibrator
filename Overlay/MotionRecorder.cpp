// The Motion Recorder module without its drawing: the one recorder, the one
// playback of its file, and what they tell the player and the log. Recording
// is the player's own act each time: nothing here is a saved setting, so no
// recording runs when Nova Calibrator starts.
#include "stdafx.h"
// The overlay's headers first: they bring in openvr.h, and the hub's protocol
// header (under PoseRecorder.h) then leaves the driver's OpenVR header out.
#include "Calibration.h"
#include "CalibrationDriver.h"
#include "Diagnostics.h"
#include "UserInterface.h"
#include "../common/Version.h"

#include "MotionRecorder.h"
#include "TrackingTrace.h"
#include "MotionRecorderText.h"
#include "PoseRecorder.h"
#include "RecordingTriggers.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace questcal::motionrecorder
{

namespace
{

using recording::Playback;

PoseRecorder Recorder;
stagetrace::Capture StageCapture;
Playback ThePlayer;
// The overlay's hub, as the first tick hands it over; null in a preview and
// for as long as the module is not installed.
PoseStreamHub *Hub = nullptr;
bool On = false;
// The mode of the recording that runs.
Mode Running = DefaultMode;
static_assert(CapChoiceCount == MotionRecorderCapChoices && DefaultCapChoice == MotionRecorderCapDefault,
	"the settings record saves the cap as one of these choices");
ULONGLONG OnSinceTick = 0;
// What was marked since the start, for the page, and what looks for more.
std::vector<Event> EventList;
constexpr size_t MaxEvents = 64;
recording::TriggerWatch Watch;
bool FileThere = false;
double DeviceScan = -1e9;
double FileScan = -1e9;
// The devices as the last scan found them, for the live view.
std::vector<RecordedDevice> LiveDevices;
// What the player has open: the file's size and time of writing.
bool PlayerOpen = false;
uint64_t PlayerFileBytes = 0;
uint64_t PlayerFileTime = 0;
double PlayerScan = -1e9;
// How long a close waits for the file. A write that takes longer leaves a
// file without its closing record, which the readers report.
constexpr unsigned CloseWaitMs = 5000;

bool Installed()
{
	return Modules::On(CalCtx.modules.motionRecorder);
}

std::string Utf8FromWide(const std::wstring &wide)
{
	if (wide.empty())
		return std::string();
	const int bytes = WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), nullptr, 0,
		nullptr, nullptr);
	if (bytes <= 0)
		return std::string();
	std::string out(static_cast<size_t>(bytes), '\0');
	WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), &out[0], bytes, nullptr, nullptr);
	return out;
}

bool Exists(const std::wstring &path)
{
	const DWORD attributes = GetFileAttributesW(path.c_str());
	return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

// What the overlay knows of each device SteamVR lists. A property SteamVR
// does not give stays empty. The serial goes no further than the recorder,
// which puts a label made up for the recording in its place.
std::vector<RecordedDevice> Devices()
{
	std::vector<RecordedDevice> devices;
	vr::IVRSystem *system = vr::VRSystem();
	if (!system)
		return devices;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		const vr::ETrackedDeviceClass deviceClass = system->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;
		RecordedDevice device;
		device.deviceId = id;
		device.deviceClass = static_cast<uint32_t>(deviceClass);
		ReadTrackedDeviceString(id, vr::Prop_SerialNumber_String, device.serial);
		ReadTrackedDeviceString(id, vr::Prop_ModelNumber_String, device.model);
		ReadTrackedDeviceString(id, vr::Prop_TrackingSystemName_String, device.trackingSystem);
		// SteamVR's role for a tracker ("vive_tracker_waist") is its controller type.
		ReadTrackedDeviceString(id, vr::Prop_ControllerType_String, device.controllerType);
		ReadTrackedDeviceString(id, vr::Prop_RenderModelName_String, device.renderModel);
		vr::ETrackedPropertyError roleError = vr::TrackedProp_Success;
		const int32_t role = system->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &roleError);
		if (roleError == vr::TrackedProp_Success)
			device.controllerRole = role;
		devices.push_back(std::move(device));
	}
	return devices;
}

// The calibration the driver gives the target system's devices now. The
// recording keeps it beside the samples, which stay as the driver made them,
// so that a playback can put both tracking systems in one space.
recording::Alignment AlignmentNow(const CalibrationContext &ctx)
{
	recording::Alignment alignment;
	alignment.applied = ctx.enabled && ctx.validProfile;
	alignment.trackingSystem = ctx.targetTrackingSystem;
	alignment.rotation[0] = ctx.transform.rotation.w();
	alignment.rotation[1] = ctx.transform.rotation.x();
	alignment.rotation[2] = ctx.transform.rotation.y();
	alignment.rotation[3] = ctx.transform.rotation.z();
	for (int i = 0; i < 3; ++i)
		alignment.translation[i] = ctx.transform.translationMeters[i];
	alignment.scale = ctx.transform.scale;
	return alignment;
}

// A mark in the recording, in the page's list and in the session log.
void MarkNow(const char *name, const RecordedDevice *device, bool keepAfter)
{
	Recorder.Mark(name, device ? device->deviceId : recording::NoDevice, keepAfter);
	Event event;
	event.unixSeconds = static_cast<int64_t>(std::time(nullptr));
	event.name = name;
	if (device)
	{
		event.hasDevice = true;
		event.device.deviceId = device->deviceId;
		event.device.deviceClass = device->deviceClass;
		event.device.controllerRole = device->controllerRole;
		event.device.controllerType = device->controllerType;
	}
	if (EventList.size() >= MaxEvents)
		EventList.erase(EventList.begin());
	EventList.push_back(std::move(event));
	AppendSessionLog(std::string("motion recording mark: ") + name +
		(device ? ", device " + std::to_string(device->deviceId) : std::string()));
}

// What Nova Calibrator's monitors count, for the watch that turns a change
// into a mark.
recording::TriggerCounts CountsNow(const CalibrationContext &ctx)
{
	recording::TriggerCounts counts;
	counts.universeJumps = ctx.jumpsCompensated;
	counts.baseStationMoves = ctx.lighthouseFrameMoves;
	counts.deviceSlides = ctx.driftSlideEvents;
	counts.trackingJumps = ctx.discontinuousLossEvents;
	counts.realignments = ctx.continuousReanchors;
	counts.calibrationStopped = ctx.continuousState == ContinuousAlignment::State::Frozen;
	return counts;
}

// The session log always; the player only for a stretch that ended while
// Nova Calibrator stays open.
void Report(CalibrationContext &ctx, const PoseRecorder::Finished &finished, bool tellPlayer)
{
	using recording::EndReason;
	const std::string path = PathForLog(Utf8FromWide(finished.path));
	char numbers[192];
	snprintf(numbers, sizeof numbers, ", %.1f s, %llu samples, %llu lost, %llu bytes", finished.seconds,
		static_cast<unsigned long long>(finished.samples), static_cast<unsigned long long>(finished.lost),
		static_cast<unsigned long long>(finished.bytes));
	AppendSessionLog(std::string("motion recording ") +
		(finished.paused ? "stopped by the player" : std::string("ended (") +
			recording::EndReasonText(static_cast<uint8_t>(finished.reason)) + ")") + ": " + path + numbers +
		(finished.saved ? "" : ", no sample arrived and no file was kept") +
		(finished.saved && !finished.closedCleanly ? ", no closing record" : "") +
		(finished.trims > 0 ? ", oldest part dropped " + std::to_string(finished.trims) + " time(s)" : "") +
		(finished.replaceRetries > 0 ? ", waited " + std::to_string(finished.replaceRetries) +
			" time(s) for another program to let go of the file" : "") +
		(finished.error.empty() ? "" : " [" + finished.error + "]"));
	FileThere = Exists(finished.path);
	PlayerScan = -1e9;
	if (!tellPlayer)
		return;
	if (!finished.paused && finished.reason == EndReason::WriteError)
		ctx.ReportError(std::string(text::WriteFailed) + "\n");
	else if (!finished.saved && finished.clipMode)
		// Nothing went wrong: no clip was called for.
		ctx.Tell(text::NoClips, CalibrationContext::Tone::Neutral);
	else if (!finished.saved)
		ctx.Tell(text::NoData, CalibrationContext::Tone::Warn);
	else if (!finished.paused && finished.reason == EndReason::SizeLimit)
		ctx.Tell(text::StoppedAtCap, CalibrationContext::Tone::Warn);
	else if (!finished.paused && finished.reason == EndReason::DiskSpaceLow)
		ctx.Tell(text::StoppedDiskFull, CalibrationContext::Tone::Warn);
	if (finished.saved)
		ctx.Tell(text::SavedTo + path, CalibrationContext::Tone::Good);
}

// A stretch that closed since the last look, the player's own stop included.
void TakeFinished(CalibrationContext &ctx, bool tellPlayer)
{
	PoseRecorder::Finished finished;
	if (!Recorder.TakeFinished(finished))
		return;
	On = false;
	Report(ctx, finished, tellPlayer);
}

// The player lets go of the file: a new recording replaces it, a resumed one
// is appended to it, and neither works on a file that is open.
void ClosePlayer()
{
	ThePlayer.Close();
	PlayerOpen = false;
	PlayerScan = -1e9;
}

// A file stream with a buffer large enough to read hours of recording in
// seconds. The buffer is a base so that it outlives the stream.
struct ReadBuffer
{
	std::vector<char> bytes = std::vector<char>(1u << 20);
};
struct RecordingFile : private ReadBuffer, public std::ifstream
{
	explicit RecordingFile(const std::wstring &path)
	{
		rdbuf()->pubsetbuf(bytes.data(), static_cast<std::streamsize>(bytes.size()));
		open(path, std::ios::in | std::ios::binary);
	}
};

// ---------------------------------------------------------------------------
// The preview's recording: a player who moves for twelve seconds, with one
// stop, a clip behind it, two marks and one loss, so that every part of the
// page has something to show.

struct PreviewBody
{
	uint32_t id;
	uint32_t deviceClass;
	int32_t role;
	const char *system;
	const char *type;
};
constexpr PreviewBody PreviewBodies[] = {
	{ 0, 1, 0, "oculus", "" },
	{ 1, 2, 1, "lighthouse", "knuckles" },
	{ 2, 2, 2, "lighthouse", "knuckles" },
	{ 3, 3, 0, "lighthouse", "vive_tracker_waist" },
	{ 4, 3, 0, "lighthouse", "vive_tracker_left_foot" },
	{ 5, 3, 0, "lighthouse", "vive_tracker_right_foot" },
};

void PreviewPosition(uint32_t id, double t, double position[3])
{
	const double sway = 0.05 * std::sin(0.8 * t), step = std::sin(2.0 * t);
	switch (id)
	{
	case 0: position[0] = sway; position[1] = 1.62 + 0.01 * std::sin(2.0 * t); position[2] = 0.03 * std::cos(0.6 * t); break;
	case 1: position[0] = -0.34 - 0.14 * std::sin(1.7 * t); position[1] = 1.06 + 0.26 * std::sin(1.3 * t); position[2] = -0.18; break;
	case 2: position[0] = 0.36 + 0.12 * std::sin(1.5 * t + 1.0); position[1] = 1.16 + 0.30 * std::sin(1.1 * t + 2.0); position[2] = -0.22; break;
	case 3: position[0] = 0.8 * sway; position[1] = 0.98; position[2] = 0.0; break;
	case 4: position[0] = -0.14; position[1] = 0.08 + 0.07 * (std::max)(0.0, step); position[2] = 0.06 * step; break;
	default: position[0] = 0.14; position[1] = 0.08 + 0.07 * (std::max)(0.0, -step); position[2] = -0.06 * step; break;
	}
}

std::string PreviewRecording()
{
	constexpr int64_t frequency = 10000000;
	std::string bytes;
	recording::Header header;
	header.qpcFrequency = frequency;
	header.startQpc = 1000;
	// 2026-11-09 21:41:00 UTC.
	header.startUnixMicroseconds = 1794260460ll * 1000000ll;
	header.writer = "Nova Calibrator preview";
	for (const PreviewBody &body : PreviewBodies)
	{
		recording::DeviceInfo device;
		device.deviceId = body.id;
		device.deviceClass = body.deviceClass;
		device.controllerRole = body.role;
		device.trackingSystem = body.system;
		device.controllerType = body.type;
		header.devices.push_back(device);
	}
	recording::AppendHeader(bytes, header);
	const auto stretch = [&](double from, double seconds, int64_t baseQpc, double lossAt)
	{
		bool lost = false;
		for (int tick = 0; tick <= static_cast<int>(seconds * 100.0); ++tick)
		{
			const double into = tick / 100.0;
			if (lossAt >= 0.0 && !lost && into >= lossAt)
			{
				lost = true;
				recording::AppendGap(bytes, { baseQpc + static_cast<int64_t>(into * frequency), 240, 240 });
			}
			for (const PreviewBody &body : PreviewBodies)
			{
				recording::Sample sample;
				sample.qpc = baseQpc + static_cast<int64_t>(into * frequency);
				sample.deviceId = body.id;
				sample.trackingResult = body.id == 4 && from + into >= 3.95 && from + into < 4.25
					? vr::TrackingResult_Calibrating_OutOfRange : vr::TrackingResult_Running_OK;
				sample.poseValid = true;
				sample.connected = true;
				PreviewPosition(body.id, from + into, sample.position);
				recording::AppendSample(bytes, sample);
			}
		}
	};
	stretch(0.0, 5.5, 1000, -1.0);
	recording::AppendMark(bytes, { 1000 + 55 * frequency / 10, header.startUnixMicroseconds + 5500000, 3,
		recording::markname::TrackingLost });
	stretch(5.5, 1.5, 1000 + 55 * frequency / 10, -1.0);
	recording::AppendPause(bytes, { 1, 1000 + 7 * frequency, header.startUnixMicroseconds + 7000000 });
	recording::AppendResume(bytes, { 2, 1000 + 120 * frequency, header.startUnixMicroseconds + 120000000,
		recording::markname::BaseStationMoved });
	stretch(7.0, 3.0, 1000 + 120 * frequency, 1.4);
	recording::AppendMark(bytes, { 1000 + 123 * frequency, header.startUnixMicroseconds + 123000000, recording::NoDevice,
		recording::markname::BaseStationMoved });
	stretch(10.0, 2.0, 1000 + 123 * frequency, -1.0);
	recording::AppendPause(bytes, { 2, 1000 + 125 * frequency, header.startUnixMicroseconds + 125000000 });
	return bytes;
}

bool PreviewHasFile()
{
	return g_uiPreviewScenario == PreviewScenario::Recorder || g_uiPreviewScenario == PreviewScenario::RecorderLive ||
		g_uiPreviewScenario == PreviewScenario::RecorderLiveFull || g_uiPreviewScenario == PreviewScenario::RecorderDelete;
}

bool PreviewIsLive()
{
	return g_uiPreviewScenario == PreviewScenario::RecorderLive || g_uiPreviewScenario == PreviewScenario::RecorderLiveFull;
}

} // namespace

std::wstring Folder()
{
	wchar_t base[MAX_PATH];
	const DWORD len = GetEnvironmentVariableW(L"LOCALAPPDATA", base, MAX_PATH);
	if (len == 0 || len >= MAX_PATH)
		return std::wstring();
	return std::wstring(base, len) + L"\\NovaCalibrator\\recordings";
}

Status Now()
{
	Status status;
	if (g_uiPreviewMode)
	{
		// A recording twelve minutes in, or one that is there to play.
		status.on = PreviewIsLive();
		status.fileExists = PreviewHasFile();
		status.seconds = 754.0;
		status.bytes = 569ull << 20;
		status.mode = g_uiPreviewScenario == PreviewScenario::RecorderLiveFull ? Mode::Full : ModeChoice();
		status.onSeconds = 754.0;
		return status;
	}
	const PoseRecorder::Progress progress = Recorder.GetProgress();
	status.on = On && progress.recording;
	status.fileExists = FileThere;
	status.seconds = progress.seconds;
	status.bytes = progress.bytes;
	status.mode = status.on ? Running : ModeChoice();
	status.onSeconds = status.on ? static_cast<double>(GetTickCount64() - OnSinceTick) / 1000.0 : 0.0;
	return status;
}

const std::vector<Event> &Events()
{
	if (g_uiPreviewMode && g_uiPreviewScenario == PreviewScenario::RecorderLive && EventList.empty())
	{
		// 2026-11-09, 22:41 and 22:47 on the player's clock.
		struct tm at{};
		at.tm_year = 126;
		at.tm_mon = 10;
		at.tm_mday = 9;
		at.tm_hour = 22;
		at.tm_min = 41;
		at.tm_isdst = -1;
		Event moved;
		moved.unixSeconds = static_cast<int64_t>(mktime(&at));
		moved.name = recording::markname::BaseStationMoved;
		Event lost;
		lost.unixSeconds = moved.unixSeconds + 6 * 60;
		lost.name = recording::markname::TrackingLost;
		lost.hasDevice = true;
		lost.device.deviceId = 3;
		lost.device.deviceClass = 3;
		lost.device.controllerType = "vive_tracker_waist";
		EventList = { moved, lost };
	}
	return EventList;
}

Mode ModeChoice()
{
	return CalCtx.motionRecorderFull ? Mode::Full : Mode::Automatic;
}

void SaveLast()
{
	if (!On || g_uiPreviewMode || Running != Mode::Automatic)
		return;
	MarkNow(recording::markname::Player, nullptr, false);
}

int CapChoice()
{
	const int choice = CalCtx.motionRecorderCap;
	return choice >= 0 && choice < CapChoiceCount ? choice : DefaultCapChoice;
}

bool BeginsNewRecording()
{
	const std::wstring folder = Folder();
	return folder.empty() || !Recorder.CanResume(folder);
}

bool Start()
{
	if (!Hub || g_uiPreviewMode || !Installed())
		return false;
	auto &ctx = CalCtx;
	// A stretch the player stopped a moment ago may still be closing.
	if (Recorder.GetProgress().recording)
		Recorder.StopAndWait(recording::EndReason::AppExit, CloseWaitMs);
	TakeFinished(ctx, true);
	ClosePlayer();

	const std::wstring folder = Folder();
	std::string detail = "Windows names no local application data folder";
	auto result = PoseRecorder::StartResult::NoFolder;
	if (!folder.empty())
	{
		// The session log made this folder already, unless it could not.
		CreateDirectoryW(folder.substr(0, folder.rfind(L'\\')).c_str(), nullptr);
		LiveDevices = Devices();
		PoseRecordingLimits limits;
		limits.maxFileBytes = CapChoicesBytes[CapChoice()];
		// At the cap the oldest part goes and the recording goes on.
		limits.keepNewest = true;
		limits.clips = ModeChoice() == Mode::Automatic;
		// A new recording is compact unless the settings record asks for raw.
		limits.compact = !ctx.motionRecorderRaw;
		Recorder.SetAlignment(AlignmentNow(ctx));
		result = Recorder.Start(*Hub, folder, std::string("Nova Calibrator ") + QUESTCAL_VERSION_STRING, LiveDevices,
			limits, &detail);
		if (result == PoseRecorder::StartResult::Started || result == PoseRecorder::StartResult::Resumed)
		{
			On = true;
			Running = ModeChoice();
			OnSinceTick = GetTickCount64();
			EventList.clear();
			Watch.Reset();
			DeviceScan = -1e9;
			AppendSessionLog(std::string("motion recording ") +
				(result == PoseRecorder::StartResult::Resumed ? "resumed: " : "started: ") +
				PathForLog(Utf8FromWide(PoseRecorder::FilePath(folder))) + ", " + std::to_string(LiveDevices.size()) +
				" device(s), cap " + std::to_string(limits.maxFileBytes >> 30) + " GiB, " +
				(limits.clips ? "clips" : "every sample") + (limits.compact ? ", compact asked for" : ", raw asked for"));
			return true;
		}
	}
	AppendSessionLog("motion recording not started: " + detail);
	// The sentence names the floor.
	static_assert(PoseRecordingLimits::DefaultMinFreeDiskBytes == 4ull << 30,
		"MotionRecorderText.h's DiskTooFull names the free-space floor");
	if (result == PoseRecorder::StartResult::DiskSpaceLow)
		ctx.ReportError(std::string(text::DiskTooFull) + "\n");
	else
		ctx.ReportError(text::CannotCreateIn + PathForLog(Utf8FromWide(folder)) + ".\n");
	return false;
}

void Pause()
{
	On = false;
	Recorder.RequestPause();
}

bool StageCaptureActive()
{
	return StageCapture.Status() == stagetrace::Result::Working;
}

void ToggleStageCapture()
{
	if (g_uiPreviewMode || !Installed())
		return;
	if (StageCaptureActive())
	{
		StageCapture.Stop();
		return;
	}
	const std::wstring folder = Folder();
	if (!folder.empty())
	{
		CreateDirectoryW(folder.substr(0, folder.rfind(L'\\')).c_str(), nullptr);
		CreateDirectoryW(folder.c_str(), nullptr);
	}
	StageCapture.Start(vr::VRSystem(), folder);
}

// Says once how a capture began and how it ended, as the recorder does of
// its file. The capture has a thread of its own; this is the tick's look at it.
void ReportStageCapture(CalibrationContext &ctx)
{
	using stagetrace::Result;
	static Result told = Result::Idle;
	const Result status = StageCapture.Status();
	if (status == told)
		return;
	told = status;
	switch (status)
	{
	case Result::Working:
		AppendSessionLog("tracking comparison: capture started");
		ctx.Tell(text::TraceWorking, CalibrationContext::Tone::Neutral);
		break;
	case Result::Complete:
	case Result::Stopped:
		AppendSessionLog(status == Result::Complete ? "tracking comparison: saved after 30 seconds"
			: "tracking comparison: saved, stopped by the player");
		ctx.Tell(text::TraceSaved, CalibrationContext::Tone::Good);
		break;
	case Result::DriverMissing:
	case Result::DriverEnded:
		AppendSessionLog(status == Result::DriverMissing ? "tracking comparison: the driver sent no poses"
			: "tracking comparison: the driver's poses ended early");
		ctx.Tell(text::TraceDriver, CalibrationContext::Tone::Warn);
		break;
	case Result::SizeLimit:
		AppendSessionLog("tracking comparison: stopped at the size limit");
		ctx.Tell(text::TraceLimit, CalibrationContext::Tone::Warn);
		break;
	case Result::Failed:
		AppendSessionLog("tracking comparison: capture failed");
		ctx.ReportError(std::string(text::TraceFailed) + "\n");
		break;
	case Result::Idle:
		break;
	}
}

void DeleteRecording()
{
	if (g_uiPreviewMode)
		return;
	auto &ctx = CalCtx;
	const std::wstring folder = Folder();
	if (folder.empty())
		return;
	// The file must be closed first. A stretch in progress ends here.
	On = false;
	Recorder.StopAndWait(recording::EndReason::AppExit, CloseWaitMs);
	TakeFinished(ctx, false);
	Recorder.Forget();
	ClosePlayer();
	const std::wstring path = PoseRecorder::FilePath(folder);
	DeleteFileW((path + L".new").c_str());
	DeleteFileW((path + L".keep").c_str());
	const bool gone = DeleteFileW(path.c_str()) != 0 || GetLastError() == ERROR_FILE_NOT_FOUND;
	FileThere = Exists(path);
	if (gone)
	{
		AppendSessionLog("motion recording deleted by the player");
		ctx.Tell(text::Deleted, CalibrationContext::Tone::Neutral);
	}
	else
	{
		AppendSessionLog("motion recording not deleted: error " + std::to_string(GetLastError()));
		ctx.ReportError(std::string(text::CannotDelete) + "\n");
	}
}

void Tick(CalibrationContext &ctx, PoseStreamHub &hub, double time)
{
	if (!Modules::On(ctx.modules.motionRecorder))
		return;
	ReportStageCapture(ctx);
	Hub = &hub;
	// A stretch that a limit, a failed write or the player's stop closed.
	TakeFinished(ctx, true);
	if (time - FileScan >= 1.0)
	{
		FileScan = time;
		const std::wstring folder = Folder();
		FileThere = !folder.empty() && Exists(PoseRecorder::FilePath(folder));
	}
	// Once a second, the devices as SteamVR lists them now and the calibration
	// in use.
	if (On && vr::VRSystem() && time - DeviceScan >= 1.0)
	{
		DeviceScan = time;
		LiveDevices = Devices();
		Recorder.SetDevices(LiveDevices);
		Recorder.SetAlignment(AlignmentNow(ctx));
	}
	// What is worth finding again in the recording: a clip starts at it, or a
	// full recording gets a mark.
	if (On && vr::VRSystem())
	{
		static vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, poses,
			vr::k_unMaxTrackedDeviceCount);
		std::vector<recording::TriggerDevice> bodies;
		for (const RecordedDevice &known : LiveDevices)
		{
			const bool body = known.deviceClass == static_cast<uint32_t>(vr::TrackedDeviceClass_HMD) ||
				known.deviceClass == static_cast<uint32_t>(vr::TrackedDeviceClass_Controller) ||
				known.deviceClass == static_cast<uint32_t>(vr::TrackedDeviceClass_GenericTracker);
			if (!body || known.deviceId >= vr::k_unMaxTrackedDeviceCount)
				continue;
			const vr::TrackedDevicePose_t &pose = poses[known.deviceId];
			bodies.push_back({ known.deviceId, pose.bPoseIsValid && pose.bDeviceIsConnected &&
				pose.eTrackingResult == vr::TrackingResult_Running_OK });
		}
		std::vector<recording::Trigger> triggers;
		Watch.Step(time, CountsNow(ctx), bodies, triggers);
		for (const recording::Trigger &trigger : triggers)
		{
			const RecordedDevice *about = nullptr;
			for (const RecordedDevice &known : LiveDevices)
				if (trigger.deviceId != recording::NoDevice && known.deviceId == trigger.deviceId)
					about = &known;
			MarkNow(trigger.name, about, true);
		}
	}
}

void Close(bool steamVrEnded)
{
	StageCapture.Close();
	using recording::EndReason;
	const bool closed = Recorder.StopAndWait(steamVrEnded ? EndReason::SteamVrQuit : EndReason::AppExit, CloseWaitMs);
	On = false;
	PoseRecorder::Finished finished;
	if (Recorder.TakeFinished(finished))
		Report(CalCtx, finished, false);
	else if (!closed)
		AppendSessionLog("motion recording: the file did not close in time and has no closing record");
	ClosePlayer();
}

recording::Playback &Player()
{
	return ThePlayer;
}

void KeepPlayerCurrent(double time)
{
	if (g_uiPreviewMode)
	{
		if (!PlayerOpen && g_uiPreviewScenario != PreviewScenario::RecorderEmpty &&
			g_uiPreviewScenario != PreviewScenario::RecorderWarning)
		{
			PlayerOpen = true;
			const auto bytes = std::make_shared<const std::string>(PreviewRecording());
			ThePlayer.Open([bytes] {
				return std::unique_ptr<std::istream>(new std::istringstream(*bytes, std::ios::in | std::ios::binary));
			}, 569ull << 20, true);
		}
		return;
	}
	// A file that is being written is not played.
	if (On || Recorder.GetProgress().recording)
	{
		if (PlayerOpen)
			ClosePlayer();
		return;
	}
	if (time - PlayerScan < 1.0)
		return;
	PlayerScan = time;
	const std::wstring folder = Folder();
	WIN32_FILE_ATTRIBUTE_DATA data{};
	const std::wstring path = folder.empty() ? std::wstring() : PoseRecorder::FilePath(folder);
	if (path.empty() || !GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &data) ||
		(data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
	{
		if (PlayerOpen)
			ClosePlayer();
		return;
	}
	const uint64_t bytes = (static_cast<uint64_t>(data.nFileSizeHigh) << 32) | data.nFileSizeLow;
	const uint64_t written = (static_cast<uint64_t>(data.ftLastWriteTime.dwHighDateTime) << 32) |
		data.ftLastWriteTime.dwLowDateTime;
	if (PlayerOpen && bytes == PlayerFileBytes && written == PlayerFileTime)
		return;
	PlayerOpen = true;
	PlayerFileBytes = bytes;
	PlayerFileTime = written;
	ThePlayer.Open([path] { return std::unique_ptr<std::istream>(new RecordingFile(path)); }, bytes);
}

void LivePoses(double time, std::vector<recording::Playback::DevicePose> &devices)
{
	devices.clear();
	if (g_uiPreviewMode)
	{
		ThePlayer.Frame(3.0, devices);
		return;
	}
	vr::IVRSystem *system = vr::VRSystem();
	if (!system)
		return;
	static vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);
	// Each device's path over the last moments, by id.
	static std::map<uint32_t, std::vector<std::pair<double, std::array<float, 3>>>> trails;
	for (const RecordedDevice &known : LiveDevices)
	{
		if (known.deviceId >= vr::k_unMaxTrackedDeviceCount)
			continue;
		const vr::TrackedDevicePose_t &pose = poses[known.deviceId];
		auto &trail = trails[known.deviceId];
		size_t old = 0;
		while (old < trail.size() && trail[old].first < time - Playback::TrailSeconds)
			++old;
		trail.erase(trail.begin(), trail.begin() + static_cast<std::ptrdiff_t>(old));
		const bool tracked = pose.bPoseIsValid && pose.bDeviceIsConnected;
		if (!tracked && trail.empty())
			continue;
		Playback::DevicePose out;
		out.info.deviceId = known.deviceId;
		out.info.deviceClass = known.deviceClass;
		out.info.controllerRole = known.controllerRole;
		out.info.trackingSystem = known.trackingSystem;
		out.info.controllerType = known.controllerType;
		out.renderModel = known.renderModel;
		out.tracked = tracked;
		out.limited = tracked && pose.eTrackingResult != vr::TrackingResult_Running_OK;
		if (tracked)
		{
			const std::array<float, 3> at = { pose.mDeviceToAbsoluteTracking.m[0][3],
				pose.mDeviceToAbsoluteTracking.m[1][3], pose.mDeviceToAbsoluteTracking.m[2][3] };
			trail.push_back({ time, at });
		}
		for (int i = 0; i < 3; ++i)
			out.position[i] = trail.back().second[static_cast<size_t>(i)];
		if (tracked)
		{
			// The pose's rotation part, as the quaternion the picture takes.
			const auto &m = pose.mDeviceToAbsoluteTracking.m;
			const Eigen::Matrix3d turn = (Eigen::Matrix3d() << m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2],
				m[2][0], m[2][1], m[2][2]).finished();
			const Eigen::Quaterniond q(turn);
			out.rotation[0] = q.w();
			out.rotation[1] = q.x();
			out.rotation[2] = q.y();
			out.rotation[3] = q.z();
		}
		out.trail.reserve(trail.size());
		for (const auto &entry : trail)
			out.trail.push_back(entry.second);
		devices.push_back(std::move(out));
	}
}

} // namespace questcal::motionrecorder
