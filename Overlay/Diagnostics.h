#pragma once

// The diagnostics file a player attaches to a bug report, written on request
// from Settings and again as Nova Calibrator exits: version, settings and
// profile summary, recent activity and the session logs, with the account name,
// profile directories and computer name replaced. Device serials stay: they
// identify hardware, not people.

#include <openvr.h>
#include <array>
#include <string>
#include "PoseStreamHub.h"
#include "DriverSyncTracker.h"

struct CalibrationContext;
namespace vr { class IVRSystem; }

// The same snapshot goes to the export and the throttled detailed log.
// qpcNow uses the raw pose clock, or zero when that clock is unavailable.
std::string DescribeContinuousDiagnostics(const CalibrationContext &ctx, double qpcNow);
struct DiagnosticCapture
{
	PoseStreamHub::Diagnostics poseStream;
	questcal::DriverSyncTracker driverSync;
	double sampleClock = 0.0;
	std::string steamVrRuntimePath;
	float steamVrWorldScale = 0.0f;
	vr::EVRSettingsError worldScaleError = vr::VRSettingsError_ReadFailed;
	// The lighthouse frame each device, base stations aside, was last seen in
	// (LighthouseFrameWatch::FrameGroup): devices in one frame share the lowest
	// id among them, and a device with no frame is -1.
	std::array<int, vr::k_unMaxTrackedDeviceCount> frameGroup = [] {
		std::array<int, vr::k_unMaxTrackedDeviceCount> none{};
		none.fill(-1);
		return none;
	}();
};
// queryRuntime false leaves out what only SteamVR can answer (its world scale
// setting), for a SteamVR that is gone.
DiagnosticCapture CaptureCalibrationDiagnostics(bool queryRuntime = true);
std::string DescribeFrameMoveSnapshot(const CalibrationContext &ctx, vr::IVRSystem *system,
	const DiagnosticCapture &capture, double qpcToSeconds);

// Why a file is written. Every kind but a request ends its file name in its
// own words ("-steamvr-quit"...), and each kind keeps its own newest ten, so a
// week of sessions never pushes out a file the player saved on purpose, nor
// routine exits a crash's.
enum class DiagnosticsTrigger
{
	Request,       // Settings, "Save diagnostics file"
	SteamVrQuit,   // SteamVR's quit event
	SteamVrCrash,  // SteamVR's server process ended without the quit event
	WindowClosed,  // the player closed the window
	FatalError,    // Nova Calibrator stops on an error it shows
};

// How the file's first line and the session log say why it was written:
// "as SteamVR quit"; empty for a request.
const char *DiagnosticsTriggerText(DiagnosticsTrigger trigger);

// Writes the file under %LOCALAPPDATA%\NovaCalibrator\diagnostics. On
// success pathOut is the file's UTF-8 path; on failure error says why.
bool WriteDiagnosticsFile(const CalibrationContext &ctx, std::string &pathOut, std::string &error,
	vr::IVRSystem *system, const DiagnosticCapture &capture,
	DiagnosticsTrigger trigger = DiagnosticsTrigger::Request);

// As Nova Calibrator exits, for any trigger but a request: flushes pending
// profile and settings writes first, as SteamVR ends a process that is slow to
// quit, then writes the file, so a player who leaves without saving one still
// has it. system is null when SteamVR is gone. Failures, thrown ones too, go to
// the session log only.
void SaveDiagnosticsAtExit(DiagnosticsTrigger trigger, vr::IVRSystem *system);

// Opens Explorer with the file selected, so "send this file" needs no search.
void RevealInExplorer(const std::string &utf8Path);

// The substitution the file applies, exposed for the test harness.
std::string AnonymiseDiagnosticsText(const std::string &text,
	const std::string &userProfileDir, const std::string &userName, const std::string &computerName);

// A path for the session log and the screen: under %LOCALAPPDATA% or
// %USERPROFILE% it is written with the variable, which still pastes into
// Explorer but carries no account name. The raw log gets shared as often as
// the diagnostics file.
std::string PathForLog(const std::string &utf8Path);
std::string ShortenUserPath(const std::string &path,
	const std::string &localAppData, const std::string &userProfileDir);
