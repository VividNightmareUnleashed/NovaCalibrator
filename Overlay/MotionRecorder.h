#pragma once

// The Motion Recorder module (docs/modules.md): the one recorder and the one
// playback of its file, as the overlay and the module's page use them.
// MotionRecorder.cpp is the part without drawing; UiMotionRecorder.cpp draws
// the page, the two dialogs and the sign in the sidebar. Nothing here runs
// unless the module is installed, and a recording is the player's own act
// each time: no setting starts one.

#include "RecordingPlayback.h"

#include <cstdint>
#include <string>
#include <vector>

struct CalibrationContext;
struct ImVec2;
class PoseStreamHub;

namespace questcal::motionrecorder
{

// The sizes a player picks the cap from, and the one picked until they do.
constexpr uint64_t CapChoicesBytes[] = { 1ull << 30, 5ull << 30, 20ull << 30 };
constexpr int CapChoiceCount = 3;
constexpr int DefaultCapChoice = 1;

struct Status
{
	bool on = false;           // recording now
	bool fileExists = false;   // there is a recording file to play, open or delete
	double seconds = 0.0;      // the recording's length, pauses left out
	uint64_t bytes = 0;
};
Status Now();

// Whether a start would begin a new recording, which replaces the previous
// one, and not go on with the one that was stopped.
bool BeginsNewRecording();
// Starts or goes on. When it cannot, the error banner tells the player why.
bool Start();
// The player's stop: nothing published after this returns is recorded.
void Pause();
// Stops any recording and deletes the file.
void DeleteRecording();
std::wstring Folder();
int CapChoice();
void SetCapChoice(int choice);

// The overlay's two calls: once a tick while SteamVR runs, and when SteamVR
// or Nova Calibrator ends, before the diagnostics are written.
void Tick(CalibrationContext &ctx, PoseStreamHub &hub, double time);
void Close(bool steamVrEnded);

// The recording file read for playback. The page calls this while it shows:
// it opens the file when there is one and it is not being written, and opens
// it again after it changed.
recording::Playback &Player();
void KeepPlayerCurrent(double time);
// Where SteamVR has each device now, for the page while a recording runs.
void LivePoses(double time, std::vector<recording::Playback::DevicePose> &devices);

// UiMotionRecorder.cpp.
void OpenWarning();
void OpenDeleteConfirm();
void BuildDialog();
// A sign on every page for as long as a recording runs.
void BuildSidebarSign(const ImVec2 &min, const ImVec2 &max);

} // namespace questcal::motionrecorder
