#pragma once

// Every sentence the Motion Recorder module shows a player, in English, in
// this one place, so that the wording can be read and changed as a whole. A
// change here is a change to the keys in lang/ja.txt and lang/it.txt
// (Tests/PoseRecordingTests.cpp fails when a string here has no entry there).
// The text follows ASD-STE100 as the rest of Nova Calibrator's does.

namespace questcal::motionrecorder::text
{

inline constexpr const char *PageTitle = "Motion Recorder";

// The row that starts and stops a recording.
inline constexpr const char *RowTitle = "Record motion";
inline constexpr const char *PrivacyLabel = "Privacy risk";
inline constexpr const char *LineFirst =
	"A recording shows how your body moves. Nova Calibrator records only after you start a recording.";
inline constexpr const char *LineContinues =
	"A recording shows how your body moves. The recording continues in the same file.";
inline constexpr const char *LineReplaces = "A recording shows how your body moves. A new recording replaces this one.";
// Minutes, then GB.
inline constexpr const char *StatusPattern = "The recording is %d min long. It uses %.2f GB.";
// Minutes. While a recording of clips runs.
inline constexpr const char *OnForPattern = "The recording is on for %d min.";
// The two modes, and what each saves.
inline constexpr const char *ModeAutomatic = "Automatic";
inline constexpr const char *ModeFull = "Full";
inline constexpr const char *AutomaticLine =
	"A clip starts when a tracking problem occurs. Between clips, nothing is saved.";
inline constexpr const char *FullLine = "Nova Calibrator saves all motion until you stop the recording.";
inline constexpr const char *SaveLast = "Save the last 30 seconds";
inline constexpr const char *StartButton = "Start recording";
inline constexpr const char *StopButton = "Stop recording";
inline constexpr const char *Sign = "Motion recording is on";

// The warning a player reads before a new recording starts. One entry is one
// paragraph; the sheet shows them in this order.
inline constexpr const char *const Disclaimer[] = {
	"A recording shows how your body moves, almost as a motion capture does.",
	"Record only when you accept that another person can see your movements.",
	"A recording is the most useful data for a problem report.",
	"You can stop and start the recording at any time.",
	"Nova Calibrator records only after you start a recording and until you stop it.",
	"A recording does not change the diagnostics file.",
	"A recording stays on your computer until you send it.",
	"A new recording replaces the previous recording. To keep the previous recording, copy the file first.",
};
inline constexpr const char *CancelButton = "Cancel";

// The size cap.
inline constexpr const char *CapTitle = "Maximum recording size";
inline constexpr const char *CapLine =
	"One hour of recording can use 3 GB. When the file has this size, Nova Calibrator deletes the oldest part of "
	"the recording.";

// The file.
inline constexpr const char *OpenFolder = "Open recording folder";
inline constexpr const char *DeleteRecording = "Delete recording";
inline constexpr const char *DeleteTitle = "Delete the recording?";
inline constexpr const char *DeleteLine =
	"Nova Calibrator deletes the recording file from this computer. You cannot get the file back.";
inline constexpr const char *KeepButton = "Keep recording";

// Playback. RecordedPattern takes a date and a time; DevicesPattern a count,
// the plural's "s" and GB.
inline constexpr const char *RecordedPattern = "Recorded %s";
inline constexpr const char *DevicesPattern = "%d device%s, %.2f GB";
inline constexpr const char *ViewHint = "Drag to turn the view";
inline constexpr const char *LegendRecorded = "Recorded";
inline constexpr const char *LegendOff = "Off: nothing recorded";
inline constexpr const char *LegendLost = "Data lost";
inline constexpr const char *LegendEvent = "Event";

// What a mark in a recording says, by its name in the file
// (recording::markname, in that order).
inline constexpr const char *const MarkLines[] = {
	"You saved this clip.",
	"The headset tracking space moved.",
	"A base station moved.",
	"A device lost tracking.",
	"A stationary device changed position.",
	"A device got tracking again in a different position.",
	"Continuous calibration stopped.",
	"Continuous calibration aligned the trackers again.",
};
inline constexpr const char *Live = "Live";
inline constexpr const char *LiveLine = "Nova Calibrator records now. Stop the recording to play it.";
inline constexpr const char *Reading = "Nova Calibrator reads the recording.";
inline constexpr const char *NoRecordingTitle = "No recording";
inline constexpr const char *NoRecordingLine = "Start a recording. Then you can play it here.";
inline constexpr const char *BadFileTitle = "Nova Calibrator cannot read the recording";
inline constexpr const char *BadFileLine = "The recording file is not correct. Delete the recording or start a new recording.";
inline constexpr const char *NoSamplesLine = "The recording has no data.";
inline constexpr const char *CutShortLine = "The recorder stopped before it closed the file. The end of the recording can be missing.";

// What a recorded device is called beside its dot.
inline constexpr const char *const DeviceNames[] = {
	"Headset", "Left hand", "Right hand", "Waist", "Chest", "Left foot", "Right foot", "Left knee", "Right knee",
	"Left elbow", "Right elbow", "Left shoulder", "Right shoulder", "Tracker", "Controller",
};

// Messages. The two that end in a path are the start of a sentence: the path
// follows (and, after CannotCreateIn, a period).
inline constexpr const char *SavedTo = "Nova Calibrator saved the recording to ";
inline constexpr const char *CannotCreateIn = "Nova Calibrator could not create the recording file in ";
inline constexpr const char *StoppedAtCap = "The recording stopped because the file reached its maximum size.";
inline constexpr const char *StoppedDiskFull = "The recording stopped because the disk is almost full.";
inline constexpr const char *WriteFailed =
	"Nova Calibrator could not write to the recording file. The recording stopped.";
inline constexpr const char *DiskTooFull =
	"Nova Calibrator did not start the recording. The disk must have more than 4 GB of free space.";
inline constexpr const char *NoData = "Nova Calibrator did not save a recording because it received no data.";
inline constexpr const char *NoClips = "No tracking problem occurred. Nova Calibrator saved no clip.";
inline constexpr const char *Deleted = "Nova Calibrator deleted the recording.";
inline constexpr const char *CannotDelete = "Nova Calibrator could not delete the recording file.";

// Every string above that is shown as it stands, for the test that each has a
// translation. The warning's paragraphs, the device names, the
// patterns and the two sentences that end in a path are checked beside it. A
// new string goes here.
inline constexpr const char *const Whole[] = {
	PageTitle, RowTitle, PrivacyLabel, LineFirst, LineContinues, LineReplaces, StartButton, StopButton, Sign,
	CancelButton, CapTitle, CapLine, OpenFolder, DeleteRecording, DeleteTitle, DeleteLine, KeepButton,
	ViewHint, LegendRecorded, LegendOff, LegendLost, Live, LiveLine, Reading, NoRecordingTitle, NoRecordingLine,
	BadFileTitle, BadFileLine, NoSamplesLine, CutShortLine, StoppedAtCap, StoppedDiskFull, WriteFailed,
	DiskTooFull, NoData, Deleted, CannotDelete, ModeAutomatic, ModeFull, AutomaticLine, FullLine, SaveLast,
	LegendEvent, NoClips,
};

} // namespace questcal::motionrecorder::text
