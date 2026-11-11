# Modules

Most people install Nova Calibrator for the classic space calibrator, with
its improvements. Features the original calibrator never had, and that a
player may not want, ship as optional modules: the installer asks about each
one, and a plain install leaves them out.

A module is part of the same executable. It is not loaded or left out at
build time; the installer records which ones the player chose and the
overlay only turns on what was chosen.

## The modules

| Module | Status | What it controls |
| --- | --- | --- |
| Lighthouse | optional | Base station tools. So far one: the Lighthouse tab, each base station and which lighthouse devices see it (`docs/lighthouse-visibility.md`). More are planned. |
| Smoothing | not built | Its tab only, greyed out with "Work in progress". No installer offers it yet. |
| Motion Recorder | optional | For development and playtests. Its page records the driver's pose samples to one file (`docs/recording-format.md`) while the player has a recording on, as clips around tracking problems ("Automatic") or all of them ("Full"), and plays that file back. A recording shows how the player's body moves. |

What a module does not control stays core. Reading SteamVR's lighthouse
log, its session log lines, the `[base stations]` diagnostics section and
the rule that keeps a tracker's own base station changes out of the drift
evidence all run on every install, with or without the Lighthouse module:
they make calibration better whether or not the player looks at the tab.

## Module or core

Lighthouse is slim today and will grow, and so will any module after it.
Each new feature is sorted by one question: does it make the classic
calibrator better for someone who never asked for the module?

- **Core** if yes, and it changes nothing a classic user would notice
  beyond better calibration: shared data (the lighthouse log reader and
  `LighthouseVisibility`), rules that clean up calibration evidence, and
  diagnostics, which support needs from every install.
- **The module** if it is something the player sees or does that the
  original calibrator had not: new screens, controls, settings, overlays,
  notifications, or anything that acts on SteamVR's base stations. It
  checks `Modules::On(CalCtx.modules.<name>)` everywhere it shows up, not
  only on its tab.

Module features build on the core data; core never depends on a module.

## How a choice travels

1. `Install.ps1` asks about each module before it changes anything. On an
   upgrade the default is the previous install's choice, otherwise no.
   `-Lighthouse` installs the module without asking; an `-Unattended`
   install gets only the modules named on its command line. The Motion
   Recorder is asked about differently, because of what it records: the
   answer is no unless the player presses Y within 10 seconds, on an upgrade
   too, and no without a console to ask on. `-MotionRecorder` installs it
   without asking.
2. It records each choice as a DWORD, 1 or 0, under
   `HKLM\Software\NovaCalibrator\Modules` (values `Lighthouse` and
   `MotionRecorder`). Running the installer again is how a player adds or
   removes a module.
3. At startup the overlay reads that key once (`ReadInstalledModules`,
   `Overlay/Modules.cpp`) into `CalCtx.modules`. A missing key or value
   means not installed. Everything a module gates asks `CalCtx.modules`.
4. `Uninstall.ps1` removes `HKLM\Software\NovaCalibrator` and the choices
   with it.

A manual install has no installer to ask; the install readme gives the
`reg add` line that turns a module on.

## What the player sees

- A module's page in the sidebar is greyed out until it is installed.
  Hovering it says the module is not installed and to select it during
  installation; a module that is not built is marked "Soon" and says "Work in
  progress".
- A diagnostics export has a `[modules]` section with each module's
  status (`installed`, `not installed`, `not built`).
- The preview never reads the registry: the Lighthouse module is on, as the
  design shows the sidebar, and the Motion Recorder module is on only in the
  `-uipreview-recorder` screens.

## The Motion Recorder

Everything of it is gated on `Modules::On(CalCtx.modules.motionRecorder)`:
without the module the overlay makes no recorder thread, opens no file and
shows only the greyed-out page link. With it, nothing is recorded until the
player presses "Start recording" on the page and reads the warning; the
choice is not a setting, so no recording runs when Nova Calibrator starts.

- `Overlay/PoseRecorder.h` is one more consumer of the pose stream hub, on
  its own thread: a slow disk delays neither the hub nor the window, and
  samples it cannot fetch in time are a gap record. There is one recording,
  `%LOCALAPPDATA%\NovaCalibrator\recordings\NovaCalibrator-recording.novapose`.
  A stop closes it with a pause record and the next start goes on in it; a new
  recording replaces it when its first sample is written. Nothing published
  while the recorder is off reaches the file. The file names no hardware: a
  device has a label made up for the recording in place of its serial number.
- The player picks one of two modes beside the start button, "Automatic"
  unless they pick the other. "Full" writes every sample. "Automatic" writes
  nothing until something is marked: the last 30 seconds wait in the
  recorder's memory, a mark writes them as the start of a clip, and the clip
  ends 10 seconds after its last mark (`PoseRecordingLimits::clips`). "Save
  the last 30 seconds" on the page is the player's own mark. In both modes a
  mark is a record at its moment, which the playback shows on its timeline.
- `Overlay/RecordingTriggers.h` decides what is marked, from what Nova
  Calibrator's monitors already count (a universe jump, a move of the
  lighthouse frame, a device that slid or came out of a loss somewhere else,
  continuous calibration stopped or aligned again) and from SteamVR's poses (a
  device that had tracking for two seconds and lost it). `MotionRecorder.cpp`
  shows it those numbers once a tick while a recording runs; nothing in the
  calibration code calls the recorder.
- `Overlay/RecordingPlayback.h` reads the file once on its own thread to find
  its stretches, losses and record positions, then reads only the span around
  the moment shown. The file is untrusted input (fuzz target `recording`).
- `Overlay/SceneView.h` draws the devices as a 3D picture into a texture of
  its own: SteamVR's render model for each device at its position and
  orientation, a plain shape where SteamVR has no model, and a floor grid. The
  page turns its camera by a drag and moves it by the wheel. A model's name
  comes from the recording, so only a plain name is given to SteamVR.
- `Overlay/MotionRecorder.h` is the module's side of the overlay,
  `Overlay/UiMotionRecorder.cpp` its page, its two dialogs and the sign in the
  sidebar while a recording runs. Every sentence is in
  `Overlay/MotionRecorderText.h`.
- At the size cap (1, 5 or 20 GB, picked on the page) the recorder drops the
  oldest part of the file and goes on (`PoseRecordingLimits::keepNewest`;
  `docs/recording-format.md`, "The size cap"). A drive with less than 4 GB
  free ends the recording.
- The mode and the size cap are saved settings (`motion_recorder_full` and
  `motion_recorder_cap` in the settings record). Whether a recording runs is
  never saved.
- A new recording is a compact one (format version 2 of
  `docs/recording-format.md`: position to a micrometer, rotation to a
  microradian, a Quest headset's numbers bit for bit; about 0.17 GB an hour
  for nine devices). With `"motion_recorder_raw": true` in the settings
  record it keeps every sample as the driver sent it (format version 1, about
  2.5 GB an hour). No screen has a control for it. A recording that goes on
  after a stop stays in the format it began in, and
  `tools/novapose-convert.cpp` writes a recording again in the other one.

## Adding a module

1. A `ModuleStatus` field in `questcal::Modules` (`Overlay/Modules.h`),
   `NotBuilt` until it ships.
2. Read its registry value in `ReadInstalledModules`.
3. Gate what it owns on `Modules::On(CalCtx.modules.<name>)`; a page gets a
   `Page` value (`Overlay/UiInternal.h`) and a `ui::NavRow` in `BuildSidebar`
   (`Overlay/UserInterface.cpp`) with its not-installed tooltip, translated in
   every table.
4. A question and a switch in `Install.ps1`, a line in the install readme,
   a line in the `[modules]` diagnostics section, and a row above.
