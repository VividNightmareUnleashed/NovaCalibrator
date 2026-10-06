# Base station visibility

Which base station sees a tracker, and when it stops seeing it, is not in the
OpenVR API. SteamVR's lighthouse driver does write it to its own log, one line
per change, per device, as it happens. QuestCalibrator follows that log and
uses it in three places: a figure on every lighthouse device row, a
per-station drop count, and a rule that keeps a tracker's own tracking trouble
out of the drift evidence.

Everything here is read from SteamVR's log file. Nothing is read from the
headset, and nothing changes when the log is missing or unreadable.

The Lighthouse tab is the first feature of the Lighthouse module, which the
installer leaves out unless asked and which will gain more base station
tools (`docs/modules.md`). Reading the log, the session log lines, the
diagnostics section and the drift rule are core and run on every install;
module features build on them.

## Where it comes from

`<Steam>\logs\vrserver.txt`, the Steam folder from the registry
(`HKEY_CURRENT_USER\Software\Valve\Steam\SteamPath`) with the default install
location as the fallback. The lines that matter, with the fixed prefix
`Fri Sep 11 2026 22:08:55.819 [Info] - lighthouse: ` removed:

```
LHR-A3C36EA5 C: SOB: add S-16 also seeing S-5 (D3D4E73B) S-8 (170EE067) S-9 (F210FBA6)
LHR-A3C36EA5 C: SOB: add S-5
LHR-A3C36EA5 C: SOB: add S-9 (generation changed) also seeing S-5 (D3D4E73B) S-16 ( 4D47FB4)
LHR-A3C36EA5 C: SOB: drop S-5 (D3D4E73B) seeing S-8 (170EE067) S-9 (F210FBA6) S-16 ( 4D47FB4)
LHR-A3C36EA5 C: SOB: drop S-8 seeing S-5 S-9 S-16
LHR-A3C36EA5 C: No base stations seen...
LHR-A3C36EA5 C: ----- BOOTSTRAPPED base F210FBA6 (best) distance 2.1m velocity 0.3m/s ... -----
LHR-A3C36EA5 C: Trying to start tracking from base D3D4E73B: Not enough contiguous samples for a bootstrap pose
Device LHR-A3C36EA5 powering off upon entering standby.
LHR-A3C36EA5: Disconnected from receiver 4BF089B604
LHR-A3C36EA5: Connected to receiver 4BF089B604
```

- `LHR-…` is the device's serial, the same string OpenVR returns as its
  serial-number property. That is how a line is joined to a device index.
- `S-N` is a base station's channel. The hex in parentheses is the station's
  id, which is the serial SteamVR keeps for it in its lighthouse database
  (`D3D4E73B` is `3553945403` there). A leading zero is printed as a space.
- `SOB` is sync on beam, the base station 2.0 mode the driver selects at
  startup. Base station 1.0 setups have not been checked; if their lines
  differ they are simply not parsed.
- `add` and `drop` list the stations the device still sees after the change,
  so every line carries the full set. `No base stations seen` and
  `BOOTSTRAPPED` mark a full loss and a solution started from scratch.
- The last three are the device's radio link. `powering off upon entering
  standby` is SteamVR switching the device off after it sat still for its
  **Turn off controllers after** time (Startup / Shutdown settings, 5 minutes
  unless changed). A headset tracker sits still whenever the headset is off,
  and stays off until someone turns it back on. These set the device's state
  and add a session-log line (`LHR-A3C36EA5 switched off by SteamVR after
  sitting still`, `... connected again`); they are not disturbances, since an
  off device reports no pose to disturb.

Four more lines name no device:

```
Selected existing universe 1744988537 (170EE067 is primary)
Creating new universe 1744988537 because there were no existing universes
Stopped tracking with universe 1744988537
vrserver 2.17.10 startup with PID=5976, ...   (the server's own first line)
```

The universe is the set of base station poses every lighthouse pose is
reported in. SteamVR has none when it starts, and none again after it stopped
tracking with one because no device was left tracking. The first devices to
track are reported in the frame of the station each started from, placed
where the driver guesses, until it chooses a universe a few seconds later; it
then moves those stations to where they stand in it. The server's start also
forgets the devices of the session before: `vrserver.txt` is not always
rotated when SteamVR starts, and one file held five days of sessions.

The file is re-opened on every poll (four times a second) and never held, so
SteamVR can rename it at its next start. The first poll replays up to the
last 4 MB of the existing file to learn the current sets; those lines are
marked historical and only ever set the sets and the counters. A file that
shrank was rotated: SteamVR restarted, and the sets and counters start over.

## What is shown

- **On the Lighthouse page** (with the Lighthouse module installed): one card per station (channel, id, how many of
  the switched-on devices see it, its drops this session), then one row per
  lighthouse device with a dot per station, filled while it is in view, and
  the in-view figure (`3 of 4 in view`, red for one or none) with the
  device's drop count. Hovering a row lists the stations by channel and id
  and the last change. A station no device sees is outlined in red: that is
  the one to look at. The pair sheet's device tiles carry none of this.
  `-uipreview-lighthouse` opens the page in the preview.
- **In the session log:** one line per disturbance (`LHR-A3C36EA5 down to one
  station S-9 (F210FBA6) after losing S-16 (04D47FB4)`), and, with detailed
  logging on, one line per routine handoff.
- **In a diagnostics export:** a `[base stations]` section with all of the
  above.

## What it changes

The drift monitor scores two things as evidence that the two universes have
moved apart: a stationary device that slides more than 12 mm across an 8 s
rest window, and a device that reappears after a short absence more than
25 cm from where it vanished. A lighthouse device does both on its own when
it loses stations: with a single station the optical solution has one
baseline and can swim along that station's line of sight; with none it coasts
on the IMU and then reports itself out of range; when a station returns the
solution snaps back.

So a drift-monitor event on a device is attributed to its base stations, and
not counted, while the device is **degraded** (fewer than two stations in
view, or no solution at all; a state, so lines replayed at startup set it
too) and for the ten seconds after a **disturbance** of that device (or up
to one second before one, since a log line can be stamped just after the pose
it explains). A disturbance is any of:

- the device fell to a single station, or to none;
- the device saw no station at all (`No base stations seen`);
- a solution was bootstrapped, or a bootstrap failed;
- the device came back from none or one station to two or more.

A station leaving or joining a solution that keeps two or more is counted but
is not a disturbance: the fit keeps a second baseline and the pose barely
moves. Attributed events are logged with the reason (`Device 7 slid 1.4 cm
after it down to one station S-9 (F210FBA6) ... -- its base stations, not
drift`) and summed in the diagnostics.

Cost: a genuine slide that happens to fall inside such a window is missed
once. In the session below, disturbances covered a few percent of any one
device's time.

A calibration uses the same state before it measures: it waits until a
lighthouse device in the pair is neither degraded nor within ten seconds of a
disturbance, until fifteen seconds after its last new solution (a solution
still settling was measured 1.2 deg off on 2026-09-26), and until ten seconds
after SteamVR chose its universe. It starts on its own once that holds, and
after 30 s measures a device that tracks but never settled.

Continuous calibration reads the headset tracker's lines the same way: no
verdict while it settles, and a pause within two minutes of one of its
disturbances is put down to that. Only a `BOOTSTRAPPED` line starts a
solution from scratch; when the solution it starts reads the same deviation
as the one blamed, the blame is lifted (see [how it works](how-it-works.md)).
A station coming or going, even the first back after none was in view,
leaves the solution and its bias in place.

The universe decides one more thing. A station SteamVR re-solves carries
every device in its frame. Each affected device receives its own inverse
frame correction (see [how it works](how-it-works.md)); a stationary device
in another frame receives none. While the universe is set up this is skipped:
the move places a station SteamVR guessed at startup, and a saved calibration
belongs to the universe, not to the guess. On 2026-09-27 the first station
was placed 26 ms after the universe was chosen (`Moving base F210FBA6 1205mm
and 42.7 deg`), carrying four trackers 120 cm and 30.5 deg of yaw, and two
more stations moved 0.5 and 1.3 m in the next 2.5 s. So no frame move is
followed from SteamVR's start until ten seconds after it chose a universe.
When the log names no universe (it is unreadable, or the part replayed at
startup holds none), the first fifteen seconds after the lighthouse devices
start tracking stand in for it.

Lighthouse devices are target-side devices, so none of this touches the
universe-jump detector, which only reads the reference system's stream.

Frame corrections are runtime state, separate from the shared calibration.
Complete driver updates retain them; ordinary tracking loss and return do too.
A new device starts with no inferred correction: identical current frames do
not establish identical history. A reused slot with a different serial, a new
profile, and a new SteamVR pose session clear the relevant state. A full
recalibration within the same active profile keeps the per-device corrections: its
raw solve is expressed in the existing normalized space. The frame watch keeps
running during the measurement, including when it is cancelled. Repairing a
disabled or unsafe profile starts a fresh solve; it does not wait for that
profile's stopped frame monitor.

An overlay restart reads the last complete checkpoint from the running driver
before publishing a replacement. The profile's calibration timestamp, tracking
systems and HMD identify its target space; physical serial keys identify the
trackers. Sleeping trackers retain their checkpoint until they enumerate,
including when a different device occupies their previous index. Those saved
entries remain inactive until their serial is identified; a replacement device
never inherits the correction. Temporary calibration
neutralization does not overwrite the checkpoint. Every restored publication
is pinned to the driver's session ID, so reconnecting to a new SteamVR process
cannot replay old corrections. No frame matrices are persisted across SteamVR
sessions. Corrections cannot recover disagreement that existed before observation,
or infer frame changes during an interval when the overlay was closed.

The raw pose ring stays unchanged. Continuous calibration waits for the frame
watch to examine each pose, then normalizes the mounted tracker before solving
or looking up its field correction. Samples older than the latest compensated
move are discarded when the observation window restarts. The diagnostic export
includes the live per-device corrections; each frame event records the serial,
old and new frames, and desired correction. Detailed logging also captures raw
and runtime poses after notable moves, coalesced to at most one capture every
two seconds. These are asynchronous samples, not an exact before/after pair.
The driver separately logs the actual transformed pose before hiding a device
whenever its frame correction changes; the export includes the latest 256 KiB
of that driver log. Runtime snapshots include valid disconnected physical
trackers alongside virtual devices such as Standable's outputs.
IPC protocol 10 requires updating
the overlay and driver together and restarting SteamVR.

## One session, in numbers

`vrserver.txt` from 2026-09-11, eight lighthouse devices, about eighty
minutes:

| Measure | Value |
| --- | --- |
| Drop and re-add pairs | 141 |
| Median time a station stayed lost | 9.4 s |
| Longest ten percent of losses | over 85 s |
| Drops on channel 16 (04D47FB4) | 59 |
| Drops on channel 5 (D3D4E73B) | 48 |
| Drops on channel 8 (170EE067) | 23 |
| Drops on channel 9 (F210FBA6) | 11 |

Most drops left three stations in view, which is why they never appeared as
tracking loss. `No base stations seen` appeared once.

## The simulated tracker

`Tests/VirtualLighthouse.h` generates the pose stream and the log lines of one
tracker whose stations come and go on a schedule, so the drift monitor and
the visibility state can be run together exactly as the overlay runs them.
The line sequence it writes for a change is the one the real driver writes
(drops one at a time listing what remains, `No base stations seen` when the
set empties, a bootstrap when it refills, adds one at a time), and the
scenarios check that the parser and the model agree.

Assumed, not measured, and named as such in its configuration: the
single-station swim (3 mm/s along the station's line of sight, capped at
3 cm), how long a coasting pose stays valid (0.2 s) and its bias
(0.3 m/s²). The scenarios only need those failures to exist, not their exact
size.

Scenarios (`RunLighthouseScenarios`):

- a slide with four stations in view stays drift evidence;
- the same slide while down to one station is raised by the monitor, every
  time it recurs, and attributed to the station; it would have counted
  without the log;
- a 0.6 s blackout during a 40 cm reposition is raised as a re-localization
  and attributed to the lost stations;
- lines replayed at startup set the state but open no window: a genuine
  slide just after a replayed return to four stations counts, the same slide
  after a live return is attributed;
- the parser accepts every line shape above and rejects the driver's other
  lines; the tailer replays history, follows appends, completes a half-written
  line once and survives a rotation;
- the 2026-09-27 startup, read as the overlay read it: the universe is being
  set up from the server's start until ten seconds after it was chosen (the
  station placed 26 ms after it included), settled after, down again when
  SteamVR stopped tracking with it, and unnamed when a minute passes with
  devices tracking and no universe chosen; a choice replayed when the overlay
  starts counts from its stamp, unlike the per-device lines.

## Limits

- An undocumented log format of a closed driver. The parser is tolerant and a
  changed format degrades to "no information", but it would go unnoticed until
  the figure stops appearing.
- Log lines are not synchronized to the pose stream. The ten-second window and
  the one-second tolerance cover the lag seen so far; per-frame gating is out
  of reach this way.
- The headset is not a lighthouse device and never appears.
- Whether SteamVR flushes the file promptly on every machine is not something
  the parser can check; the session log's first lighthouse line says whether
  anything is being read at all.
