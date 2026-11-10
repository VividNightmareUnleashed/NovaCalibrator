# The pose recording format

Format version 1, raw mode. File extension `.novapose`.

This document is the contract for a pose recording. A reader written from it
alone, in any language, is correct when it reports for each file in
`Tests/recordings/` what the `.txt` file beside it says.
`Overlay/PoseRecordingFormat.h` is the same layout as C++ and needs only the
standard library; `tools/read-recording.py` is a second reader written from
this document. The Motion Recorder module (`docs/modules.md`) writes the file
and plays it back (`Overlay/RecordingPlayback.h`).

A recording holds every pose the driver captured while recording was on: for
each tracked device, its position, rotation and velocities as SteamVR's driver
reported them, before any calibration, with the capture time. It shows how the
player's body moved. It holds no serial number, no device name the player
typed, no account or computer name, and nothing recorded while recording was
off. It is not encrypted or compressed: what is in the file is what this
document says, and anyone can check it.

## Rules for every field

- Every number is little-endian. Nothing is padded or aligned: fields follow
  each other byte after byte.
- `u8`, `u16`, `u32`, `u64` are unsigned integers of 1, 2, 4 and 8 bytes.
  `i32` and `i64` are two's-complement signed integers.
- `f64` is an IEEE-754 double, stored as its 64 bits. A reader must keep the
  bits: NaN payloads and negative zero are data, and they are what the driver
  sent.
- A `string` is a `u8` length followed by that many bytes of UTF-8. It has no
  terminator and is at most 255 bytes. A writer may cut a longer text at 255
  bytes, so the last character of a 255-byte string may be incomplete.
- A "count" is a value of the machine's performance counter
  (`QueryPerformanceCounter`). The header gives its rate.
- A "time" is a wall-clock time in microseconds since 1970-01-01 00:00:00 UTC.

## The file

```
header
record
record
...
```

The header comes once. Records follow until the end of the file. There is no
index, no footer and no checksum; the file is written front to back and each
record is complete before the next begins, so a file is readable up to its
last whole record at every moment, a crash included.

### Header

| Field | Type | Meaning |
| --- | --- | --- |
| magic | 8 bytes | The ASCII text `NOVAPOSE` |
| format version | u32 | 1 |
| mode | u32 | 1, raw |
| clock rate | i64 | Counts per second. Greater than zero |
| start count | i64 | The count when the recording began |
| start time | i64 | The wall-clock time when the recording began |
| device count | u32 | The number of device entries below, 0 to 64 |
| writer | string | The program and version that wrote the file |
| devices | device entry, `device count` times | The devices known when the recording began |

A reader refuses a file when the magic is different, when the format version
is not one it knows, when the mode is not 1, when the clock rate is zero or
negative, or when the device count is over 64. A format version above 1 is a
layout this document does not describe: a version 1 reader must refuse it, not
guess.

### Device entry

| Field | Type | Meaning |
| --- | --- | --- |
| id | u32 | The device's OpenVR index. Samples name their device by it |
| class | u32 | `vr::ETrackedDeviceClass`: 1 headset, 2 controller, 3 tracker, 4 base station, 5 display redirect |
| controller role | i32 | `vr::ETrackedControllerRole`: 0 none, 1 left hand, 2 right hand |
| label | string | Eight hexadecimal digits made up for this recording |
| model | string | The model name SteamVR gives, such as `VIVE Tracker 3.0 MV` |
| tracking system | string | The tracking system SteamVR gives, such as `lighthouse` or `oculus` |
| controller type | string | SteamVR's controller type. For a tracker it is the role the player gave it in SteamVR, such as `vive_tracker_waist` |

The label stands where a serial number would. It is random, it is the same
for one physical device through the whole recording (also when the device
comes back under another id, and across pauses), and another recording gives
the same device another label. A string SteamVR did not give is empty.

### Record

Every record starts with the same three bytes:

| Field | Type | Meaning |
| --- | --- | --- |
| kind | u8 | Which record this is. Never 0 |
| size | u16 | The number of payload bytes that follow |
| payload | `size` bytes | Depends on the kind |

| Kind | Name | Payload size |
| --- | --- | --- |
| 1 | sample | 186 |
| 2 | gap | 24 |
| 3 | boundary | 16 |
| 4 | device | 16 to 1036 |
| 5 | end | 25 |
| 6 | pause | 17 |
| 7 | resume | 18 to 273 |
| 8 | alignment | 74 to 329 |
| 9 | model | 5 to 260 |

A reader skips a record whose kind it does not know by its size and goes on.
A reader refuses the file from that record on when a kind is 0, when a known
kind has a payload size other than the one above, or when a payload's fields
do not fill the payload exactly. Records before it stay valid.

#### 1: sample

One pose of one device. 186 bytes.

| Field | Type | Meaning |
| --- | --- | --- |
| capture count | i64 | The count when the driver captured the pose |
| device id | u32 | Which device. Stored as received: check it before use as an index |
| tracking result | u32 | `vr::ETrackingResult`: 200 running OK, 100 and 101 calibrating, 201 out of range, 1 uninitialized |
| pose valid | u8 | 0 or 1 |
| connected | u8 | 0 or 1 |
| pose time offset | f64 | Seconds. The driver's own estimate of the pose's age |
| world-from-driver rotation | 4 f64 | Quaternion, in the order w, x, y, z |
| world-from-driver translation | 3 f64 | x, y, z in meters |
| rotation | 4 f64 | The device's orientation in driver space: w, x, y, z |
| position | 3 f64 | The device's position in driver space: x, y, z in meters |
| velocity | 3 f64 | x, y, z in meters per second |
| angular velocity | 3 f64 | x, y, z in radians per second |

A flag byte other than 0 or 1 makes the record one a reader refuses. The pose
in the tracking universe is the world-from-driver transform applied to the
driver-space pose. A lighthouse device sends about 430 samples a second and a
Quest headset about 55; the headset's values are single-precision numbers
widened to doubles.

Samples are in the order the recorder received them. Across devices that is
close to, but not exactly, the order of their capture counts.

#### 2: gap

Samples are missing in front of the next sample.

| Field | Type | Meaning |
| --- | --- | --- |
| count | i64 | The count when the recorder learned of the gap |
| lost | u64 | Samples missing, as this record adds to the hole |
| hole size | u64 | The size of the whole hole so far |

A hole can reach the recorder in parts: two gap records in a row with no
sample between them describe one hole, and the second record's hole size is
the sum. A session boundary (below) adds one to a hole and hides how many
samples went with it, so treat a gap as "there is a hole here", not as an
exact loss.

#### 3: boundary

SteamVR's driver session ended or a new one began, or the recorder's source
started over.

| Field | Type | Meaning |
| --- | --- | --- |
| count | i64 | The count when the recorder learned of the boundary |
| boundaries | u64 | A running count of boundaries since the recording program started |

Poses on the two sides of a boundary may be in different tracking universes.
A boundary is followed by a gap record that counts it (1, or more when samples were lost with it).

#### 4: device

One device entry, in the layout above. It is written when a device is first
seen after the header was written, when something about a device changes, and
for every device after a resume. The newest entry for an id is the one that
holds from there on. A device that sends a sample before anything is known of
it gets an entry in front of that sample with its id and nothing else: class
0, role 0 and four empty strings.

#### 5: end

The recorder closed the file for good. 25 bytes. Nothing follows it.

| Field | Type | Meaning |
| --- | --- | --- |
| reason | u8 | 1 the file reached its size cap, 2 the disk was almost full, 3 SteamVR quit, 4 the recording program closed, 5 a write failed |
| count | i64 | The count when recording stopped |
| time | i64 | The wall-clock time when recording stopped |
| samples | u64 | The number of sample records in the file |

A reader shows a reason it does not know as a number.

#### 6: pause

Recording stopped here and may go on later in the same file. 17 bytes.

| Field | Type | Meaning |
| --- | --- | --- |
| cause | u8 | 1 the player turned recording off. 2 a clip ended (not written yet) |
| count | i64 | The count when recording stopped |
| time | i64 | The wall-clock time when recording stopped |

Nothing that happened between a pause and the next resume is in the file.

#### 7: resume

Recording goes on here.

| Field | Type | Meaning |
| --- | --- | --- |
| cause | u8 | 1 the player turned recording on again. 2 a clip began (not written yet) |
| count | i64 | The count when recording went on |
| time | i64 | The wall-clock time when recording went on |
| trigger | string | Empty for cause 1. For a clip, the name of what started it |

A resume with cause 1 is followed by a device record for every device, so the stretch
behind it can be read by itself.

#### 8: alignment

The calibration Nova Calibrator gives one tracking system's devices from here
on. The samples are never changed by it: they stay as the driver made them. A
reader that wants every device in one space, as SteamVR showed them, uses it.

| Field | Type | Meaning |
| --- | --- | --- |
| applied | u8 | 1 the calibration is in use. 0 it is known but not in use. No other value |
| count | i64 | The count when the record was written |
| rotation | 4 f64 | w, x, y, z |
| translation | 3 f64 | Meters |
| scale | f64 | |
| tracking system | string | The system whose devices it is for, as in their device entries |

For a sample of a device in that system, with `p` the sample's position put in
its world (world-from-driver rotation times position, plus world-from-driver
translation), the position SteamVR showed is `rotation * (scale * p) +
translation` while `applied` is 1, and `p` while it is 0 or before the first
alignment record for the system. A device of another system is at `p`.

The recorder writes one at each start and each resume when it knows the
calibration, and one when a value changes. This is the calibration of the
profile in use; the finer corrections Nova Calibrator makes on top of it
(field anchors, a tracker's own frame corrections) are not in the file. A
reader that does not know kind 8 skips it, as any unknown kind.

#### 9: model

SteamVR's name of the render model that shows a device, such as
`{htc}vr_tracker_vive_3_0`, for a playback that draws the device. It names a
shape that every device of that kind has, not one device.

| Field | Type | Meaning |
| --- | --- | --- |
| device id | u32 | The device, as in its device entry |
| render model | string | The name. Never empty in a file the recorder wrote |

The recorder writes one for each device that has a model at each start and
each resume, behind the device entries, and one when a device's model changes.
The newest one for an id holds. A reader that does not know kind 9 skips it.

## What a reader must do

1. Never join the samples on the two sides of a gap, a boundary, or a pause
   and its resume: do not interpolate, filter or difference across them. The
   time between a pause and its resume is the difference of their counts.
2. A sample's time in seconds from the start of the recording is
   `(capture count - start count) / clock rate`.
3. Take a device's description from the newest device entry for its id at
   that point in the file.
4. How the file ends says how the recording ended:
   - an end record: closed for the reason it gives;
   - a pause record as the last record: the player turned recording off and
     did not turn it on again;
   - anything else on a record's edge: the recorder was ended without warning
     (the program was ended or the machine lost power) between two records;
   - the file ends inside a record (fewer than three bytes of a record's
     start, or fewer payload bytes than its size says): the same, in the
     middle of a write. Everything before that record is good. This is not an
     error in the file.
5. Treat the file as untrusted. Read no more bytes for a record than its size
   says, bound every allocation by the sizes above, and check a device id
   before using it as an index.

## The size cap

The player picks a cap of 1, 5 or 20 GB. In this version the recording ends
when the file reaches the cap, with end reason 1, and the data it holds stays.

Keeping the newest data and dropping the oldest instead is not built. A file
that is written front to back cannot lose its front in place, and a ring
inside the file would need an index a reader must trust. The way that keeps
one whole file at every moment is to write the newer part to a second file (a
new header, the device entries, then the records from a record's edge on) and
put it in the old file's place in one step, as a new recording replaces an
old one. A crash at any point then leaves one complete file, and the result
is an ordinary version 1 file that starts later: the layout does not change.

The cost is the copy. For clips, which are tens of megabytes, it is short.
For a full recording at a 20 GB cap it moves gigabytes while samples keep
arriving, so the recorder must hold them or record a gap.

## The listing

A listing is what a reader found in a file, as lines of text that two readers
can be compared by. Each line ends in a line feed. `tools/read-recording.py
--listing FILE` and `tools/novapose-listing.cpp` print it.

```
novapose listing 1
format: version V, mode M
writer: WRITER
clock: RATE ticks per second, start count COUNT, start time TIME us
header devices: N
device ID: class C, role R, label "L", model "M", system "S", type "T"     (one line for each header device)
...one line for each record that is not a sample, in file order...
samples: TOTAL
samples of device ID: N, valid V, first POSE, last POSE                     (one line for each device, by id)
file: complete
```

The lines for records, where `K` is the number of samples in front of the
record:

```
gap after sample K: lost L, hole H, count C
boundary after sample K: boundaries B, count C
device record after sample K: device ID: class C, role R, label "L", model "M", system "S", type "T"
pause after sample K: cause N, count C, time T us
resume after sample K: cause N, trigger "TEXT", count C, time T us
alignment after sample K: system "S", applied N, count C, rotation W X Y Z, translation X Y Z, scale S
model after sample K: device ID, render model "NAME"
end after sample K: reason N, count C, time T us, samples S
unknown record after sample K: kind N, B bytes
```

`POSE` is `count C position X Y Z rotation W X Y Z`, the sample's capture
count, position and rotation. `valid` counts the samples whose pose-valid flag
is 1. A double is printed as C's `printf("%.17g")` prints it.

The last line is `file: complete` when the file ends on a record's edge,
`file: cut short inside a record after B bytes` when it ends inside a record,
and `file: not a recording after B bytes` when a record is refused; `B` is the
size of the header and of every whole record in front. A file whose header is
refused has the first line and then `file: not a recording`.

## The reference recordings

`Tests/recordings/` holds six small recordings made from fixed input
(`Tests/ReferenceRecordings.h`), each with its listing.
`tools/check-recording-readers.ps1` compiles the C++ reader by itself and
compares both readers' listings with them:

| File | What it shows |
| --- | --- |
| `clean` | A header with three devices, 218 samples (a headset at 72 a second, two trackers at 400), an end record |
| `gap-and-boundary` | A gap of 40, a boundary with its gap of 1, and a device the header does not list: first an entry with only its id, later its description |
| `pause-and-resume` | 100 samples, a pause, a resume thirty seconds later, every device again, 60 more samples, an end record |
| `cut-short` | `clean` without its end record, cut 77 bytes into its last sample |
| `newer-records` | A record of kind 77, and a pause and resume with cause 2 and a trigger |
| `alignment` | An alignment record and a model record at the start, and a second alignment record, not applied, after 20 samples |

The harness fails when the generator no longer makes these exact bytes, so a
change to the layout cannot be made without a change here. A change that a
version 1 reader would misread (a field added to a known record, a field's
meaning changed, a new rule for joining samples) needs a new format version.
A new record kind does not: version 1 readers skip it. The values in the
reference recordings are made with integers and binary fractions only, so
they do not depend on a compiler.
