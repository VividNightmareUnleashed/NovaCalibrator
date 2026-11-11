# The pose recording format

Format version 1 (raw mode) and format version 2 (compact mode). File
extension `.novapose`.

This document is the contract for a pose recording. A reader written from it
alone, in any language, is correct when it reports for each file in
`Tests/recordings/` what the `.txt` file beside it says.
`Overlay/PoseRecordingFormat.h` is the same layout as C++ and needs only the
standard library; `tools/read-recording.py` is a second reader written from
this document. The Motion Recorder module (`docs/modules.md`) writes the file
and plays it back (`Overlay/RecordingPlayback.h`).
`tools/novapose-convert.cpp` writes a recording again in the other mode and
says how far two recordings are apart. `tools/measure-recording.py` holds the
measurements the compact mode was designed from; it defines no format.

A recording holds every pose the driver captured while recording was on (or,
in "Automatic", the poses around each tracking problem: see "Clips"): for
each tracked device, its position, rotation and velocities as SteamVR's driver
reported them, before any calibration, with the capture time. It shows how the
player's body moved. It holds no serial number, no device name the player
typed, no account or computer name, and nothing recorded while recording was
off. It is not encrypted: what is in the file is what this document says, and
anyone can check it.

The two versions hold the same records. They differ in how a sample is stored:

| | Version 1, raw | Version 2, compact |
| --- | --- | --- |
| A sample | 189 bytes, every number as the driver sent it | Most samples a few bits in a block, to the steps in "Compact samples". A sample that does not fit those steps is stored as in version 1 |
| Position | The driver's double | Within 0.5 micrometers |
| Rotation | The driver's doubles | Within 1 microradian |
| A sample of single-precision numbers (a Quest headset's) | The driver's doubles | Bit for bit |
| Size, measured (see "Measured sizes") | 189.0 bytes a sample, 2.48 GB an hour | 12.6 bytes a sample, 0.17 GB an hour |

The Motion Recorder writes version 2. It writes version 1 when the settings
record has `motion_recorder_raw` set, for a capture that must hold the
driver's own doubles. The sections up to "The size cap" describe both
versions; where version 2 differs, they say so.

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
| format version | u32 | 1 or 2 |
| mode | u32 | 1, raw, in version 1. 2, compact, in version 2 |
| velocity scale | u32 | Version 2 only: the velocity steps in one meter a second. 10000 is a step of 100 micrometers a second |
| angular velocity scale | u32 | Version 2 only: the angular velocity steps in one radian a second |
| clock rate | i64 | Counts per second. Greater than zero |
| start count | i64 | The count when the recording began |
| start time | i64 | The wall-clock time when the recording began |
| device count | u32 | The number of device entries below, 0 to 64 |
| writer | string | The program and version that wrote the file |
| devices | device entry, `device count` times | The devices known when the recording began |

A version 1 header has no scale fields: the clock rate follows the mode.

A reader refuses a file when the magic is different, when the format version
is not one it knows, when the mode is not the one of its version, when the
clock rate is zero or negative, or when the device count is over 64. It
refuses a version 2 file when a scale is under 1000 or over 10,000,000, or
when the clock rate is under 1,000,000 or over 1,000,000,000 (a pose time is
kept to one count: on a slower clock that is too coarse). A format version
above 2 is a layout this document does not describe: a reader must refuse it,
not guess. A reader that knows version 1 alone refuses version 2 in the same
way.

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
| 10 | mark | 21 to 276 |
| 11 | samples | Version 2 only. 3 to 65535 |
| 12 | sync | Version 2 only. 0 |

Kinds 11 and 12 are in "Compact samples". In a version 1 file they are kinds
the format does not know.

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

In a version 2 file this record is a sample stored whole. Most samples are in
blocks (kind 11), which a reader turns into the same fields; the order of the
samples in the file is the same.

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
| cause | u8 | 1 the player turned recording off. 2 a clip ended, or a recording of clips began and waits for its first |
| count | i64 | The count when recording stopped |
| time | i64 | The wall-clock time when recording stopped |

Nothing that happened between a pause and the next resume is in the file.

#### 7: resume

Recording goes on here.

| Field | Type | Meaning |
| --- | --- | --- |
| cause | u8 | 1 the player turned recording on again. 2 a clip began |
| count | i64 | The count when recording went on. For a clip, the count from which the clip holds data |
| time | i64 | The wall-clock time at that count |
| trigger | string | Empty for cause 1. For a clip, the name of the mark that started it |

A resume is followed by a device record for every device, a model record for
each device that has a model and the alignment records in use, so the stretch
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

#### 10: mark

Something worth finding again happened at this moment: a tracking problem Nova
Calibrator saw, or the player's own "save".

| Field | Type | Meaning |
| --- | --- | --- |
| count | i64 | The count of the moment |
| time | i64 | The wall-clock time of the moment |
| device id | u32 | The device it is about, as in its device entry. 4294967295 when it is about no one device |
| name | string | What happened |

The names this version writes:

| Name | What happened |
| --- | --- |
| `player` | The player saved what just happened |
| `universe-jump` | The headset's tracking space jumped, and Nova Calibrator corrected for it |
| `base-station-moved` | SteamVR moved the lighthouse devices' frame: a base station moved, or SteamVR placed it again |
| `tracking-lost` | The named device had tracking for at least two seconds and then had none for half a second. A shorter break is not marked: the driver's poses carry on through it |
| `device-slid` | A device that was still changed position |
| `tracking-jumped` | A device got tracking again in a different position |
| `calibration-stopped` | Continuous calibration stopped because the values moved too far from the calibration |
| `calibration-realigned` | Continuous calibration aligned the trackers again |

A reader shows a name it does not know as it is. The record's place in the
file is near its moment and not exact (the recorder writes it behind the
samples it had fetched): use the count. A full recording has a mark record at
each such moment; in a recording of clips the first mark of a clip is also the
trigger its resume record names. A reader that does not know kind 10 skips it.

## Compact samples

Format version 2. A sample is in the file in one of two ways:

- **Whole**: a sample record (kind 1), as in version 1. Every number is the
  driver's.
- **In a block**: some bits in a samples record (kind 11). The bits say how the
  sample differs from a guess, and the guess is made from the sample of the
  same device before it.

### What a block keeps

| Field | A reader gets |
| --- | --- |
| capture count, device id, tracking result, the two flags, the world-from-driver transform | The recorded value, bit for bit |
| position | The nearest micrometer: within 0.5 micrometers of the recorded number |
| rotation | Each of the four numbers to the nearest 0.0000005: within 0.00000025. As an angle the rotation is within 1 microradian of the recorded one |
| velocity | The nearest step of the header's velocity scale: within 50 micrometers a second for a scale of 10000 |
| angular velocity | The nearest step of the header's angular velocity scale: within 50 microradians a second for a scale of 10000 |
| pose time offset | The nearest clock count: within 50 nanoseconds for a clock of 10,000,000 counts a second |
| all of a sample whose thirteen pose numbers are single-precision numbers | The recorded value, bit for bit, the pose time offset too |

The thirteen pose numbers are the rotation (w, x, y, z), the position, the
velocity and the angular velocity, in that order: numbers 0 to 12. "Within"
is half a step, plus the last digit of the double itself (one part in 2^51 of
the number).

A Quest headset sends single-precision numbers, so its samples come back as
they were recorded. A lighthouse device sends doubles, whose last digits are
far under the sensor's noise (a tracker that does not move changes position
by 30 to 40 micrometers from one sample to the next).

A sample that cannot be kept to these steps is stored whole: a number that is
not a number or is infinite, a number over a limit (below), and a sample of a
device whose sample before it was one of those. So is the first sample of each
device behind every place where a reader forgets the devices. An exact repeat
(the thirteen pose numbers of the device's sample before it again, bit for
bit) is one bit in a block, and a reader gets its own previous numbers again:
a repeat stays a repeat.

The numbers a reader gets do not depend on the reader. Every guess is
arithmetic with whole numbers that fit 64 bits, and a floating-point number
is made by one division. Two readers that follow this section print the same
`sample digest` in a listing.

### What a reader keeps

To read a block a reader keeps a **model**:

- The **devices** that had a whole sample since the model was last emptied, in
  the order of their first whole sample. At most 64.
- For each device: its **previous sample**, as this reader has it (whole or
  from a block); `usable`, whether that sample can be guessed from; `single`,
  whether its thirteen pose numbers are single-precision numbers; a `period`
  in clock counts, 0 when not known; and 44 **contexts**: `steps[2][14]`,
  `time[2]` and `singles[14]`.
- The **last count**: the capture count of the sample before, of any device.
- One more context, `capture`.

The model is emptied (no devices, last count 0, `capture` at its first value)
at the header, at a sync record (kind 12) and at a resume record (kind 7).
Nothing else empties it: a block reaches back over a gap, a boundary, a
device record or a mark, never over a sync or a resume.

A **whole sample** (kind 1) in a version 2 file sets the last count to its
capture count. When its device is not among the devices and there are fewer
than 64, the device is added at the end. When the device is (now) among them,
it starts anew: its previous sample is this sample, `usable` and `single` are
found from it, its period is 0, and its contexts are at their first values. A
whole sample of a 65th device changes the last count and nothing else.

A sample is **usable** when its capture count is from -2^58 to 2^58, and the
steps of its pose time offset and of each of its thirteen pose numbers exist:

| Number | Scale: steps in one unit | Limit in steps |
| --- | --- | --- |
| pose time offset | the clock rate | 2^40 |
| rotation, each of four | 2,000,000 | 2^22 |
| position | 1,000,000 | 2^40 |
| velocity | the header's velocity scale | 2^30 |
| angular velocity | the header's angular velocity scale | 2^30 |

**Steps** of a double `x` at a scale: `p = x * scale`, one double
multiplication. When `p` is not a number, or is under `-limit` or over
`limit`, there are no steps. Otherwise `w = floor(p)`, and the steps are
`w + 1` when `p >= w + 0.5`, else `w`: the nearest whole number, a half
upward. The comparison is made on `p` itself; do not compute `p + 0.5` or
`p - w` in a way a compiler can join with the multiplication.

**A double from steps**: `steps / scale`, one double division of two whole
numbers that a double holds exactly.

**A single-precision number** is a double whose value a 32-bit IEEE float
holds exactly: a zero of either sign, or a finite number with at most 24
significant bits in the float's range (the smallest are multiples of 2^-149).
Infinity and not-a-number are none. Its **32 bits** are that float's bits as
an unsigned number. Find them from the double's 64 bits, not with a
conversion that a processor's settings for very small numbers can change.

A **context** is one whole number, `sum`: about eight times the mean of the
numbers it has coded. Its first value is 128; 524288 for each of `singles`;
8192 for `capture`.

### 11: samples

A block.

| Field | Type | Meaning |
| --- | --- | --- |
| count | u16 | The number of samples in the block. Never 0 |
| bits | the rest of the payload | The samples, one behind the other |

The bits fill each byte from its lowest bit to its highest. Behind the last
sample the last byte is filled up with zero bits. In the layout below:

- `bit` is one bit.
- `bits(n)` is a whole number of `n` bits, the lowest bit first.
- `step(context, guess)` is a whole number, written as its distance from the
  guess:
  1. `k` is the number of bits `context.sum` takes (0 for 0, 8 for 128 to
     255) less 5; not under 0 and not over 40.
  2. Count zero bits up to the next one bit: `z` zeros. The one bit is used
     up. More than `61 - k` zeros are not a number: refuse the block.
  3. `h = bits(z)`, `l = bits(k)`, and `u = ((2^z + h - 1) * 2^k) + l`.
  4. `context.sum` becomes `context.sum + min(u, 2^40) - floor(context.sum / 8)`.
  5. The distance `d` is `u / 2` when `u` is even, else `-(u + 1) / 2`. The
     number is `guess + d`.

A writer does the reverse: `u = 2 * d` for `d` of 0 or more, else
`-2 * d - 1`; `v = floor(u / 2^k) + 1`, which takes `n` bits; `n - 1` zero
bits, a one bit, the low `n - 1` bits of `v`, the low `k` bits of `u`.

Each sample, in this order:

1. **The device**: `bits(L)`, where `L` is the number of bits that the number
   of devices less one takes (no bits for one device, 2 bits for three or
   four). It is the device's place among the model's devices, from 0. Refuse
   the block when there is no such device or the device is not usable.
2. **The capture count**: `step(capture, last count)`. Refuse when the last
   count or the new count is outside -2^58 to 2^58. The last count becomes
   this count.
3. **What is unusual**: `plain = bit`. When it is 1, `status`, `moved` and
   `repeat` are 0 and `single` is the device's `single`. When it is 0, four
   bits follow: `status`, `moved`, `single`, `repeat`.
   - With `status`: pose valid is `bit`, connected is `bit`, and the tracking
     result is `bits(32)`. Without it they are the previous sample's.
   - With `moved`: seven doubles, each `bits(64)`: the world-from-driver
     rotation (w, x, y, z) and translation. Without it they are the previous
     sample's.
4. **The pose time offset.** `wasOffset` is the steps of the previous
   sample's offset (scale: the clock rate), and `wasPoseTime` is the previous
   capture count plus `wasOffset`.
   - With `single`: `wide = bit`. With `wide` the offset is a double,
     `bits(64)`. Without it the offset is a single-precision number (below)
     with the context `singles[0]`.
   - Without `single`: the offset is stored as the pose time, which for a
     lighthouse device stays or goes on by a steady period. `periods` is the
     count of one bits up to a zero bit, four at most (four ones have no zero
     behind them). `onward` is 1 when `periods` is over 0, else 0.
     - `periods` 4: the offset in counts is `wasOffset`, and `elapsed` is the
       new capture count plus `wasOffset` less `wasPoseTime`.
     - Otherwise `elapsed = step(time[onward], periods * period)`, and the
       offset in counts is `wasPoseTime + elapsed` less the new capture count.
       Refuse when it is outside -2^40 to 2^40. Then the period follows: when
       `periods` is 1 or more and the period is 0, the period becomes
       `elapsed` if `elapsed` is from 1 to 2^30; else when `periods` is 1,
       with `miss = elapsed - period`, the period changes by
       `min(floor(|miss| / 4), 8)` toward the miss, and is 1 at least.
     - The offset is the offset in counts divided by the clock rate.
5. **The thirteen pose numbers.**
   - With `repeat` they are the previous sample's, bit for bit.
   - Else with `single`, number `i` is a single-precision number with the
     context `singles[1 + i]`.
   - Else they are in steps. `from[i]` is the steps of the previous sample's
     number `i`, `to[i]` the steps of the new one, and `C` is
     `steps[onward]`.
     1. The velocities: for `i` from 0 to 5, `to[7 + i] = step(C[i], from[7 + i])`.
     2. `micros` is `floor(elapsed * 1000000 / clock rate)` when `elapsed` is
        over 0 and not over `floor(clock rate / 4)`, else 0.
     3. The position, moved on by the mean of the two velocities: for `i`
        from 0 to 2, `to[4 + i] = step(C[6 + i], from[4 + i] + floor(((from[7 + i] + to[7 + i]) * micros + V) / (2 * V)))`,
        with `V` the velocity scale.
     4. The rotation, turned on by the new angular velocity in the device's
        own frame. With `w, x, y, z = from[0..3]` and `a, b, c = to[10..12]`:
        `turn = (-(x*a + y*b + z*c), w*a + y*c - z*b, w*b + z*a - x*c, w*c + x*b - y*a)`.
        `largest` is the place of the largest of `|w|, |x|, |y|, |z|`, the
        first of equals. For `i` from 0 to 3 without `largest`:
        `to[i] = step(C[9 + i], from[i] + floor((floor(turn[i] / A) * micros + 1000000) / 2000000))`,
        with `A` the angular velocity scale.
     5. The rotation's largest number, from the other three, because the four
        keep their length: `rest = w*w + x*x + y*y + z*z` less the squares of
        the three new numbers; `root` is the largest whole number whose
        square is not over `rest`, or 0 when `rest` is under 1; the guess is
        `-root` when `from[largest]` is under 0, else `root`;
        `to[largest] = step(C[13], guess)`.
     6. Refuse when a number is over its limit in the table above. Number
        `i` is `to[i]` divided by its scale.
6. The device's previous sample is this sample as read, `usable` is found
   from it, and its `single` is this sample's.

Every `floor` is toward minus infinity.

**A single-precision number with a context** is
`step(context, base)`, where `base` is the 32 bits of the value that was in
its place in the previous sample when that value is a single-precision
number, else 0. Refuse when the result is under 0 or over 4294967295, or is
not the 32 bits of a finite number (the eight exponent bits all 1). The
number is the double with that single-precision value.

A reader refuses a block that has fewer than two bytes or a count of 0, whose
bits end inside a sample, that has 8 or more bits left behind its last
sample, or any one bit among those left; and every block for which the layout
above says "refuse". Records before it stay valid.

### 12: sync

No payload. The model is emptied here: behind it the first sample of each
device is a whole one. A recorder writes one every two seconds of capture
time, so that a reader can start in the middle of a file and a damaged byte
costs two seconds at most, and one where the file can later be cut (see "The
size cap").

### What the writer does

None of this is needed to read a file. The Motion Recorder's writer
(`CompactWriter` in `Overlay/PoseRecordingFormat.h`):

- stores a sample whole when its device is not in the model or not usable,
  when the sample is not usable, and when the last count is outside -2^58 to
  2^58;
- sets `single` when all thirteen numbers are single-precision numbers, and
  `repeat` when they are the thirteen it was given for the device's sample
  before it, bit for bit;
- chooses `periods` 4 when the offset in counts did not change, else the
  count from 0 to 3 that leaves the smallest distance to write; with no
  period yet, 1 when `elapsed` is over a 2000th of a second, else 0;
- writes a sync record in front of a sample whose capture count is two
  seconds or more from the first sample behind the last place the model was
  emptied;
- puts at most 1024 samples and 60000 bytes in one block, and closes the open
  block in front of every other record.

### Measured sizes

The first real recording: 24.5 minutes, one Quest headset, six trackers and
two controllers that all move, 5,353,159 samples (3,639 a second), with 110
gaps and 331 marks.

| Stored as | Bytes | Bytes a sample | One hour at this rate |
| --- | --- | --- | --- |
| Version 1, raw | 1,011,763,718 | 189.0 | 2.48 GB |
| Version 2, velocity scales 10000 (steps of 100 micrometers and microradians a second) | 67,645,591 | 12.64 | 0.166 GB |
| Version 2, velocity scales 100000 (steps of 10) | 78,558,120 | 14.68 | 0.192 GB |

Read back and compared with the raw file, sample by sample
(`novapose-convert --compare`): every sample has the same capture count,
device, state and world-from-driver transform; all 94,571 samples of
single-precision numbers are the same bit for bit; the largest distance is
0.5 micrometers for a position, 0.00000025 for a rotation number (0.985
microradians as an angle), half a velocity step, and half a clock count (50
nanoseconds) for a pose time offset. 5,346,533 samples are in blocks and
6,626 are stored whole: the first of each device at the start and behind each
of the 735 sync records. Both readers print the same listing for the compact
file, sample digest included.

The Motion Recorder writes velocity scales of 10000. Most of what a block
still holds is the sensors' noise, which no encoding removes and which the
recording is for: a tracker that does not move changes its velocity by 350 to
430 micrometers a second from one sample to the next.

## Clips

In "Automatic" the recorder writes nothing until something is marked. What
arrives waits in memory, at most the last 30 seconds. A mark then starts a
clip:

```
resume, cause 2, with the mark's name    count: where the clip's data begins
device, model and alignment records      everything known at that count
the records that waited in memory        samples, gaps, boundaries, changes
mark                                     the moment itself
records as they arrive
pause, cause 2                           10 seconds after the last mark
```

A mark inside an open clip is one more mark record, and the clip goes on for
10 seconds after it. The player's save (`player`) ends its clip at the mark.
A new recording of clips has a pause record with cause 2 behind its header,
and its file takes the place of the previous recording when the first clip is
written. When the player turns recording off between two clips nothing is
added: the file already ends with a pause record. What waited in memory at
that moment is dropped, as nothing recorded while recording is off reaches a
file.

A clip is a stretch like any other: the rules below hold for it, and a reader
that knows nothing of clips reads the file correctly.

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
   before using it as an index. In a block, read no bit behind the payload,
   and refuse a number that is over its limit before it is used in
   arithmetic.
6. In a version 2 file, start reading only where the model is empty: at the
   header, or at a sync or a resume record. A block behind any other place
   has nothing to be read against. A reader that keeps places to jump to
   keeps only those.
7. In a version 2 file a number from a block is the recorded number to the
   steps in "What a block keeps", not its last digits. Two samples of one
   device with the same thirteen pose numbers are an exact repeat in the
   recording, as they are in a version 1 file.

## The size cap

The player picks a cap of 1, 5 or 20 GB. The file never grows past it. Near
the cap the recorder drops the oldest part of the file, so that about half the
cap stays, and goes on.

A file that is written front to back cannot lose its front in place, and a
ring inside the file would need an index a reader must trust. So the recorder
writes the newer part to a second file and puts it in the old file's place in
one step, as a new recording replaces an old one. The second file is:

```
header                                   start count and time: the cut
device, model and alignment records      everything the dropped part said
the old file's records from the cut on   byte for byte
```

The cut is on a record's edge. The header's start count and start time are
those of the cut, its device entries are the newest ones the dropped part held
for each id, and the labels stay the same. The result is an ordinary file of
the same version that starts later: a reader needs to know nothing of this,
and nothing in the file says how much was dropped. In a recording of clips the
file can start at a clip's resume record or inside a clip.

In a version 2 file a cut is only where the model is empty. The recorder
writes a sync record at each place it may later cut at (one for each 64th of
the cap), so the part that stays starts with whole samples behind its new
header.

Until the one step that swaps the files, the old file is whole and is still
the recording: the recorder keeps writing to it while it copies, the copy
follows, and a crash or a stop at any moment leaves one complete file. The
copy starts a sixteenth of the cap before the cap. When the disk is too slow
for the copy to be done before the cap is reached, the recorder finishes it
first, and the samples it could not fetch meanwhile are a gap record.

Windows refuses the swap while any other program has the old file open, and a
virus scanner opens a file the moment it was written. The recorder therefore
tries the swap again for up to two seconds before it calls it failed, here and
where a new recording replaces an old one. A program that reads a recording
while it is recorded should not keep the file open for longer than that.

When the drop cannot be done (the new file cannot be written or cannot take
the old one's place), the recording ends when the file reaches the cap, with
end reason 1, and the data it holds stays.

## The listing

A listing is what a reader found in a file, as lines of text that two readers
can be compared by. Each line ends in a line feed. `tools/read-recording.py
--listing FILE` and `tools/novapose-listing.cpp` print it.

```
novapose listing 2
format: version V, mode M
velocity steps: V in a meter a second, A in a radian a second               (version 2 only: the two scales)
writer: WRITER
clock: RATE ticks per second, start count COUNT, start time TIME us
header devices: N
device ID: class C, role R, label "L", model "M", system "S", type "T"     (one line for each header device)
...one line for each record that holds no sample, in file order...
samples: TOTAL
samples in blocks: N                                                        (version 2 only)
sample digest: HHHHHHHH
samples of device ID: N, valid V, first POSE, last POSE                     (one line for each device, by id)
file: complete
```

`TOTAL` counts every sample, whole or in a block. The sample digest is the
CRC-32 of zlib and PNG, as eight lowercase hexadecimal digits, over every
sample in file order, each as the 186 payload bytes a version 1 sample record
holds it in (a sample from a block is put in that layout first). Two readers
print the same digest only when they agree on every bit of every sample.

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
mark after sample K: name "NAME", device ID, count C, time T us
end after sample K: reason N, count C, time T us, samples S
sync after sample K                                                         (version 2 only)
unknown record after sample K: kind N, B bytes
```

A samples record (kind 11) of a version 2 file has no line of its own, as a
sample record has none. In a version 1 file kinds 11 and 12 are unknown
records.

`POSE` is `count C position X Y Z rotation W X Y Z`, the sample's capture
count, position and rotation. `valid` counts the samples whose pose-valid flag
is 1. A double is printed as C's `printf("%.17g")` prints it.

The last line is `file: complete` when the file ends on a record's edge,
`file: cut short inside a record after B bytes` when it ends inside a record,
and `file: not a recording after B bytes` when a record is refused; `B` is the
size of the header and of every whole record in front. A file whose header is
refused has the first line and then `file: not a recording`.

## The reference recordings

`Tests/recordings/` holds eight small recordings made from fixed input
(`Tests/ReferenceRecordings.h`), each with its listing.
`tools/check-recording-readers.ps1` compiles the C++ reader by itself and
compares both readers' listings with them. It also writes each of them again
in the other mode with `tools/novapose-convert.cpp` and checks that both
readers print one listing for the result:

| File | What it shows |
| --- | --- |
| `clean` | A header with three devices, 218 samples (a headset at 72 a second, two trackers at 400), an end record |
| `gap-and-boundary` | A gap of 40, a boundary with its gap of 1, and a device the header does not list: first an entry with only its id, later its description |
| `pause-and-resume` | 100 samples, a pause, a resume thirty seconds later, every device again, 60 more samples, an end record |
| `cut-short` | `clean` without its end record, cut 77 bytes into its last sample |
| `newer-records` | A record of kind 77, and a pause and resume with cause 2 and a trigger |
| `alignment` | An alignment record and a model record at the start, and a second alignment record, not applied, after 20 samples |
| `clips` | A pause with cause 2 behind the header, then two clips: each a resume with its trigger, every device, the samples from before its mark, the mark and the samples after it. The second has a mark about device 3 and the player's own |
| `compact` | Format version 2. 318 samples, 305 of them in blocks: a headset of single-precision numbers (one pose time offset that is none), a tracker whose pose time stays and goes on by its period, once by two periods, and a tracker whose offset never changes. One state change, one exact repeat, a moved driver frame, a tracker pose of single-precision numbers, and two poses that fit no block (too far away, not a number), each with the whole sample behind it. A gap that a sync follows, because three seconds pass; a mark; a pause and a resume with whole samples behind it; an end record |

The harness fails when the generator no longer makes these exact bytes, so a
change to the layout cannot be made without a change here. A change that a
reader of the same version would misread (a field added to a known record, a
field's meaning changed, a new rule for joining samples, another way to store
a sample) needs a new format version, as blocks needed version 2. A new
record kind that holds nothing a reader must have does not: readers skip it.
The values in the reference recordings are made with integers and binary
fractions only (and one division for a pose time), so they do not depend on a
compiler.
