# Tracking comparison

In Motion Recorder, choose **Compare tracking** and move as usual for 30
seconds. **Stop capture** ends it early. **Open recording folder** opens the
folder containing the new `Tracking-stages-*.novatrace`. All three
observations share this one file and the machine's performance-counter clock.
The folder keeps the five newest captures: a capture that starts removes the
older ones, and one that fails leaves no file.

Run `python tools/read-tracking-trace.py FILE` for device coverage, reported
loss and measured runtime query cadence. To inspect a window, use
`python tools/read-tracking-trace.py FILE --csv --device 5 --start 2 --end 3`.
The CSV goes to standard output. Time in seconds starts at the capture request.

| Stage | What was measured |
| --- | --- |
| `input` | Pose at Nova's processing hook before Nova restores, filters, calibrates or hides it. An upstream driver hook can have already modified it. |
| `submitted` | Final pose prepared for forwarding to the host. The same device and sequence identify the input, even for revisions with identical device pose times. |
| `resent` | A quiet base station's new submission after a calibration change. Its sequence refers to the earlier input, which can predate the capture. |
| `runtime` | The standing-space matrix, velocities, connection and tracking status returned by SteamVR to the overlay, at zero requested prediction. Each query has start and end QPC. |

Use each driver's capture QPC and pose time offset to find its estimated pose
time. Compare **changes over time within each coordinate system**. Driver rows
include local position, rotation, velocities and the world-from-driver transform.
Nonidentity driver-from-head transforms have additional `*_head` rows, with
the translation in `x/y/z` and rotation in `qw/qx/qy/qz`. Runtime rows use
the SDK's original 3×4 matrix and standing-space velocities. Changes to the
runtime's reported raw-zero-to-standing matrix are `raw_to_standing` rows.
Account for tracking origin, device offsets, prediction and query timing before
attributing a discrepancy to a processing stage. The runtime query samples
the application API independently of headset refresh, SSW and VRChat's IK.

The sampler targets 120 queries per second. Inspect the actual intervals,
driver loss rows, tracking status and closing reason for the event in question.
Short-lived driver changes can fall between queries. A device that bypasses
Nova's hook can appear only at the runtime stage. The capture stops if the
driver disconnects or the file reaches 128 MiB. An interrupted file can have
a readable prefix and no closing record.

## Storage

The 32-byte little-endian header contains ASCII `NOVASTG1`, the 64-bit
QPC frequency, the starting QPC, and UTC Unix microseconds. Each block has two
16-bit lengths (plain and stored), both at most 32768. Equal lengths mean
literal bytes. A smaller stored length contains an independent MSZIP stream
(`CK` and raw DEFLATE). Record boundaries may cross blocks.

The lossless field codec preserves the bit patterns of incoming and submitted
doubles and runtime floats. For each pose it writes a mask of changed fields
and the XOR of each changed word against its base, using unsigned LEB128.
A matched submission uses its input as the base. An unchanged submission
thus costs only its record header and a zero mask before block compression.
Other observations use the previous value for the same stage and device.
Each runtime poll shares one pair of timestamps across all its devices.
Timestamps use zigzag-encoded deltas in file order, which can be negative.
The reader provides the field order and the record kinds. Device descriptions
contain the OpenVR slot, class, role hint and controller type. Hardware serial
numbers are omitted.
