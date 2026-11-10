#!/usr/bin/env python3
"""Reads a Nova Calibrator pose recording (.novapose).

    python read-recording.py FILE
        The recording's devices, each one's sample count, rate and time span,
        the gaps, session boundaries and pauses, and how the file ended.

    python read-recording.py FILE --csv DEVICE [--out FILE.csv]
        One device's samples as CSV. DEVICE is its id or its label.

    python read-recording.py FILE --listing
        The plain-text listing docs/recording-format.md defines: what a
        correct reader finds in the file. Tests/recordings/*.txt are the
        listings of Tests/recordings/*.novapose, and this prints the same
        bytes for each.

This reader is written from docs/recording-format.md and shares no code with
the C++ reader (Overlay/PoseRecordingFormat.h), so the listings agreeing is a check of
both. The layout is little-endian with no padding, every double as its 64
bits. The file is read front to back, one record at a time, so its size does
not matter. A recording is untrusted input: what does not fit the layout stops
the read with a message, and a file that ends inside a record is reported as
cut short. Only the standard library is used.
"""

import argparse
import datetime
import os
import struct
import sys

MAGIC = b"NOVAPOSE"
FORMAT_VERSION = 1
MODE_RAW = 1
MAX_HEADER_DEVICES = 64

KIND_SAMPLE, KIND_GAP, KIND_BOUNDARY, KIND_DEVICE, KIND_END, KIND_PAUSE, KIND_RESUME = 1, 2, 3, 4, 5, 6, 7
KIND_ALIGNMENT = 8

HEADER = struct.Struct("<8sIIqqqI")
PREFIX = struct.Struct("<BH")
# Clock count, id, tracking result, pose valid, connected, pose time offset,
# world-from-driver rotation (w x y z) and translation, rotation (w x y z),
# position, velocity, angular velocity.
SAMPLE = struct.Struct("<qIIBB21d")
GAP = struct.Struct("<qQQ")
BOUNDARY = struct.Struct("<qQ")
END = struct.Struct("<BqqQ")
PAUSE = struct.Struct("<Bqq")
# Applied, clock count, rotation (w x y z), translation, scale; then a string.
ALIGNMENT = struct.Struct("<Bq8d")
DEVICE_START = struct.Struct("<IIi")
FIXED_SIZES = {KIND_SAMPLE: SAMPLE.size, KIND_GAP: GAP.size, KIND_BOUNDARY: BOUNDARY.size, KIND_END: END.size,
               KIND_PAUSE: PAUSE.size}

END_REASONS = {
    1: "the file reached its size limit",
    2: "the disk was almost full",
    3: "SteamVR quit",
    4: "the recording program closed",
    5: "a write failed",
}
PAUSE_CAUSES = {1: "the player", 2: "a clip"}
DEVICE_CLASSES = {1: "headset", 2: "controller", 3: "tracker", 4: "base station", 5: "display"}
CONTROLLER_ROLES = {1: "left hand", 2: "right hand", 3: "opt out", 4: "treadmill", 5: "stylus"}

CSV_COLUMNS = (
    "time_s,count,tracking_result,pose_valid,connected,pose_time_offset_s,"
    "world_qw,world_qx,world_qy,world_qz,world_x,world_y,world_z,"
    "qw,qx,qy,qz,x,y,z,vx,vy,vz,wx,wy,wz,lost_before,boundary_before,pause_before"
)


class NotARecording(Exception):
    """The bytes do not fit the layout."""


class Device:
    def __init__(self, device_id, device_class, role, label, model, system, controller_type):
        self.id = device_id
        self.device_class = device_class
        self.role = role
        self.label = label
        self.model = model
        self.system = system
        self.controller_type = controller_type

    def role_text(self):
        return self.controller_type or CONTROLLER_ROLES.get(self.role, "")

    def line(self):
        return (f'device {self.id}: class {self.device_class}, role {self.role}, label "{self.label}", '
                f'model "{self.model}", system "{self.system}", type "{self.controller_type}"')


def take_string(data, at):
    if at >= len(data):
        raise NotARecording("a record ends inside a string")
    end = at + 1 + data[at]
    if end > len(data):
        raise NotARecording("a record ends inside a string")
    return data[at + 1:end].decode("utf-8", errors="replace"), end


def parse_device(data):
    """One device entry that fills `data` exactly."""
    if len(data) < DEVICE_START.size:
        raise NotARecording("a device entry is cut short")
    device_id, device_class, role = DEVICE_START.unpack_from(data, 0)
    at = DEVICE_START.size
    strings = []
    for _ in range(4):
        text, at = take_string(data, at)
        strings.append(text)
    if at != len(data):
        raise NotARecording("a device record has bytes left over")
    return Device(device_id, device_class, role, *strings)


class Recording:
    """The file, read front to back. `consumed` counts the bytes of the header
    and of every whole record returned so far."""

    def __init__(self, stream):
        self.stream = stream
        self.consumed = 0
        self._taken = 0
        fixed =self._exact(HEADER.size, "the file is shorter than a recording header")
        (magic, self.version, self.mode, self.clock_rate, self.start_count, self.start_unix_us,
         device_count) = HEADER.unpack(fixed)
        if magic != MAGIC:
            raise NotARecording("the file does not start with NOVAPOSE")
        if self.version != FORMAT_VERSION:
            raise NotARecording(
                f"the recording has format version {self.version}, and this reader knows version {FORMAT_VERSION}")
        if self.mode != MODE_RAW:
            raise NotARecording("the recording is not in raw mode")
        if self.clock_rate <= 0:
            raise NotARecording("the recording's clock rate is not positive")
        if device_count > MAX_HEADER_DEVICES:
            raise NotARecording("the header lists more than 64 devices")
        self.writer = self._string("the file is shorter than a recording header").decode("utf-8", errors="replace")
        self.devices = []
        for _ in range(device_count):
            cut = "the file ends inside the header's device table"
            data = bytearray(self._exact(DEVICE_START.size, cut))
            for _ in range(4):
                text = self._string(cut)
                data += bytes([len(text)]) + text
            self.devices.append(parse_device(bytes(data)))
        self.consumed = self._taken

    def _exact(self, count, why):
        data = self.stream.read(count)
        self._taken += len(data)
        if len(data) != count:
            raise NotARecording(why)
        return data

    def _string(self, why):
        return self._exact(self._exact(1, why)[0], why)

    def seconds(self, count):
        return (count - self.start_count) / self.clock_rate

    def records(self):
        """Yields (kind, payload) for each whole record. Returns "complete" at
        the end of the file, or "cut" when the file ends inside a record.
        Raises NotARecording for a record that cannot be one."""
        while True:
            prefix = self.stream.read(PREFIX.size)
            self._taken += len(prefix)
            if not prefix:
                return "complete"
            if len(prefix) < PREFIX.size:
                return "cut"
            kind, size = PREFIX.unpack(prefix)
            if kind == 0:
                raise NotARecording("a record has kind 0: the file has a hole or is not a recording")
            payload = self.stream.read(size)
            self._taken += len(payload)
            if len(payload) != size:
                return "cut"
            if kind in FIXED_SIZES and size != FIXED_SIZES[kind]:
                raise NotARecording(f"a record of kind {kind} has {size} bytes, not {FIXED_SIZES[kind]}")
            if kind == KIND_SAMPLE and (payload[16] > 1 or payload[17] > 1):
                raise NotARecording("a sample has a flag that is neither 0 nor 1")
            if kind == KIND_DEVICE:
                payload = parse_device(payload)
            elif kind == KIND_RESUME:
                if size < PAUSE.size:
                    raise NotARecording("a resume record is cut short")
                cause, count, time = PAUSE.unpack_from(payload, 0)
                trigger, end = take_string(payload, PAUSE.size)
                if end != size:
                    raise NotARecording("a resume record has bytes left over")
                payload = (cause, count, time, trigger)
            elif kind == KIND_ALIGNMENT:
                if size < ALIGNMENT.size:
                    raise NotARecording("an alignment record is cut short")
                values = ALIGNMENT.unpack_from(payload, 0)
                if values[0] > 1:
                    raise NotARecording("an alignment record has a flag that is neither 0 nor 1")
                system, end = take_string(payload, ALIGNMENT.size)
                if end != size:
                    raise NotARecording("an alignment record has bytes left over")
                payload = values + (system,)
            self.consumed = self._taken
            yield kind, payload


def each_record(recording, state):
    """recording.records(), with how the file ended left in state["file"]:
    "complete", "cut" or "invalid" (and the reason in state["error"])."""
    try:
        state["file"] = yield from recording.records()
    except NotARecording as problem:
        state["file"] = "invalid"
        state["error"] = str(problem)


# ---------------------------------------------------------------------------
# The listing

def real(value):
    return "%.17g" % value


def pose_text(values):
    return ("count %d position %s %s %s rotation %s %s %s %s"
            % (values[0], real(values[17]), real(values[18]), real(values[19]),
               real(values[13]), real(values[14]), real(values[15]), real(values[16])))


def listing(path):
    lines = ["novapose listing 1"]
    with open(path, "rb") as stream:
        try:
            recording = Recording(stream)
        except NotARecording:
            return "\n".join(lines + ["file: not a recording"]) + "\n"
        lines.append(f"format: version {recording.version}, mode {recording.mode}")
        lines.append(f"writer: {recording.writer}")
        lines.append(f"clock: {recording.clock_rate} ticks per second, start count {recording.start_count}, "
                     f"start time {recording.start_unix_us} us")
        lines.append(f"header devices: {len(recording.devices)}")
        lines += [device.line() for device in recording.devices]
        per_device = {}
        samples = 0
        state = {}
        for kind, payload in each_record(recording, state):
            after = f" after sample {samples}: "
            if kind == KIND_SAMPLE:
                values = SAMPLE.unpack(payload)
                entry = per_device.setdefault(values[1], [0, 0, values, values])
                entry[0] += 1
                entry[1] += values[3]
                entry[3] = values
                samples += 1
            elif kind == KIND_GAP:
                count, lost, hole = GAP.unpack(payload)
                lines.append(f"gap{after}lost {lost}, hole {hole}, count {count}")
            elif kind == KIND_BOUNDARY:
                count, boundaries = BOUNDARY.unpack(payload)
                lines.append(f"boundary{after}boundaries {boundaries}, count {count}")
            elif kind == KIND_DEVICE:
                lines.append(f"device record{after}{payload.line()}")
            elif kind == KIND_END:
                reason, count, time, total = END.unpack(payload)
                lines.append(f"end{after}reason {reason}, count {count}, time {time} us, samples {total}")
            elif kind == KIND_PAUSE:
                cause, count, time = PAUSE.unpack(payload)
                lines.append(f"pause{after}cause {cause}, count {count}, time {time} us")
            elif kind == KIND_RESUME:
                cause, count, time, trigger = payload
                lines.append(f'resume{after}cause {cause}, trigger "{trigger}", count {count}, time {time} us')
            elif kind == KIND_ALIGNMENT:
                turn = " ".join(real(value) for value in payload[2:6])
                shift = " ".join(real(value) for value in payload[6:9])
                lines.append(f'alignment{after}system "{payload[10]}", applied {payload[0]}, count {payload[1]}, '
                             f"rotation {turn}, translation {shift}, scale {real(payload[9])}")
            else:
                lines.append(f"unknown record{after}kind {kind}, {len(payload)} bytes")
        lines.append(f"samples: {samples}")
        for device_id in sorted(per_device):
            count, valid, first, last = per_device[device_id]
            lines.append(f"samples of device {device_id}: {count}, valid {valid}, first {pose_text(first)}, "
                         f"last {pose_text(last)}")
        if state["file"] == "complete":
            lines.append("file: complete")
        elif state["file"] == "cut":
            lines.append(f"file: cut short inside a record after {recording.consumed} bytes")
        else:
            lines.append(f"file: not a recording after {recording.consumed} bytes")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# The summary and the CSV

class DeviceStats:
    def __init__(self, count, paused_before):
        self.samples = 0
        self.valid = 0
        self.first = count
        self.last = count
        # Clock counts of the pauses before its first sample, and of those
        # between its first and last: the second is not recorded time.
        self.paused_before = paused_before
        self.paused = 0


def scan(path, csv_device=None, csv_out=None):
    """Reads the whole file. Returns what the summary prints; writes the CSV
    rows of `csv_device` (an id) to `csv_out` when given."""
    with open(path, "rb") as stream:
        recording = Recording(stream)
        devices = {device.id: device for device in recording.devices}
        stats = {}
        totals = {"samples": 0, "gaps": 0, "lost": 0, "boundaries": 0, "pauses": 0, "paused": 0, "unknown": 0,
                  "alignments": 0, "end": None, "last": None, "open_pause": None}
        lost_since = 0
        boundary_since = False
        pause_since = False
        state = {}
        for kind, payload in each_record(recording, state):
            if kind == KIND_SAMPLE:
                values = SAMPLE.unpack(payload)
                count, device_id, result, valid, connected = values[:5]
                entry = stats.get(device_id)
                if entry is None:
                    entry = stats[device_id] = DeviceStats(count, totals["paused"])
                entry.samples += 1
                entry.valid += valid
                entry.last = count
                entry.paused = totals["paused"] - entry.paused_before
                totals["samples"] += 1
                totals["last"] = count if totals["last"] is None else max(totals["last"], count)
                if csv_out is not None and device_id == csv_device:
                    row = [repr(recording.seconds(count)), str(count), str(result), str(valid), str(connected)]
                    row += [repr(value) for value in values[5:]]
                    row += [str(lost_since), "1" if boundary_since else "0", "1" if pause_since else "0"]
                    csv_out.write(",".join(row) + "\n")
                    lost_since = 0
                    boundary_since = False
                    pause_since = False
            elif kind == KIND_GAP:
                totals["gaps"] += 1
                totals["lost"] += GAP.unpack(payload)[1]
                lost_since += GAP.unpack(payload)[1]
            elif kind == KIND_BOUNDARY:
                totals["boundaries"] += 1
                boundary_since = True
            elif kind == KIND_DEVICE:
                # The newest entry for an id is the one that holds.
                devices[payload.id] = payload
            elif kind == KIND_PAUSE:
                totals["pauses"] += 1
                totals["open_pause"] = PAUSE.unpack(payload)[1]
                pause_since = True
            elif kind == KIND_RESUME:
                if totals["open_pause"] is not None:
                    totals["paused"] += max(payload[1] - totals["open_pause"], 0)
                    totals["open_pause"] = None
                pause_since = True
            elif kind == KIND_END:
                totals["end"] = END.unpack(payload)
            elif kind == KIND_ALIGNMENT:
                totals["alignments"] += 1
            else:
                totals["unknown"] += 1
        totals["file"] = state["file"]
        totals["error"] = state.get("error", "")
    return recording, devices, stats, totals


def size_text(count):
    for unit in ("bytes", "KB", "MB", "GB"):
        if count < 1024 or unit == "GB":
            return f"{count:.0f} {unit}" if unit == "bytes" else f"{count:.2f} {unit}"
        count /= 1024
    return ""


def print_summary(path, recording, devices, stats, totals):
    start = datetime.datetime.fromtimestamp(0, datetime.timezone.utc) + datetime.timedelta(
        microseconds=recording.start_unix_us)
    print(os.path.basename(path))
    print(f"format {recording.version} (raw), written by {recording.writer or '(no name)'}")
    print(f"started {start:%Y-%m-%d %H:%M:%S} UTC, clock {recording.clock_rate} ticks a second")
    end = totals["end"]
    # To the end record or the last sample, whichever is later: the end is
    # stamped when the file closes, the samples when they were captured.
    ends = [count for count in (end[1] if end else None, totals["last"]) if count is not None]
    span = max(recording.seconds(max(ends)), 0.0) if ends else 0.0
    paused = totals["paused"] / recording.clock_rate
    print(f"recorded {max(span - paused, 0.0):.3f} s over {span:.3f} s, {totals['samples']} samples, "
          f"{size_text(os.path.getsize(path))}")
    if totals["file"] == "invalid":
        print(f"ended: the file stops being a recording {recording.consumed} bytes in ({totals['error']})")
    elif end:
        reason = END_REASONS.get(end[0], f"reason {end[0]}")
        match = "" if end[3] == totals["samples"] else f" (its end record counts {end[3]} samples)"
        print(f"ended: {reason}{match}")
    elif totals["open_pause"] is not None and totals["file"] == "complete":
        print("ended: paused and not resumed")
    else:
        print("ended: no end record; the recorder was cut short"
              + (" inside a record" if totals["file"] == "cut" else ""))
    print(f"gaps: {totals['gaps']} ({totals['lost']} samples lost), session boundaries: {totals['boundaries']}, "
          f"pauses: {totals['pauses']} ({paused:.3f} s not recorded)")
    if totals["unknown"]:
        print(f"records of a kind this reader does not know: {totals['unknown']}")
    if totals["alignments"]:
        print(f"alignment records (the calibration in use, see --listing): {totals['alignments']}")
    print()

    rows = [("id", "class", "label", "model", "system", "role", "samples", "Hz", "first s", "last s", "valid")]
    for device_id in sorted(set(devices) | set(stats)):
        device = devices.get(device_id)
        entry = stats.get(device_id)
        described = [
            DEVICE_CLASSES.get(device.device_class, str(device.device_class)) if device else "?",
            (device.label if device else "") or "-",
            (device.model if device else "") or "-",
            (device.system if device else "") or "-",
            (device.role_text() if device else "") or "-",
        ]
        if entry is None:
            measured = ["0", "-", "-", "-", "-"]
        else:
            recorded = (entry.last - entry.first - entry.paused) / recording.clock_rate
            rate = f"{(entry.samples - 1) / recorded:.1f}" if recorded > 0 and entry.samples > 1 else "-"
            measured = [str(entry.samples), rate, f"{recording.seconds(entry.first):.3f}",
                        f"{recording.seconds(entry.last):.3f}", f"{100.0 * entry.valid / entry.samples:.0f}%"]
        rows.append((str(device_id), *described, *measured))
    widths = [max(len(row[column]) for row in rows) for column in range(len(rows[0]))]
    right = {0, 6, 7, 8, 9, 10}
    for row in rows:
        cells = [cell.rjust(widths[i]) if i in right else cell.ljust(widths[i]) for i, cell in enumerate(row)]
        print("  ".join(cells).rstrip())


def find_device(path, wanted):
    """The id `wanted` names: a number, or a label from the device table."""
    if wanted.isdigit():
        return int(wanted)
    _, devices, _, _ = scan(path)
    matches = [device.id for device in devices.values() if device.label and device.label.lower() == wanted.lower()]
    if len(matches) != 1:
        raise SystemExit(f"{len(matches)} devices have the label {wanted}; give the device's id.")
    return matches[0]


def main():
    parser = argparse.ArgumentParser(description="Reads a Nova Calibrator pose recording (.novapose).")
    parser.add_argument("file", help="the recording")
    parser.add_argument("--csv", metavar="DEVICE", help="write this device's samples as CSV: an id or a label")
    parser.add_argument("--out", metavar="FILE", help="where the CSV goes (default: standard output)")
    parser.add_argument("--listing", action="store_true", help="print the listing docs/recording-format.md defines")
    arguments = parser.parse_args()
    for stream in (sys.stdout, sys.stderr):
        # A console code page may not hold every model name.
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    try:
        if arguments.listing:
            # Bytes, so the lines end the same on every system.
            sys.stdout.flush()
            sys.stdout.buffer.write(listing(arguments.file).encode("utf-8"))
            return 0
        if arguments.csv is None:
            print_summary(arguments.file, *scan(arguments.file))
            return 0
        device_id = find_device(arguments.file, arguments.csv)
        out = open(arguments.out, "w", newline="", encoding="utf-8") if arguments.out else sys.stdout
        try:
            out.write(CSV_COLUMNS + "\n")
            _, _, stats, totals = scan(arguments.file, device_id, out)
        finally:
            if arguments.out:
                out.close()
        rows = stats[device_id].samples if device_id in stats else 0
        note = "" if totals["end"] else "; the file has no end record"
        print(f"device {device_id}: {rows} samples{note}", file=sys.stderr)
        return 0 if rows else 1
    except NotARecording as problem:
        print(f"{arguments.file} is not a recording this reader can use: {problem}.", file=sys.stderr)
        return 1
    except BrokenPipeError:
        # The reader of a piped CSV stopped early (`| head`): not a failure.
        # Point stdout at nothing so the exit does not fail on its flush.
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        return 0
    except OSError as problem:
        print(f"{arguments.file} could not be read: {problem}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
