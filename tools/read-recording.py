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
both. It reads both format versions: 1 (raw) and 2 (compact, whose samples
are mostly a few bits each in blocks). The layout is little-endian with no
padding, every double as its 64 bits. The file is read front to back, one
record at a time, so its size does not matter. A recording is untrusted input:
what does not fit the layout stops the read with a message, and a file that
ends inside a record is reported as cut short. Only the standard library is
used.
"""

import argparse
import datetime
import math
import os
import struct
import sys
import zlib

MAGIC = b"NOVAPOSE"
FORMAT_VERSION = 1
MODE_RAW = 1
FORMAT_VERSION_COMPACT = 2
MODE_COMPACT = 2
MAX_HEADER_DEVICES = 64

KIND_SAMPLE, KIND_GAP, KIND_BOUNDARY, KIND_DEVICE, KIND_END, KIND_PAUSE, KIND_RESUME = 1, 2, 3, 4, 5, 6, 7
KIND_ALIGNMENT = 8
KIND_MODEL = 9
KIND_MARK = 10
# Version 2 only. In a version 1 file they are kinds it does not know.
KIND_SAMPLES = 11
KIND_SYNC = 12

HEADER_START = struct.Struct("<8sII")
HEADER_SCALES = struct.Struct("<II")
HEADER_REST = struct.Struct("<qqqI")
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
# Clock count, wall-clock time, device id; then a string.
MARK = struct.Struct("<qqI")
NO_DEVICE = 0xFFFFFFFF
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


# ---------------------------------------------------------------------------
# Compact samples (format version 2): docs/recording-format.md, "Compact
# samples". A sample here is the tuple SAMPLE unpacks: capture count, device
# id, tracking result, pose valid, connected, then the 21 doubles.

POSITION_SCALE = 1000000            # steps in one meter
ROTATION_SCALE = 2000000            # steps in 1, for each of a rotation's four numbers
MIN_VELOCITY_SCALE, MAX_VELOCITY_SCALE = 1000, 10000000
MIN_CLOCK_RATE, MAX_CLOCK_RATE = 1000000, 1000000000
POSITION_LIMIT = 1 << 40
ROTATION_LIMIT = 1 << 22
VELOCITY_LIMIT = 1 << 30
OFFSET_LIMIT = 1 << 40
COUNT_LIMIT = 1 << 58
PERIOD_LIMIT = 1 << 30
MAX_DEVICES = 64
MAX_PARAMETER = 40
MAX_CODE_LENGTH = 61
MAX_CONTEXT_STEP = 1 << 40
FIRST_CONTEXT_SUM = 128
FIRST_SINGLE_SUM = 8 << 16
FIRST_CAPTURE_SUM = 8 << 10
# The thirteen pose numbers: rotation (4), position (3), velocity (3), angular velocity (3).
POSE_LIMITS = (ROTATION_LIMIT,) * 4 + (POSITION_LIMIT,) * 3 + (VELOCITY_LIMIT,) * 6
DOUBLE = struct.Struct("<d")
DOUBLE_BITS = struct.Struct("<Q")


def steps_of(value, scale, limit):
    """`value` in steps of 1 / `scale`: the nearest whole number, a half
    upward. None when it is not a number or is over `limit` steps from zero."""
    product = value * scale
    if not abs(product) <= limit:
        return None
    whole = math.floor(product)
    return whole + 1 if product >= whole + 0.5 else whole


def single_bits(value):
    """The 32 bits of `value` as a single-precision number, or None when it is
    not exactly one. By its bits, as the C++ reader does it."""
    wide = DOUBLE_BITS.unpack(DOUBLE.pack(value))[0]
    sign = 0x80000000 if wide >> 63 else 0
    exponent = (wide >> 52) & 0x7FF
    fraction = wide & ((1 << 52) - 1)
    if exponent == 0x7FF:
        return None
    if exponent == 0:
        return sign if fraction == 0 else None
    power = exponent - 1023
    if power > 127:
        return None
    if power >= -126:
        return None if fraction & ((1 << 29) - 1) else sign | ((power + 127) << 23) | (fraction >> 29)
    if power < -149:
        return None
    shift = -97 - power
    significand = fraction | (1 << 52)
    return None if significand & ((1 << shift) - 1) else sign | (significand >> shift)


def single_value(bits):
    """The double a single-precision number's 32 bits stand for, or None for
    bits that are not a finite number."""
    sign = (bits >> 31) << 63
    exponent = (bits >> 23) & 0xFF
    fraction = bits & 0x7FFFFF
    if exponent == 0xFF:
        return None
    wide = sign
    if exponent:
        wide |= ((exponent + 896) << 52) | (fraction << 29)
    elif fraction:
        top = fraction.bit_length() - 1
        wide |= ((top - 149 + 1023) << 52) | ((fraction << (52 - top)) & ((1 << 52) - 1))
    return DOUBLE.unpack(DOUBLE_BITS.pack(wide))[0]


class Bits:
    """A block's bits, the first in the lowest bit of a byte."""

    def __init__(self, data, start):
        self.data = data
        self.at = start * 8
        self.end = len(data) * 8

    def take(self, count):
        if count == 0:
            return 0
        at = self.at
        if count > self.end - at:
            raise NotARecording("a block ends inside a sample")
        self.at = at + count
        return (int.from_bytes(self.data[at >> 3:(at + count + 7) >> 3], "little") >> (at & 7)) & ((1 << count) - 1)

    def zeros(self, limit):
        """The zero bits in front of the next one bit, which is taken too."""
        at = self.at
        ahead = int.from_bytes(self.data[at >> 3:(at >> 3) + 9], "little") >> (at & 7)
        count = (ahead & -ahead).bit_length() - 1
        if ahead == 0 or count > limit:
            raise NotARecording("a block ends inside a sample, or holds a number that is too long")
        self.at = at + count + 1
        return count

    def at_end(self):
        left = self.end - self.at
        return left < 8 and self.take(left) == 0


class CompactDevice:
    """What a reader keeps of one device between two syncs."""
    __slots__ = ("id", "previous", "usable", "single", "period", "steps", "time", "singles")


class Compact:
    """What a compact file's blocks are read against. A sync, a resume and the
    header empty it; a sample stored whole starts its device."""

    def __init__(self, rate, velocity_scale, angular_scale):
        self.rate = rate
        self.velocity_scale = velocity_scale
        self.angular_scale = angular_scale
        self.scales = (ROTATION_SCALE,) * 4 + (POSITION_SCALE,) * 3 + (velocity_scale,) * 3 + (angular_scale,) * 3
        self.reset()

    def reset(self):
        self.devices = []
        self.last = 0
        self.capture = [FIRST_CAPTURE_SUM]

    def usable(self, values):
        if not -COUNT_LIMIT <= values[0] <= COUNT_LIMIT or steps_of(values[5], self.rate, OFFSET_LIMIT) is None:
            return False
        return all(steps_of(values[13 + i], self.scales[i], POSE_LIMITS[i]) is not None for i in range(13))

    def whole(self, values):
        """A sample stored whole: its device starts here."""
        self.last = values[0]
        device = next((known for known in self.devices if known.id == values[1]), None)
        if device is None:
            if len(self.devices) >= MAX_DEVICES:
                return
            device = CompactDevice()
            device.id = values[1]
            self.devices.append(device)
        device.previous = values
        device.usable = self.usable(values)
        device.single = all(single_bits(value) is not None for value in values[13:26])
        device.period = 0
        device.steps = [[FIRST_CONTEXT_SUM] * 14, [FIRST_CONTEXT_SUM] * 14]
        device.time = [FIRST_CONTEXT_SUM] * 2
        device.singles = [FIRST_SINGLE_SUM] * 14

    def block(self, payload):
        """A block's payload as its samples."""
        if len(payload) < 2 or payload[0] | (payload[1] << 8) == 0:
            raise NotARecording("a block holds no sample")
        bits = Bits(payload, 2)
        samples = []
        for _ in range(payload[0] | (payload[1] << 8)):
            known = len(self.devices)
            # 1. The device, by its place among those that have a whole sample.
            place = bits.take((known - 1).bit_length()) if known else 0
            if place >= known or not self.devices[place].usable:
                raise NotARecording("a block has a sample of a device with no usable sample before it")
            samples.append(self.sample(bits, self.devices[place]))
        if not bits.at_end():
            raise NotARecording("a block has bits left over")
        return samples

    @staticmethod
    def step(bits, contexts, index, guess):
        """A whole number as its distance from `guess`."""
        total = contexts[index]
        parameter = min(max(total.bit_length() - 5, 0), MAX_PARAMETER)
        zeros = bits.zeros(MAX_CODE_LENGTH - parameter)
        number = ((((1 << zeros) | bits.take(zeros)) - 1) << parameter) | bits.take(parameter)
        contexts[index] = total + min(number, MAX_CONTEXT_STEP) - (total >> 3)
        return guess + ((number >> 1) if not number & 1 else -((number + 1) >> 1))

    def single(self, bits, contexts, index, before):
        """A single-precision number as the distance of its 32 bits from the
        bits of the value before it (from 0 when that is not one)."""
        number = self.step(bits, contexts, index, single_bits(before) or 0)
        value = single_value(number) if 0 <= number <= 0xFFFFFFFF else None
        if value is None:
            raise NotARecording("a block holds a single-precision number that is none")
        return value

    @staticmethod
    def double(bits):
        return DOUBLE.unpack(bits.take(64).to_bytes(8, "little"))[0]

    def sample(self, bits, device):
        previous = device.previous
        rate = self.rate
        step = self.step
        if not -COUNT_LIMIT <= self.last <= COUNT_LIMIT:
            raise NotARecording("a block follows a sample whose count is out of range")

        # The numbers are the document's; 1, the device, is the caller's.
        # 2. The capture count, from the count of the sample before it in the file.
        count = step(bits, self.capture, 0, self.last)
        if not -COUNT_LIMIT <= count <= COUNT_LIMIT:
            raise NotARecording("a block holds a capture count that is out of range")
        self.last = count

        # 3. What is unusual about it. One bit says that nothing is.
        status = moved = repeat = 0
        single = device.single
        if not bits.take(1):
            status = bits.take(1)
            moved = bits.take(1)
            single = bool(bits.take(1))
            repeat = bits.take(1)
        result, valid, connected = previous[2], previous[3], previous[4]
        if status:
            valid = bits.take(1)
            connected = bits.take(1)
            result = bits.take(32)
        transform = tuple(self.double(bits) for _ in range(7)) if moved else previous[6:13]

        # 4. The pose time offset.
        was_offset = steps_of(previous[5], rate, OFFSET_LIMIT)
        was_pose_time = previous[0] + was_offset
        elapsed = 0
        onward = 0
        if single:
            offset = self.double(bits) if bits.take(1) else self.single(bits, device.singles, 0, previous[5])
        else:
            periods = 0
            while periods < 4 and bits.take(1):
                periods += 1
            onward = 1 if periods else 0
            if periods == 4:
                counts = was_offset
                elapsed = count + counts - was_pose_time
            else:
                elapsed = step(bits, device.time, onward, periods * device.period)
                counts = was_pose_time + elapsed - count
                if not -OFFSET_LIMIT <= counts <= OFFSET_LIMIT:
                    raise NotARecording("a block holds a pose time that is out of range")
                if periods >= 1 and device.period == 0:
                    if 0 < elapsed <= PERIOD_LIMIT:
                        device.period = elapsed
                elif periods == 1:
                    miss = elapsed - device.period
                    change = min(abs(miss) >> 2, 8)
                    device.period = max(1, device.period + (change if miss >= 0 else -change))
            offset = counts / rate

        # 5. The thirteen pose numbers.
        was = previous[13:26]
        if repeat:
            now = was
        elif single:
            now = tuple(self.single(bits, device.singles, 1 + i, was[i]) for i in range(13))
        else:
            scales = self.scales
            start = [steps_of(was[i], scales[i], POSE_LIMITS[i]) for i in range(13)]
            contexts = device.steps[onward]
            to = [0] * 13

            def number(context, guess, limit):
                value = step(bits, contexts, context, guess)
                if not -limit <= value <= limit:
                    raise NotARecording("a block holds a pose number that is out of range")
                return value

            # The two velocities, each number from the one before it.
            for i in range(6):
                to[7 + i] = number(i, start[7 + i], VELOCITY_LIMIT)
            micros = elapsed * 1000000 // rate if 0 < elapsed <= rate // 4 else 0
            # Position: the previous one moved on by the mean of the two velocities.
            velocity_scale = self.velocity_scale
            for i in range(3):
                to[4 + i] = number(6 + i, start[4 + i] + ((start[7 + i] + to[7 + i]) * micros + velocity_scale)
                                   // (2 * velocity_scale), POSITION_LIMIT)
            # Rotation: the previous one turned on by the new angular velocity
            # in the device's own frame; its largest number last, from the
            # other three.
            w, x, y, z = start[0:4]
            a, b, c = to[10:13]
            turn = (-(x * a + y * b + z * c), w * a + y * c - z * b, w * b + z * a - x * c, w * c + x * b - y * a)
            largest = 0
            for i in range(1, 4):
                if abs(start[i]) > abs(start[largest]):
                    largest = i
            rest = w * w + x * x + y * y + z * z
            for i in range(4):
                if i != largest:
                    to[i] = number(9 + i, start[i] + ((turn[i] // self.angular_scale) * micros + 1000000) // 2000000,
                                   ROTATION_LIMIT)
                    rest -= to[i] * to[i]
            root = math.isqrt(rest) if rest > 0 else 0
            to[largest] = number(13, -root if start[largest] < 0 else root, ROTATION_LIMIT)
            now = tuple(to[i] / scales[i] for i in range(13))

        values = (count, device.id, result, valid, connected, offset) + tuple(transform) + tuple(now)
        device.previous = values
        device.usable = self.usable(values)
        device.single = single
        return values


class Recording:
    """The file, read front to back. `consumed` counts the bytes of the header
    and of every whole record returned so far."""

    def __init__(self, stream):
        self.stream = stream
        self.consumed = 0
        self._taken = 0
        short = "the file is shorter than a recording header"
        magic, self.version, self.mode = HEADER_START.unpack(self._exact(HEADER_START.size, short))
        if magic != MAGIC:
            raise NotARecording("the file does not start with NOVAPOSE")
        if self.version not in (FORMAT_VERSION, FORMAT_VERSION_COMPACT):
            raise NotARecording(f"the recording has format version {self.version}, and this reader knows versions "
                                f"{FORMAT_VERSION} and {FORMAT_VERSION_COMPACT}")
        self.compact = self.version == FORMAT_VERSION_COMPACT
        if self.mode != (MODE_COMPACT if self.compact else MODE_RAW):
            raise NotARecording("the recording is not in compact mode" if self.compact
                                else "the recording is not in raw mode")
        self.velocity_scale = self.angular_scale = 0
        if self.compact:
            self.velocity_scale, self.angular_scale = HEADER_SCALES.unpack(self._exact(HEADER_SCALES.size, short))
        self.clock_rate, self.start_count, self.start_unix_us, device_count = HEADER_REST.unpack(
            self._exact(HEADER_REST.size, short))
        if self.clock_rate <= 0:
            raise NotARecording("the recording's clock rate is not positive")
        if self.compact and not (MIN_CLOCK_RATE <= self.clock_rate <= MAX_CLOCK_RATE
                                 and MIN_VELOCITY_SCALE <= self.velocity_scale <= MAX_VELOCITY_SCALE
                                 and MIN_VELOCITY_SCALE <= self.angular_scale <= MAX_VELOCITY_SCALE):
            raise NotARecording("the recording's clock rate or velocity scales are not ones a compact recording has")
        self.model = Compact(self.clock_rate, self.velocity_scale, self.angular_scale) if self.compact else None
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
        Raises NotARecording for a record that cannot be one. A compact file's
        block of samples comes as (KIND_SAMPLES, a list of samples), each the
        tuple SAMPLE unpacks from a sample stored whole."""
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
            if self.compact and kind == KIND_SAMPLE:
                # A whole sample is where a block's guesses for its device start.
                self.model.whole(SAMPLE.unpack(payload))
            elif self.compact and kind == KIND_SAMPLES:
                payload = self.model.block(payload)
            elif self.compact and kind == KIND_SYNC:
                if size != 0:
                    raise NotARecording("a sync record has a payload")
                self.model.reset()
            elif self.compact and kind == KIND_RESUME:
                # A stretch reads by itself: its blocks never reach back over this.
                self.model.reset()
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
            elif kind == KIND_MODEL:
                if size < 4:
                    raise NotARecording("a model record is cut short")
                name, end = take_string(payload, 4)
                if end != size:
                    raise NotARecording("a model record has bytes left over")
                payload = (struct.unpack_from("<I", payload, 0)[0], name)
            elif kind == KIND_MARK:
                if size < MARK.size:
                    raise NotARecording("a mark record is cut short")
                count, time, device = MARK.unpack_from(payload, 0)
                name, end = take_string(payload, MARK.size)
                if end != size:
                    raise NotARecording("a mark record has bytes left over")
                payload = (count, time, device, name)
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


def each_sample(recording, kind, payload):
    """The samples a record holds, each as (the tuple SAMPLE unpacks, its 186
    bytes as a version 1 file holds them)."""
    if kind == KIND_SAMPLE:
        yield SAMPLE.unpack(payload), payload
    elif kind == KIND_SAMPLES and recording.compact:
        for values in payload:
            yield values, SAMPLE.pack(*values)


def listing(path):
    lines = ["novapose listing 2"]
    with open(path, "rb") as stream:
        try:
            recording = Recording(stream)
        except NotARecording:
            return "\n".join(lines + ["file: not a recording"]) + "\n"
        lines.append(f"format: version {recording.version}, mode {recording.mode}")
        if recording.compact:
            lines.append(f"velocity steps: {recording.velocity_scale} in a meter a second, "
                         f"{recording.angular_scale} in a radian a second")
        lines.append(f"writer: {recording.writer}")
        lines.append(f"clock: {recording.clock_rate} ticks per second, start count {recording.start_count}, "
                     f"start time {recording.start_unix_us} us")
        lines.append(f"header devices: {len(recording.devices)}")
        lines += [device.line() for device in recording.devices]
        per_device = {}
        samples = 0
        in_blocks = 0
        digest = 0
        state = {}
        for kind, payload in each_record(recording, state):
            after = f" after sample {samples}: "
            if kind == KIND_SAMPLE or (kind == KIND_SAMPLES and recording.compact):
                for values, whole in each_sample(recording, kind, payload):
                    entry = per_device.setdefault(values[1], [0, 0, values, values])
                    entry[0] += 1
                    entry[1] += values[3]
                    entry[3] = values
                    digest = zlib.crc32(whole, digest)
                    samples += 1
                    in_blocks += kind == KIND_SAMPLES
            elif kind == KIND_SYNC and recording.compact:
                lines.append(f"sync after sample {samples}")
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
            elif kind == KIND_MODEL:
                lines.append(f'model{after}device {payload[0]}, render model "{payload[1]}"')
            elif kind == KIND_MARK:
                lines.append(f'mark{after}name "{payload[3]}", device {payload[2]}, count {payload[0]}, '
                             f"time {payload[1]} us")
            else:
                lines.append(f"unknown record{after}kind {kind}, {len(payload)} bytes")
        lines.append(f"samples: {samples}")
        if recording.compact:
            lines.append(f"samples in blocks: {in_blocks}")
        lines.append(f"sample digest: {digest:08x}")
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
                  "alignments": 0, "end": None, "last": None, "open_pause": None, "clips": 0, "marks": [],
                  "in_blocks": 0}
        lost_since = 0
        boundary_since = False
        pause_since = False
        state = {}
        for kind, payload in each_record(recording, state):
            if kind == KIND_SAMPLE or (kind == KIND_SAMPLES and recording.compact):
                for values, _ in each_sample(recording, kind, payload):
                    count, device_id, result, valid, connected = values[:5]
                    entry = stats.get(device_id)
                    if entry is None:
                        entry = stats[device_id] = DeviceStats(count, totals["paused"])
                    entry.samples += 1
                    entry.valid += valid
                    entry.last = count
                    entry.paused = totals["paused"] - entry.paused_before
                    totals["samples"] += 1
                    totals["in_blocks"] += kind == KIND_SAMPLES
                    totals["last"] = count if totals["last"] is None else max(totals["last"], count)
                    if csv_out is not None and device_id == csv_device:
                        row = [repr(recording.seconds(count)), str(count), str(result), str(valid), str(connected)]
                        row += [repr(value) for value in values[5:]]
                        row += [str(lost_since), "1" if boundary_since else "0", "1" if pause_since else "0"]
                        csv_out.write(",".join(row) + "\n")
                        lost_since = 0
                        boundary_since = False
                        pause_since = False
            elif kind == KIND_SYNC and recording.compact:
                pass
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
                if payload[0] == 2:
                    totals["clips"] += 1
            elif kind == KIND_END:
                totals["end"] = END.unpack(payload)
            elif kind == KIND_ALIGNMENT:
                totals["alignments"] += 1
            elif kind == KIND_MODEL:
                pass
            elif kind == KIND_MARK:
                totals["marks"].append(payload)
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
    how = "raw"
    if recording.compact:
        # Position to a micrometer and rotation to a microradian; the velocities as the header says.
        how = (f"compact, velocity steps of {1e6 / recording.velocity_scale:g} um/s and "
               f"{1e6 / recording.angular_scale:g} urad/s")
    print(f"format {recording.version} ({how}), written by {recording.writer or '(no name)'}")
    print(f"started {start:%Y-%m-%d %H:%M:%S} UTC, clock {recording.clock_rate} ticks a second")
    end = totals["end"]
    # To the end record or the last sample, whichever is later: the end is
    # stamped when the file closes, the samples when they were captured.
    ends = [count for count in (end[1] if end else None, totals["last"]) if count is not None]
    span = max(recording.seconds(max(ends)), 0.0) if ends else 0.0
    paused = totals["paused"] / recording.clock_rate
    print(f"recorded {max(span - paused, 0.0):.3f} s over {span:.3f} s, {totals['samples']} samples, "
          f"{size_text(os.path.getsize(path))}")
    if recording.compact and totals["samples"]:
        print(f"samples in blocks: {totals['in_blocks']}, stored whole: {totals['samples'] - totals['in_blocks']}; "
              f"{os.path.getsize(path) / totals['samples']:.2f} bytes a sample")
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
    if totals["clips"]:
        print(f"clips: {totals['clips']}")
    for count, time, device, name in totals["marks"]:
        at = datetime.datetime.fromtimestamp(0, datetime.timezone.utc) + datetime.timedelta(microseconds=time)
        about = "" if device == NO_DEVICE else f", device {device}"
        print(f"mark at {recording.seconds(count):.3f} s ({at:%H:%M:%S} UTC): {name}{about}")
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
