#!/usr/bin/env python3
"""Measures how small a Nova Calibrator pose recording (.novapose) could be.

    python measure-recording.py FILE
        A plain-text report: for each device and for the whole file, the bytes
        a sample takes and the size of an hour at the recording's own rates,
        under the candidate encodings below.

    python measure-recording.py FILE --json OUT.json     (or --json - )
        Every figure of the report as JSON as well.

    python measure-recording.py FILE --max-seconds 60
        Stops after that much recorded time: a quick look at a large file.

    python measure-recording.py --self-test
        Encodes and decodes the reference recordings and a made-up noisy
        stream with the same functions the figures come from, and checks the
        round trip.

This tool only measures. It writes no recording and defines no format: the
"practical" sizes are those of simple encodings that the self-test proves can
be decoded, so that a compact format can be designed from numbers measured on
real trackers. The candidates:

  1. As stored: 189 bytes a sample.
  2. Constants removed: a field that seldom changes (the world-from-driver
     transform, the pose time offset, the flags, the tracking result) is
     stored only when it changes.
  3. Bit-exact: every double as the XOR of its bits with a prediction's.
  4. Quantized: position to 1 micrometer, rotation to 1 microradian, the
     velocities to two candidate steps, each as a whole number of steps from
     a prediction.
  5. The capture time as a difference from the expected interval.
  6. All of it again for the recording thinned to 10 samples a second.

A sample is never predicted from the one before it across a gap, a session
boundary, a pause, a resume (which is also where a clip begins) or an end
record: the first sample of each device behind such an edge is stored whole.

The file is read front to back with tools/read-recording.py's reader, and what
is kept in memory grows with the number of devices, not with the file. Only
the standard library is used. Pure Python is slow at this: expect a few
thousand samples a second, minutes for a five-minute recording.
"""

import argparse
import collections
import importlib.util
import json
import lzma
import math
import os
import random
import struct
import sys
import time
import zlib


def load_reader():
    """tools/read-recording.py as a module (its name has a hyphen)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "read-recording.py")
    spec = importlib.util.spec_from_file_location("read_recording", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


reader = load_reader()

# ---------------------------------------------------------------------------
# The sample's layout (docs/recording-format.md, "1: sample"): 186 payload
# bytes behind a 3-byte record prefix.

RECORD_BYTES = 189
PAYLOAD_BYTES = 186
HEAD = struct.Struct("<qIIBB")      # capture count, device id, tracking result, pose valid, connected
DOUBLES = struct.Struct("<21d")     # at byte 18: see the offsets below
BITS = struct.Struct("<21Q")        # the same 21 doubles as their 64 bits
MOVING_BITS = struct.Struct("<13Q")
ONE_DOUBLE = struct.Struct("<d")
ONE_BITS = struct.Struct("<Q")
SINGLES = struct.Struct("<21f")

# The fields that are a device's mounting and state: candidates 2 to 4 store
# one only when it differs from the device's previous sample. Name, payload
# bytes.
CONSTANTS = (
    ("pose time offset", 18, 26),
    ("world-from-driver rotation", 26, 58),
    ("world-from-driver translation", 58, 82),
    ("flags", 16, 18),
    ("tracking result", 12, 16),
)
CONSTANT_BYTES = sum(end - start for _, start, end in CONSTANTS)   # 70
# The pose itself: 13 doubles at payload bytes 82 to 186. Name, index of the
# first double among the sample's 21, how many, index among the 13.
MOVING_START = 82
MOVING_BYTES = 104
GROUPS = (
    ("rotation", 8, 4, 0),
    ("position", 12, 3, 4),
    ("velocity", 15, 3, 7),
    ("angular velocity", 18, 3, 10),
)
# What "repeats the previous sample" is counted for. Name, payload bytes.
REPEAT_GROUPS = (
    ("position", 114, 138),
    ("rotation", 82, 114),
    ("velocity", 138, 162),
    ("angular velocity", 162, 186),
    ("world-from-driver", 26, 82),
    ("pose time offset", 18, 26),
    ("all pose fields", 18, 186),
)
# What "is an exact single-precision number" is counted for. Name, first
# double, how many.
SINGLE_GROUPS = (
    ("position", 12, 3),
    ("rotation", 8, 4),
    ("velocity", 15, 3),
    ("angular velocity", 18, 3),
    ("world-from-driver", 1, 7),
    ("pose time offset", 0, 1),
)

# Candidate 4's steps.
POSITION_STEP = 1e-6            # meters
ROTATION_STEP = 1e-6            # radians
VELOCITY_STEPS = (1e-5, 1e-4)   # meters a second
ANGULAR_STEPS = (1e-5, 1e-4)    # radians a second
STEP_NAMES = ("10", "100")      # micro-units a second, for the two steps above
# How each group can be predicted. A prediction from a velocity uses the
# velocity as the decoder has it, so it exists once for each velocity step.
POSITION_VARIANTS = ("previous", "velocity@10", "velocity@100")
ROTATION_VARIANTS = ("previous", "world@10", "body@10", "world@100", "body@100")
# A quaternion this far from unit length is not a rotation: the sample is
# stored whole.
UNIT_TOLERANCE = 1e-3
# A residual of more steps than this is stored whole too.
MAX_SYMBOL = 1 << 40
# A prediction from a velocity is used only over a time step this long at most.
MAX_PREDICTED_SECONDS = 1.0

THIN_HZ = 10


# ---------------------------------------------------------------------------
# Small pieces both codecs share

def zigzag(number):
    return (number << 1) if number >= 0 else ((-number << 1) - 1)


def unzigzag(code):
    return (code >> 1) if not code & 1 else -((code + 1) >> 1)


def put_varint(out, code):
    while code >= 0x80:
        out.append((code & 0x7F) | 0x80)
        code >>= 7
    out.append(code)


def get_varint(data, at):
    code = 0
    shift = 0
    while True:
        byte = data[at]
        at += 1
        code |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return code, at
        shift += 7


def symbols_bytes(symbols):
    """Whole numbers as zigzag varints: one byte up to 63 either way."""
    out = bytearray()
    for symbol in symbols:
        put_varint(out, zigzag(symbol))
    return out


def get_symbols(data, at, count):
    symbols = []
    for _ in range(count):
        code, at = get_varint(data, at)
        symbols.append(unzigzag(code))
    return symbols, at


def bits_of(value):
    return ONE_BITS.unpack(ONE_DOUBLE.pack(value))[0]


def seconds_between(count, earlier, rate):
    """The time step a velocity prediction may use, or None."""
    step = (count - earlier) / rate
    return step if 0.0 < step <= MAX_PREDICTED_SECONDS else None


# ---- The front of an encoded sample: which device, when, and the fields that
# seldom change. One control byte says what follows.

WHOLE = 0x01     # nothing is predicted: the first sample behind an edge
ESCAPE = 0x40    # candidate 4 only: the pose is stored as its 13 doubles


class Front:
    """What both sides know of a device's previous sample."""
    __slots__ = ("payload", "count", "interval")

    def __init__(self):
        self.payload = None
        self.count = 0
        self.interval = 0


def put_device(out, device_id):
    if device_id < 255:
        out.append(device_id)
    else:
        out.append(255)
        out += struct.pack("<I", device_id)


def get_device(data, at):
    if data[at] < 255:
        return data[at], at + 1
    return struct.unpack_from("<I", data, at + 1)[0], at + 5


def encode_front(front, payload, flags):
    """The front of one sample. Returns (bytes, time bytes, constant bytes);
    the rest of the bytes are the device id and the control byte."""
    count, device_id = struct.unpack_from("<qI", payload, 0)
    out = bytearray()
    put_device(out, device_id)
    if flags & WHOLE:
        out.append(flags)
        out += payload[0:8]
        out += payload[12:MOVING_START]
        return out, 8, CONSTANT_BYTES
    control = flags
    changed = bytearray()
    for bit, (_, start, end) in enumerate(CONSTANTS):
        if payload[start:end] != front.payload[start:end]:
            control |= 2 << bit
            changed += payload[start:end]
    out.append(control)
    before = len(out)
    put_varint(out, zigzag(count - front.count - front.interval))
    timed = len(out) - before
    out += changed
    return out, timed, len(changed)


def decode_front(fronts, data, at):
    """Returns (device id, control byte, the payload's first 82 bytes, at)."""
    device_id, at = get_device(data, at)
    control = data[at]
    at += 1
    head = bytearray(MOVING_START)
    struct.pack_into("<I", head, 8, device_id)
    if control & WHOLE:
        head[0:8] = data[at:at + 8]
        head[12:MOVING_START] = data[at + 8:at + 8 + CONSTANT_BYTES]
        return device_id, control, head, at + 8 + CONSTANT_BYTES
    front = fronts[device_id]
    code, at = get_varint(data, at)
    struct.pack_into("<q", head, 0, front.count + front.interval + unzigzag(code))
    head[12:MOVING_START] = front.payload[12:MOVING_START]
    for bit, (_, start, end) in enumerate(CONSTANTS):
        if control & (2 << bit):
            head[start:end] = data[at:at + end - start]
            at += end - start
    return device_id, control, head, at


def advance_front(front, payload, whole):
    count = struct.unpack_from("<q", payload, 0)[0]
    front.interval = 0 if whole else count - front.count
    front.count = count
    front.payload = bytes(payload)


# ---------------------------------------------------------------------------
# Candidate 3: bit-exact. Each double is stored as the XOR of its 64 bits with
# a prediction's. A group of doubles is a byte that says which residuals are
# not zero, then for each of those a byte with the count of leading and of
# trailing zero bytes and the bytes between them.

def xor_bytes(residuals):
    out = bytearray(1)
    mask = 0
    for index, residual in enumerate(residuals):
        if residual:
            mask |= 1 << index
            lead = (64 - residual.bit_length()) >> 3
            trail = ((residual & -residual).bit_length() - 1) >> 3
            out.append((lead << 4) | trail)
            out += (residual >> (8 * trail)).to_bytes(8 - lead - trail, "big")
    out[0] = mask
    return out


def get_xor(data, at, count):
    mask = data[at]
    at += 1
    residuals = []
    for index in range(count):
        if not mask & (1 << index):
            residuals.append(0)
            continue
        lead, trail = data[at] >> 4, data[at] & 15
        size = 8 - lead - trail
        residuals.append(int.from_bytes(data[at + 1:at + 1 + size], "big") << (8 * trail))
        at += 1 + size
    return residuals, at


class ExactState(Front):
    __slots__ = ("bits", "doubles")

    def __init__(self):
        super().__init__()
        self.bits = None
        self.doubles = None


def position_from_velocity(state, count, rate):
    """The bits of the previous position plus the previous velocity times the
    time step, for each axis; the previous position's bits where that is not
    a number or the time step is not usable."""
    step = seconds_between(count, state.count, rate)
    predicted = list(state.bits[12:15])
    if step is None:
        return predicted
    for axis in range(3):
        value = state.doubles[12 + axis] + state.doubles[15 + axis] * step
        if math.isfinite(value):
            predicted[axis] = bits_of(value)
    return predicted


def encode_exact(state, payload, rate):
    """One sample against `state` (None behind an edge). Returns
    (front, groups, position by velocity, sizes, residuals): `groups` holds
    the four groups' bytes with every double predicted by its previous value,
    and the third item the position group predicted from the velocity."""
    if state is None or state.payload is None:
        front, timed, constant = encode_front(None, payload, WHOLE)
        return front, None, None, (timed, constant), None
    bits = BITS.unpack_from(payload, 18)
    count = struct.unpack_from("<q", payload, 0)[0]
    front, timed, constant = encode_front(state, payload, 0)
    residuals = [bits[index] ^ state.bits[index] for index in range(8, 21)]
    groups = [xor_bytes(residuals[at:at + size]) for _, _, size, at in GROUPS]
    predicted = position_from_velocity(state, count, rate)
    by_velocity = [bits[12 + axis] ^ predicted[axis] for axis in range(3)]
    return front, groups, xor_bytes(by_velocity), (timed, constant), (residuals, by_velocity)


def advance_exact(state, payload, whole):
    advance_front(state, payload, whole)
    state.bits = BITS.unpack_from(payload, 18)
    state.doubles = DOUBLES.unpack_from(payload, 18)


def exact_blob(states, payload, rate, by_velocity):
    """The whole encoded sample, as the self-test decodes it."""
    device_id = struct.unpack_from("<I", payload, 8)[0]
    state = states.get(device_id)
    front, groups, velocity_group, _, _ = encode_exact(state, payload, rate)
    if state is None:
        state = states[device_id] = ExactState()
    if groups is None:
        blob = front + payload[MOVING_START:]
    else:
        blob = front + groups[0] + (velocity_group if by_velocity else groups[1]) + groups[2] + groups[3]
    advance_exact(state, payload, groups is None)
    return bytes(blob)


def decode_exact(states, data, at, rate, by_velocity):
    """Returns (the sample's 186 payload bytes, at)."""
    device_id, control, head, at = decode_front(states, data, at)
    if control & WHOLE:
        payload = bytes(head) + bytes(data[at:at + MOVING_BYTES])
        at += MOVING_BYTES
        state = states.setdefault(device_id, ExactState())
    else:
        state = states[device_id]
        count = struct.unpack_from("<q", head, 0)[0]
        moving = []
        for name, first, size, _ in GROUPS:
            residuals, at = get_xor(data, at, size)
            predicted = state.bits[first:first + size]
            if name == "position" and by_velocity:
                predicted = position_from_velocity(state, count, rate)
            moving += [residual ^ predicted[index] for index, residual in enumerate(residuals)]
        payload = bytes(head) + MOVING_BITS.pack(*moving)
    advance_exact(state, payload, bool(control & WHOLE))
    return payload, at


# ---------------------------------------------------------------------------
# Candidate 4: quantized. Each value is a whole number of steps from a
# prediction, and the prediction is made from what the decoder has (the
# values as decoded, not as recorded), so an error never adds up.

def quantize(predicted, values, step):
    symbols = [round((value - guess) / step) for guess, value in zip(predicted, values)]
    return symbols, [guess + symbol * step for guess, symbol in zip(predicted, symbols)]


def dequantize(predicted, symbols, step):
    return [guess + symbol * step for guess, symbol in zip(predicted, symbols)]


def predict_position(position, velocity, step, by_velocity):
    if not by_velocity or step is None:
        return position
    return [position[axis] + velocity[axis] * step for axis in range(3)]


def multiply(a, b):
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def unit(q):
    length = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] / length, q[1] / length, q[2] / length, q[3] / length)


def rotation_of(vector):
    """The quaternion of a rotation vector (axis times angle, radians)."""
    angle = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if angle < 1e-9:
        return unit((1.0, 0.5 * vector[0], 0.5 * vector[1], 0.5 * vector[2]))
    scale = math.sin(0.5 * angle) / angle
    return (math.cos(0.5 * angle), vector[0] * scale, vector[1] * scale, vector[2] * scale)


def vector_of(q):
    """The rotation vector of a unit quaternion, by the shorter way round."""
    if q[0] < 0.0:
        q = (-q[0], -q[1], -q[2], -q[3])
    length = math.sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if length < 1e-12:
        return [2.0 * q[1], 2.0 * q[2], 2.0 * q[3]]
    scale = 2.0 * math.atan2(length, q[0]) / length
    return [q[1] * scale, q[2] * scale, q[3] * scale]


def predict_rotation(rotation, angular, step, frame):
    """`rotation` turned on by the angular velocity over `step`, with the
    angular velocity taken as a vector in driver space ("world") or in the
    device's own frame ("body"). `frame` None is no prediction."""
    if frame is None or step is None:
        return rotation
    turn = rotation_of([angular[0] * step, angular[1] * step, angular[2] * step])
    return unit(multiply(turn, rotation) if frame == "world" else multiply(rotation, turn))


def quantize_rotation(predicted, rotation, step):
    """The small rotation from the prediction to the sample, as a rotation
    vector in whole steps. Returns (symbols, the rotation as decoded, the
    vector before it was rounded).

    Each of the vector's three numbers is off by half a step at most, so the
    decoded rotation is within sqrt(3) / 2 steps of the sample's, as an angle:
    two rotations are never further apart than their vectors are."""
    exact = vector_of(multiply(conjugate(predicted), rotation))
    symbols = [round(part / step) for part in exact]
    return symbols, dequantize_rotation(predicted, symbols, step), exact


def dequantize_rotation(predicted, symbols, step):
    return unit(multiply(predicted, rotation_of([symbol * step for symbol in symbols])))


def usable(doubles):
    """Whether a sample's pose can be quantized: 13 numbers, and a rotation
    that is one."""
    for value in doubles[8:21]:
        if not math.isfinite(value):
            return False
    length = math.sqrt(sum(value * value for value in doubles[8:12]))
    return abs(length - 1.0) <= UNIT_TOLERANCE


def as_decoded(doubles):
    """A pose stored whole, as the decoder holds it."""
    return unit(doubles[8:12]), list(doubles[12:15]), list(doubles[15:18]), list(doubles[18:21])


class QuantConfig:
    """One way to store a pose: how position and rotation are predicted, and
    the steps of the two velocities."""

    def __init__(self, position_by_velocity, rotation_frame, velocity_step, angular_step):
        self.position_by_velocity = position_by_velocity
        self.rotation_frame = rotation_frame        # None, "world" or "body"
        self.velocity_step = velocity_step
        self.angular_step = angular_step

    def __str__(self):
        return (f"position from {'velocity' if self.position_by_velocity else 'previous'}, rotation "
                f"{self.rotation_frame or 'previous'}, steps {self.velocity_step:g} / {self.angular_step:g}")


class QuantState(Front):
    __slots__ = ("valid", "rotation", "position", "velocity", "angular")

    def __init__(self):
        super().__init__()
        self.valid = False
        self.rotation = self.position = self.velocity = self.angular = None


def advance_quant(state, payload, whole, decoded, valid):
    advance_front(state, payload, whole)
    state.valid = valid
    if decoded is not None:
        state.rotation, state.position, state.velocity, state.angular = decoded


def quant_symbols(state, doubles, step, config):
    """The twelve whole numbers of one pose and the pose as decoded, or None
    when a number is too large to be worth a residual."""
    velocity, new_velocity = quantize(state.velocity, doubles[15:18], config.velocity_step)
    angular, new_angular = quantize(state.angular, doubles[18:21], config.angular_step)
    guess = predict_position(state.position, state.velocity, step, config.position_by_velocity)
    position, new_position = quantize(guess, doubles[12:15], POSITION_STEP)
    turn = predict_rotation(state.rotation, state.angular, step, config.rotation_frame)
    rotation, new_rotation, _ = quantize_rotation(turn, unit(doubles[8:12]), ROTATION_STEP)
    symbols = velocity + angular + position + rotation
    if max(abs(symbol) for symbol in symbols) > MAX_SYMBOL:
        return None, None
    return symbols, (new_rotation, new_position, new_velocity, new_angular)


def quant_blob(states, payload, rate, config):
    """The whole encoded sample, as the self-test decodes it."""
    count, device_id = struct.unpack_from("<qI", payload, 0)
    doubles = DOUBLES.unpack_from(payload, 18)
    state = states.get(device_id)
    whole = state is None or state.payload is None
    if state is None:
        state = states[device_id] = QuantState()
    good = usable(doubles)
    symbols = decoded = None
    if not whole and good and state.valid:
        symbols, decoded = quant_symbols(state, doubles, seconds_between(count, state.count, rate), config)
    if symbols is None:
        flags = WHOLE if whole else ESCAPE
        front, _, _ = encode_front(None if whole else state, payload, flags)
        blob = front + payload[MOVING_START:]
        decoded = as_decoded(doubles) if good else None
    else:
        front, _, _ = encode_front(state, payload, 0)
        blob = front + symbols_bytes(symbols)
    advance_quant(state, payload, whole, decoded, good)
    return bytes(blob)


def decode_quant(states, data, at, rate, config):
    """Returns (the payload's first 82 bytes, the pose, at). The pose is its
    104 stored bytes for a sample stored whole, and (rotation, position,
    velocity, angular velocity) as decoded otherwise."""
    device_id, control, head, at = decode_front(states, data, at)
    state = states.setdefault(device_id, QuantState()) if control & WHOLE else states[device_id]
    count = struct.unpack_from("<q", head, 0)[0]
    if control & (WHOLE | ESCAPE):
        pose = bytes(data[at:at + MOVING_BYTES])
        at += MOVING_BYTES
        payload = bytes(head) + pose
        doubles = DOUBLES.unpack_from(payload, 18)
        good = usable(doubles)
        advance_quant(state, payload, bool(control & WHOLE), as_decoded(doubles) if good else None, good)
        return bytes(head), pose, at
    symbols, at = get_symbols(data, at, 12)
    step = seconds_between(count, state.count, rate)
    velocity = dequantize(state.velocity, symbols[0:3], config.velocity_step)
    angular = dequantize(state.angular, symbols[3:6], config.angular_step)
    position = dequantize(predict_position(state.position, state.velocity, step, config.position_by_velocity),
                          symbols[6:9], POSITION_STEP)
    rotation = dequantize_rotation(predict_rotation(state.rotation, state.angular, step, config.rotation_frame),
                                   symbols[9:12], ROTATION_STEP)
    decoded = (rotation, position, velocity, angular)
    # The front only needs the count and the constants of the previous sample.
    advance_quant(state, bytes(head) + bytes(MOVING_BYTES), False, decoded, True)
    return bytes(head), decoded, at


# ---------------------------------------------------------------------------
# The measurement

class Spread:
    """Mean and variance of numbers that come one at a time, for each axis."""
    __slots__ = ("count", "total", "squares")

    def __init__(self, axes=3):
        self.count = 0
        self.total = [0.0] * axes
        self.squares = [0.0] * axes

    def add(self, values):
        self.count += 1
        for axis, value in enumerate(values):
            self.total[axis] += value
            self.squares[axis] += value * value

    def deviations(self):
        if self.count == 0:
            return None
        return [math.sqrt(max(self.squares[axis] / self.count - (self.total[axis] / self.count) ** 2, 0.0))
                for axis in range(len(self.total))]

    def pooled(self):
        parts = self.deviations()
        return None if parts is None else math.sqrt(sum(part * part for part in parts) / len(parts))


class Symbols:
    """How often each whole number came, for each axis, to measure the bits
    an ideal coder would need. Bounded: past the limit a new number is
    counted as "another", which makes the entropy a little too low."""
    LIMIT = 200000
    __slots__ = ("counts", "others", "samples", "size", "spread")

    def __init__(self, axes=3):
        self.counts = [collections.Counter() for _ in range(axes)]
        self.others = 0
        self.samples = 0
        self.size = 0
        self.spread = Spread(axes)

    def add(self, symbols, size, residual=None):
        self.samples += 1
        self.size += size
        for axis, symbol in enumerate(symbols):
            counter = self.counts[axis]
            if symbol in counter or len(counter) < self.LIMIT:
                counter[symbol] += 1
            else:
                self.others += 1
        if residual is not None:
            self.spread.add(residual)

    def bits(self):
        """Bits a sample: the sum of each axis's own entropy."""
        if self.samples == 0:
            return None
        total = 0.0
        for counter in self.counts:
            seen = sum(counter.values())
            for count in counter.values():
                total -= count / seen * math.log2(count / seen)
        return total


class Pipe:
    """A stream of bytes through zlib and lzma, to say how far a general
    compressor gets on it. Only the sizes are kept."""
    CHUNK = 1 << 16

    def __init__(self):
        self.raw = 0
        self.deflate = zlib.compressobj(6)
        # A small dictionary: many of these run at once, and the residuals of
        # one device repeat within a few thousand samples or not at all.
        self.xz = lzma.LZMACompressor(format=lzma.FORMAT_XZ, filters=[
            {"id": lzma.FILTER_LZMA2, "preset": 6, "dict_size": 1 << 20}])
        self.zlib_bytes = 0
        self.lzma_bytes = 0
        self.waiting = bytearray()

    def write(self, data):
        self.raw += len(data)
        self.waiting += data
        if len(self.waiting) >= self.CHUNK:
            self._push()

    def _push(self):
        self.zlib_bytes += len(self.deflate.compress(self.waiting))
        self.lzma_bytes += len(self.xz.compress(self.waiting))
        self.waiting = bytearray()

    def close(self):
        self._push()
        self.zlib_bytes += len(self.deflate.flush())
        self.lzma_bytes += len(self.xz.flush())
        return {"bytes": self.raw, "zlib": self.zlib_bytes, "lzma": self.lzma_bytes}


class BitStats:
    """What the XOR residuals of one group look like."""
    __slots__ = ("values", "zero", "leading", "trailing", "size")

    def __init__(self):
        self.values = 0
        self.zero = 0
        self.leading = [0] * 65
        self.trailing = [0] * 65
        self.size = 0

    def add(self, residuals, size):
        self.size += size
        for residual in residuals:
            self.values += 1
            if residual == 0:
                self.zero += 1
                self.leading[64] += 1
                self.trailing[64] += 1
            else:
                self.leading[64 - residual.bit_length()] += 1
                self.trailing[(residual & -residual).bit_length() - 1] += 1

    @staticmethod
    def _summary(histogram):
        total = sum(histogram)
        if total == 0:
            return None
        mean = sum(bits * count for bits, count in enumerate(histogram)) / total
        quantiles = []
        for share in (0.1, 0.5, 0.9):
            seen = 0
            for bits, count in enumerate(histogram):
                seen += count
                if seen >= share * total:
                    quantiles.append(bits)
                    break
        return {"mean": mean, "p10": quantiles[0], "median": quantiles[1], "p90": quantiles[2]}

    def result(self):
        return {
            "values": self.values,
            "zero_share": self.zero / self.values if self.values else None,
            "leading_zero_bits": self._summary(self.leading),
            "trailing_zero_bits": self._summary(self.trailing),
            "bytes": self.size,
        }


class Options:
    def __init__(self, still_angular=0.02, still_linear=0.01, still_window=0.25):
        self.still_angular = still_angular    # radians a second
        self.still_linear = still_linear      # meters a second
        self.still_window = still_window      # seconds under both before a device counts as still


class DeviceMeasure:
    """Everything counted for one device."""

    def __init__(self):
        self.samples = 0
        self.whole = 0              # first samples behind an edge
        self.ticks = 0              # clock counts between samples of one stretch
        self.exact = ExactState()
        # Candidate 2.
        self.changed = [0] * len(CONSTANTS)
        self.front_bytes = 0        # device id, control byte, time and constants, as candidates 3 and 4 share them
        self.time_bytes = 0
        self.constant_bytes = 0
        # Candidate 3.
        self.bit_stats = {name: BitStats() for name, _, _, _ in GROUPS}
        self.bit_stats["position by velocity"] = BitStats()
        # Candidate 4.
        self.valid = False
        self.escapes = 0
        self.velocity = {}          # step name -> the velocity as decoded
        self.angular = {}
        self.position = {}          # variant -> the position as decoded
        self.rotation = {}
        self.symbols = {}
        for step in STEP_NAMES:
            self.symbols["velocity@" + step] = Symbols()
            self.symbols["angular@" + step] = Symbols()
        for variant in POSITION_VARIANTS:
            self.symbols["position " + variant] = Symbols()
        for variant in ROTATION_VARIANTS:
            self.symbols["rotation " + variant] = Symbols()
        # Candidate 5.
        self.intervals = collections.deque(maxlen=9)
        self.time_previous = Symbols(1)
        self.time_median = Symbols(1)
        # The rest.
        self.repeats = [0] * len(REPEAT_GROUPS)
        self.single = [0] * len(SINGLE_GROUPS)
        self.still_since = None
        self.spreads = {state + " " + what: Spread()
                        for state in ("still", "moving")
                        for what in ("position", "position by velocity", "velocity")}
        # Thinning: the 100 ms bin of the last sample kept in this stretch.
        self.first_count = None
        self.bin = None

    def edge(self):
        self.exact.payload = None
        self.valid = False
        self.intervals.clear()
        self.still_since = None
        self.first_count = None
        self.bin = None


class Analyzer:
    """One pass over the samples and edges of a recording. `thin` keeps the
    first sample of each device in every 1 / THIN_HZ seconds of a stretch."""

    def __init__(self, rate, options, thin=False):
        self.rate = rate
        self.options = options
        self.thin = thin
        self.devices = {}
        self.pipes = {}

    def edge(self):
        for device in self.devices.values():
            device.edge()

    def pipe(self, name):
        if name not in self.pipes:
            self.pipes[name] = Pipe()
        return self.pipes[name]

    def sample(self, payload):
        count, device_id = struct.unpack_from("<qI", payload, 0)
        device = self.devices.get(device_id)
        if device is None:
            device = self.devices[device_id] = DeviceMeasure()
        if self.thin:
            if device.first_count is None:
                device.first_count = count
            slot = (count - device.first_count) * THIN_HZ // self.rate
            if slot == device.bin:
                return
            device.bin = slot
        previous = device.exact if device.exact.payload is not None else None
        doubles = DOUBLES.unpack_from(payload, 18)
        device.samples += 1
        if previous is None:
            device.whole += 1
        else:
            device.ticks += max(count - previous.count, 0)
            self._describe(device, previous, payload, doubles, count)
        self._time(device, previous, count)
        self._exact(device, previous, payload)
        self._quantized(device, previous, doubles, count)
        advance_exact(device.exact, payload, previous is None)

    # ---- Candidate 2 and the plain facts about the stream.
    def _describe(self, device, previous, payload, doubles, count):
        for index, (_, start, end) in enumerate(CONSTANTS):
            if payload[start:end] != previous.payload[start:end]:
                device.changed[index] += 1
        for index, (_, start, end) in enumerate(REPEAT_GROUPS):
            if payload[start:end] == previous.payload[start:end]:
                device.repeats[index] += 1
        # Still: the device turns and moves slower than the thresholds, and
        # has done so for the window.
        options = self.options
        turn = math.sqrt(sum(value * value for value in doubles[18:21]))
        speed = math.sqrt(sum(value * value for value in doubles[15:18]))
        if turn < options.still_angular and speed < options.still_linear:
            if device.still_since is None:
                device.still_since = count
        else:
            device.still_since = None
        still = device.still_since is not None and count - device.still_since >= options.still_window * self.rate
        state = "still" if still else "moving"
        before = previous.doubles
        step = seconds_between(count, previous.count, self.rate)
        moved = [doubles[12 + axis] - before[12 + axis] for axis in range(3)]
        if all(math.isfinite(value) for value in moved):
            device.spreads[state + " position"].add(moved)
            if step is not None:
                missed = [moved[axis] - before[15 + axis] * step for axis in range(3)]
                if all(math.isfinite(value) for value in missed):
                    device.spreads[state + " position by velocity"].add(missed)
        faster = [doubles[15 + axis] - before[15 + axis] for axis in range(3)]
        if all(math.isfinite(value) for value in faster):
            device.spreads[state + " velocity"].add(faster)

    def _singles(self, device, doubles):
        try:
            narrowed = SINGLES.unpack(SINGLES.pack(*doubles))
        except OverflowError:
            # A value past what single precision holds: none of this sample's
            # values is counted, which is exact for that value at least.
            return
        for index, (_, first, size) in enumerate(SINGLE_GROUPS):
            device.single[index] += sum(1 for at in range(first, first + size) if narrowed[at] == doubles[at])

    # ---- Candidate 5.
    def _time(self, device, previous, count):
        if previous is None:
            return
        interval = count - previous.count
        expected = previous.interval
        device.time_previous.add([interval - expected], len(symbols_bytes([interval - expected])))
        middle = sorted(device.intervals)[len(device.intervals) // 2] if device.intervals else 0
        device.time_median.add([interval - middle], len(symbols_bytes([interval - middle])))
        device.intervals.append(interval)

    # ---- Candidate 3.
    def _exact(self, device, previous, payload):
        self._singles(device, DOUBLES.unpack_from(payload, 18))
        front, groups, by_velocity, sizes, residuals = encode_exact(previous, payload, self.rate)
        device.front_bytes += len(front)
        device.time_bytes += sizes[0]
        device.constant_bytes += sizes[1]
        if groups is None:
            whole = front + payload[MOVING_START:]
            self.pipe("bit-exact").write(whole)
            self.pipe("bit-exact, position by velocity").write(whole)
            return
        for (name, _, size, at), group in zip(GROUPS, groups):
            device.bit_stats[name].add(residuals[0][at:at + size], len(group))
        device.bit_stats["position by velocity"].add(residuals[1], len(by_velocity))
        self.pipe("bit-exact").write(front + groups[0] + groups[1] + groups[2] + groups[3])
        self.pipe("bit-exact, position by velocity").write(front + groups[0] + by_velocity + groups[2] + groups[3])

    # ---- Candidate 4: every variant at once, each with its own decoder's
    # view of the previous sample.
    def _quantized(self, device, previous, doubles, count):
        good = usable(doubles)
        if previous is None or not good or not device.valid:
            if previous is not None:
                device.escapes += 1
            device.valid = good
            if good:
                rotation, position, velocity, angular = as_decoded(doubles)
                device.velocity = {step: velocity for step in STEP_NAMES}
                device.angular = {step: angular for step in STEP_NAMES}
                device.position = {variant: position for variant in POSITION_VARIANTS}
                device.rotation = {variant: rotation for variant in ROTATION_VARIANTS}
            return
        step = seconds_between(count, previous.count, self.rate)
        results = []    # (key, symbols, residual before rounding), then the new state
        velocities, angulars, positions, rotations = {}, {}, {}, {}
        for name, size in zip(STEP_NAMES, VELOCITY_STEPS):
            symbols, velocities[name] = quantize(device.velocity[name], doubles[15:18], size)
            results.append(("velocity@" + name, symbols,
                            [doubles[15 + axis] - device.velocity[name][axis] for axis in range(3)]))
        for name, size in zip(STEP_NAMES, ANGULAR_STEPS):
            symbols, angulars[name] = quantize(device.angular[name], doubles[18:21], size)
            results.append(("angular@" + name, symbols,
                            [doubles[18 + axis] - device.angular[name][axis] for axis in range(3)]))
        for variant in POSITION_VARIANTS:
            by_velocity = variant != "previous"
            velocity = device.velocity[variant.split("@")[1]] if by_velocity else None
            guess = predict_position(device.position[variant], velocity, step, by_velocity)
            symbols, positions[variant] = quantize(guess, doubles[12:15], POSITION_STEP)
            results.append(("position " + variant, symbols, [doubles[12 + axis] - guess[axis] for axis in range(3)]))
        rotation = unit(doubles[8:12])
        for variant in ROTATION_VARIANTS:
            frame = None if variant == "previous" else variant.split("@")[0]
            angular = device.angular[variant.split("@")[1]] if frame else None
            turn = predict_rotation(device.rotation[variant], angular, step, frame)
            symbols, rotations[variant], exact = quantize_rotation(turn, rotation, ROTATION_STEP)
            results.append(("rotation " + variant, symbols, exact))
        if max(abs(symbol) for _, symbols, _ in results for symbol in symbols) > MAX_SYMBOL:
            device.escapes += 1
            rotation, position, velocity, angular = as_decoded(doubles)
            device.velocity = {name: velocity for name in STEP_NAMES}
            device.angular = {name: angular for name in STEP_NAMES}
            device.position = {variant: position for variant in POSITION_VARIANTS}
            device.rotation = {variant: rotation for variant in ROTATION_VARIANTS}
            return
        for key, symbols, residual in results:
            encoded = symbols_bytes(symbols)
            device.symbols[key].add(symbols, len(encoded), residual)
            self.pipe("quantized " + key).write(encoded)
        device.velocity, device.angular, device.position, device.rotation = velocities, angulars, positions, rotations

    # ---- The figures.
    def results(self):
        pipes = {name: pipe.close() for name, pipe in sorted(self.pipes.items())}
        devices = {device_id: self._device_result(device) for device_id, device in sorted(self.devices.items())}
        return {"devices": devices, "all": self._total(devices), "streams": pipes}

    def _device_result(self, device):
        samples = device.samples
        later = samples - device.whole     # samples that had a previous one
        seconds = device.ticks / self.rate
        rate = later / seconds if seconds > 0 else None
        out = {"samples": samples, "stored_whole": device.whole, "seconds": seconds, "rate_hz": rate}

        def each(total):
            return total / samples if samples else None

        def share(count, of):
            return count / of if of else None

        out["as_stored"] = float(RECORD_BYTES)
        out["constants"] = {
            "changed_share": {name: share(device.changed[index], later) for index, (name, _, _) in enumerate(CONSTANTS)},
            # The record as it is, with its constant fields behind a byte that
            # says which of them are there.
            "bytes_per_sample": each(samples * (RECORD_BYTES - CONSTANT_BYTES + 1) + device.constant_bytes),
        }
        front = device.front_bytes
        whole_pose = device.whole * MOVING_BYTES
        stats = {name: entry.result() for name, entry in device.bit_stats.items()}
        common = front + whole_pose + sum(stats[name]["bytes"] for name in ("rotation", "velocity", "angular velocity"))
        by_previous = each(common + stats["position"]["bytes"])
        by_velocity = each(common + stats["position by velocity"]["bytes"])
        out["bit_exact"] = {
            "bytes_per_sample": by_previous,
            "bytes_per_sample_position_by_velocity": by_velocity,
            "better_position": None if samples == 0 else "velocity" if by_velocity < by_previous else "previous",
            "front_bytes_per_sample": each(front),
            "groups": stats,
        }
        variants = {}
        for key, entry in device.symbols.items():
            bits = entry.bits()
            variants[key] = {
                "samples": entry.samples,
                "deviation": entry.spread.pooled(),
                "entropy_bits": bits,
                "bytes_per_coded_sample": entry.size / entry.samples if entry.samples else None,
                "bytes": entry.size,
                "entropy_bytes": None if bits is None else bits * entry.samples / 8.0,
            }
        totals = {}
        for step in STEP_NAMES:
            position = min(("position previous", "position velocity@" + step), key=lambda key: variants[key]["bytes"])
            rotation = min(("rotation previous", "rotation world@" + step, "rotation body@" + step),
                           key=lambda key: variants[key]["bytes"])
            chosen = (position, rotation, "velocity@" + step, "angular@" + step)
            raw_poses = (device.whole + device.escapes) * MOVING_BYTES
            ideal = None
            if all(variants[key]["entropy_bytes"] is not None for key in chosen) and device.time_previous.samples:
                # An ideal coder for the numbers and for the time; the rest as stored above.
                ideal = each(front - device.time_bytes + device.whole * 8 + raw_poses +
                             device.time_previous.bits() * device.time_previous.samples / 8.0 +
                             sum(variants[key]["entropy_bytes"] for key in chosen))
            totals[step] = {
                "position": position[len("position "):],
                "rotation": rotation[len("rotation "):],
                "bytes_per_sample": each(front + raw_poses + sum(variants[key]["bytes"] for key in chosen)),
                "entropy_bytes_per_sample": ideal,
            }
        out["quantized"] = {"escapes": device.escapes, "variants": variants, "totals": totals}
        out["capture_time"] = {}
        for name, entry in (("previous interval", device.time_previous), ("median of 9", device.time_median)):
            out["capture_time"][name] = {
                "entropy_bits": entry.bits(),
                "bytes_per_sample": entry.size / entry.samples if entry.samples else None,
            }
        out["repeats"] = {name: share(device.repeats[index], later) for index, (name, _, _) in enumerate(REPEAT_GROUPS)}
        out["single_precision"] = {name: share(device.single[index], samples * size)
                                   for index, (name, _, size) in enumerate(SINGLE_GROUPS)}
        out["residuals"] = {key: {"samples": spread.count, "deviation_by_axis": spread.deviations(),
                                  "deviation": spread.pooled()}
                            for key, spread in device.spreads.items()}
        return out

    @staticmethod
    def _total(devices):
        """The whole file: each device weighted by its own samples, and the
        hour's size as the sum of each device's bytes a second."""
        samples = sum(device["samples"] for device in devices.values())
        out = {"samples": samples, "rate_hz": sum(device["rate_hz"] or 0.0 for device in devices.values())}

        def weighted(pick):
            total = 0.0
            for device in devices.values():
                value = pick(device)
                if value is None:
                    return None
                total += value * device["samples"]
            return total / samples if samples else None

        out["as_stored"] = float(RECORD_BYTES)
        out["constants"] = weighted(lambda device: device["constants"]["bytes_per_sample"])
        out["bit_exact"] = weighted(lambda device: device["bit_exact"]["bytes_per_sample"])
        out["bit_exact_position_by_velocity"] = weighted(
            lambda device: device["bit_exact"]["bytes_per_sample_position_by_velocity"])
        out["bit_exact_best"] = weighted(lambda device: min(
            device["bit_exact"]["bytes_per_sample"], device["bit_exact"]["bytes_per_sample_position_by_velocity"]))
        for step in STEP_NAMES:
            out["quantized@" + step] = weighted(lambda device: device["quantized"]["totals"][step]["bytes_per_sample"])
            out["quantized@" + step + " entropy"] = weighted(
                lambda device: device["quantized"]["totals"][step]["entropy_bytes_per_sample"])
        return out


def bytes_per_hour(per_sample, device):
    """An hour of one device at the rate it was recorded at."""
    if per_sample is None or not device["rate_hz"]:
        return None
    return per_sample * device["rate_hz"] * 3600.0


def hour_of_all(result, pick):
    total = 0.0
    for device in result["devices"].values():
        value = bytes_per_hour(pick(device), device)
        if value is None:
            continue
        total += value
    return total


# ---------------------------------------------------------------------------
# Reading a file

def measure(path, options, max_seconds=None, progress=None):
    """Returns everything the report prints, as a dictionary."""
    with open(path, "rb") as stream:
        recording = reader.Recording(stream)
        if recording.compact:
            # Its samples are already in steps: there is nothing raw to measure.
            raise reader.NotARecording("it is a compact recording, and this tool measures raw ones (format version 1)")
        rate = recording.clock_rate
        described = {device.id: device for device in recording.devices}
        full = Analyzer(rate, options)
        thinned = Analyzer(rate, options, thin=True)
        edges = collections.Counter()
        marks = 0
        recorded = 0        # clock counts between samples of one stretch, all devices as one stream
        last = None
        state = {}
        started = time.monotonic()
        shown = started
        stopped_early = False
        size = os.path.getsize(path)
        for kind, payload in reader.each_record(recording, state):
            if kind == reader.KIND_SAMPLE:
                count = struct.unpack_from("<q", payload, 0)[0]
                if last is not None and count > last:
                    recorded += count - last
                last = count if last is None else max(last, count)
                full.sample(payload)
                thinned.sample(payload)
                if max_seconds is not None and recorded >= max_seconds * rate:
                    stopped_early = True
                    break
                if progress and time.monotonic() - shown >= 5.0:
                    shown = time.monotonic()
                    progress(recording.consumed, size, shown - started)
            elif kind in (reader.KIND_GAP, reader.KIND_BOUNDARY, reader.KIND_PAUSE, reader.KIND_RESUME,
                          reader.KIND_END):
                edges[kind] += 1
                full.edge()
                thinned.edge()
                last = None
            elif kind == reader.KIND_DEVICE:
                described[payload.id] = payload
            elif kind == reader.KIND_MARK:
                marks += 1
        ending = "stopped at --max-seconds" if stopped_early else state.get("file", "complete")
        result = {
            "file": os.path.basename(path),
            "writer": recording.writer,
            "clock_rate": rate,
            "recorded_seconds": recorded / rate,
            "file_end": ending,
            "file_error": state.get("error", ""),
            "edges": {"gaps": edges[reader.KIND_GAP], "boundaries": edges[reader.KIND_BOUNDARY],
                      "pauses": edges[reader.KIND_PAUSE], "resumes": edges[reader.KIND_RESUME],
                      "ends": edges[reader.KIND_END]},
            "marks": marks,
            "synthetic": is_synthetic(path, recording.writer),
            "still_rule": {"angular_below": options.still_angular, "linear_below": options.still_linear,
                           "for_seconds": options.still_window},
            "full": full.results(),
            "thinned": thinned.results(),
        }
        result["described"] = {
            str(device_id): {"class": reader.DEVICE_CLASSES.get(device.device_class, str(device.device_class)),
                             "label": device.label, "role": device.role_text(), "system": device.system}
            for device_id, device in described.items()}
        return result


def is_synthetic(path, writer):
    where = os.path.abspath(path).replace("\\", "/").lower()
    return (writer.startswith("novapose reference generator") or writer.startswith("Nova Calibrator preview") or
            "/tests/recordings/" in where)


# ---------------------------------------------------------------------------
# The report

SYNTHETIC_NOTE = ("NOTE: this recording is made-up test data (round numbers, no noise). Its figures show that the\n"
                  "tool works and say nothing about real trackers.")


def number(value, digits=2):
    return "-" if value is None else f"{value:.{digits}f}"


def percent(value):
    return "-" if value is None else f"{100.0 * value:.1f}%"


def size_text(count):
    if count is None:
        return "-"
    if count >= 1 << 30:
        return f"{count / (1 << 30):.2f} GiB"
    return f"{count / (1 << 20):.1f} MiB"


def small(value, unit_scale, digits=2):
    """A deviation in micro-units."""
    return "-" if value is None else f"{value * unit_scale:.{digits}f}"


def table(rows, right):
    widths = [max(len(row[column]) for row in rows) for column in range(len(rows[0]))]
    lines = []
    for row in rows:
        cells = [cell.rjust(widths[index]) if index in right else cell.ljust(widths[index])
                 for index, cell in enumerate(row)]
        lines.append("  " + "  ".join(cells).rstrip())
    return lines


def device_name(result, device_id):
    known = result["described"].get(str(device_id))
    if not known:
        return str(device_id)
    return f"{device_id} {known['role'] or known['class']}"


def report(result):
    lines = []
    out = lines.append
    full, thin = result["full"], result["thinned"]
    devices = full["devices"]
    names = {device_id: device_name(result, device_id) for device_id in devices}
    every = range(1, 20)

    out(f"{result['file']}: written by {result['writer'] or '(no name)'}")
    edges = result["edges"]
    out(f"{full['all']['samples']} samples of {len(devices)} device(s) in {result['recorded_seconds']:.1f} s recorded; "
        f"file {result['file_end']}" + (f" ({result['file_error']})" if result["file_error"] else ""))
    out(f"Edges: {edges['gaps']} gap(s), {edges['boundaries']} boundary(ies), {edges['pauses']} pause(s), "
        f"{edges['resumes']} resume(s), {edges['ends']} end(s). Marks: {result['marks']}.")
    if result["synthetic"]:
        out(SYNTHETIC_NOTE)
    out("Sizes are bytes a sample; an hour is at each device's own rate, in binary units (1 MiB = 1048576 bytes).")
    out("")

    def hours(pick):
        return size_text(hour_of_all(full, pick))

    out("SUMMARY, whole file")
    summary = full["all"]
    rows = [("candidate", "bytes/sample", "one hour")]
    rows.append(("1 as stored", number(summary["as_stored"]), hours(lambda device: device["as_stored"])))
    rows.append(("2 constants only when they change", number(summary["constants"]),
                 hours(lambda device: device["constants"]["bytes_per_sample"])))
    rows.append(("3 bit-exact, each value from the previous", number(summary["bit_exact"]),
                 hours(lambda device: device["bit_exact"]["bytes_per_sample"])))
    rows.append(("3 bit-exact, position from velocity", number(summary["bit_exact_position_by_velocity"]),
                 hours(lambda device: device["bit_exact"]["bytes_per_sample_position_by_velocity"])))
    for step in STEP_NAMES:
        rows.append((f"4 quantized, velocities to {step} micro-units/s", number(summary["quantized@" + step]),
                     hours(lambda device: device["quantized"]["totals"][step]["bytes_per_sample"])))
        rows.append(("    the same with an ideal coder", number(summary["quantized@" + step + " entropy"]),
                     hours(lambda device: device["quantized"]["totals"][step]["entropy_bytes_per_sample"])))
    lines.extend(table(rows, {1, 2}))
    out("  Candidates 3 and 4 are sizes of real encodings (below), each sample with its device, its time and its")
    out("  constants. \"Ideal coder\" replaces the varints of candidate 4 and of the time by their measured entropy.")
    out("")

    out("DEVICES")
    rows = [("device", "system", "samples", "Hz", "stored whole", "seconds")]
    for device_id, device in devices.items():
        known = result["described"].get(str(device_id), {})
        rows.append((names[device_id], known.get("system", "") or "-", str(device["samples"]),
                     number(device["rate_hz"], 1), str(device["stored_whole"]), number(device["seconds"], 1)))
    lines.extend(table(rows, set(every[1:])))
    out("  \"Stored whole\": first samples behind a gap, a boundary, a pause, a resume or the start. Nothing is")
    out("  predicted across those.")
    out("")

    out("2. CONSTANTS: share of samples in which the field differs from the device's previous sample")
    rows = [("device",) + tuple(name.replace("world-from-driver", "w-f-d").replace("pose time", "time").replace("tracking ", "")
                                for name, _, _ in CONSTANTS) + ("bytes", "one hour")]
    for device_id, device in devices.items():
        constants = device["constants"]
        rows.append((names[device_id],) + tuple(percent(constants["changed_share"][name]) for name, _, _ in CONSTANTS) +
                    (number(constants["bytes_per_sample"]),
                     size_text(bytes_per_hour(constants["bytes_per_sample"], device))))
    lines.extend(table(rows, set(every)))
    out("  w-f-d: world-from-driver. Size: the record as stored, with a byte that says which of these fields follow.")
    out("")

    out("3. BIT-EXACT: every double as the XOR of its bits with a prediction's")
    out("  Encoding: device byte, control byte, the time as a zigzag varint of the difference from the previous")
    out("  interval, the changed constants, then for each group a byte that says which residuals are not zero and,")
    out("  for each of those, one byte (leading and trailing zero bytes) and the bytes between.")
    rows = [("device", "from previous", "position from velocity", "better", "one hour (better)")]
    for device_id, device in devices.items():
        exact = device["bit_exact"]
        best = min(exact["bytes_per_sample"], exact["bytes_per_sample_position_by_velocity"])
        rows.append((names[device_id], number(exact["bytes_per_sample"]),
                     number(exact["bytes_per_sample_position_by_velocity"]), exact["better_position"] or "-",
                     size_text(bytes_per_hour(best, device))))
    rows.append(("all", number(summary["bit_exact"]), number(summary["bit_exact_position_by_velocity"]), "",
                 hours(lambda device: min(device["bit_exact"]["bytes_per_sample"],
                                          device["bit_exact"]["bytes_per_sample_position_by_velocity"]))))
    lines.extend(table(rows, {1, 2, 4}))
    for name in ("bit-exact", "bit-exact, position by velocity"):
        stream = full["streams"].get(name)
        if stream and full["all"]["samples"]:
            count = full["all"]["samples"]
            out(f"  The whole stream ({name}): {stream['bytes'] / count:.2f} bytes/sample; through zlib "
                f"{stream['zlib'] / count:.2f}, through lzma {stream['lzma'] / count:.2f}.")
    out("  Residual bits (leading zeros are the sign, the exponent and the high digits that agree; trailing zeros")
    out("  are low digits both lack). Mean / median, of 64:")
    rows = [("device", "group", "exactly zero", "leading zero bits", "trailing zero bits", "bytes/value")]
    for device_id, device in devices.items():
        for group, stats in device["bit_exact"]["groups"].items():
            if not stats["values"]:
                continue
            lead, trail = stats["leading_zero_bits"], stats["trailing_zero_bits"]
            rows.append((names[device_id], group, percent(stats["zero_share"]),
                         f"{lead['mean']:.1f} / {lead['median']}", f"{trail['mean']:.1f} / {trail['median']}",
                         number(stats["bytes"] / stats["values"])))
    lines.extend(table(rows, {2, 3, 4, 5}))
    out("")

    out("4. QUANTIZED: position to 1 um, rotation to 1 urad (the small rotation from the prediction to the sample),")
    out("  velocity to 10 or 100 um/s, angular velocity to 10 or 100 urad/s. Each value is a whole number of steps")
    out("  from a prediction made of decoded values; encoding: zigzag varints, three a group. A decoded value is")
    out("  within half a step of the recorded one; a decoded rotation within 0.87 urad of it, as an angle.")
    out("  Deviation: of the prediction's miss, before rounding, in um, urad, um/s, urad/s. Entropy: bits a sample,")
    out("  the sum over the three axes of each axis's own entropy. Bytes: a sample, as varints.")
    rows = [("device", "group and prediction", "deviation", "entropy bits", "bytes")]
    for device_id, device in devices.items():
        for key, variant in device["quantized"]["variants"].items():
            if not variant["samples"]:
                continue
            rows.append((names[device_id], key, small(variant["deviation"], 1e6), number(variant["entropy_bits"], 1),
                         number(variant["bytes_per_coded_sample"])))
    lines.extend(table(rows, {2, 3, 4}))
    out("  Rotation predictions: \"previous\" is none; \"world\" and \"body\" turn the previous rotation on by the")
    out("  angular velocity, taken as a vector in driver space or in the device's own frame. The smaller wins.")
    out("  Steps: of the two velocities, in micro-units a second. Whole: samples whose pose is stored as it is.")
    rows = [("device", "steps", "position from", "rotation from", "bytes/sample", "ideal coder", "one hour",
             "whole")]
    for device_id, device in devices.items():
        for step in STEP_NAMES:
            total = device["quantized"]["totals"][step]
            rows.append((names[device_id], step, total["position"], total["rotation"], number(total["bytes_per_sample"]),
                         number(total["entropy_bytes_per_sample"]),
                         size_text(bytes_per_hour(total["bytes_per_sample"], device)),
                         str(device["stored_whole"] + device["quantized"]["escapes"])))
    for step in STEP_NAMES:
        rows.append(("all", step, "", "", number(summary["quantized@" + step]),
                     number(summary["quantized@" + step + " entropy"]),
                     hours(lambda device: device["quantized"]["totals"][step]["bytes_per_sample"]), ""))
    lines.extend(table(rows, {1, 4, 5, 6, 7}))
    out("  Each group's varints through zlib and lzma, whole file, bytes a coded sample:")
    rows = [("group and prediction", "varints", "zlib", "lzma")]
    for name, stream in full["streams"].items():
        if not name.startswith("quantized "):
            continue
        key = name[len("quantized "):]
        coded = sum(device["quantized"]["variants"][key]["samples"] for device in devices.values())
        if coded:
            rows.append((key, number(stream["bytes"] / coded), number(stream["zlib"] / coded),
                         number(stream["lzma"] / coded)))
    lines.extend(table(rows, {1, 2, 3}))
    out("")

    out("5. CAPTURE TIME: the count as a difference from the expected interval, in clock counts")
    rows = [("device", "expected", "entropy bits", "bytes as varint")]
    for device_id, device in devices.items():
        for name, entry in device["capture_time"].items():
            rows.append((names[device_id], name, number(entry["entropy_bits"], 1), number(entry["bytes_per_sample"])))
    lines.extend(table(rows, {2, 3}))
    out("  Candidates 3 and 4 use the previous interval.")
    out("")

    out(f"6. THINNED to {THIN_HZ} samples a second for each device (the first sample of each {1000 // THIN_HZ} ms). Bit-exact: the")
    out("  better of its two position predictions. One hour: quantized, velocities to 100 micro-units a second.")
    rows = [("device", "samples", "Hz", "as stored", "bit-exact", "quantized 10", "quantized 100",
             "one hour (100)")]
    for device_id, device in thin["devices"].items():
        exact = device["bit_exact"]
        totals = device["quantized"]["totals"]
        rows.append((names.get(device_id, str(device_id)), str(device["samples"]), number(device["rate_hz"], 1),
                     number(device["as_stored"]),
                     number(min(exact["bytes_per_sample"], exact["bytes_per_sample_position_by_velocity"])),
                     number(totals["10"]["bytes_per_sample"]), number(totals["100"]["bytes_per_sample"]),
                     size_text(bytes_per_hour(totals["100"]["bytes_per_sample"], device))))
    all_thin = thin["all"]
    rows.append(("all", str(all_thin["samples"]), number(all_thin["rate_hz"], 1), number(all_thin["as_stored"]),
                 number(all_thin["bit_exact_best"]), number(all_thin["quantized@10"]), number(all_thin["quantized@100"]),
                 size_text(hour_of_all(thin, lambda device: device["quantized"]["totals"]["100"]["bytes_per_sample"]))))
    lines.extend(table(rows, set(every)))
    out("")

    out("REPEATS: share of samples whose field group is the previous sample's, bit for bit")
    rows = [("device",) + tuple(name.replace("world-from-driver", "w-f-d").replace("pose time", "time")
                                for name, _, _ in REPEAT_GROUPS)]
    for device_id, device in devices.items():
        rows.append((names[device_id],) + tuple(percent(device["repeats"][name]) for name, _, _ in REPEAT_GROUPS))
    lines.extend(table(rows, set(every)))
    out("")

    out("SINGLE PRECISION: share of values that a 32-bit float holds exactly")
    rows = [("device",) + tuple(name.replace("world-from-driver", "w-f-d").replace("pose time", "time")
                                for name, _, _ in SINGLE_GROUPS)]
    for device_id, device in devices.items():
        rows.append((names[device_id],) +
                    tuple(percent(device["single_precision"][name]) for name, _, _ in SINGLE_GROUPS))
    lines.extend(table(rows, set(every)))
    out("")

    rule = result["still_rule"]
    out("STILL AND MOVING: deviation of the change from one sample to the next, all three axes together")
    out(f"  Still: the angular velocity is under {rule['angular_below']:g} rad/s and the velocity under "
        f"{rule['linear_below']:g} m/s,")
    out(f"  and both have been for {rule['for_seconds']:g} s. Moving: every other sample.")
    rows = [("device", "state", "samples", "position um", "position less velocity x time um", "velocity um/s")]
    for device_id, device in devices.items():
        for state in ("still", "moving"):
            residuals = device["residuals"]
            rows.append((names[device_id], state, str(residuals[state + " position"]["samples"]),
                         small(residuals[state + " position"]["deviation"], 1e6),
                         small(residuals[state + " position by velocity"]["deviation"], 1e6),
                         small(residuals[state + " velocity"]["deviation"], 1e6)))
    lines.extend(table(rows, {2, 3, 4, 5}))
    if result["synthetic"]:
        out("")
        out(SYNTHETIC_NOTE)
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# The self-test

def synthetic_events(seed=7):
    """A made-up recording as a list of ("sample", payload) and ("edge",):
    three devices at different rates, still and then moving, with noise, a gap
    and a pause in the middle, one sample that is not a number, and a headset
    whose values are single-precision numbers."""
    noise = random.Random(seed)
    rate = 10000000
    events = []
    # Device id, samples a second, the frame its angular velocity is in.
    plans = ((0, 72, "world"), (3, 400, "world"), (5, 250, "body"))
    samples = []
    for device_id, hertz, frame in plans:
        rotation = (1.0, 0.0, 0.0, 0.0)
        for index in range(int(4.0 * hertz)):
            at = index / hertz
            moving = at >= 2.0
            sway = (at - 2.0) if moving else 0.0
            position = [0.3 * device_id + 0.4 * math.sin(1.7 * sway), 1.0 + 0.2 * math.sin(2.3 * sway),
                        -0.5 + 0.3 * math.sin(0.9 * sway)]
            velocity = [0.4 * 1.7 * math.cos(1.7 * sway), 0.2 * 2.3 * math.cos(2.3 * sway),
                        0.3 * 0.9 * math.cos(0.9 * sway)] if moving else [0.0, 0.0, 0.0]
            angular = [0.9 * math.sin(1.1 * sway), 1.4, 0.6 * math.cos(0.7 * sway)] if moving else [0.0, 0.0, 0.0]
            turn = rotation_of([value / hertz for value in angular])
            rotation = unit(multiply(turn, rotation) if frame == "world" else multiply(rotation, turn))
            shaken = unit(multiply(rotation, rotation_of([noise.gauss(0.0, 2e-4) for _ in range(3)])))
            doubles = [-0.004 if device_id else 0.0]
            doubles += [0.5, 0.5, -0.5, 0.5] if device_id else [1.0, 0.0, 0.0, 0.0]
            doubles += [0.125, -1.5, 2.25] if device_id else [0.0, 0.0, 0.0]
            doubles += list(shaken)
            doubles += [value + noise.gauss(0.0, 1e-4) for value in position]
            doubles += [value + noise.gauss(0.0, 2e-3) for value in velocity]
            doubles += [value + noise.gauss(0.0, 5e-3) for value in angular]
            if device_id == 0:
                doubles = [struct.unpack("<f", struct.pack("<f", value))[0] for value in doubles]
            if device_id == 3 and index == 700:
                doubles[12] = float("nan")
            count = 5000000000 + int(at * rate) + noise.randint(-300, 300)
            lost = device_id == 5 and index % 97 == 96
            samples.append((count, device_id, HEAD.pack(count, device_id, 101 if lost else 200, 0 if lost else 1, 1) +
                            DOUBLES.pack(*doubles)))
    samples.sort(key=lambda sample: (sample[0], sample[1]))
    for index, (_, _, payload) in enumerate(samples):
        if index in (len(samples) * 3 // 8, len(samples) * 3 // 4):
            events.append(("edge",))
        events.append(("sample", payload))
    return rate, events


def file_events(path):
    """A recording file as the same list. A compact file's samples come as
    the 186 bytes a raw file would hold them in."""
    events = []
    with open(path, "rb") as stream:
        recording = reader.Recording(stream)
        for kind, payload in reader.each_record(recording, {}):
            if kind == reader.KIND_SAMPLE or (kind == reader.KIND_SAMPLES and recording.compact):
                events += [("sample", whole) for _, whole in reader.each_sample(recording, kind, payload)]
            elif kind in (reader.KIND_GAP, reader.KIND_BOUNDARY, reader.KIND_PAUSE, reader.KIND_RESUME,
                          reader.KIND_END):
                events.append(("edge",))
        return recording.clock_rate, events


def first_samples(events):
    """Which samples are the first of their device behind an edge."""
    seen = set()
    first = []
    for event in events:
        if event[0] == "edge":
            seen.clear()
            continue
        device_id = struct.unpack_from("<I", event[1], 8)[0]
        first.append(device_id not in seen)
        seen.add(device_id)
    return first


def blob_is_whole(blob):
    _, at = get_device(blob, 0)
    return bool(blob[at] & WHOLE)


def check_exact(name, rate, events, by_velocity):
    """Encodes and decodes every sample bit for bit. Returns the bytes."""
    encoders, decoders = {}, {}
    total = 0
    expected = first_samples(events)
    index = 0
    for event in events:
        if event[0] == "edge":
            for state in encoders.values():
                state.payload = None
            continue
        blob = exact_blob(encoders, event[1], rate, by_velocity)
        total += len(blob)
        if blob_is_whole(blob) != expected[index]:
            raise AssertionError(f"{name}: sample {index} is {'not ' if expected[index] else ''}stored whole")
        payload, at = decode_exact(decoders, blob, 0, rate, by_velocity)
        if at != len(blob) or payload != bytes(event[1]):
            raise AssertionError(f"{name}: sample {index} does not come back bit for bit")
        index += 1
    return total


def angle_between(a, b):
    return vector_of(multiply(conjugate(a), b))


def check_quant(name, rate, events, config):
    """Encodes and decodes every sample to within half a step. Returns the
    bytes."""
    encoders, decoders = {}, {}
    total = 0
    expected = first_samples(events)
    index = 0
    slack = 1.0 + 1e-6
    for event in events:
        if event[0] == "edge":
            for state in encoders.values():
                state.payload = None
            continue
        payload = bytes(event[1])
        blob = quant_blob(encoders, payload, rate, config)
        total += len(blob)
        if blob_is_whole(blob) != expected[index]:
            raise AssertionError(f"{name}: sample {index} is {'not ' if expected[index] else ''}stored whole")
        head, pose, at = decode_quant(decoders, blob, 0, rate, config)
        where = f"{name}, {config}: sample {index}"
        if at != len(blob) or head != payload[:MOVING_START]:
            raise AssertionError(f"{where}: the device, the time or a constant does not come back exactly")
        if isinstance(pose, bytes):
            if pose != payload[MOVING_START:]:
                raise AssertionError(f"{where}: a pose stored whole does not come back bit for bit")
        else:
            doubles = DOUBLES.unpack_from(payload, 18)
            rotation, position, velocity, angular = pose
            for axis in range(3):
                if abs(position[axis] - doubles[12 + axis]) > 0.5 * POSITION_STEP * slack + 1e-12:
                    raise AssertionError(f"{where}: position is off by more than half a step")
                if abs(velocity[axis] - doubles[15 + axis]) > 0.5 * config.velocity_step * slack + 1e-12:
                    raise AssertionError(f"{where}: velocity is off by more than half a step")
                if abs(angular[axis] - doubles[18 + axis]) > 0.5 * config.angular_step * slack + 1e-12:
                    raise AssertionError(f"{where}: angular velocity is off by more than half a step")
            # Half a step on each axis of the stored vector is this much as an angle.
            off = math.sqrt(sum(part * part for part in angle_between(rotation, unit(doubles[8:12]))))
            if off > 0.5 * math.sqrt(3.0) * ROTATION_STEP * slack + 1e-12:
                raise AssertionError(f"{where}: rotation is off by {off:.3g} rad")
        index += 1
    return total


def analyzer_of(rate, events):
    analyzer = Analyzer(rate, Options())
    for event in events:
        if event[0] == "edge":
            analyzer.edge()
        else:
            analyzer.sample(event[1])
    return analyzer.results()


def self_test():
    folder = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Tests", "recordings")
    sets = []
    for entry in sorted(os.listdir(folder)):
        if entry.endswith(".novapose"):
            rate, events = file_events(os.path.join(folder, entry))
            sets.append((entry, rate, events))
    if len(sets) < 7:
        print(f"self-test failed: {len(sets)} reference recordings in {folder}, expected 7 or more")
        return 1
    rate, events = synthetic_events()
    sets.append(("the noisy made-up stream", rate, events))
    configs = [QuantConfig(by_velocity, frame, VELOCITY_STEPS[step], ANGULAR_STEPS[step])
               for by_velocity in (False, True) for frame in (None, "world", "body") for step in (0, 1)]
    try:
        for name, rate, events in sets:
            count = sum(1 for event in events if event[0] == "sample")
            edges = len(events) - count
            sizes = [check_exact(name, rate, events, by_velocity) for by_velocity in (False, True)]
            print(f"ok  bit-exact round trip, {name}: {count} samples, {edges} edge(s), "
                  f"{sizes[0] / max(count, 1):.1f} and {sizes[1] / max(count, 1):.1f} bytes/sample")
            quant_sizes = {str(config): check_quant(name, rate, events, config) for config in configs}
            print(f"ok  quantized round trip within half a step (rotation: 0.87 urad), {name}: {len(configs)} ways, "
                  f"{min(quant_sizes.values()) / max(count, 1):.1f} to "
                  f"{max(quant_sizes.values()) / max(count, 1):.1f} bytes/sample")

            # What the report prints is the size of exactly these streams.
            measured = analyzer_of(rate, events)
            streams = measured["streams"]
            if count and (streams["bit-exact"]["bytes"] != sizes[0] or
                          streams["bit-exact, position by velocity"]["bytes"] != sizes[1]):
                raise AssertionError(f"{name}: the measured bit-exact size is not the size of the decoded stream")
            whole = sum(device["stored_whole"] for device in measured["devices"].values())
            if whole != sum(first_samples(events)):
                raise AssertionError(f"{name}: a sample was predicted across an edge")
            for by_velocity, frame, step in ((False, None, 0), (True, "world", 0), (True, "body", 1)):
                config = QuantConfig(by_velocity, frame, VELOCITY_STEPS[step], ANGULAR_STEPS[step])
                position = "position " + ("velocity@" + STEP_NAMES[step] if by_velocity else "previous")
                rotation = "rotation " + (frame + "@" + STEP_NAMES[step] if frame else "previous")
                reported = 0
                for device in measured["devices"].values():
                    variants = device["quantized"]["variants"]
                    reported += round(device["bit_exact"]["front_bytes_per_sample"] * device["samples"])
                    reported += (device["stored_whole"] + device["quantized"]["escapes"]) * MOVING_BYTES
                    reported += sum(variants[key]["bytes"] for key in (
                        position, rotation, "velocity@" + STEP_NAMES[step], "angular@" + STEP_NAMES[step]))
                if reported != quant_sizes[str(config)]:
                    raise AssertionError(f"{name}, {config}: the measured size {reported} is not the size of the "
                                         f"decoded stream, {quant_sizes[str(config)]}")
            print(f"ok  the measured sizes are the sizes of the decoded streams, {name}")

        # The made-up stream, as a check of the checks: the device that reports
        # its angular velocity in its own frame is predicted best that way.
        measured = analyzer_of(*synthetic_events()[0:2])
        winners = {device_id: device["quantized"]["totals"]["10"]["rotation"]
                   for device_id, device in measured["devices"].items()}
        if not winners[3].startswith("world") or not winners[5].startswith("body"):
            raise AssertionError(f"the rotation predictions do not tell the two frames apart: {winners}")
        print(f"ok  rotation is predicted best in the frame each made-up device reports in: {winners}")
        single = measured["devices"][0]["single_precision"]["position"]
        if single != 1.0 or measured["devices"][3]["single_precision"]["position"] > 0.01:
            raise AssertionError("single-precision values are not told from doubles")
        print("ok  single-precision values are told from doubles")
        if measured["devices"][3]["quantized"]["escapes"] < 1:
            raise AssertionError("the sample that is not a number was not stored whole")
        print("ok  a sample that is not a number is stored whole, and so is the one behind it")
    except AssertionError as problem:
        print(f"self-test failed: {problem}")
        return 1
    print("self-test passed")
    return 0


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Measures how small a Nova Calibrator pose recording (.novapose) could be stored.")
    parser.add_argument("file", nargs="?", help="the recording")
    parser.add_argument("--json", metavar="FILE", help="also write every figure as JSON (- for standard output, "
                        "in place of the report)")
    parser.add_argument("--max-seconds", type=float, metavar="N", help="stop after N seconds of recorded time")
    parser.add_argument("--still-angular", type=float, default=0.02, metavar="RAD_S",
                        help="a device is still under this angular velocity (default 0.02 rad/s)")
    parser.add_argument("--still-linear", type=float, default=0.01, metavar="M_S",
                        help="and under this velocity (default 0.01 m/s)")
    parser.add_argument("--still-window", type=float, default=0.25, metavar="S",
                        help="for this long (default 0.25 s)")
    parser.add_argument("--self-test", action="store_true",
                        help="check the encodings' round trips on the reference recordings and a made-up stream")
    arguments = parser.parse_args()
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    if arguments.self_test:
        return self_test()
    if not arguments.file:
        parser.error("give a recording, or --self-test")

    def progress(done, size, seconds):
        print(f"  {100.0 * done / max(size, 1):.0f}% read after {seconds:.0f} s", file=sys.stderr)

    options = Options(arguments.still_angular, arguments.still_linear, arguments.still_window)
    try:
        result = measure(arguments.file, options, arguments.max_seconds, progress)
    except reader.NotARecording as problem:
        print(f"{arguments.file} is not a recording this tool can use: {problem}.", file=sys.stderr)
        return 2
    except OSError as problem:
        print(f"{arguments.file} could not be read: {problem}", file=sys.stderr)
        return 2
    if arguments.json == "-":
        json.dump(result, sys.stdout, indent=1)
        sys.stdout.write("\n")
        return 0
    sys.stdout.write(report(result))
    if arguments.json:
        with open(arguments.json, "w", encoding="utf-8") as out:
            json.dump(result, out, indent=1)
            out.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
