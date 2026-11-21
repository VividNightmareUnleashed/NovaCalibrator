"""Read a Motion Recorder .novatrace. Use --csv --device 5 --start 2 --end 3 to inspect a window."""
import argparse
import collections
import csv
import math
import struct
import sys
import zlib


class Stream:
    """Little-endian binary blocks, independently compressed. At most 32 KiB of output per block."""
    def __init__(self, file):
        self.file = file
        self.block = b""
        self.at = 0
        header = file.read(32)
        if len(header) != 32 or header[:8] != b"NOVASTG1":
            raise ValueError("missing or unsupported tracking trace header")
        self.hz, self.start, self.utc = struct.unpack("<QqQ", header[8:])
        if not self.hz:
            raise ValueError("zero clock frequency")

    def next_block(self):
        header = self.file.read(4)
        if not header:
            return False
        if len(header) != 4:
            raise ValueError("partial block header")
        n, stored = struct.unpack("<HH", header)
        if not 0 < stored <= n <= 32768:
            raise ValueError("invalid block size")
        data = self.file.read(stored)
        if len(data) != stored:
            raise ValueError("partial block")
        if stored < n:
            if data[:2] != b"CK":
                raise ValueError("invalid MSZIP block")
            inflater = zlib.decompressobj(-15)
            data = inflater.decompress(data[2:], n + 1)
            if len(data) != n or not inflater.eof or inflater.unused_data:
                raise ValueError("damaged MSZIP block")
        self.block, self.at = data, 0
        return True

    def byte(self):
        if self.at == len(self.block) and not self.next_block():
            raise ValueError("recording ended before its closing record")
        value = self.block[self.at]
        self.at += 1
        return value

    def uint(self):
        result = 0
        for shift in range(0, 64, 7):
            byte = self.byte()
            if shift == 63 and byte > 1:
                break
            result |= (byte & 127) << shift
            if byte < 128:
                return result
        raise ValueError("invalid variable-length integer")

    def changes(self, base, width=64):
        mask = self.uint()
        if mask >> len(base):
            raise ValueError("unexpected pose field")
        value = base.copy()
        while mask:
            bit = (mask & -mask).bit_length() - 1
            value[bit] ^= self.uint()
            if value[bit] >= (1 << width):
                raise ValueError("pose field out of range")
            mask &= mask - 1
        return value


STAGES = ("input", "submitted", "resent", "input_head", "submitted_head", "resent_head")
REASONS = ("complete", "stopped", "driver data missing", "driver ended", "size limit", "capture failed")


def records(s):
    drivers = [[[0]*22 for _ in range(64)] for _ in STAGES]
    inputs = [None]*64
    runtime = [[0]*19 for _ in range(64)]
    origin = [0]*12
    time = s.start

    def stamp():
        nonlocal time
        n = s.uint()
        time += (n >> 1) ^ -(n & 1)
        if not -(1 << 63) <= time < 1 << 63:
            raise ValueError("clock count out of range")
        return time

    def device():
        n = s.byte()
        if n >= 64:
            raise ValueError("invalid device index")
        return n

    while True:
        kind = s.byte()
        if kind in range(1, 7) or kind == 130:
            paired = kind == 130
            k = (kind & 127) - 1
            dev, seq, t = device(), s.uint(), stamp()
            if seq >= 1 << 23 or (paired and inputs[dev] != seq):
                raise ValueError("invalid driver pair")
            base = drivers[0 if paired else k][dev]
            w = s.changes(base)
            drivers[k][dev] = w
            if k == 0:
                inputs[dev] = seq
            yield STAGES[k], t, dev, seq, w, (paired and w == base)
        elif kind == 7:
            t, span, mask = stamp(), s.uint(), s.uint()
            yield "poll", t, None, None, span, None
            while mask:
                dev = (mask & -mask).bit_length() - 1
                runtime[dev] = s.changes(runtime[dev], 32)
                yield "runtime", t, dev, None, runtime[dev], span
                mask &= mask - 1
        elif kind == 8:
            t, lost = stamp(), s.uint()
            inputs = [None]*64
            yield "gap", t, None, None, lost, None
        elif kind == 9:
            dev, cls = device(), s.uint()
            role = struct.unpack("<i", bytes(s.byte() for _ in range(4)))[0]
            name = bytes(s.byte() for _ in range(s.byte())).decode("utf-8", errors="replace")
            yield "device", None, dev, None, (cls, role, name), None
        elif kind == 10:
            t = stamp()
            origin = s.changes(origin, 32)
            yield "raw_to_standing", t, None, None, origin, None
        elif kind == 11:
            t, reason = stamp(), s.byte()
            if reason >= len(REASONS) or s.at != len(s.block) or s.file.read(1):
                raise ValueError("invalid closing record or trailing data")
            yield "end", t, None, None, REASONS[reason], None
            return
        else:
            raise ValueError("unknown record kind")


def numbers(values, bits):
    integer, real = ("I", "f") if bits == 32 else ("Q", "d")
    return struct.unpack(f"<{len(values)}{real}", struct.pack(f"<{len(values)}{integer}", *values))


TRANSFORM = "world_qw world_qx world_qy world_qz world_x world_y world_z".split()
MOTION = "qw qx qy qz x y z vx vy vz wx wy wz".split()
MATRIX = [f"m{i}{j}" for i in range(3) for j in range(4)]
FIELDS = ["stage", "time_s", "qpc", "query_end_qpc", "device", "sequence", "result", "valid", "connected",
          "pose_offset_s", *TRANSFORM, *MOTION, *MATRIX, "detail"]


def csv_row(s, stage, t, dev, seq, value, extra):
    row = dict(stage=stage, time_s=None if t is None else (t-s.start)/s.hz, qpc=t, device=dev, sequence=seq)
    if stage in STAGES:
        row.update(zip(["pose_offset_s", *TRANSFORM, *MOTION], numbers(value[1:], 64)))
        flags = value[0]
    elif stage == "runtime":
        row.update(zip(MATRIX + MOTION[-6:], numbers(value[:18], 32)))
        row.update(x=row["m03"], y=row["m13"], z=row["m23"], query_end_qpc=t+extra)
        flags = value[18]
    else:
        if stage == "raw_to_standing":
            row.update(zip(MATRIX, numbers(value, 32)))
        elif stage == "poll":
            row.update(query_end_qpc=t+value)
        else:
            row.update(detail=value)
        return row
    row.update(result=flags >> 2, valid=flags & 1, connected=(flags >> 1) & 1)
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("file")
    p.add_argument("--csv", action="store_true")
    p.add_argument("--device", type=int, choices=range(64), metavar="0..63")
    p.add_argument("--start", type=float, default=-math.inf, help="seconds from capture start")
    p.add_argument("--end", type=float, default=math.inf)
    args = p.parse_args()
    if args.start > args.end:
        p.error("--start must precede --end")
    counts = collections.defaultdict(collections.Counter)
    devices = {}
    with open(args.file, "rb") as file:
        s = Stream(file)
        out = csv.DictWriter(sys.stdout, fieldnames=FIELDS)
        if args.csv:
            out.writeheader()
        else:
            print(f"Start QPC {s.start}, frequency {s.hz} Hz, UTC Unix microseconds {s.utc}")
        losses = polls = 0
        last = None
        largest = 0
        for stage, t, dev, seq, v, extra in records(s):
            if stage == "device":
                devices[dev] = v
            if stage == "end":
                reason = v
            if dev is not None and args.device not in (None, dev):
                continue
            if t is not None and not args.start <= (t-s.start)/s.hz <= args.end:
                continue
            if args.csv:
                out.writerow(csv_row(s, stage, t, dev, seq, v, extra))
            elif stage == "poll":
                polls += 1
                if last is not None:
                    largest = max(largest, t-last)
                last = t
            elif stage == "gap":
                losses += v
            elif dev is not None and stage != "device":
                counts[dev][stage] += 1
                if stage == "submitted" and extra:
                    counts[dev]["identical"] += 1
        if not args.csv:
            print("Device   Input   Submitted   Identical*   Runtime   Resent   Controller type")
            for dev, c in sorted(counts.items()):
                print(f"{dev:6} {c['input']:7} {c['submitted']:11} {c['identical']:12} {c['runtime']:9} "
                      f"{c['resent']:8}   {devices.get(dev, ('','',''))[2]}")
            print(f"{polls} runtime queries, longest interval {1000*largest/s.hz:.2f} ms, "
                  f"{losses} lost driver records. Capture ended: {reason}.")
            print("* Identical main pose fields in a matched driver pair. Head transforms have separate rows.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, zlib.error) as error:
        sys.exit(str(error))
