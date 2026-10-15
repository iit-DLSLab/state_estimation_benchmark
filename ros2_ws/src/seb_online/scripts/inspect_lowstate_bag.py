#!/usr/bin/env python3
"""Inspect unitree_go/LowState in a rosbag2 (sqlite3 .db3) recording.

Reports what the online pipeline relies on and what must be validated per robot:
  * message rate and reception-time jitter (bag reception timestamps)
  * LowState.tick: increments per message, estimated tick period (fit of tick
    vs reception time), repeated ticks, exact duplicate messages
  * leg ordering evidence: correlation of foot_force[i] with the calf torque of
    LowState motor block j (diagonal dominant => foot_force uses the motor leg
    order) and mean hip-abduction angle per block (sign alternates R/L)
  * foot_force statistics per LowState index

For a definitive foot-order check, record a bag while lifting ONE foot at a time
(robot supported) and run with --segments to see which index drops to ~0.

The CDR decoder below needs no ROS installation (pure Python + numpy).
Usage: inspect_lowstate_bag.py BAG_DIR_OR_DB3 [--topic /lowstate] [--segments 10]
"""

import argparse
import glob
import os
import sqlite3
import struct
import sys

import numpy as np


class _Reader:
    def __init__(self, buf):
        self.b = buf
        self.o = 4  # skip CDR encapsulation header

    def read(self, fmt, align):
        self.o += (align - ((self.o - 4) % align)) % align
        v = struct.unpack_from("<" + fmt, self.b, self.o)
        self.o += struct.calcsize("<" + fmt)
        return v


def parse_lowstate(buf):
    """Decode the fields of unitree_go/msg/LowState used by seb_online."""
    r = _Reader(buf)
    r.read("2B", 1); r.read("B", 1); r.read("B", 1); r.read("2I", 4); r.read("2I", 4); r.read("H", 2)
    d = {"quat": r.read("4f", 4), "gyro": r.read("3f", 4), "acc": r.read("3f", 4), "rpy": r.read("3f", 4)}
    r.read("b", 1)
    q, dq, tau = [], [], []
    for _ in range(20):
        r.read("B", 1)
        v = r.read("7f", 4)  # q dq ddq tau_est q_raw dq_raw ddq_raw
        q.append(v[0]); dq.append(v[1]); tau.append(v[3])
        r.read("b", 1); r.read("I", 4); r.read("2I", 4)
    d["q"], d["dq"], d["tau"] = q[:12], dq[:12], tau[:12]
    r.read("4B", 1); r.read("i", 4); r.read("H", 2); r.read("2b", 1); r.read("2b", 1); r.read("15H", 2)  # BmsState
    d["foot_force"] = r.read("4h", 2)
    d["foot_force_est"] = r.read("4h", 2)
    d["tick"] = r.read("I", 4)[0]
    return d


def read_bag(path, topic):
    dbs = [path] if path.endswith(".db3") else sorted(glob.glob(os.path.join(path, "*.db3")))
    if not dbs:
        sys.exit(f"no .db3 file found in {path} (mcap bags: convert with 'ros2 bag convert' first)")
    out = []
    for db in dbs:
        con = sqlite3.connect(db)
        row = con.execute("select id from topics where name=?", (topic,)).fetchone()
        if row is None:
            continue
        for ts, data in con.execute("select timestamp, data from messages where topic_id=? order by timestamp", (row[0],)):
            out.append((ts, bytes(data)))
    if not out:
        sys.exit(f"topic {topic} not found")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/lowstate")
    ap.add_argument("--segments", type=int, default=0, help="print mean foot_force per index over N time segments")
    args = ap.parse_args()

    raw = read_bag(args.bag, args.topic)
    ts = np.array([t for t, _ in raw], dtype=np.int64)
    msgs = [parse_lowstate(b) for _, b in raw]
    payload = [b[4:] for _, b in raw]
    tick = np.array([m["tick"] for m in msgs], dtype=np.int64)
    n = len(msgs)
    dt_ms = np.diff(ts) * 1e-6
    dtick = np.diff(tick)

    print(f"messages: {n}   duration: {(ts[-1] - ts[0]) * 1e-9:.2f} s   rate: {n / ((ts[-1] - ts[0]) * 1e-9):.1f} Hz")
    print(f"reception dt [ms]: mean {dt_ms.mean():.3f} median {np.median(dt_ms):.3f} std {dt_ms.std():.3f} "
          f"min {dt_ms.min():.3f} p99 {np.percentile(dt_ms, 99):.3f} max {dt_ms.max():.3f}")
    vals, counts = np.unique(dtick, return_counts=True)
    order = np.argsort(-counts)[:8]
    print("tick deltas (value: count):", ", ".join(f"{vals[i]}: {counts[i]}" for i in order))
    fit = np.polyfit((ts - ts[0]) * 1e-9, tick - tick[0], 1)
    print(f"tick rate from linear fit vs reception time: {fit[0]:.2f} counts/s  (tick period {1e3 / fit[0]:.4f} ms)")
    repeated = int(np.sum(dtick == 0))
    duplicates = sum(1 for i in range(1, n) if tick[i] == tick[i - 1] and payload[i] == payload[i - 1])
    print(f"repeated ticks: {repeated}   exact duplicate messages: {duplicates}   tick decreases: {int(np.sum(dtick < 0))}")

    ff = np.array([m["foot_force"] for m in msgs], dtype=float)
    tau = np.array([m["tau"] for m in msgs], dtype=float)
    q = np.array([m["q"] for m in msgs], dtype=float)
    print("\nfoot_force per LowState index: mean", np.round(ff.mean(0), 1), " std", np.round(ff.std(0), 1),
          " min", ff.min(0), " max", ff.max(0))
    calf = np.abs(tau[:, [2, 5, 8, 11]])
    if np.all(ff.std(0) > 0) and np.all(calf.std(0) > 0):
        c = np.corrcoef(np.hstack([ff, calf]).T)[:4, 4:]
        print("corr(foot_force[i], |calf torque| of motor block j)   (rows i, cols j):")
        for i in range(4):
            print("   ", " ".join(f"{c[i, j]:+.2f}" for j in range(4)), "  <- diagonal max" if np.argmax(c[i]) == i else "")
    print("mean hip-abduction angle of motor blocks 0..3:", np.round(q[:, [0, 3, 6, 9]].mean(0), 3),
          "  (Unitree order FR, FL, RR, RL: expected signs -, +, -, + in a neutral stance)")
    acc = np.array([m["acc"] for m in msgs], dtype=float)
    print("mean accelerometer [m/s^2]:", np.round(acc.mean(0), 3), " (z ~ +9.8 at rest = specific force, z up)")

    if args.segments > 0:
        print(f"\nmean foot_force per index over {args.segments} segments (lift one foot at a time to identify it):")
        for k, seg in enumerate(np.array_split(np.arange(n), args.segments)):
            t0 = (ts[seg[0]] - ts[0]) * 1e-9
            print(f"  t={t0:7.2f}s  ", np.round(ff[seg].mean(0), 1))


if __name__ == "__main__":
    main()
