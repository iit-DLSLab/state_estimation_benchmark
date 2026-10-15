#!/usr/bin/env python3
"""Compare two sets of offline benchmark outputs (regression check).

Usage: compare_offline_regression.py REFERENCE_DATASET_DIR NEW_DATASET_DIR

Both directories are dataset roots containing muse/, iekf/, invariant_smoother/
output CSVs (and optionally feet_kinematics.csv). Reports, per file, whether the
files are byte-identical and otherwise the maximum absolute difference per column.
Exit code 0 if every compared file is identical.
"""

import csv
import filecmp
import os
import sys

FILES = [
    "feet_kinematics.csv",
    "muse/fused_state.csv",
    "muse/attitude_estimate.csv",
    "muse/leg_odometry.csv",
    "iekf/fused_state.csv",
    "invariant_smoother/fused_state.csv",
]


def max_diff(a, b):
    with open(a, newline="") as fa, open(b, newline="") as fb:
        ra, rb = csv.reader(fa), csv.reader(fb)
        ha, hb = next(ra), next(rb)
        if ha != hb:
            return {"header": "different"}
        worst = [0.0] * len(ha)
        rows_a = rows_b = 0
        for x, y in zip(ra, rb):
            rows_a += 1
            rows_b += 1
            for i, (u, v) in enumerate(zip(x, y)):
                try:
                    worst[i] = max(worst[i], abs(float(u) - float(v)))
                except ValueError:
                    pass
        rows_a += sum(1 for _ in ra)
        rows_b += sum(1 for _ in rb)
    out = {h: w for h, w in zip(ha, worst) if w > 0}
    if rows_a != rows_b:
        out["rows"] = f"{rows_a} vs {rows_b}"
    return out


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    ref, new = sys.argv[1], sys.argv[2]
    ok = True
    for f in FILES:
        a, b = os.path.join(ref, f), os.path.join(new, f)
        if not (os.path.exists(a) and os.path.exists(b)):
            print(f"{f:40s} skipped (missing)")
            continue
        if filecmp.cmp(a, b, shallow=False):
            print(f"{f:40s} IDENTICAL")
        else:
            ok = False
            print(f"{f:40s} DIFFERENT: {max_diff(a, b)}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
