#!/usr/bin/env python3
import numpy as np
import pandas as pd
from pathlib import Path

# Fixed paths (relative to repo root)
INPUT_CSV = "data/anymalD_grandtour/groundtruth.csv"
OUTPUT_CSV = "data/anymalD_grandtour/groundtruth_rotated_vel.csv"

# Rotation to apply: v_rot = R * v
ROT_GT2EST = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=float)

def main():
    repo_root = Path(__file__).resolve().parents[2]
    in_path = repo_root / INPUT_CSV
    out_path = repo_root / OUTPUT_CSV

    if not in_path.exists():
        raise FileNotFoundError(f"Input file not found: {in_path}")

    df = pd.read_csv(in_path)

    required = ["vx", "vy", "vz"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in {in_path}: {missing}")

# ...existing code...

    v_gt = df[["vx", "vy", "vz"]].to_numpy(dtype=float)
    v_gt_rotated = (ROT_GT2EST @ v_gt.T).T

    # Replace original velocity columns
    df.loc[:, "vx"] = v_gt_rotated[:, 0]
    df.loc[:, "vy"] = v_gt_rotated[:, 1]
    df.loc[:, "vz"] = v_gt_rotated[:, 2]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_path, index=False)

    print(f"Read:   {in_path}")
    print(f"Wrote:  {out_path}")
    print("Replaced columns: vx, vy, vz")

# ...existing code...

if __name__ == "__main__":
    main()