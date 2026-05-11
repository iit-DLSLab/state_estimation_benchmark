#!/usr/bin/env python3

import csv
import math
from pathlib import Path
from typing import List, Optional, Tuple


VelRec = Tuple[Optional[float], float, float, float]  # (t, vx, vy, vz)
TIME_CANDIDATES = ("t", "time", "timestamp", "stamp")

# Dataset location (relative to repo root)
DATASET_DIR = "data/anymalD_grandtour"

# Umeyama rotations (estimator frame -> groundtruth frame)
ROT_MUSE = (
    (0.71744476, -0.69589424, -0.03168947),
    (0.69644101, 0.71754045, 0.01027768),
    (0.01558629, -0.02944351, 0.99944492),
)

ROT_IEKF = (
    (0.71849142, -0.69482244, -0.03149382),
    (0.69527739, 0.71872202, 0.00529164),
    (0.01895855, -0.02569894, 0.99948994),
)

ROT_IS = (
    (0.71790964, -0.69544522, -0.03101111),
    (0.69589757, 0.71811755, 0.00580931),
    (0.01822957, -0.02575112, 0.99950216),
)


def parse_float(value: str) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def read_velocity_csv(path: Path) -> List[VelRec]:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    records: List[VelRec] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        headers = set(reader.fieldnames or [])

        for req in ("vx", "vy", "vz"):
            if req not in headers:
                raise ValueError(f"{path} is missing required column '{req}'")

        time_col = next((c for c in TIME_CANDIDATES if c in headers), None)

        for row in reader:
            vx = parse_float(row.get("vx", ""))
            vy = parse_float(row.get("vy", ""))
            vz = parse_float(row.get("vz", ""))
            if vx is None or vy is None or vz is None:
                continue

            t = parse_float(row.get(time_col, "")) if time_col else None
            records.append((t, vx, vy, vz))

    if not records:
        raise ValueError(f"No valid velocity rows found in {path}")
    return records


def align_records(gt: List[VelRec], est: List[VelRec]) -> Tuple[List[VelRec], List[VelRec], str]:
    gt_has_time = all(r[0] is not None for r in gt)
    est_has_time = all(r[0] is not None for r in est)

    if gt_has_time and est_has_time:
        gt_map = {round(r[0], 9): r for r in gt if r[0] is not None}
        est_map = {round(r[0], 9): r for r in est if r[0] is not None}
        common_t = sorted(set(gt_map.keys()) & set(est_map.keys()))
        if len(common_t) > 1:
            return [gt_map[t] for t in common_t], [est_map[t] for t in common_t], "timestamp"

    n = min(len(gt), len(est))
    return gt[:n], est[:n], "index"


def mat3_vec3_mul(R, v):
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def rotate_records(records: List[VelRec], R) -> List[VelRec]:
    out: List[VelRec] = []
    for t, vx, vy, vz in records:
        rx, ry, rz = mat3_vec3_mul(R, (vx, vy, vz))
        out.append((t, rx, ry, rz))
    return out


def compute_rmse(gt: List[VelRec], est: List[VelRec]) -> Tuple[float, float, float, float]:
    if len(gt) != len(est) or len(gt) == 0:
        raise ValueError("Aligned arrays must have same non-zero length")

    se_x = se_y = se_z = se_norm = 0.0
    n = len(gt)

    for g, e in zip(gt, est):
        ex = e[1] - g[1]
        ey = e[2] - g[2]
        ez = e[3] - g[3]
        se_x += ex * ex
        se_y += ey * ey
        se_z += ez * ez
        se_norm += ex * ex + ey * ey + ez * ez

    rmse_x = math.sqrt(se_x / n)
    rmse_y = math.sqrt(se_y / n)
    rmse_z = math.sqrt(se_z / n)
    rmse_vec = math.sqrt(se_norm / n)
    return rmse_x, rmse_y, rmse_z, rmse_vec


def evaluate(gt_path: Path, est_path: Path, label: str, R_est_to_gt) -> None:
    gt = read_velocity_csv(gt_path)
    est = read_velocity_csv(est_path)

    gt_a, est_a, mode = align_records(gt, est)
    est_a = rotate_records(est_a, R_est_to_gt)  # rotate velocities into GT frame

    rmse_x, rmse_y, rmse_z, rmse_vec = compute_rmse(gt_a, est_a)

    print(f"{label}:")
    print(f"  files         : GT={gt_path} | EST={est_path}")
    print(f"  aligned by    : {mode}")
    print(f"  frame align   : applied R_est_to_gt")
    print(f"  samples       : {len(gt_a)}")
    print(f"  RMSE vx [m/s] : {rmse_x:.6f}")
    print(f"  RMSE vy [m/s] : {rmse_y:.6f}")
    print(f"  RMSE vz [m/s] : {rmse_z:.6f}")
    print(f"  RMSE |v| [m/s]: {rmse_vec:.6f}")
    print("")


def main() -> None:
    # script path: <repo>/data_process/scripts/compute_vel_rmse.py
    repo_root = Path(__file__).resolve().parents[2]
    dataset_dir = repo_root / DATASET_DIR

    gt_path = dataset_dir / "groundtruth.csv"
    muse_path = dataset_dir / "muse/fused_state.csv"
    iekf_path = dataset_dir / "iekf/fused_state.csv"
    inv_smoother_path = dataset_dir / "invariant_smoother/fused_state.csv"

    evaluate(gt_path, muse_path, "MUSE", ROT_MUSE)
    evaluate(gt_path, iekf_path, "IEKF", ROT_IEKF)
    evaluate(gt_path, inv_smoother_path, "INVARIANT_SMOOTHER", ROT_IS)


if __name__ == "__main__":
    main()