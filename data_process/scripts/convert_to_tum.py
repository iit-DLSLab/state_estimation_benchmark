#!/usr/bin/env python3
from pathlib import Path
import pandas as pd
import numpy as np


def ensure_outdir(outdir: Path) -> None:
    outdir.mkdir(parents=True, exist_ok=True)


def save_tum_like(df: pd.DataFrame, out_path: Path) -> None:
    """
    Save in TUM format: t tx ty tz qx qy qz qw
    Space-separated, no header.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)
    arr = df[["t", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    np.savetxt(out_path, arr, fmt="%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f")
    print(f"[OK] {out_path}  ({len(df)} poses)")


def load_groundtruth(path: Path) -> pd.DataFrame:
    """
    Input: groundtruth.csv
    Columns: t,x,y,z,qx,qy,qz,qw,(...)
    """
    df = pd.read_csv(path)
    required = ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{path} missing columns: {missing}")

    out = pd.DataFrame(
        {
            "t": df["t"].astype(float),
            "tx": df["x"].astype(float),
            "ty": df["y"].astype(float),
            "tz": df["z"].astype(float),
            "qx": df["qx"].astype(float),
            "qy": df["qy"].astype(float),
            "qz": df["qz"].astype(float),
            "qw": df["qw"].astype(float),
        }
    ).sort_values("t").reset_index(drop=True)
    return out


def load_anymal_state(path: Path) -> pd.DataFrame:
    """
    Input: anymal_state.csv
    Columns: t,px,py,pz,qx,qy,qz,qw,(...)
    NOTE: your example shows tab-separated; we handle both commas and tabs.
    """
    # Try flexible separator: comma OR tab OR spaces
    df = pd.read_csv(path, sep=None, engine="python")
    required = ["t", "px", "py", "pz", "qx", "qy", "qz", "qw"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{path} missing columns: {missing}")

    out = pd.DataFrame(
        {
            "t": df["t"].astype(float),
            "tx": df["px"].astype(float),
            "ty": df["py"].astype(float),
            "tz": df["pz"].astype(float),
            "qx": df["qx"].astype(float),
            "qy": df["qy"].astype(float),
            "qz": df["qz"].astype(float),
            "qw": df["qw"].astype(float),
        }
    ).sort_values("t").reset_index(drop=True)
    return out


def load_muse_like(path: Path) -> pd.DataFrame:
    """
    Input: fused_state.csv (muse / iekf / invariant_smoother)
    Columns: t_rel,t_abs,px,py,pz,vx,vy,vz,qw,qx,qy,qz
    Uses t_abs as timestamp.
    """
    df = pd.read_csv(path)
    required = ["t_abs", "px", "py", "pz", "qw", "qx", "qy", "qz"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{path} missing columns: {missing}")

    out = pd.DataFrame(
        {
            "t": df["t_abs"].astype(float),
            "tx": df["px"].astype(float),
            "ty": df["py"].astype(float),
            "tz": df["pz"].astype(float),
            # TUM wants qx qy qz qw
            "qx": df["qx"].astype(float),
            "qy": df["qy"].astype(float),
            "qz": df["qz"].astype(float),
            "qw": df["qw"].astype(float),
        }
    ).sort_values("t").reset_index(drop=True)
    return out


def main() -> None:
    # Run from: data/anymalD_grandtour/
    root = Path(".")
    outdir = root / "tum"
    ensure_outdir(outdir)

    # Inputs
    gt_path = root / "groundtruth.csv"
    anymal_state_path = root / "anymal_state.csv"

    muse_path = root / "muse" / "fused_state.csv"
    iekf_path = root / "iekf" / "fused_state.csv"
    is_path = root / "invariant_smoother" / "fused_state.csv"

    # Outputs (requested names + new anymal_state output)
    gt_out = outdir / "groundtruth_traj_tum.csv"

    muse_out = outdir / "muse_traj_tum.csv"
    iekf_out = outdir / "iekf_traj_tum.csv"
    is_out = outdir / "is_traj_tum.csv"

    # Convert + save
    if not gt_path.exists():
        raise FileNotFoundError(f"Missing: {gt_path}")
    save_tum_like(load_groundtruth(gt_path), gt_out)

    if muse_path.exists():
        save_tum_like(load_muse_like(muse_path), muse_out)
    else:
        print(f"[SKIP] not found: {muse_path}")

    if iekf_path.exists():
        save_tum_like(load_muse_like(iekf_path), iekf_out)
    else:
        print(f"[SKIP] not found: {iekf_path}")

    if is_path.exists():
        save_tum_like(load_muse_like(is_path), is_out)
    else:
        print(f"[SKIP] not found: {is_path}")

    print("\nATE [m] (evo):")
    print(f"  evo_ape tum {gt_out} {muse_out} -a")
    print(f"  evo_ape tum {gt_out} {iekf_out} -a")
    print(f"  evo_ape tum {gt_out} {is_out}   -a")

    print("\nRPE [m] (delta=1 m)):")
    print(f"  evo_rpe tum {gt_out} {muse_out} --delta 1 --delta_unit m --pose_relation point_distance -a")
    print(f"  evo_rpe tum {gt_out} {iekf_out} --delta 1 --delta_unit m --pose_relation point_distance -a")
    print(f"  evo_rpe tum {gt_out} {is_out}   --delta 1 --delta_unit m --pose_relation point_distance -a")

    print("\nRPE [m] (delta=1 f)):")
    print(f"  evo_rpe tum {gt_out} {muse_out} --delta 1 --delta_unit f --pose_relation point_distance -a")
    print(f"  evo_rpe tum {gt_out} {iekf_out} --delta 1 --delta_unit f --pose_relation point_distance -a")
    print(f"  evo_rpe tum {gt_out} {is_out}   --delta 1 --delta_unit f --pose_relation point_distance -a")

    print("\nRPE [deg] (delta=1 m)):")
    print(f"  evo_rpe tum {gt_out} {muse_out} --pose_relation angle_deg --delta 1 --delta_unit m --pose_relation rot_part -a")
    print(f"  evo_rpe tum {gt_out} {iekf_out} --pose_relation angle_deg --delta 1 --delta_unit m --pose_relation rot_part -a")
    print(f"  evo_rpe tum {gt_out} {is_out}   --pose_relation angle_deg --delta 1 --delta_unit m --pose_relation rot_part -a")

    print("\nRPE [deg] (delta=1 f)):")
    print(f"  evo_rpe tum {gt_out} {muse_out} --pose_relation angle_deg --delta 1 --delta_unit f --pose_relation rot_part -a")
    print(f"  evo_rpe tum {gt_out} {iekf_out} --pose_relation angle_deg --delta 1 --delta_unit f --pose_relation rot_part -a")
    print(f"  evo_rpe tum {gt_out} {is_out}   --pose_relation angle_deg --delta 1 --delta_unit f --pose_relation rot_part -a")

    print("\nVelocity RMSE [m/s]:")
    print(f"  python3 ../../data_process/scripts/compute_vel_rmse.py ")

if __name__ == "__main__":
    main()