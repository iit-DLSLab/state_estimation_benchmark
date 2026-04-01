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
    is_path = root / "invariant_smoother" / "fused_state_ws15.csv"
    is_ws1_path = root / "invariant_smoother" / "fused_state_ws1.csv"
    is_ws2_path = root / "invariant_smoother" / "fused_state_ws2.csv"
    is_ws3_path = root / "invariant_smoother" / "fused_state_ws3.csv"
    is_ws4_path = root / "invariant_smoother" / "fused_state_ws4.csv"
    is_ws5_path = root / "invariant_smoother" / "fused_state_ws5.csv"

    # Outputs (requested names + new anymal_state output)
    gt_out = outdir / "groundtruth_traj_tum.csv"
    anymal_state_out = outdir / "anymal_state_traj_tum.csv"

    muse_out = outdir / "muse_traj_tum.csv"
    iekf_out = outdir / "iekf_traj_tum.csv"
    is_out = outdir / "is_traj_tum_ws15.csv"
    is_ws1_out = outdir / "is_traj_tum_ws1.csv"
    is_ws2_out = outdir / "is_traj_tum_ws2.csv"
    is_ws3_out = outdir / "is_traj_tum_ws3.csv"
    is_ws4_out = outdir / "is_traj_tum_ws4.csv"
    is_ws5_out = outdir / "is_traj_tum_ws5.csv"

    # Convert + save
    if not gt_path.exists():
        raise FileNotFoundError(f"Missing: {gt_path}")
    save_tum_like(load_groundtruth(gt_path), gt_out)

    if anymal_state_path.exists():
        save_tum_like(load_anymal_state(anymal_state_path), anymal_state_out)
    else:
        print(f"[SKIP] not found: {anymal_state_path}")

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

    if is_ws1_path.exists():
        save_tum_like(load_muse_like(is_ws1_path), is_ws1_out)
    else:
        print(f"[SKIP] not found: {is_ws1_path}")

    if is_ws2_path.exists():
        save_tum_like(load_muse_like(is_ws2_path), is_ws2_out)
    else:
        print(f"[SKIP] not found: {is_ws2_path}")

    if is_ws3_path.exists():
        save_tum_like(load_muse_like(is_ws3_path), is_ws3_out)
    else:
        print(f"[SKIP] not found: {is_ws3_path}")

    if is_ws4_path.exists():
        save_tum_like(load_muse_like(is_ws4_path), is_ws4_out)
    else:
        print(f"[SKIP] not found: {is_ws4_path}")

    if is_ws5_path.exists():
        save_tum_like(load_muse_like(is_ws5_path), is_ws5_out)
    else:
        print(f"[SKIP] not found: {is_ws5_path}")

    print("\nExamples (evo):")
    print(f"  evo_ape tum {gt_out} {muse_out} -a --plot")
    print(f"  evo_ape tum {gt_out} {iekf_out} -a --plot")
    print(f"  evo_ape tum {gt_out} {is_out} -a --plot")
    print(f"  evo_ape tum {gt_out} {anymal_state_out} -a --plot")

    print("\nIf you want RPE:")
    print(f"  evo_rpe tum {gt_out} {muse_out} -a --plot")
    print(f"  evo_rpe tum {gt_out} {iekf_out} -a --plot")
    print(f"  evo_rpe tum {gt_out} {is_out} -a --plot")
    print(f"  evo_rpe tum {gt_out} {anymal_state_out} -a --plot")


if __name__ == "__main__":
    main()