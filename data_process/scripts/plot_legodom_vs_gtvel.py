#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

DATASET_ROOT = "data/anymalD_grandtour"

LEG_FILE = f"{DATASET_ROOT}/muse/leg_odometry.csv"
GT_FILE  = f"{DATASET_ROOT}/groundtruth.csv"

def unwrap_deg(a):
    return np.rad2deg(np.unwrap(np.deg2rad(a)))

def main():
    # ------------------------
    # Load leg odometry output
    # ------------------------
    leg = pd.read_csv(LEG_FILE)

    # Expect: t_rel,t_abs,v_base_b_x,v_base_b_y,v_base_b_z,v_base_w_x,v_base_w_y,v_base_w_z
    if "t_rel" not in leg.columns or "t_abs" not in leg.columns:
        raise RuntimeError("leg_odometry.csv must contain columns: t_rel, t_abs")

    t_leg = leg["t_rel"].to_numpy()

    v_leg_w = np.column_stack([
        leg["v_base_b_x"].to_numpy(),
        leg["v_base_b_y"].to_numpy(),
        leg["v_base_b_z"].to_numpy(),
    ])

    # ------------------------
    # Load ground truth
    # ------------------------
    gt = pd.read_csv(GT_FILE)

    # Expected: timestamp, px,py,pz, qx,qy,qz,qw, vx,vy,vz
    # If your GT uses different column names, adjust here.
    if "timestamp" not in gt.columns:
        # fallback: assume first col is timestamp
        t_gt_abs = gt.iloc[:, 0].to_numpy()
    else:
        t_gt_abs = gt["timestamp"].to_numpy()

    # Make GT time relative
    t_gt = t_gt_abs - t_gt_abs[0]

    # Vel columns: try common names first
    vel_cols_candidates = [
        ("vx", "vy", "vz"),
        ("vel_x", "vel_y", "vel_z"),
        ("v_x", "v_y", "v_z"),
    ]

    vel_cols = None
    for c in vel_cols_candidates:
        if all(col in gt.columns for col in c):
            vel_cols = c
            break

    if vel_cols is None:
        raise RuntimeError(
            "Could not find GT velocity columns. Expected one of: "
            "(vx,vy,vz) or (vel_x,vel_y,vel_z) or (v_x,v_y,v_z). "
            f"Columns found: {list(gt.columns)}"
        )

    v_gt = np.column_stack([gt[vel_cols[0]], gt[vel_cols[1]], gt[vel_cols[2]]]).astype(float)

    # ------------------------
    # Interpolate GT velocity to leg-odom timestamps
    # ------------------------
    v_gt_i = np.zeros_like(v_leg_w)
    for k in range(3):
        v_gt_i[:, k] = np.interp(t_leg, t_gt, v_gt[:, k])

    # simple test to compare leg odom world vs leg odom body
    # v_gt_i = np.column_stack([
    #     leg["v_base_b_x"].to_numpy(),
    #     leg["v_base_b_y"].to_numpy(),
    #     leg["v_base_b_z"].to_numpy(),
    # ])

    # ------------------------
    # Plot XYZ velocities
    # ------------------------
    labels = ["vx [m/s]", "vy [m/s]", "vz [m/s]"]
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    for i in range(3):
        axs[i].plot(t_leg, v_leg_w[:, i], label="Leg odom (world)", linewidth=1.4)
        axs[i].plot(t_gt, v_gt[:, i], "--", label="GT", linewidth=1.0)
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)
        axs[i].legend()

    axs[-1].set_xlabel("time [s]")
    axs[0].set_title("Base linear velocity: Leg Odometry vs Ground Truth")
    plt.tight_layout()
    plt.show()

    # ------------------------
    # Plot speed norm
    # ------------------------
    # speed_leg = np.linalg.norm(v_leg_w, axis=1)
    # speed_gt  = np.linalg.norm(v_gt_i, axis=1)

    # plt.figure(figsize=(11, 4))
    # plt.plot(t_leg, speed_leg, label="Leg odom speed", linewidth=1.4)
    # plt.plot(t_leg, speed_gt, "--", label="GT speed", linewidth=1.0)
    # plt.grid(True)
    # plt.xlabel("time [s]")
    # plt.ylabel("|v| [m/s]")
    # plt.title("Speed magnitude")
    # plt.legend()
    # plt.tight_layout()
    # plt.show()

if __name__ == "__main__":
    main()
