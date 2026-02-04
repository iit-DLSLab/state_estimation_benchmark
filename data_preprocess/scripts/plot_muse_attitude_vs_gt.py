#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

DATASET_ROOT = "data/anymalD_grandtour"

MUSE_FILE = f"{DATASET_ROOT}/muse/attitude_estimate_muse.csv"
GT_FILE   = f"{DATASET_ROOT}/groundtruth.csv"

def quat_to_rpy_deg(qw, qx, qy, qz):
    """
    Quaternion -> roll, pitch, yaw in degrees
    Convention: XYZ (roll-pitch-yaw), same as Eigen default
    """
    r = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = r.as_euler("xyz", degrees=True)
    return roll, pitch, yaw

def interp_quat_components(t_src, q_src_xyzw, t_dst):
    """
    Interpola i 4 componenti quaternion (x,y,z,w) component-wise,
    poi rinormalizza per mantenere unit norm.
    """
    q_dst = np.zeros((len(t_dst), 4))
    for k in range(4):
        q_dst[:, k] = np.interp(t_dst, t_src, q_src_xyzw[:, k])
    # rinormalizza
    n = np.linalg.norm(q_dst, axis=1)
    n[n < 1e-12] = 1.0
    q_dst = q_dst / n[:, None]
    return q_dst


def main():
    # ------------------------
    # Load MUSE attitude
    # ------------------------
    muse = pd.read_csv(MUSE_FILE)

    t_muse = muse["t_rel"].to_numpy()

    r_muse = R.from_quat(
        np.column_stack([
            muse["qx"], muse["qy"], muse["qz"], muse["qw"]
        ])
    )
    rpy_muse = r_muse.as_euler("xyz", degrees=True)

    # ------------------------
    # Load ground truth
    # ------------------------
    gt = pd.read_csv(GT_FILE)

    t_gt_abs = gt.iloc[:, 0].to_numpy()   # timestamp
    t_gt = t_gt_abs - t_gt_abs[0]         # make relative

    # Ground truth quaternion: qx,qy,qz,qw
    r_gt = R.from_quat(
        np.column_stack([
            gt["qx"], gt["qy"], gt["qz"], gt["qw"]
        ])
    )
    rpy_gt = r_gt.as_euler("xyz", degrees=True)

    def unwrap_deg(a):
        return np.rad2deg(np.unwrap(np.deg2rad(a)))

    rpy_muse[:,2] = unwrap_deg(rpy_muse[:,2])
    rpy_gt[:,0]   = unwrap_deg(rpy_gt[:,0])
    rpy_gt[:,2]   = unwrap_deg(rpy_gt[:,2])


    # ------------------------
    # Interpolate GT to MUSE time
    # ------------------------
    rpy_gt_interp = np.zeros_like(rpy_muse)
    for i in range(3):
        rpy_gt_interp[:, i] = np.interp(
            t_muse, t_gt, rpy_gt[:, i]
        )

    # ------------------------
    # Plot
    # ------------------------
    labels = ["roll [deg]", "pitch [deg]", "yaw [deg]"]

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(3):
        axs[i].plot(t_muse, rpy_muse[:, i]-rpy_muse[0, i], label="MUSE", linewidth=1.5)
        axs[i].plot(t_gt, rpy_gt[:, i] - rpy_gt[0, i], "--", label="GT", linewidth=1.0)
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)
        axs[i].legend()

    axs[-1].set_xlabel("time [s]")
    axs[0].set_title("Attitude comparison: MUSE vs Ground Truth")

    # Quaternion MUSE in formato [x,y,z,w]
    q_muse_xyzw = np.column_stack([muse["qx"], muse["qy"], muse["qz"], muse["qw"]])

    # Quaternion GT in formato [x,y,z,w]  (ATTENZIONE: ordine giusto)
    q_gt_xyzw = np.column_stack([gt["qx"], gt["qy"], gt["qz"], gt["qw"]])

    # Interpola GT su t_muse
    q_gt_i_xyzw = interp_quat_components(t_gt, q_gt_xyzw, t_muse)

    # Plot componenti
    fig_q, axs_q = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    labs = ["qx", "qy", "qz", "qw"]

    for i, lab in enumerate(labs):
        axs_q[i].plot(t_muse, q_muse_xyzw[:, i], label="MUSE", linewidth=1.2)
        axs_q[i].plot(t_muse, q_gt_i_xyzw[:, i], "--", label="GT (interp)", linewidth=1.0)
        axs_q[i].set_ylabel(lab)
        axs_q[i].grid(True)
        axs_q[i].legend()

    axs_q[-1].set_xlabel("time [s]")
    axs_q[0].set_title("Quaternion components: MUSE vs Ground Truth (GT interpolated)")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
