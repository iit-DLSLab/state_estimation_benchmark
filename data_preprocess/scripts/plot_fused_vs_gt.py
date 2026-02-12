#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

DATASET_ROOT = "data/anymalD_grandtour"
# GT_FILE      = f"{DATASET_ROOT}/groundtruth.csv"
GT_FILE      = f"{DATASET_ROOT}/anymal_state.csv"
FUSED_FILE   = f"{DATASET_ROOT}/muse/fused_state.csv"
SMOOTHER_FILE = f"{DATASET_ROOT}/invariant_smoother/result.csv"

def interp_vec(t_src, y_src, t_dst):
    """Linear interpolation column-wise."""
    out = np.zeros((len(t_dst), y_src.shape[1]))
    for i in range(y_src.shape[1]):
        out[:, i] = np.interp(t_dst, t_src, y_src[:, i])
    return out

def unwrap_deg(a_deg):
    return np.rad2deg(np.unwrap(np.deg2rad(a_deg)))

def align_quat_sign(q_xyzw_ref, q_xyzw):
    """
    Ensure quaternion continuity by flipping sign if dot(ref, q) < 0.
    q and -q represent the same rotation, but discontinuities ruin plots.
    """
    q = q_xyzw.copy()
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0.0:
            q[i] *= -1.0
    # also align first sample to reference to avoid constant sign mismatch
    if np.dot(q_xyzw_ref[0], q[0]) < 0.0:
        q *= -1.0
    return q

def main():
    gt = pd.read_csv(GT_FILE)
    fused = pd.read_csv(FUSED_FILE)
    smoother = pd.read_csv(SMOOTHER_FILE)
    # --- time vectors
    t_gt_abs = gt["t"].to_numpy(dtype=float)
    t_fu_abs = fused["t_abs"].to_numpy(dtype=float)
    t_sm_abs = smoother["t_abs"].to_numpy(dtype=float)

    t_gt = t_gt_abs - t_gt_abs[0]
    t_fu = t_fu_abs - t_fu_abs[0]
    t_sm = t_sm_abs - t_sm_abs[0]

    # --- GT signals
    # p_gt = gt[["x","y","z"]].to_numpy(dtype=float)
    # v_gt = gt[["vx","vy","vz"]].to_numpy(dtype=float)

    # # GT quat is (qx,qy,qz,qw) -> scipy expects (x,y,z,w)
    # q_gt_xyzw = gt[["qx","qy","qz","qw"]].to_numpy(dtype=float)

    # --- ANYmal state from internal state estimator
    p_gt = gt[["px","py","pz"]].to_numpy(dtype=float)
    v_gt = gt[["vx","vy","vz"]].to_numpy(dtype=float)

    # ANYmal quat is (qx,qy,qz,qw) -> scipy expects (x,y,z,w)
    q_gt_xyzw = gt[["qx","qy","qz","qw"]].to_numpy(dtype=float)

    # --- Fused signals
    p_fu = fused[["px","py","pz"]].to_numpy(dtype=float)
    v_fu = fused[["vx","vy","vz"]].to_numpy(dtype=float)
    # Fused quat is (qw,qx,qy,qz) -> convert to (x,y,z,w)
    q_fu_xyzw = fused[["qx","qy","qz","qw"]].to_numpy(dtype=float)

    # --- Smoother signals 
    p_sm = smoother[["px","py","pz"]].to_numpy(dtype=float)
    v_sm = smoother[["vx","vy","vz"]].to_numpy(dtype=float) 
    q_sm_xyzw = smoother[["qx","qy","qz","qw"]].to_numpy(dtype=float)

    # --- interpolate GT to fused timeline (pos/vel)
    p_gt_i = interp_vec(t_gt, p_gt, t_fu)
    v_gt_i = interp_vec(t_gt, v_gt, t_fu)

    # --- interpolate GT quaternions component-wise then renormalize (OK for plotting)
    q_gt_i = interp_vec(t_gt, q_gt_xyzw, t_fu)
    q_gt_i = q_gt_i / np.linalg.norm(q_gt_i, axis=1, keepdims=True)

    # --- align quaternion signs for clean plots
    q_fu_xyzw = align_quat_sign(q_gt_i, q_fu_xyzw)
    q_gt_i    = align_quat_sign(q_fu_xyzw, q_gt_i)
    q_sm_xyzw = align_quat_sign(q_gt_i, q_sm_xyzw)

    # --- Euler angles
    r_gt = R.from_quat(q_gt_i)
    r_fu = R.from_quat(q_fu_xyzw)
    r_sm = R.from_quat(q_sm_xyzw)

    rpy_gt = r_gt.as_euler("xyz", degrees=True)
    rpy_fu = r_fu.as_euler("xyz", degrees=True)
    rpy_sm = r_sm.as_euler("xyz", degrees=True)

    # unwrap yaw (often the problematic one); if you want also roll/pitch, uncomment
    rpy_gt[:,2] = unwrap_deg(rpy_gt[:,2])
    rpy_fu[:,2] = unwrap_deg(rpy_fu[:,2])
    rpy_sm[:,2] = unwrap_deg(rpy_sm[:,2])
    # rpy_gt[:,0] = unwrap_deg(rpy_gt[:,0]); rpy_fu[:,0] = unwrap_deg(rpy_fu[:,0])
    # rpy_gt[:,1] = unwrap_deg(rpy_gt[:,1]); rpy_fu[:,1] = unwrap_deg(rpy_fu[:,1])

    # --- RMSE
    # pos_rmse = np.sqrt(np.mean((p_fu - p_gt_i)**2, axis=0))
    # vel_rmse = np.sqrt(np.mean((v_fu - v_gt_i)**2, axis=0))
    # print("RMSE position [m]   (x,y,z):", pos_rmse)
    # print("RMSE velocity [m/s] (x,y,z):", vel_rmse)

    # =======================
    # Position plots
    # =======================
    labels = ["x","y","z"]
    fig1, ax1 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i in range(3):
        ax1[i].plot(t_fu, p_fu[:,i], label="Fused", linewidth=1.5)
        ax1[i].plot(t_fu, p_gt_i[:,i], "--", label="GT", linewidth=1.0)
        ax1[i].plot(t_fu, p_sm[:,i], "-.", label="Smoothed", linewidth=1.0)
        ax1[i].set_ylabel(f"p_{labels[i]} [m]")
        ax1[i].grid(True)
        ax1[i].legend()
    ax1[-1].set_xlabel("time [s]")
    ax1[0].set_title("Position: Fused vs Ground Truth")
    plt.tight_layout()

    # =======================
    # Velocity plots
    # =======================
    fig2, ax2 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i in range(3):
        ax2[i].plot(t_fu, v_fu[:,i], label="Fused", linewidth=1.5)
        ax2[i].plot(t_fu, v_gt_i[:,i], "--", label="GT", linewidth=1.0)
        ax2[i].plot(t_fu, v_sm[:,i], "-.", label="Smoothed", linewidth=1.0)
        ax2[i].set_ylabel(f"v_{labels[i]} [m/s]")
        ax2[i].grid(True)
        ax2[i].legend()
    ax2[-1].set_xlabel("time [s]")
    ax2[0].set_title("Velocity: Fused vs Ground Truth")
    plt.tight_layout()

    # =======================
    # RPY plots
    # =======================
    fig3, ax3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    rpy_names = ["roll","pitch","yaw"]
    for i in range(3):
        ax3[i].plot(t_fu, rpy_fu[:,i], label="Fused", linewidth=1.5)
        rpy_gt[:,i] = unwrap_deg(rpy_gt[:,i])  # ensure GT is also unwrapped for plotting
        rpy_sm[:,i] = unwrap_deg(rpy_sm[:,i])  # ensure Smoothed is also unwrapped for plotting
        ax3[i].plot(t_fu, rpy_gt[:,i], "--", label="GT", linewidth=1.0)
        ax3[i].plot(t_fu, rpy_sm[:,i], "-.", label="Smoothed", linewidth=1.0)
        ax3[i].set_ylabel(f"{rpy_names[i]} [deg]")
        ax3[i].grid(True)
        ax3[i].legend()
    ax3[-1].set_xlabel("time [s]")
    ax3[0].set_title("Attitude (RPY): Fused vs Ground Truth")
    plt.tight_layout()

    # =======================
    # Quaternion component plots
    # =======================
    fig4, ax4 = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    qlabs = ["qx","qy","qz","qw"]
    for i, lab in enumerate(qlabs):
        ax4[i].plot(t_fu, q_fu_xyzw[:,i] if lab!="qw" else q_fu_xyzw[:,3], label="Fused", linewidth=1.2)
        ax4[i].plot(t_fu, q_gt_i[:,i]   if lab!="qw" else q_gt_i[:,3], "--", label="GT", linewidth=1.0)
        ax4[i].plot(t_fu, q_sm_xyzw[:,i] if lab!="qw" else q_sm_xyzw[:,3], "-.", label="Smoothed", linewidth=1.0)
        ax4[i].set_ylabel(lab)
        ax4[i].grid(True)
        ax4[i].legend()
    ax4[-1].set_xlabel("time [s]")
    ax4[0].set_title("Quaternion components (aligned sign)")
    plt.tight_layout()

    # =======================
    # Trajectory XY
    # =======================
    plt.figure(figsize=(8,8))
    plt.plot(p_gt[:,0], p_gt[:,1], "--", label="GT", linewidth=1.2)
    plt.plot(p_fu[:,0], p_fu[:,1], label="Fused", linewidth=1.5)
    plt.plot(p_sm[:,0], p_sm[:,1], "-.", label="Smoothed", linewidth=1.0)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory XY")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
