#!/usr/bin/env python3
"""
plots.py — plotting for GT vs MUSE / IEKF / IS

Fixes:
- Quaternion resampling uses SLERP (no component-wise interp).
- Time-shifts are applied first, then we crop to the common time window across ALL signals,
  so SLERP never goes out of range.
- Uses one common timeline = (cropped) MUSE timeline after time-shift.
- Optional unwrap for Euler angles (kept, but commentable).
"""

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R, Slerp
from scipy.signal import savgol_filter


# =======================
# Paths
# =======================
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DATASET_ROOT = REPO_ROOT / "data" / "anymalD_grandtour"


def first_existing_path(*candidates):
    for path in candidates:
        if path.exists():
            return path
    tried = "\n".join(f"- {path}" for path in candidates)
    raise FileNotFoundError(f"None of the expected dataset files exist:\n{tried}")


GT_FILE = first_existing_path(
    DATASET_ROOT / "groundtruth.csv",
)
MUSE_FILE = DATASET_ROOT / "muse" / "fused_state.csv"
IEKF_FILE = DATASET_ROOT / "iekf" / "fused_state.csv"
SMOOTHER_FILE = DATASET_ROOT / "invariant_smoother" / "fused_state.csv"

GT_COLOR = "tab:blue"
MUSE_COLOR = "tab:orange"
IEKF_COLOR = "tab:green"
IS_COLOR = "tab:red"

# =======================
# Umeyama alignment (est -> GT)
# position: p_gt ≈ R_align @ p_est + t_align
# orientation: R_wb_gt ≈ R_align * R_wb_est
# these values can be derived from the evo tool: https://github.com/michaelgrupp/evo
# =======================
ALIGN = {
    "muse": {
        "R": np.array([
            [ 0.71744476, -0.69589424, -0.03168947],
            [ 0.69644101,  0.71754045,  0.01027768],
            [ 0.01558629, -0.02944351,  0.99944492],
        ], dtype=float),
        "t": np.array([ 1.84438775, -0.20790437, -0.18639427], dtype=float),
    },
    "iekf": {
        "R": np.array([
            [ 0.71849142, -0.69482244, -0.03149382],
            [ 0.69527739,  0.71872202,  0.00529164],
            [ 0.01895855, -0.02569894,  0.99948994],
        ], dtype=float),
        "t": np.array([-0.72336851,  0.29125441, -0.46869969], dtype=float),
    },
    "is": {
        "R": np.array([
            [ 0.71790964, -0.69544522, -0.03101111],
            [ 0.69589757,  0.71811755,  0.00580931],
            [ 0.01822957, -0.02575112,  0.99950216],
        ], dtype=float),
        "t": np.array([-0.71558956,  0.23444711, -0.44472576], dtype=float),
    },
}


# =======================
# Helpers
# =======================
def interp_vec(t_src, y_src, t_dst):
    """Linear interpolation column-wise for vector signals (pos/vel)."""
    t_src = np.asarray(t_src, dtype=float)
    y_src = np.asarray(y_src, dtype=float)
    t_dst = np.asarray(t_dst, dtype=float)

    out = np.zeros((len(t_dst), y_src.shape[1]))
    for i in range(y_src.shape[1]):
        out[:, i] = np.interp(t_dst, t_src, y_src[:, i])
    return out


def slerp_quat(t_src, q_xyzw_src, t_dst):
    """
    SLERP quaternion interpolation.
    q must be in (x,y,z,w) order.
    SciPy Slerp requires strictly increasing t_src, so duplicates are removed.
    """
    t_src = np.asarray(t_src, dtype=float)
    q_xyzw_src = np.asarray(q_xyzw_src, dtype=float)

    # Remove duplicate timestamps (keep first)
    t_unique, idx = np.unique(t_src, return_index=True)
    q_unique = q_xyzw_src[idx]

    r_src = R.from_quat(q_unique)
    slerp = Slerp(t_unique, r_src)
    r_dst = slerp(t_dst)
    return r_dst.as_quat()


def unwrap_deg(a_deg):
    return np.rad2deg(np.unwrap(np.deg2rad(a_deg)))


def align_quat_sign(q_xyzw_ref, q_xyzw):
    """
    Make quaternion signs consistent over time (avoid sign flips), and also
    keep same “hemisphere” as reference at the first sample.
    """
    q = q_xyzw.copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0.0:
            q[i] *= -1.0
    if np.dot(q_xyzw_ref[0], q[0]) < 0.0:
        q *= -1.0
    return q


def apply_alignment(p, v, q_xyzw, R_align, t_align, vel_in_body=False):
    """
    Apply SE(3) alignment (R_align, t_align) to pos, vel, orientation.
    Assumes q is R_wb (world-from-body) in estimator world.
    Returns aligned in GT world.

    - pos: p' = R_align p + t
    - ori: R_wb' = R_align * R_wb
    - vel:
        if vel_in_body: v_body -> v_world_est = R_wb_est * v_body -> v' = R_align v_world_est
        else: v already in estimator world -> v' = R_align v
    """
    r_wb_est = R.from_quat(q_xyzw)

    p_al = (R_align @ p.T).T + t_align

    r_align = R.from_matrix(R_align)
    r_wb_al = r_align * r_wb_est
    q_al = r_wb_al.as_quat()

    if vel_in_body:
        v_world_est = r_wb_est.apply(v)
        v_al = (R_align @ v_world_est.T).T
    else:
        v_al = (R_align @ v.T).T

    return p_al, v_al, q_al


def estimate_time_shift_rmse(t_ref, sig_ref, t_est, sig_est, search_s=10.0, step_s=0.005):
    """
    Find tau minimizing RMSE between sig_ref(t) and sig_est(t) after shifting est time:
        t_est_shifted = t_est + tau
    Uses linear interp of sig_est onto t_ref over overlapping support.
    """
    taus = np.arange(-search_s, search_s + 0.5 * step_s, step_s)
    best_tau = 0.0
    best_rmse = np.inf

    for tau in taus:
        t_es = t_est + tau
        # overlap region (in ref time)
        mask = (t_ref >= t_es[0]) & (t_ref <= t_es[-1])
        if np.count_nonzero(mask) < 100:
            continue

        est_i = np.interp(t_ref[mask], t_es, sig_est)
        err = est_i - sig_ref[mask]
        rmse = np.sqrt(np.mean(err * err))

        if rmse < best_rmse:
            best_rmse = rmse
            best_tau = tau

    return best_tau, best_rmse


def crop_by_time(t, *arrays, t_min, t_max):
    """Crop time and aligned arrays to [t_min, t_max]."""
    t = np.asarray(t, dtype=float)
    mask = (t >= t_min) & (t <= t_max)
    out = [t[mask]]
    for a in arrays:
        out.append(a[mask])
    return out


def common_time_window(*time_vectors):
    """Intersection window [t_min, t_max] across all vectors (assumed sorted)."""
    t_min = max(tv[0] for tv in time_vectors)
    t_max = min(tv[-1] for tv in time_vectors)
    if not (t_min < t_max):
        raise ValueError(f"No common time window: t_min={t_min}, t_max={t_max}")
    return t_min, t_max


def compute_rmse(err):
    """Per-axis and 3D RMSE from an Nx3 position error array."""
    err = np.asarray(err, dtype=float)
    rmse_xyz = np.sqrt(np.mean(err * err, axis=0))
    rmse_norm = np.sqrt(np.mean(np.sum(err * err, axis=1)))
    return rmse_xyz, rmse_norm


# =======================
# Main
# =======================
def main():
    # --- Load
    gt = pd.read_csv(GT_FILE)
    muse = pd.read_csv(MUSE_FILE)          # MUSE
    smoother = pd.read_csv(SMOOTHER_FILE)   # IS
    iekf = pd.read_csv(IEKF_FILE)           # IEKF

    # --- Time vectors (relative)
    t_gt = gt["t"].to_numpy(float)
    t_gt = t_gt - t_gt[0]

    t_muse = muse["t_abs"].to_numpy(float)
    t_muse = t_muse - t_muse[0]

    t_sm = smoother["t_abs"].to_numpy(float)
    t_sm = t_sm - t_sm[0]

    t_ik = iekf["t_abs"].to_numpy(float)
    t_ik = t_ik - t_ik[0]

    # --- GT signals
    p_gt = gt[["x", "y", "z"]].to_numpy(float)
    v_gt = gt[["vx", "vy", "vz"]].to_numpy(float)
    q_gt = gt[["qx", "qy", "qz", "qw"]].to_numpy(float)

    # --- Estimator signals
    p_muse = muse[["px", "py", "pz"]].to_numpy(float)
    v_muse = muse[["vx", "vy", "vz"]].to_numpy(float)
    q_muse = muse[["qx", "qy", "qz", "qw"]].to_numpy(float)

    p_sm = smoother[["px", "py", "pz"]].to_numpy(float)
    v_sm = smoother[["vx", "vy", "vz"]].to_numpy(float)
    q_sm = smoother[["qx", "qy", "qz", "qw"]].to_numpy(float)

    p_ik = iekf[["px", "py", "pz"]].to_numpy(float)
    v_ik = iekf[["vx", "vy", "vz"]].to_numpy(float)
    q_ik = iekf[["qx", "qy", "qz", "qw"]].to_numpy(float)

    # --- Apply spatial alignment into GT frame
    p_muse, v_muse, q_muse = apply_alignment(p_muse, v_muse, q_muse, ALIGN["muse"]["R"], ALIGN["muse"]["t"])
    p_sm, v_sm, q_sm = apply_alignment(p_sm, v_sm, q_sm, ALIGN["is"]["R"], ALIGN["is"]["t"])
    p_ik, v_ik, q_ik = apply_alignment(p_ik, v_ik, q_ik, ALIGN["iekf"]["R"], ALIGN["iekf"]["t"])

    # --- Estimate time shifts (using speed magnitude AFTER spatial alignment)
    s_gt = np.linalg.norm(v_gt, axis=1)
    s_muse = np.linalg.norm(v_muse, axis=1)
    s_sm = np.linalg.norm(v_sm, axis=1)
    s_ik = np.linalg.norm(v_ik, axis=1)

    tau_muse, rmse_muse = estimate_time_shift_rmse(t_gt, s_gt, t_muse, s_muse)
    tau_sm, rmse_sm = estimate_time_shift_rmse(t_gt, s_gt, t_sm, s_sm)
    tau_ik, rmse_ik = estimate_time_shift_rmse(t_gt, s_gt, t_ik, s_ik)

    # --- Apply time shifts
    t_muse = t_muse + tau_muse
    t_sm = t_sm + tau_sm
    t_ik = t_ik + tau_ik

    # --- Common time window across GT + all estimators (after shifts)
    t_min, t_max = common_time_window(t_gt, t_muse, t_sm, t_ik)

    # --- Define common timeline as cropped MUSE timeline
    t_common, p_muse, v_muse, q_muse = crop_by_time(t_muse, p_muse, v_muse, q_muse, t_min=t_min, t_max=t_max)

    # --- Resample others onto common timeline
    p_gt_i = interp_vec(t_gt, p_gt, t_common)
    v_gt_i = interp_vec(t_gt, v_gt, t_common)
    q_gt_i = slerp_quat(t_gt, q_gt, t_common)

    p_sm_i = interp_vec(t_sm, p_sm, t_common)
    v_sm_i = interp_vec(t_sm, v_sm, t_common)
    q_sm_i = slerp_quat(t_sm, q_sm, t_common)

    p_ik_i = interp_vec(t_ik, p_ik, t_common)
    v_ik_i = interp_vec(t_ik, v_ik, t_common)
    q_ik_i = slerp_quat(t_ik, q_ik, t_common)

    # --- Trajectory error with all trajectories rebased to start at zero
    p_gt_zero = p_gt_i   # - p_gt_i[0]
    p_muse_zero = p_muse # - p_muse[0]
    p_sm_zero = p_sm_i   # - p_sm_i[0]
    p_ik_zero = p_ik_i   # - p_ik_i[0]

    e_muse = p_muse_zero - p_gt_zero
    e_sm = p_sm_zero - p_gt_zero
    e_ik = p_ik_zero - p_gt_zero

    e_v_muse = v_muse - v_gt_i
    e_v_sm = v_sm_i - v_gt_i
    e_v_ik = v_ik_i - v_gt_i

    e_muse_norm = np.linalg.norm(e_muse, axis=1)
    e_sm_norm = np.linalg.norm(e_sm, axis=1)
    e_ik_norm = np.linalg.norm(e_ik, axis=1)

    e_v_muse_norm = np.linalg.norm(e_v_muse, axis=1)
    e_v_sm_norm = np.linalg.norm(e_v_sm, axis=1)
    e_v_ik_norm = np.linalg.norm(e_v_ik, axis=1)

    rmse_muse_xyz, rmse_muse_norm = compute_rmse(e_muse)
    rmse_sm_xyz, rmse_sm_norm = compute_rmse(e_sm)
    rmse_ik_xyz, rmse_ik_norm = compute_rmse(e_ik)

    # Normalize quats (safety)
    q_gt_i = q_gt_i / np.linalg.norm(q_gt_i, axis=1, keepdims=True)
    q_muse = q_muse / np.linalg.norm(q_muse, axis=1, keepdims=True)
    q_sm_i = q_sm_i / np.linalg.norm(q_sm_i, axis=1, keepdims=True)
    q_ik_i = q_ik_i / np.linalg.norm(q_ik_i, axis=1, keepdims=True)

    # Sign-consistency for clean plots
    q_muse = align_quat_sign(q_gt_i, q_muse)
    q_gt_i = align_quat_sign(q_muse, q_gt_i)
    q_sm_i = align_quat_sign(q_gt_i, q_sm_i)
    q_ik_i = align_quat_sign(q_gt_i, q_ik_i)

    # --- Euler angles
    rpy_gt = R.from_quat(q_gt_i).as_euler("xyz", degrees=True)
    rpy_muse = R.from_quat(q_muse).as_euler("xyz", degrees=True)
    rpy_sm = R.from_quat(q_sm_i).as_euler("xyz", degrees=True)
    rpy_ik = R.from_quat(q_ik_i).as_euler("xyz", degrees=True)

    # Optional unwrap (often helps readability; comment out if you prefer raw wrap)
    for arr in (rpy_gt, rpy_muse, rpy_sm, rpy_ik):
        arr[:, 0] = unwrap_deg(arr[:, 0])  # roll
        arr[:, 1] = unwrap_deg(arr[:, 1])  # pitch
        arr[:, 2] = unwrap_deg(arr[:, 2])  # yaw

    # =======================
    # Plots
    # =======================
    labels = ["x", "y", "z"]

    # Position
    fig1, ax1 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i in range(3):
        ax1[i].plot(t_common, p_gt_i[:, i] - p_gt_i[0, i], linestyle="--", label="GT", linewidth=2.0, color=GT_COLOR)
        ax1[i].plot(t_common, p_muse[:, i] - p_muse[0, i], label="MUSE", linewidth=2.0, color=MUSE_COLOR)
        ax1[i].plot(t_common, p_ik_i[:, i] - p_ik_i[0, i], label="IEKF", linewidth=2.0, color=IEKF_COLOR)
        ax1[i].plot(t_common, p_sm_i[:, i] - p_sm_i[0, i], linestyle=":", label="IS", linewidth=4.0, color=IS_COLOR)
        ax1[i].set_ylabel(rf"$p_{{{labels[i]}}}$ [m]", fontsize=30)
        if i == 0:
            ax1[i].legend(fontsize=25, loc="lower right", ncol=4)
        ax1[i].grid(True, color='gray', alpha=0.3)
        ax1[i].set_xlim(t_common[0], t_common[-1])
        ax1[i].set_ylim(-80, 130)
        ax1[i].tick_params(axis='both', labelsize=30)
    ax1[-1].set_xlabel("Time [s]", fontsize=30)
    ax1[0].set_title("Position", fontsize=30)
    plt.tight_layout()

    # Velocity
    fig2, ax2 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i in range(3):
        ax2[i].plot(t_common, v_gt_i[:, i], linestyle="--", label="GT", linewidth=2.0, color=GT_COLOR)
        ax2[i].plot(t_common, v_muse[:, i], label="MUSE", linewidth=2.0, color=MUSE_COLOR)
        ax2[i].plot(t_common, v_ik_i[:, i], label="IEKF", linewidth=2.0, color=IEKF_COLOR)
        ax2[i].plot(t_common, v_sm_i[:, i], linestyle=":", label="IS", linewidth=1.0, color=IS_COLOR)
        ax2[i].set_ylabel(rf"$v_{{{labels[i]}}}$ [m/s]", fontsize=30)
        if i == 2:
            ax2[i].legend(fontsize=20, loc="upper left", ncol=4)
        ax2[i].grid(True, color='gray', alpha=0.3)
        ax2[i].set_xlim(t_common[0], t_common[-1])
        ax2[i].set_ylim(-1.5, 1.5)
        ax2[i].axvspan(50.0, 60.0, alpha=0.2, color='yellow')
        ax2[i].tick_params(axis='both', labelsize=30)
    ax2[-1].set_xlabel("Time [s]", fontsize=30)
    ax2[0].set_title("Linear Velocity", fontsize=30)
    plt.tight_layout()

    # Velocity (zoomed region)
    zoom_t_min = 50.0
    zoom_t_max = 60.0
    mask_zoom = (t_common >= zoom_t_min) & (t_common <= zoom_t_max)
    
    fig2_zoom, ax2_zoom = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i in range(3):
        ax2_zoom[i].plot(t_common[mask_zoom], v_gt_i[mask_zoom, i], linestyle="--", label="GT", linewidth=2.0, color=GT_COLOR)
        ax2_zoom[i].plot(t_common[mask_zoom], v_muse[mask_zoom, i], label="MUSE", linewidth=2.0, color=MUSE_COLOR)
        ax2_zoom[i].plot(t_common[mask_zoom], v_ik_i[mask_zoom, i], label="IEKF", linewidth=2.0, color=IEKF_COLOR)
        ax2_zoom[i].plot(t_common[mask_zoom], v_sm_i[mask_zoom, i], linestyle=":", label="IS", linewidth=1.0, color=IS_COLOR)
        ax2_zoom[i].set_ylabel(rf"$v_{{{labels[i]}}}$ [m/s]", fontsize=30)
        if i == 2:
            ax2_zoom[i].legend(fontsize=20, loc="upper left", ncol=4)
        ax2_zoom[i].grid(True, color='gray', alpha=0.3)
        ax2_zoom[i].set_xlim(zoom_t_min, zoom_t_max)
        # ylims = [(0.25, 1.35), (-1.2, 0.3), (-0.2, 0.45)]
        ylims = [(-1.2, 1.35), (-1.2, 1.35), (-1.2, 1.35)]
        ax2_zoom[i].set_ylim(ylims[i])
        ax2_zoom[i].tick_params(axis='both', labelsize=30)
    ax2_zoom[-1].set_xlabel("Time [s]", fontsize=30)
    ax2_zoom[0].set_title(f"Linear Velocity (zoomed: {zoom_t_min:.1f}–{zoom_t_max:.1f} s)", fontsize=30)
    plt.tight_layout()

    # RPY
    fig3, ax3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    rpy_names = ["roll", "pitch", "yaw"]
    for i in range(3):
        ax3[i].plot(t_common, rpy_gt[:, i]-rpy_gt[0, i], linestyle="--", label="GT", linewidth=2.0, color=GT_COLOR)
        ax3[i].plot(t_common, rpy_muse[:, i]-rpy_muse[0, i], label="MUSE", linewidth=2.0, color=MUSE_COLOR)
        ax3[i].plot(t_common, rpy_ik[:, i]-rpy_ik[0, i], label="IEKF", linewidth=2.0, color=IEKF_COLOR)
        ax3[i].plot(t_common, rpy_sm[:, i]-rpy_sm[0, i], linestyle=":", label="IS", linewidth=1.0, color=IS_COLOR)
        ax3[i].set_ylabel(rf"${rpy_names[i]}$ [°]", fontsize=30)
        if i == 0:
            ax3[i].legend(fontsize=25, loc="upper right", ncol=4)
        ax3[i].grid(True, color='gray', alpha=0.3)
        ax3[i].set_xlim(t_common[0], t_common[-1])
        ylims = [(-35, 35), (-35, 35), (-250, 250)]
        ax3[i].set_ylim(ylims[i])
        ax3[i].tick_params(axis='both', labelsize=30)
    ax3[-1].set_xlabel("Time [s]", fontsize=30)
    ax3[0].set_title("Orientation (roll, pitch, yaw)", fontsize=30)
    plt.tight_layout()

    # Trajectory error norm
    fig4, ax4 = plt.subplots(1, 1, figsize=(11, 4.5), sharex=True)
    ax4.plot(t_common, e_muse_norm-e_muse_norm[0], label="MUSE", linewidth=2.0, color=MUSE_COLOR)
    ax4.plot(t_common, e_ik_norm-e_ik_norm[0], label="IEKF", linewidth=2.0, color=IEKF_COLOR)
    ax4.plot(t_common, e_sm_norm-e_sm_norm[0], linestyle=":", label="IS", linewidth=2.5, color=IS_COLOR)
    ax4.set_ylabel(r"$||e_p||$ [m]", fontsize=30)
    ax4.set_xlabel("Time [s]", fontsize=30)
    ax4.set_title("Trajectory Error Norm", fontsize=30)
    ax4.legend(fontsize=18, loc="upper right", ncol=3)
    ax4.grid(True, color="gray", alpha=0.3)
    ax4.set_xlim(t_common[0], t_common[-1])
    ax4.tick_params(axis="both", labelsize=30)
    plt.tight_layout()

    # Linear velocity error norm
    fig5, ax5 = plt.subplots(1, 1, figsize=(11, 4.5), sharex=True)
    ax5.plot(t_common, e_v_muse_norm- e_v_muse_norm[0], label="MUSE", linewidth=2.0, color=MUSE_COLOR)
    ax5.plot(t_common, e_v_ik_norm- e_v_ik_norm[0], label="IEKF", linewidth=2.0, color=IEKF_COLOR)
    ax5.plot(t_common, e_v_sm_norm- e_v_sm_norm[0], linestyle=":", label="IS", linewidth=2.5, color=IS_COLOR)
    ax5.set_ylabel(r"$||e_v||$ [m/s]", fontsize=30)
    ax5.set_xlabel("Time [s]", fontsize=30)
    ax5.set_title("Linear Velocity Error Norm", fontsize=30)
    ax5.legend(fontsize=18, loc="upper right", ncol=3)
    ax5.grid(True, color="gray", alpha=0.3)
    ax5.set_xlim(t_common[0], t_common[-1])
    ax5.tick_params(axis="both", labelsize=30)
    plt.tight_layout()

    # Trajectory XY (aligned frame)
    plt.figure(figsize=(8, 8))
    plt.plot(p_gt_i[:, 0], p_gt_i[:, 1] - p_gt_i[0, 1], label="GT", linewidth=1.2, color=GT_COLOR)
    plt.plot(p_muse[:, 0], p_muse[:, 1] - p_muse[0, 1],   label="MUSE", linewidth=1.5, color=MUSE_COLOR)
    plt.plot(p_ik_i[:, 0], p_ik_i[:, 1] - p_ik_i[0, 1], label="IEKF", linewidth=1.0, color=IEKF_COLOR)
    plt.plot(p_sm_i[:, 0], p_sm_i[:, 1] - p_sm_i[0, 1], label="IS", linewidth=1.0, color=IS_COLOR)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory XY (aligned frame)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Runtime boxplot
    fig_runtime = plt.figure(figsize=(12, 3))
    muse_runtime = 0.010  
    iekf_runtime = 0.020  
    is1_runtime = 0.110
    is2_runtime = 0.201
    is3_runtime = 0.301
    is4_runtime = 0.428
    is5_runtime = 0.583
    data = [muse_runtime, iekf_runtime, is1_runtime, is2_runtime, is3_runtime, is4_runtime, is5_runtime]
    labels = ["MUSE", "IEKF", "IS\n(WS:1)", "IS\n(WS:2)", "IS\n(WS:3)", "IS\n(WS:4)", "IS\n(WS:5)"]
    plt.bar(labels, data, color=['#FFC107', '#4CAF50', "#E32756", '#E3274C', '#E32742', '#E32738', '#E3272E'])
    plt.yscale('log')
    plt.ylabel("Runtime\nper step\n(Avg) [ms]", fontsize=25)
    plt.title("Runtime Comparison", fontsize=28)
    plt.xticks(fontsize=25)
    plt.yticks(fontsize=25)
    plt.grid(axis='y', which='both', alpha=0.3, linestyle='-', linewidth=0.5)
    plt.grid(axis='y', which='minor', alpha=0.15, linestyle=':', linewidth=0.5)
    plt.tight_layout()
    
   
    # Save plots as PDF
    fig1.savefig("position.pdf", format="pdf", bbox_inches="tight")
    fig2.savefig("velocity.pdf", format="pdf", bbox_inches="tight")
    fig2_zoom.savefig("velocity_zoomed.pdf", format="pdf", bbox_inches="tight")
    fig3.savefig("orientation.pdf", format="pdf", bbox_inches="tight")
    fig4.savefig("trajectory_error_norm.pdf", format="pdf", bbox_inches="tight")
    fig5.savefig("velocity_error_norm.pdf", format="pdf", bbox_inches="tight")
    fig_runtime.savefig("runtime_comparison.pdf", format="pdf", bbox_inches="tight")

    plt.show()

if __name__ == "__main__":
    main()
