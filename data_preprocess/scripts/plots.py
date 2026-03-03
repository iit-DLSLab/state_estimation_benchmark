#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.signal import savgol_filter


DATASET_ROOT = "data/anymalD_grandtour"
GT_FILE       = f"{DATASET_ROOT}/groundtruth.csv"
FUSED_FILE    = f"{DATASET_ROOT}/muse/fused_state.csv"
SMOOTHER_FILE = f"{DATASET_ROOT}/invariant_smoother/fused_state.csv"
IEKF_FILE     = f"{DATASET_ROOT}/iekf/fused_state.csv"

# Umeyama alignment: p_gt = R_align @ p_est + t_align
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

USE_NUMERIC_GT_VEL = False        # set False to use logged gt["vx","vy","vz"]
VEL_SAVGOL_WINDOW = 31           # odd number
VEL_SAVGOL_POLY = 3

def numerical_velocity_from_position(t, p, smooth=True, window=31, poly=3):
    """
    Compute velocity by differentiating position with non-uniform time support.
    Uses central differences (np.gradient) + optional Savitzky-Golay smoothing.
    """
    # dp/dt for each axis
    vx = np.gradient(p[:, 0], t)
    vy = np.gradient(p[:, 1], t)
    vz = np.gradient(p[:, 2], t)
    v = np.column_stack((vx, vy, vz))

    if smooth and len(v) >= max(window, poly + 2):
        # window must be odd and <= number of samples
        w = min(window, len(v) if len(v) % 2 == 1 else len(v) - 1)
        w = max(w, poly + 2 + ((poly + 2) % 2 == 0))  # ensure > poly and odd
        if w % 2 == 0:
            w += 1
        if w <= len(v):
            v[:, 0] = savgol_filter(v[:, 0], w, poly)
            v[:, 1] = savgol_filter(v[:, 1], w, poly)
            v[:, 2] = savgol_filter(v[:, 2], w, poly)

    return v

def interp_vec(t_src, y_src, t_dst):
    """Linear interpolation column-wise."""
    out = np.zeros((len(t_dst), y_src.shape[1]))
    for i in range(y_src.shape[1]):
        out[:, i] = np.interp(t_dst, t_src, y_src[:, i])
    return out

def unwrap_deg(a_deg):
    return np.rad2deg(np.unwrap(np.deg2rad(a_deg)))

def align_quat_sign(q_xyzw_ref, q_xyzw):
    q = q_xyzw.copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0.0:
            q[i] *= -1.0
    if np.dot(q_xyzw_ref[0], q[0]) < 0.0:
        q *= -1.0
    return q

def apply_alignment(p, v, q_xyzw, R_align, t_align, vel_in_body=False):
    # orientation in estimator world frame
    r_wb_est = R.from_quat(q_xyzw)

    # position: p' = R_align p + t
    p_al = (R_align @ p.T).T + t_align

    # orientation: R_wb' = R_align * R_wb
    r_align = R.from_matrix(R_align)
    r_wb_al = r_align * r_wb_est
    q_al = r_wb_al.as_quat()

    # velocity
    if vel_in_body:
        # v is body-frame -> estimator world -> GT world
        v_world_est = r_wb_est.apply(v)
        v_al = (R_align @ v_world_est.T).T
    else:
        # v already in estimator world frame
        v_al = (R_align @ v.T).T

    return p_al, v_al, q_al

def estimate_time_shift_rmse(t_ref, sig_ref, t_est, sig_est, search_s=10.0, step_s=0.005):
    """
    Find tau minimizing RMSE between sig_ref(t) and sig_est(t) after shifting est time:
        t_est_shifted = t_est + tau
    """
    taus = np.arange(-search_s, search_s + 0.5 * step_s, step_s)
    best_tau = 0.0
    best_rmse = np.inf

    for tau in taus:
        t_es = t_est + tau
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

def main():
    gt = pd.read_csv(GT_FILE)
    fused = pd.read_csv(FUSED_FILE)
    smoother = pd.read_csv(SMOOTHER_FILE)
    iekf = pd.read_csv(IEKF_FILE)

    # --- time vectors
    t_gt_abs = gt["t"].to_numpy(dtype=float)
    t_fu_abs = fused["t_abs"].to_numpy(dtype=float)
    t_sm_abs = smoother["t_abs"].to_numpy(dtype=float)
    t_ik_abs = iekf["t_abs"].to_numpy(dtype=float)

    t_gt = t_gt_abs - t_gt_abs[0]
    t_fu = t_fu_abs - t_fu_abs[0]
    t_sm = t_sm_abs - t_sm_abs[0]
    t_ik = t_ik_abs - t_ik_abs[0]

    # --- GT signals
    p_gt = gt[["x", "y", "z"]].to_numpy(dtype=float)
    v_gt = gt[["vx", "vy", "vz"]].to_numpy(dtype=float)
    q_gt_xyzw = gt[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)

    # if USE_NUMERIC_GT_VEL:
    #     v_gt = numerical_velocity_from_position(
    #         t_gt, p_gt, smooth=True, window=VEL_SAVGOL_WINDOW, poly=VEL_SAVGOL_POLY
    #     )
    # else:
    #     v_gt = gt[["vx", "vy", "vz"]].to_numpy(dtype=float)

    q_gt_xyzw = gt[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)

    # --- Estimator signals
    p_fu = fused[["px", "py", "pz"]].to_numpy(dtype=float)
    v_fu = fused[["vx", "vy", "vz"]].to_numpy(dtype=float)
    q_fu_xyzw = fused[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)

    p_sm = smoother[["px", "py", "pz"]].to_numpy(dtype=float)
    v_sm = smoother[["vx", "vy", "vz"]].to_numpy(dtype=float)
    q_sm_xyzw = smoother[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)

    p_ik = iekf[["px", "py", "pz"]].to_numpy(dtype=float)
    v_ik = iekf[["vx", "vy", "vz"]].to_numpy(dtype=float)
    q_ik_xyzw = iekf[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)

    # --- Apply Umeyama alignment per estimator (into GT frame)
    p_fu, v_fu, q_fu_xyzw = apply_alignment(p_fu, v_fu, q_fu_xyzw, ALIGN["muse"]["R"], ALIGN["muse"]["t"])
    p_sm, v_sm, q_sm_xyzw = apply_alignment(p_sm, v_sm, q_sm_xyzw, ALIGN["is"]["R"], ALIGN["is"]["t"])
    p_ik, v_ik, q_ik_xyzw = apply_alignment(p_ik, v_ik, q_ik_xyzw, ALIGN["iekf"]["R"], ALIGN["iekf"]["t"])

    # --- Estimate and apply per-estimator time shift using speed magnitude
    s_gt = np.linalg.norm(v_gt, axis=1)
    s_fu = np.linalg.norm(v_fu, axis=1)
    s_sm = np.linalg.norm(v_sm, axis=1)
    s_ik = np.linalg.norm(v_ik, axis=1)

    tau_fu, rmse_fu = estimate_time_shift_rmse(t_gt, s_gt, t_fu, s_fu)
    tau_sm, rmse_sm = estimate_time_shift_rmse(t_gt, s_gt, t_sm, s_sm)
    tau_ik, rmse_ik = estimate_time_shift_rmse(t_gt, s_gt, t_ik, s_ik)

    t_fu = t_fu + tau_fu #5
    t_sm = t_sm + tau_sm #5
    t_ik = t_ik + tau_ik #5

    print(f"[time-align] MUSE tau={tau_fu:+.4f}s, RMSE={rmse_fu:.4f}")
    print(f"[time-align] IS   tau={tau_sm:+.4f}s, RMSE={rmse_sm:.4f}")
    print(f"[time-align] IEKF tau={tau_ik:+.4f}s, RMSE={rmse_ik:.4f}")

    # --- Interpolate GT to MUSE timeline (for direct ovep_fu, v_fu, q_fu_xyzwrlays)
    p_gt_i = interp_vec(t_gt, p_gt, t_fu)
    v_gt_i = interp_vec(t_gt, v_gt, t_fu)
    q_gt_i = interp_vec(t_gt, q_gt_xyzw, t_fu)
    q_gt_i = q_gt_i / np.linalg.norm(q_gt_i, axis=1, keepdims=True)

    # Interpolate IS/IEKF to MUSE timeline for plotting consistency
    p_sm_i = interp_vec(t_sm, p_sm, t_fu)
    v_sm_i = interp_vec(t_sm, v_sm, t_fu)
    q_sm_i = interp_vec(t_sm, q_sm_xyzw, t_fu)
    q_sm_i = q_sm_i / np.linalg.norm(q_sm_i, axis=1, keepdims=True)

    p_ik_i = interp_vec(t_ik, p_ik, t_fu)
    v_ik_i = interp_vec(t_ik, v_ik, t_fu)
    q_ik_i = interp_vec(t_ik, q_ik_xyzw, t_fu)
    q_ik_i = q_ik_i / np.linalg.norm(q_ik_i, axis=1, keepdims=True)

    # --- Align quaternion signs for clean plots
    q_fu_xyzw = align_quat_sign(q_gt_i, q_fu_xyzw)
    q_gt_i = align_quat_sign(q_fu_xyzw, q_gt_i)
    q_sm_i = align_quat_sign(q_gt_i, q_sm_i)
    q_ik_i = align_quat_sign(q_gt_i, q_ik_i)

    # --- Euler angles
    rpy_gt = R.from_quat(q_gt_i).as_euler("xyz", degrees=True)
    rpy_fu = R.from_quat(q_fu_xyzw).as_euler("xyz", degrees=True)
    rpy_sm = R.from_quat(q_sm_i).as_euler("xyz", degrees=True)
    rpy_ik = R.from_quat(q_ik_i).as_euler("xyz", degrees=True)

    for arr in (rpy_gt, rpy_fu, rpy_sm, rpy_ik):
        arr[:, 2] = unwrap_deg(arr[:, 2])

    # =======================
    # Position plots
    # =======================
    labels = ["x", "y", "z"]
    fig1, ax1 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i in range(3):
        ax1[i].plot(t_fu, p_gt_i[:, i]-p_gt_i[0, i], label="GT", linewidth=1.0)
        ax1[i].plot(t_fu, p_fu[:, i]-p_fu[0, i], label="MUSE", linewidth=1.5)
        ax1[i].plot(t_ik, p_ik_i[:, i]-p_ik_i[0, i], label="IEKF", linewidth=1.0)
        ax1[i].plot(t_sm, p_sm_i[:, i]-p_sm_i[0, i], label="IS", linewidth=1.0)
        ax1[i].set_ylabel(f"p_{labels[i]} [m]")
        ax1[i].grid(True)
        ax1[i].legend()
    ax1[-1].set_xlabel("time [s]")
    ax1[0].set_title("Position: Fused vs Ground Truth (aligned frame)")
    plt.tight_layout()

    # =======================
    # Velocity plots
    # =======================
    rot_gt2est = np.array([[0.0, 1.0, 0.0],
                           [1.0, 0.0, 0.0],
                           [0.0, 0.0,-1.0]])
    v_gt_rotated = (rot_gt2est @ v_gt_i.T).T

    fig2, ax2 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i in range(3):
        ax2[i].plot(t_fu, v_gt_rotated[:, i]-v_gt_rotated[0, i], label="GT", linewidth=1.0)
        ax2[i].plot(t_fu, v_fu[:, i]-v_fu[0, i], label="MUSE", linewidth=1.5)
        ax2[i].plot(t_ik, v_ik_i[:, i]-v_ik_i[0, i], label="IEKF", linewidth=1.0)
        ax2[i].plot(t_sm, v_sm_i[:, i]-v_sm_i[0, i], label="IS", linewidth=1.0)
        ax2[i].set_ylabel(f"v_{labels[i]} [m/s]")
        ax2[i].grid(True)
        ax2[i].legend()
    ax2[-1].set_xlabel("time [s]")
    ax2[0].set_title("Velocity: Fused vs Ground Truth (aligned frame)")
    plt.tight_layout()

    # =======================
    # RPY plots
    # =======================
    fig3, ax3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    rpy_names = ["roll", "pitch", "yaw"]
    for i in range(3):
        ax3[i].plot(t_fu, unwrap_deg(rpy_gt[:, i]-rpy_gt[0, i]), "--", label="GT", linewidth=1.0)
        ax3[i].plot(t_fu, unwrap_deg(rpy_fu[:, i]-rpy_fu[0, i]), label="MUSE", linewidth=1.5)
        ax3[i].plot(t_ik, unwrap_deg(rpy_ik[:, i]-rpy_ik[0, i]), ":", label="IEKF", linewidth=1.0)
        ax3[i].plot(t_sm, unwrap_deg(rpy_sm[:, i]-rpy_sm[0, i]), "-.", label="IS", linewidth=1.0)
        ax3[i].set_ylabel(f"{rpy_names[i]} [deg]")
        ax3[i].grid(True)
        ax3[i].legend()
    ax3[-1].set_xlabel("time [s]")
    ax3[0].set_title("Attitude (RPY): Fused vs Ground Truth (aligned frame)")
    plt.tight_layout()

    # =======================
    # Quaternion component plots
    # =======================
    fig4, ax4 = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    qlabs = ["qx", "qy", "qz", "qw"]
    for i, lab in enumerate(qlabs):
        ax4[i].plot(t_fu, q_gt_i[:, i]-q_gt_i[0, i], "--", label="GT", linewidth=1.0)
        ax4[i].plot(t_fu, q_fu_xyzw[:, i]-q_fu_xyzw[0, i], label="MUSE", linewidth=1.2)
        ax4[i].plot(t_ik, q_ik_i[:, i]-q_ik_i[0, i], ":", label="IEKF", linewidth=1.0)
        ax4[i].plot(t_sm, q_sm_i[:, i]-q_sm_i[0, i], "-.", label="IS", linewidth=1.0)
        ax4[i].set_ylabel(lab)
        ax4[i].grid(True)
        ax4[i].legend()
    ax4[-1].set_xlabel("time [s]")
    ax4[0].set_title("Quaternion components (aligned sign, aligned frame)")
    plt.tight_layout()

    # =======================
    # Trajectory XY
    # =======================
    plt.figure(figsize=(8, 8))
    plt.plot(p_gt[:, 0], p_gt[:, 1]-p_gt[0, 1], "--", label="GT", linewidth=1.2)
    plt.plot(p_fu[:, 0], p_fu[:, 1]-p_fu[0, 1], label="MUSE", linewidth=1.5)
    plt.plot(p_sm[:, 0], p_sm[:, 1]-p_sm[0, 1], "-.", label="IS", linewidth=1.0)
    plt.plot(p_ik[:, 0], p_ik[:, 1]-p_ik[0, 1], ":", label="IEKF", linewidth=1.0)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory XY (aligned frame)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()