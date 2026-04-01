#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# -------- CONFIG --------
DATASET_ROOT = Path("data/anymalD_grandtour")
KIN_FILE = DATASET_ROOT / "feet_kinematics.csv"

LEG_NAMES = ["LF", "RF", "LH", "RH"]
# ------------------------

def main():
    print(f"Loading {KIN_FILE}")
    df = pd.read_csv(KIN_FILE)

    t = df["t"].to_numpy()
    t = t - t[0]  # start at zero for plotting

    # -------------------------
    # 1) Foot positions XYZ
    # -------------------------
    fig_pos, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for leg in LEG_NAMES:
        px = df[f"p_{leg}_x"]
        py = df[f"p_{leg}_y"]
        pz = df[f"p_{leg}_z"]

        axs[0].plot(t, px, label=leg)
        axs[1].plot(t, py, label=leg)
        axs[2].plot(t, pz, label=leg)

    axs[0].set_ylabel("x [m]")
    axs[1].set_ylabel("y [m]")
    axs[2].set_ylabel("z [m]")
    axs[2].set_xlabel("time [s]")
    axs[0].set_title("Foot positions (base frame)")
    axs[0].legend()
    for ax in axs: ax.grid(True)

    # -------------------------
    # 2) Foot velocities norm
    # -------------------------
    fig_vel, ax = plt.subplots(figsize=(10, 4))
    for leg in LEG_NAMES:
        vx = df[f"v_{leg}_x"]
        vy = df[f"v_{leg}_y"]
        vz = df[f"v_{leg}_z"]
        vnorm = np.sqrt(vx**2 + vy**2 + vz**2)
        ax.plot(t, vnorm, label=leg)

    ax.set_title("Foot velocity norm (|v|)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("m/s")
    ax.legend()
    ax.grid(True)

    # -------------------------
    # 3) XY trajectory (feet)
    # -------------------------
    fig_xy, ax = plt.subplots(figsize=(6, 6))
    for leg in LEG_NAMES:
        ax.plot(df[f"p_{leg}_x"], df[f"p_{leg}_y"], label=leg)

    ax.set_title("Foot trajectories (XY, base frame)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()

    # -------------------------
    # 4) Jacobian sanity check
    # -------------------------
    fig_jac, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    leg = "LF"

    rows = ["x", "y", "z"]
    cols = ["HAA", "HFE", "KFE"]

    for i in range(3):  # rows
        for j in range(3):  # cols
            axs[i].plot(
                t,
                df[f"J_{leg}_{i}{j}"],
                label=f"d p_{rows[i]} / d q_{cols[j]}"
            )
        axs[i].set_ylabel(f"d p_{rows[i]} / d q")
        axs[i].legend()
        axs[i].grid(True)

    axs[0].set_title(f"Foot Jacobian entries for {leg} leg")
    axs[-1].set_xlabel("time [s]")
    plt.show()



if __name__ == "__main__":
    main()
