#!/usr/bin/env python3
import os, sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_csv_no_header(path, expected_cols=None):
    df = pd.read_csv(path, header=None)
    if expected_cols is not None and df.shape[1] != expected_cols:
        raise ValueError(f"{path}: expected {expected_cols} cols, got {df.shape[1]}")
    return df.to_numpy(dtype=float)

def main():
    root = "data/anymalD_grandtour"
    gt_path = os.path.join(root, "groundtruth.csv")
    est_path = os.path.join(root, "estimate_example.csv")  # crea un esempio o copia qui

    gt = load_csv_no_header(gt_path, expected_cols=11)
    est = load_csv_no_header(est_path, expected_cols=11)

    N = min(len(gt), len(est))
    gt = gt[:N]; est = est[:N]
    t = gt[:,0]
    gt_p = gt[:,1:4]; est_p = est[:,1:4]
    gt_v = gt[:,8:11]; est_v = est[:,8:11]

    fig, axs = plt.subplots(3,1,sharex=True)
    labels = ["x","y","z"]
    for i in range(3):
        axs[i].plot(t, gt_p[:,i], label="GT")
        axs[i].plot(t, est_p[:,i], '--', label="Est")
        axs[i].set_ylabel(labels[i])
        axs[i].legend()
    axs[-1].set_xlabel("t [s]")
    plt.show()

if __name__ == "__main__":
    main()
