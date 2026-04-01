# State Estimation Benchmark
This repository provides an offline benchmarking pipeline for quadruped state estimation using CSV datasets (proprioceptive measurements and ground truth).

At the current stage, the repository supports:
- Offline data preprocessing (Pinocchio --> foot kinematics)
- Offline porting of MUSE

## Dependencies
- C++ toolchain
- Eigen
- Pinocchio
- Python (numpy, pandas, matplotlib, scipy)

We recommend using conda-forge to ensure C++ and Python dependencies across machines.
Create and activate the environment: 
```
conda env create -f environment.yml
conda activate state_est_bench
```

## Data format
`sensor_data.csv` is a CSV file containing proprioceptive measurements. Its format is:
```
timestamp, imu_wx, imu_wy, imu_wz, imu_ax, imu_ay, imu_az,
js, contacts
```
`groundtruth.csv` is a CSV file containing ground-truth data:
```
timestamp, px, py, pz, qx, qy, qz, vx, vy, vz
```

## Step 1 - Dataset sanity check
Build and run the dataset inspection tool:
```
cd data_process
mkdir -p build && cd build
cmake ..
make -j$(nproc)

./inspect_dataset ../../data/anymalD_grandtour
```
This verifies:
- CSV parsing
- timestamps
- number of samples

## Step 2 - Precompute Foot Kinematics (Pinocchio)
This step computes, offline, all kinematics required by leg-based estimators:
- Foot positions (base frame)
- Foot Jacobians
- Foot velocities

Build and run:
```
./precompute_feet_kinematics
```
Output: `data/anymalD_grandtour/feet_kinematics.csv`

## Step 3 - Run MUSE (attitude + leg odometry + fused state)
Build and run MUSE:
```
cd muse
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./main_muse
```
or you can run also separately:
```
./main_attitude_estimation
./main_leg_odometry
./main_muse
```
Default input dataset root: `data/anymalD_grandtour`

Generated outputs:
- `data/anymalD_grandtour/muse/attitude_estimate_muse.csv`
- `data/anymalD_grandtour/muse/leg_odometry.csv`
- `data/anymalD_grandtour/muse/fused_state.csv`

## Step 4 - Run IEKF
Build and run IEKF:
```
cd iekf
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# optional: pass a custom dataset root as first argument
./main_iekf
```
Generated output:
- `data/anymalD_grandtour/iekf/fused_state.csv`

## Step 5 - Run Invariant Smoother
Build and run the invariant smoother:
```
cd invariant_smoother
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# optional: pass a custom dataset root as first argument
./main_invariant_smoother
```
Generated output:
- `data/anymalD_grandtour/invariant_smoother/fused_state.csv`

## Step 6 - Plot and compare results
From the repository root, run:
```
# MUSE attitude vs GT
python3 data_process/scripts/plot_muse_attitude_vs_gt.py

# MUSE leg odometry velocity vs GT velocity
python3 data_process/scripts/plot_legodom_vs_gtvel.py

# Compare fused trajectories: MUSE vs IEKF vs Invariant Smoother vs GT
python3 data_process/scripts/plot_fused_vs_gt.py
```

Note:
- `plot_muse_attitude_vs_gt.py` and `plot_legodom_vs_gtvel.py` read `groundtruth.csv`.
- `plot_fused_vs_gt.py` currently reads `anymal_state.csv` as GT.

## One-command full pipeline
From the repository root, you can run preprocessing + MUSE + IEKF + Invariant Smoother with one command:
```
./run_all_estimators.sh
```

Optional custom dataset root (relative to repository root):
```
./run_all_estimators.sh --dataset data/anymalD_grandtour
```
