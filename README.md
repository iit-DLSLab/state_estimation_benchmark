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
cd data_preprocess
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
Output: `data/anymalD_grandtour_feet_kinematics.csv`

## Step 3 - MUSE
This is a direct offline port of the MUSE [`attitude_estimation_plugin`](https://github.com/iit-DLSLab/muse/blob/main/muse_ws/src/state_estimator/src/plugins/attitude_estimation_plugin.cpp).
It:
- reads `sensor_data.csv`
- uses the `AttitudeBiasXKF` logic
- uses timestamps from the CSV
- writes the estimated orientation to CSV

Build:
```
cd muse
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```
Run:
```
./main_attitude
```
Output: `data/anymalD_grandtour/attitude_estimate_muse.csv` in the format:
```
t_rel, t_abs,
qw, qx, qy, qz,
bgx, bgy, bgz,
roll_deg, pitch_deg, yaw_deg,
omega_filt_x, omega_filt_y, omega_filt_z
```
## Step 4 - Plotting & Validation
### Attitude estimated vs. Ground Truth
Run
```
python3 data_preprocess/scripts/plot_muse_attitude_vs_gt.py
```