<h1 align="center"> A Proprioceptive-Only Benchmark for Quadruped State Estimation: ATE, RPE, and Runtime Trade-offs Between Filters and Smoothers </h1>
<h3 align="center"> Ylenia Nisticò, João Carlos Virgolino Soares, Joan Solà, Claudio Semini</h3>
<h4 align="center"> Paper available on ArXiv (https://arxiv.org/abs/2605.11674) </h4>

##

This repository provides an offline benchmarking pipeline for quadruped state estimation using CSV datasets (proprioceptive measurements and ground truth). It provides the data and the code to replicate the results presented in the paper. 

The CSV datasets are generated from the [ANYmal GrandTour dataset](https://grand-tour.leggedrobotics.com/dataset), specifically from the rosbags of sequence **CYN-1**.



## Dependencies
We recommend using conda-forge to ensure C++ and Python dependencies across machines.
Create and activate the environment:
```
conda env create -f environment.yml
conda activate state_est_bench
```

If you prefer to install manually, the full list of dependencies is:

**Build tools**
- [CMake >= 3.22](https://github.com/Kitware/CMake)
- [Ninja](https://github.com/ninja-build/ninja)
- C/C++ compiler (gcc/clang)

**C++ libraries**
- [Eigen](https://gitlab.com/libeigen/eigen)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [fmt](https://github.com/fmtlib/fmt)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

**Python packages**
- [numpy](https://github.com/numpy/numpy)
- [pandas](https://github.com/pandas-dev/pandas)
- [matplotlib](https://github.com/matplotlib/matplotlib)
- [scipy](https://github.com/scipy/scipy)
- [pyproj](https://github.com/pyproj4/pyproj) + [PROJ](https://github.com/OSGeo/PROJ) (for GNSS georeferencing)
- [contextily](https://github.com/geopandas/contextily) (for map tile backgrounds in plots)
- [evo](https://github.com/MichaelGrupp/evo) (for trajectory metrics)

## Data format
`sensor_data.csv` is a CSV file containing proprioceptive measurements. Its format is:
```
timestamp, imu_wx, imu_wy, imu_wz, imu_ax, imu_ay, imu_az, js, contacts
```
`groundtruth.csv` is a CSV file containing ground-truth data:
```
timestamp, px, py, pz, qx, qy, qz, vx, vy, vz
```
You can download `sensor_data.csv` and `grountruth.csv` from this [link](https://drive.google.com/drive/folders/13FPdESYe10gAHfvmjsgJCjq7s2pjQzI-?usp=sharing).
For this project you need to copy these files in [`data/anymalD_grandtour`](https://github.com/iit-DLSLab/state_estimation_benchmark/tree/main/data/anymalD_grandtour)

## Step 1 - Dataset sanity check
From the root directory, build and run the dataset inspection tool:
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

## Step 3 - Run MUSE 
From the root directory, build and run MUSE:
```
cd muse
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./main_muse
```
<!-- or, since MUSE is a modular state estimator, we provide executables for each single modules. They can be run also separately:
```
./main_attitude
./main_leg_odometry
./main_sensor_fusion
``` -->
Default input dataset root: `data/anymalD_grandtour`

Generated output:
- `data/anymalD_grandtour/muse/fused_state.csv`
- `data/anymalD_grandtour/muse/attitude_estimate.csv`
- `data/anymalD_grandtour/muse/leg_odometry.csv`

<!-- Generated outputs if the modules are run separately:
- `data/anymalD_grandtour/muse/attitude_estimate_muse.csv`
- `data/anymalD_grandtour/muse/leg_odometry.csv`
- `data/anymalD_grandtour/muse/fused_state.csv` -->

## Step 4 - Run IEKF
From the root directory, build and run IEKF:
```
cd iekf
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./main_iekf
```
Generated output:
- `data/anymalD_grandtour/iekf/fused_state.csv`

## Step 5 - Run Invariant Smoother
From the root directory, build and run the invariant smoother:
```
cd invariant_smoother
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./main_invariant_smoother
```
Generated output:
- `data/anymalD_grandtour/invariant_smoother/fused_state.csv`

## Step 6 - Plot and compare results
To plot and compare fused trajectories: GT vs MUSE vs IEKF vs Invariant Smoother, from the repository root, run:
```
python3 data_process/scripts/plots.py
```

## Step 7 - Convert trajectories to TUM format
First, convert local-frame GT and estimator trajectories to TUM format (`t x y z qx qy qz qw`).
Run from the dataset folder:
```
cd data/anymalD_grandtour
python3 ../../data_process/scripts/convert_to_tum.py
```

Generated files:
- `data/anymalD_grandtour/tum/groundtruth_traj_tum.csv`
- `data/anymalD_grandtour/tum/muse_traj_tum.csv`
- `data/anymalD_grandtour/tum/iekf_traj_tum.csv`
- `data/anymalD_grandtour/tum/is_traj_tum.csv`

The script `convert_to_tum.py` also prints the command to compute the evaluation metrics from `evo` on the estimated trajectories.

Optionally, you can also georeference TUM trajectories to UTM using GNSS with `data/anymalD_grandtour/tum/georef_from_gnss.py`.

From repository root:
```
python3 data/anymalD_grandtour/tum/georef_from_gnss.py \
  --gnss data/anymalD_grandtour/tum/gt_navsatfix.csv \
  --gt data/anymalD_grandtour/tum/groundtruth_traj_tum.csv \
  --traj data/anymalD_grandtour/tum/muse_traj_tum.csv data/anymalD_grandtour/tum/iekf_traj_tum.csv data/anymalD_grandtour/tum/is_traj_tum.csv \
  --out_dir data/anymalD_grandtour/tum/georef_out \
  --fit_seconds 60 \
  --use_gnss_z
```

Outputs are written in `data/anymalD_grandtour/tum/georef_out/` as `gt_georef.tum`
and one `*_georef.tum` per trajectory.

