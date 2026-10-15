# Online (ROS 2) state estimation benchmark for the Unitree Go2

This document describes the ROS 2 extension of the offline benchmark: an online
pipeline that runs **one** of the three estimators of the paper (MUSE, IEKF,
invariant smoother) on Unitree Go2 `LowState` data, with exact sample-level
synchronisation and the instrumentation needed for a runtime comparison.

> **Monitoring only.** The node publishes estimates; it has no command or
> actuation interface (enforced by the test `test_no_actuation_interface`).
> Nothing in this package has been validated on the real robot yet — see
> [Validation status](#validation-status).

Contents: [Architecture](#architecture) · [Synchronisation guarantees](#synchronisation-guarantees) ·
[Time stamps](#time-stamps) · [Go2 interface findings](#go2-lowstate-findings) ·
[Frames and mapping](#frames-and-mapping) · [Estimators](#estimators) ·
[Changes to the offline code](#changes-to-the-offline-code) · [Queues and drop policy](#queues-backpressure-and-events) ·
[Output](#output-interface) · [Logging](#logging-and-analysis) · [Build](#build) · [Run](#run) ·
[Tests](#tests) · [Validation status](#validation-status) · [Limitations](#known-limitations) ·
[MPC checklist](#checklist-before-connecting-the-external-mpc)

---

## Architecture

```
 Unitree Go2  ── unitree_go/LowState (~500 Hz, DDS) ──┐
                                                       │  subscription callback (own callback group)
                                                       ▼  reception time: steady + ROS clock, DDS source stamp
                                   ┌──────────────── Acquisition ────────────────┐
                                   │ validate (finite) · drop exact duplicates    │
                                   │ tick unwrap · time base · non-increasing ->  │
                                   │ reject · decimation · canonical order        │
                                   └───────────────┬──────────────────────────────┘
                     capacity check (both worker queues + barrier) -> else kPipelineFull
                     sample_id = next id ; register in the barrier ; IMMUTABLE SensorSample
                         ┌─────────────────────────┴──────────────────────────┐
                         ▼ shared_ptr<const SensorSample>                     ▼ (same object)
              ┌─ Kinematics worker thread ─┐                     ┌─ Contact worker thread ─┐
              │ Pinocchio (shared offline   │                     │ Go2 foot force, MUSE    │
              │ implementation), frame E    │                     │ strategy, hysteresis    │
              └─────────────┬───────────────┘                     └────────────┬────────────┘
                            │ KinematicsResult{sample_id, stamp}               │ ContactResult{sample_id, stamp}
                            └────────────────►  ExactSampleSynchronizer  ◄─────┘
                                     exact sample_id match · in-order release · per-sample
                                     deadline (supervisor thread) · late results discarded
                                                       │ bounded queue, overflow policy
                                                       ▼
                                   ┌──────── Estimator worker thread ────────┐
                                   │ stale / dt / gap checks · ONE estimator  │
                                   │ (MUSE | IEKF | invariant smoother)       │
                                   │ frame E -> base · state stamped with the │
                                   │ SAMPLE time                              │
                                   └──────────┬───────────────────────────────┘
                                              ├─► seb_interfaces/EstimatorState (~/state)   ─► external MPC (future)
                                              ├─► /diagnostics (1 Hz)
                                              └─► run logger thread ─► samples.csv, events.csv
```

| Thread | Owns | Blocking |
|---|---|---|
| ROS executor (2 threads, separate callback groups for `LowState` and for services/timers) | subscription, reset service, diagnostics timer | — |
| `seb_kinematics` | its Pinocchio `Model`/`Data` | blocks on its input queue |
| `seb_contact` | the contact detector (hysteresis state) | blocks on its input queue |
| synchroniser supervisor | — | sleeps until the next sample deadline |
| `seb_estimator` | the estimator object (the only caller of its update functions) | blocks on the bundle queue |
| logger | CSV files | blocks on its queue |

No lock is held during the Pinocchio computation or the estimator update; each of
them runs on state owned by a single thread. All queues are fixed-capacity ring
buffers; consumers block on condition variables (no busy waiting).

Code layout:

```
ros2_ws/src/seb_interfaces/          EstimatorState.msg
ros2_ws/src/seb_online/
  include/seb_online/, src/          core pipeline (ROS independent) + adapters
  src/node/online_estimator_node.cpp lifecycle node
  src/tools/bag_replay.cpp           deterministic bag replay (seb_replay)
  config/go2_online.yaml             all parameters with defaults
  launch/go2_online.launch.py        live robot
  launch/go2_bag_replay.launch.py    safe offline replay
  scripts/                           analyze_run.py, profile_window_sizes.sh,
                                     inspect_lowstate_bag.py, compare_offline_regression.py
  test/                              gtest suites
common/include/benchmark/FootKinematics.hpp   Pinocchio kinematics shared with data_process
models/go2/go2.urdf                  Go2 model (from the MUSE repository, unitree_sdk branch)
```

The estimators are **not copied** into the ROS package: `seb_online` compiles the
libraries of `muse/`, `iekf/` and `invariant_smoother/` from this repository.

## Synchronisation guarantees

1. Every accepted `LowState` becomes one immutable `SensorSample` with a
   monotonically increasing, consecutive `sample_id` (plus `acq_seq`, which also
   counts samples lost before dispatch, and `msg_seq`, which counts every message).
2. The **same object** is handed to the kinematics and the contact worker; IMU data
   stay inside it.
3. Both workers return results carrying the `sample_id` **and** stamp of their input.
4. `ExactSampleSynchronizer` builds a bundle only from `{sample k, kinematics k,
   contact k}`; a result whose stamp differs from its sample is rejected
   (`identity_mismatches`). Results are never paired across ids, never reused,
   and stamps are never replaced.
5. Bundles are released in strictly increasing `sample_id` order; a complete
   bundle `k+1` waits while `k` is pending.
6. A sample without both results `sync.worker_timeout_s` after dispatch is expired
   and logged (`missing_kinematics`, `missing_contact`,
   `missing_kinematics_and_contact`); results arriving later are logged as
   `late_worker_result` and discarded.
7. The estimator never receives a non-positive time step (`dt < min_dt_s` →
   `dt_too_small`, not processed); updates after lost samples are processed with the
   real elapsed `dt`, flagged `gap` and published as `DEGRADED`; a gap longer than
   `estimator.max_gap_s` re-initialises the estimator (`reset_gap`); a stamp earlier
   than the last processed one (new time-base epoch) re-initialises it as well
   (`reset_time_jump`).
8. Memory is bounded everywhere (fixed-capacity queues, fixed slot ring in the
   barrier; one `SensorSample` allocation per message).

These properties are tested in `test_synchronizer` and `test_pipeline` (see [Tests](#tests)).

Every sample carries IMU **and** leg data, and all three estimators use both in
the same update. A dropped sample therefore always removes one IMU propagation
input **and** one kinematic/contact correction; the event log records the
`sample_id`, `msg_seq`, `tick` and reason of every lost sample.

## Time stamps

| Quantity | Domain | Where |
|---|---|---|
| Sensor acquisition time | **not available** — `LowState` has no stamp; see `tick` | — |
| `LowState.tick` | robot MCU counter, 1 ms/count (measured), 1 ms resolution | `tick` |
| DDS source timestamp | publisher (robot) clock, not synchronised with the PC | `dds_source_ns` (live only) |
| Reception time | ROS clock and steady clock at the start of the subscription callback | `receive_ros_ns`, `receive_steady_ns` |
| Estimator time base (sample stamp) | `receive_time` (default) or `lowstate_tick`, labelled in every output | `stamp_ns`, `EstimatorState.header.stamp`, `stamp_source` |
| Dispatch / worker start-end / bundle complete / release / dequeue | steady | `*_steady_ns` |
| Estimator start / completion | steady | `est_start_steady_ns`, `est_end_steady_ns` |
| State timestamp | = stamp of the source sample (the newest sample, for every estimator) | `header.stamp` |
| Publication time | steady + ROS clock immediately before `publish()` | `publish_*_ns` |

* Durations are differences of **steady** clock values only.
* `state_age_at_publish_s = publish_ros - stamp` is a difference inside the ROS time
  domain. With `receive_time` it equals the end-to-end latency; with `lowstate_tick`
  it also contains the (unknown, fixed) offset chosen at the first sample.
* No value from the robot clock (`tick`, DDS source stamp) is ever subtracted from a
  PC clock value.
* With `receive_time` the stamps include the robot→PC transport delay and its jitter:
  they are **reception** times, not acquisition times.

## Go2 LowState findings

Measured on lab recordings (`unitree-ros2-dls/rosbag2_2026_07_30-15_19_56`,
`rosbag2_2026_08_04-13_28_00`) with `scripts/inspect_lowstate_bag.py`:

| | 2026-07-30 15:19 | 2026-08-04 13:28 |
|---|---|---|
| rate | 500.0 Hz | 498.9 Hz |
| reception Δt mean / std / max | 2.000 / 0.351 / 5.6 ms | 2.005 / 1.837 / **111 ms** |
| tick period (fit vs reception) | 1.0000 ms | 1.0001 ms |
| tick Δ per message | 1/2/3 (12571 × 2, 5637 × 1, 5632 × 3) | 1/2/3 |
| repeated ticks / exact duplicates | 31 / 19 | 49 / 5 |

Consequences built into the pipeline:
* exact duplicate messages exist → dropped (`duplicate`);
* `tick` is a usable, drift-free counter but quantised to 1 ms (Δtick ∈ {1,2,3}
  for a 2 ms period) and occasionally repeats;
* the 2026-08-04 recording has five ~110 ms reception stalls while the tick
  never advances by more than 11 ms: these are transport/recording delays, not
  missing sensor data. With `receive_time` they appear as gaps and re-initialise
  the estimator (`reset_gap` ×5); with `lowstate_tick` they disappear, but the 45
  samples with a repeated tick are rejected (zero dt) and dt is quantised.
  Neither time base is exact; both are available and logged. **Characterise the
  real robot→PC link before choosing** (see checklist).

## Frames and mapping

**Canonical order** used by every stage: legs `LF, RF, LH, RH` (Unitree `FL, FR,
RL, RR`), joints per leg `HAA, HFE, KFE` (Unitree `hip, thigh, calf`).

**Go2 LowState order** (Unitree documentation): `motor_state[0..11]` = FR, FL, RR, RL
(3 joints each); `foot_force[0..3]` = FR, FL, RR, RL. Default maps
(`robot.lowstate_motor_index`, `robot.lowstate_foot_force_index`):

```
motor index per canonical joint : [3,4,5, 0,1,2, 9,10,11, 6,7,8]
foot_force index per canonical leg: [1, 0, 3, 2]
```

Evidence in the lab bags (consistent, not conclusive): foot-force/calf-torque
correlation is diagonal-dominant (same leg order for both arrays) and the mean hip
abduction angle alternates sign as expected for FR, FL, RR, RL in the 2026-08-04
recording.

> The reference MUSE plugin (`unitree_sdk` branch) assigns `foot_force[0..3]`
> directly to LF, RF, LH, RH. With the order above that labels the FR foot as LF.
> seb_online uses the explicit map instead. **Validate on the robot** (lift one foot
> at a time, `inspect_lowstate_bag.py --segments`).

**Internal order of the KAIST estimators** (IEKF, smoother): `RH, LH, LF, RF`
(`seb::kInvariantLegOrder`), used consistently for joints, contacts and kinematics.

**Frames**
* Base `B`: URDF link `base` (x forward, y left, z up).
* IMU: URDF frame `imu`, read from the URDF: `p_base_imu = (-0.02557, 0, 0.04232) m`,
  `R_base_imu = I` (Go2). Measured specific force at rest ≈ +9.3…9.8 m/s² along z.
* Estimation frame `E`: base axes, origin at the IMU. The acquisition stage rotates
  gyroscope/accelerometer into base axes; the kinematics worker expresses foot
  positions/Jacobians in `E` (`p_E = p_B - p_base_imu`). All three estimators
  estimate the pose/velocity of `E`, which keeps the strap-down propagation exact.
* Output conversion `E → B` (estimator worker):
  `R_WB = R_WE`, `p_WB = p_WE - R_WE p_base_imu`,
  `v_WB = v_WE + R_WE (ω × (-p_base_imu))`, ω = bias-corrected gyro.
* World `W`: gravity aligned, z up; origin and yaw = initial pose of the base.
  Gravity −9.81 m/s² along z (IEKF propagation: −9.80665, unchanged from the
  original implementation).
* Initial attitude: `estimator.init_attitude: gravity_aligned` (roll/pitch from the
  first accelerometer sample, yaw = 0) or `identity` (offline behaviour). Biases
  start at zero for every estimator (as offline).

The offline ANYmal pipeline ignored the IMU lever arm; the online Go2 pipeline does not.

## Estimators

`seb::EstimatorAdapter` converts a `SynchronizedMeasurement` into each
estimator's input and calls the **unchanged** update of the offline code. Exactly
one adapter is created (`estimator.type`). Default parameters are those of the
offline mains. With the offline configuration (`dt_mode: offline_compat`,
`init_attitude: identity`, `muse.use_updated_attitude: false`) the adapters
reproduce the offline outputs **for all 208 084 samples of the ANYmal dataset**
(positions/velocities identical at the 1e-9 precision of the CSV files, orientation
within 1.9e-9 rad, i.e. quaternion print rounding) — test `OfflineRegression.*`.

Time steps: in the default `dt_mode: measured` every update uses the measured
interval between the current and the previously processed sample.
* MUSE already uses timestamps (attitude XKF and KF compute `t - t_prev`).
* IEKF: the offline main writes `estimator_common_struct_.dt`, but the propagation
  uses the member `dt` fixed in `Initialize()` (0.0025 s); the adapter sets the
  member `dt` per step.
* Smoother: the propagation factors use one scalar dt (that of the latest step) for
  every interval in the window; the new option `use_per_interval_dt`
  (`smoother.per_interval_dt`, default true online) gives each factor its own
  measured interval. For uniform sampling both are bitwise identical (tested).

Investigation of the features requested in the task:

| | MUSE | IEKF | Invariant smoother |
|---|---|---|---|
| Separate IMU prediction | yes (`predict`/`update` of XKF and KF) | yes (`InEKF::Propagate` / `CorrectKinematics`) | no (one factor-graph update per keyframe) |
| Lower-rate kinematic correction | possible (KF update optional) | possible (`CorrectKinematics` optional) | via keyframe decimation only |
| Batch IMU processing | no | no | `preintegration_mode_` exists in the code (IMU buffered between keyframes) but is unused offline and not validated → not enabled |
| Propagation to a controller time | not implemented (would be an extension) | `Propagate` could be called with the latest IMU | no |

None of these was enabled: they change the measurement model or the update rate
w.r.t. the paper configuration. Every sample is one update for all three
estimators. The only rate-reduction mechanism provided is the explicit, logged
`acquisition.decimation` (applied to every estimator identically).

**Smoother output** (from `InvariantSmoother::send_states`): the published state is
`RS[frame_count]`, the **newest** frame of the window after the optimisation, stamped
with the newest sample time. The smoothed older frames are not output. The output
delay is the optimisation time, not the window duration. The log reports both:
`window_duration_s` (newest − oldest frame stamp in the window) and the execution
time. `smoother.window_size` is a runtime parameter (any value ≥ 1).

## Changes to the offline code

All offline outputs were regenerated after the changes and are **byte-identical** to
those of the original code (`feet_kinematics.csv`, MUSE `fused_state.csv`,
`attitude_estimate.csv`, `leg_odometry.csv`, IEKF and smoother `fused_state.csv`,
208 084 samples, Release build).

| File | Change | Effect on offline results |
|---|---|---|
| `common/…/FootKinematics.{hpp,cpp}`, `data_process/apps/precompute_feet_kinematics.cpp` | Pinocchio computation moved to a shared class used by the precomputation and the online worker; the unused second kinematics block of the app was removed | none (file identical) |
| `invariant_smoother/…/InvariantSmoother.{hpp,cpp}`, `RobotState_Smoother.hpp` | window size `WINDOW_SIZE` (compile-time 3) → runtime `window_size` (default 3); window buffers in `std::vector`; per-interval dt option (default off) | none |
| `iekf/…/RobotStateIEKF.hpp` | `factor_info::dt_interval` added (unused) to keep the class identical to the smoother copy — both libraries are linked into the node | none |
| `iekf/apps/main_iekf.cpp`, `invariant_smoother/apps/main_invariant_smoother.cpp` | `EstimatorCovariances` initialised from `EstimatorCommonStruct()` | none (see below) |
| `*/CMakeLists.txt` | library targets usable from the ROS package (`muse_lib` interface target, `SEB_BUILD_OFFLINE_APPS`, PIC, guarded `common`) | none |

**Latent bug fixed**: `EstimatorCovariances` does not initialise its 3×3 covariance
matrices and `Covariance_Reset()` only writes their diagonals, so the off-diagonal
terms were whatever was on the stack. The offline runs happened to see zeros
(outputs unchanged after the fix); in the online adapter the same pattern produced
P ≈ 1e72 and NaN after two steps. Both now start from the constructor's identity
matrices.

Other observations (not changed):
* The offline IEKF/smoother mains fill the joint blocks in the order RH, LH, RF, LF
  while contacts and kinematics use RH, LH, LF, RF. The joint blocks only enter the
  slip-rejection / variable-contact-covariance logic, disabled in the benchmark, so
  the published results are unaffected. The adapters use one consistent order.
* MUSE: the offline main uses the attitude of the **previous** sample for leg
  odometry, acceleration rotation and the output. Online default
  `muse.use_updated_attitude: true` uses the current one so that the published
  orientation matches the state time stamp; `false` reproduces the offline code.
* Pinocchio's CMake target enables OpenMP. `seb_online` defines
  `EIGEN_DONT_PARALLELIZE` so estimator updates stay single-threaded, as in the
  offline build.
* The KAIST IEKF/smoother classes contain ~190 MB of offline buffers each; the
  adapters allocate them in lazily-zeroed anonymous memory (same initial state as
  the offline globals, only touched pages become resident).

## Queues, backpressure and events

| Stage | Bound | When full |
|---|---|---|
| worker input queues | `queues.worker_capacity` (64) | sample not dispatched to either worker: `pipeline_full` |
| barrier pending samples | `sync.capacity` (64) | idem |
| estimator input queue | `queues.estimator_capacity` (50 = 100 ms at 500 Hz) | `estimator_overflow_policy`: `drop_oldest` (default, keeps the freshest data) or `reject_newest`; logged `estimator_backlog_drop` |
| stale input | `estimator.max_input_age_s` (0.1 s since reception) | dropped, `stale_input` |
| log queue | `log.queue_capacity` | record counted as dropped (reported) — never blocks the pipeline |

Event types in `events.csv`: `invalid_message, duplicate, out_of_order, decimated,
pipeline_full, missing_kinematics, missing_contact, missing_kinematics_and_contact,
late_worker_result, estimator_backlog_drop, stale_input, gap, reset_gap,
reset_diverged, reset_requested, reset_time_jump, dt_too_small, estimator_exception`.
`/diagnostics` turns WARN when samples are lost in the last period and ERROR on any
synchronisation integrity counter.

## Output interface

`seb_interfaces/msg/EstimatorState` on `~/state` (default
`/seb_online_estimator/state`): `header.stamp` = state time (source sample),
`frame_id` = world, `child_frame_id` = base, estimator name, `sample_id`,
`lowstate_tick`, `stamp_source`, `status` (INITIALIZING / VALID / DEGRADED /
INVALID), orientation (base→world), position (+`position_valid`), linear velocity
in world and base frame, angular velocity in base frame with its source
(`IMU_BIAS_CORRECTED` or `IMU_RAW`: none of the estimators estimates angular
velocity, it is the gyroscope of the same sample), gyro/accelerometer biases with
validity flags (MUSE has no accelerometer bias), the contact set used, and timing
(`latency_to_estimate_s`, `estimator_compute_time_s`, `state_age_at_publish_s`,
`estimator_queue_depth`, `dropped_samples_total`). Optional `nav_msgs/Odometry`
(`output.publish_odometry`), covariances not provided (zeros).
No covariance of the estimate is published (not exposed uniformly by the three
implementations).

## Logging and analysis

Each run writes `<log.directory>/<run_name>/`:
* `run_config.yaml` — every parameter value, estimator description, URDF, IMU placement;
* `samples.csv` — one row per estimator update: ids, tick, all time stamps, derived
  durations (kinematics, contact, synchronisation wait, queue wait, estimator
  execution, latency to estimate, end-to-end latency, state age), queue occupancy at
  dispatch and dequeue, samples missing before the update, dt, smoother iterations,
  window frames and duration, the state and the contacts;
* `events.csv` — every lost/dropped sample and every reset, with reason and ids.

`scripts/analyze_run.py RUN [RUN...]` prints mean, median, p95, p99 and maximum of
every duration, the effective output rate, backlog, drop rate per reason, and a
comparison table for several runs (`--json` for a machine-readable summary).
Execution time and latency are reported separately.

## Build

Requirements: ROS 2 (developed and tested with **Humble**; `rclcpp`,
`rclcpp_lifecycle`, `rosbag2_cpp`, `diagnostic_msgs`, `nav_msgs`, `std_srvs`),
Pinocchio (tested 3.7), Eigen 3, GTest. Python scripts need `numpy`. All of these
are in the repository conda env `state_est_bench` (ROS 2 Humble from RoboStack), so
no system ROS install is needed.

The repository contains a ready colcon workspace, `ros2_ws/`:

```
ros2_ws/src/seb_interfaces   messages of the online benchmark
ros2_ws/src/seb_online       online pipeline, node, replay tool, tests
ros2_ws/src/unitree_go       Unitree Go2 LowState messages (rosidl only)
```

`unitree_go` is vendored because it is not packaged for conda. It is the trimmed
copy from the MUSE repository, which needs only the standard rosidl generators.

```bash
conda env create -f environment.yml        # or: conda env update -n state_est_bench -f environment.yml
conda activate state_est_bench
cd ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython_EXECUTABLE=$CONDA_PREFIX/bin/python3
source install/setup.bash
```

If the build fails with `No 'rosidl_typesupport_c' found`, `AMENT_PREFIX_PATH` is
not set: the ROS activation hook of the env did not run (typically when ROS was added
to an env that was already active). Run `conda deactivate && conda activate
state_est_bench` and build again.

Make sure no Python virtualenv (for example the repository `.venv`) is ahead of the
conda env on `PATH` while building, otherwise the rosidl generators run with the
wrong interpreter. Always run colcon from `ros2_ws/`, never from the repository
root: colcon would treat the top-level `CMakeLists.txt` as a plain CMake package.

With a system ROS 2 Humble install instead of conda, source
`/opt/ros/humble/setup.bash` before the same `colcon build` (the
`-DPython_EXECUTABLE` flag can then be dropped). If your environment already
provides `unitree_go` (for example from `unitree_ros2`), skip the vendored copy with
`colcon build --packages-ignore unitree_go`.

The offline pipeline is built exactly as before (see the main README); its
executables are not part of the ROS build.

## Run

Live robot (monitoring only), one estimator per run:

```bash
ros2 launch seb_online go2_online.launch.py estimator:=muse
ros2 launch seb_online go2_online.launch.py estimator:=iekf
ros2 launch seb_online go2_online.launch.py estimator:=invariant_smoother window_size:=3
```

Useful arguments: `lowstate_topic:=/lowstate`, `config:=<yaml>`, `run_name:=...`,
`log_dir:=...`, `log:=false`. Reset the estimator:
`ros2 service call /seb_online_estimator/reset_estimator std_srvs/srv/Trigger`.

Recorded bags (never republishes on `/lowstate`):

```bash
# deterministic: recorded reception times, every sample processed (execution-time profiling)
ros2 launch seb_online go2_bag_replay.launch.py bag:=/path/to/bag estimator:=iekf
# deterministic at the recorded pace (backlog / drop behaviour as on the robot)
ros2 launch seb_online go2_bag_replay.launch.py bag:=/path/to/bag estimator:=invariant_smoother replay_mode:=realtime
# full ROS transport: ros2 bag play remapped to /seb_replay/lowstate + the online node
ros2 launch seb_online go2_bag_replay.launch.py bag:=/path/to/bag estimator:=muse mode:=ros_play
# or directly
ros2 run seb_online seb_replay --ros-args --params-file $(ros2 pkg prefix seb_online)/share/seb_online/config/go2_online.yaml \
    -p replay.bag_path:=/path/to/bag -p estimator.type:=invariant_smoother -p smoother.window_size:=5
```

Profiling and analysis:

```bash
ros2 run seb_online profile_window_sizes.sh /path/to/bag lossless 1 2 3 5 10
ros2 run seb_online analyze_run.py ~/.ros/seb_online/<run> [more runs] --json summary.json
ros2 run seb_online inspect_lowstate_bag.py /path/to/bag [--segments 10]
ros2 run seb_online compare_offline_regression.py <reference_dataset_dir> <new_dataset_dir>
```

For runtime experiments fix the CPU frequency (e.g. `performance` governor):
on the development PC the smoother update took 0.51 ms on average in lossless
replay but 1.02 ms when paced in real time, consistent with frequency scaling
while the CPU idles between samples. Thread affinity / SCHED_FIFO priorities are
available under `threads.*` (nothing is applied by default).

## Tests

```bash
colcon test --packages-select seb_online && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `test_synchronizer` | shuffled completion order of 100 samples; delayed kinematics k vs contact k+1; missing contact → timeout, expiry event, later result discarded; missing both; timestamp integrity (same object, mismatched stamp rejected); out-of-order registration; duplicate results; bounded capacity; drop_oldest / reject_newest with ids reported; reset; supervisor thread enforces deadlines on its own |
| `test_pipeline` | real threads with per-sample delays in both workers (300 samples, every bundle coherent, dt exact, single-threaded estimator access, state stamp = sample time ≠ publication time); missing contact → gap with dt across it and DEGRADED; slow estimator → bounded queue, all samples accounted for; stale inputs; backward time jump → re-initialisation; duplicates |
| `test_acquisition` | Go2 motor / foot-force order, IMU rotation, duplicates, repeated tick, tick wrap-around, non-increasing stamps, large time jump, decimation, NaN |
| `test_lowstate_message` | conversion from a real `unitree_go/msg/LowState` |
| `test_contact_detector` | reference threshold / hysteresis behaviour, identity and stamp preserved, validity range |
| `test_go2_model` | URDF joint/foot names, LowState index maps, IMU placement, front/rear/left/right foot positions, hip-abduction axis, Jacobians vs finite differences, frame E, ANYmal root-frame equivalence |
| `test_estimators` | each estimator on a standing Go2 with jittered sampling; reset; gravity-aligned init; measured dt (constant acceleration, irregular sampling); smoother window sizes 1–10 and window duration; per-interval dt identical for uniform sampling; **offline regression** (needs data, see below) |
| `test_no_actuation_interface` | no command message/topic in sources, launch or config |

Offline regression of the adapters (ANYmal dataset + outputs of the offline executables):

```bash
SEB_REGRESSION_DATASET=<dir with sensor_data.csv, feet_kinematics.csv> \
SEB_REGRESSION_REFERENCE=<dir with muse/ iekf/ invariant_smoother/ fused_state.csv> \
SEB_REGRESSION_MAX_SAMPLES=0 \
  build/seb_online/test_estimators --gtest_filter='OfflineRegression.*'
```

## Validation status

Performed (development PC, Ubuntu 24.04, ROS 2 Humble via RoboStack, GCC 14, Release):
* offline outputs byte-identical before/after the refactoring (all files, 208 084 samples);
* online adapters identical to the offline estimators on the full ANYmal sequence;
* 66 automated tests passing;
* replay of recorded Go2 `LowState` bags through the full threaded pipeline with each
  estimator (deterministic lossless and real-time, and `ros2 bag play` → node over
  DDS); estimated roll/pitch agree with the Go2 onboard attitude (`imu_state`) within
  2° RMS for the three estimators (roll 1.3–2.0°, pitch 0.5–1.7° RMS).

**Not performed**: any run on the robot or its onboard computer, any closed-loop
use, verification of the leg/foot order by a physical test, characterisation of the
real robot→PC timing. Runtime numbers measured on the development PC do not
transfer to the Go2 computer.

## Known limitations

* No hardware acquisition timestamp exists in `LowState`; both available time bases
  have the artefacts described above. A clock model combining tick and reception
  time is not implemented.
* The Go2 foot-force units/threshold (20, reference default) are not calibrated; the
  foot order must be validated physically.
* The world frame is the estimator's initial pose; there is no alignment with an
  external frame. Position is dead-reckoned (unobservable globally).
* No covariance is published. Angular velocity is the (bias-corrected) gyroscope.
* At 500 Hz with every sample a keyframe the smoother keeps up on the development
  PC for `window_size: 3` (mean 0.5–1.0 ms) but not for `window_size: 20`
  (≈28 ms per update, 93 % of samples dropped in real-time replay). Keyframe
  decimation / preintegration would change the estimator and is not enabled.
* Bag replay supports sqlite3 (`.db3`) bags in `inspect_lowstate_bag.py`; `seb_replay`
  supports every storage plugin installed for rosbag2.
* `LowState` from Unitree's own message package is required at build time
  (`unitree_go`).
* The code and license notices of the KAIST IEKF/invariant-smoother sources are
  unchanged ("All rights reserved"); check redistribution terms before publishing
  derived code.

## Checklist before connecting the external MPC

1. Physically verify the foot-force and motor index maps on the Go2 (lift one foot at
   a time; move one leg at a time) and the sign of every joint against the URDF.
2. Record bags on the target computer with the final network setup; run
   `inspect_lowstate_bag.py` and choose `time.stamp_source`; set `estimator.max_gap_s`
   and `sync.worker_timeout_s` from the observed jitter/stalls.
3. Calibrate the contact threshold / hysteresis for the Go2 foot-force sensors.
4. Measure execution time and latency of the chosen estimator **on the robot
   computer** (lossless and real-time replay, fixed CPU governor, chosen affinity /
   priorities) and check p99/max against the MPC period (10 ms at 100 Hz).
5. Verify estimator outputs against ground truth (motion capture) for standing,
   walking and turning; check drift and initialisation.
6. Define how the MPC consumes the state: use `header.stamp` (state time), check
   `status == VALID`, bound `state_age_at_publish_s`, and decide whether to predict
   the state to the controller time.
7. Define the behaviour of the controller on INVALID / DEGRADED states, on resets
   (`reset_*` events re-initialise position to the origin) and on missing messages.
8. Keep the estimator node monitoring-only; any actuation must stay in a separate,
   independently safeguarded controller with its own emergency stop.
