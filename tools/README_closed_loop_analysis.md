# Closed-loop experiment analysis (Go2 + Quadruped-PyMPC + seb_online)

`tools/analyze_closed_loop.py` turns the real-robot recordings into per-trial metrics,
per-configuration tables, figures and a Markdown report
(`results/analysis/analysis_report.md`).

## Setup

Python 3 with numpy, pandas, scipy, matplotlib, PyYAML (all in the `state_est_bench`
conda env). Two input formats per trial:

* `csv:` a folder written by `seb_online/scripts/export_bags_csv.py` (also contains
  the seb_online run log: `samples.csv`, `events.csv`, `run_config.yaml`). No ROS needed.
* `bag:` a rosbag2 directory, read with `rosbag2_py`. Needs the custom message types:
  ```bash
  conda activate state_est_bench
  source ros2_ws/install/local_setup.bash
  ```
  Add `run_log: ~/.ros/seb_online/<run>` to use the per-sample log (pre-processing,
  queue wait, monotonic publication stamps); without it the publication intervals use
  the ROS-clock stamps of `/benchmark/estimator_timing`.

## Run

```bash
# once, if only bags exist
ros2 run seb_online export_bags_csv.py ros2_ws/rosbags ros2_ws/rosbags_csv
# analysis (≈20 s for 15 trials)
python3 tools/analyze_closed_loop.py --config tools/experiment_config.yaml [--output results/analysis]
# code tests on synthetic fixtures
python3 -m pytest -q tools/test_closed_loop.py
```

## Configuration (`tools/experiment_config.yaml`)

One entry per trial: `estimator` (muse | iekf | smoother), `ws` (smoother), `trial`,
`csv` or `bag` (relative to the YAML), optional `start_s`/`end_s` (analysis window on
the bag time axis), `warmup_s`, `phases` (manual segments `{name, start, end}`
replacing the automatic ones), `topics` (role overrides), `run_log`, `notes`.
Global: `output`, `analysis` (thresholds, band, window lengths), `cpu` (documentation of
the resource sampling).

## Topic discovery

Roles are found from the bag metadata by message type, then by name when a type is
shared, and can be overridden per trial (`topics:`):

| role | type | recorded topic |
|---|---|---|
| timing | seb_interfaces/msg/EstimatorTiming | /benchmark/estimator_timing |
| resources | seb_interfaces/msg/ResourceUsage | /benchmark/resources |
| consumption | sensor_msgs/msg/TimeReference (name contains `mpc_state_consumption`) | /benchmark/mpc_state_consumption |
| mpc_cost | std_msgs/msg/Float64 (name contains `mpc_cost`) | /mpc_cost |
| state | seb_interfaces/msg/EstimatorState | /seb_online_estimator/state |
| lowstate | unitree_go/msg/LowState | /lowstate |
| diagnostics | diagnostic_msgs/msg/DiagnosticArray | /diagnostics |
| mpc_timing | dls2_interface/msg/TimeDebug | (not recorded) |

`tables/topic_mapping.csv` lists what was used for every trial.

## Task segments (no command topic was recorded)

From raw LowState only (estimator-independent): contact = `foot_force > 15` (the
estimators' threshold), switch rate = contact changes per leg per second (1 s window).

* **standup**: 1 s to 5 s after all four feet are first loaded for ≥ 1 s (goUp and the
  standing that follows; the same task event in every trial), cut at the first
  stepping (switch rate > 0.5 /leg/s) or support loss.
* **trot_straight / trot_turn**: trot bouts (switch rate > 3 /leg/s), 2 s trimmed at both
  ends, split by |yaw rate| (2 s average of the raw gyro) < 0.2 / > 0.3 rad/s.
* **lying, stand_final**: lying / four-foot bouts after the last trot.
* **support loss**: after standing, mean loaded fraction < 0.1 for 1 s with the
  controller active in the 2 s before; data after it are excluded everywhere. *Early
  termination* = support loss and the controller stopping before the end of the bag.

## Quantities

| metric (column prefix) | definition | clock |
|---|---|---|
| `est_ms` | estimator update: `EstimatorAdapter::step()` (MUSE / IEKF / smoother update) | steady |
| `kin_ms`, `contact_ms` | kinematics / contact worker execution, in parallel (never summed) | steady |
| `pre_ms` | LowState callback → sample released by the synchronizer with both results (sync-ready) | steady |
| `queue_ms` | release → start of the estimator update | steady |
| `rx_to_pub_ms` | LowState callback entry → start of the state publication (software pipeline latency, not sensor-to-publication) | steady |
| `rate_mean_hz`, `interval_ms_*` | from consecutive publication stamps (`publish_steady_ns` of the run log) inside the scope | steady |
| `frac_burst_intervals` | intervals < 0.5 ms (back-to-back publications, queue draining) | steady |
| `n_long_gaps` | intervals > 10 ms | steady |
| `missing_samples`, `frac_missing` | LowState samples with no estimate (sample_id gaps) | — |
| `state_age_ms` | controller read time − `BaseState.timestamp` of the estimate read (receive-time-based) | ROS, same host |
| `control_loop_hz_mean`, `frac_state_reused` | rate of the controller loop that reads the state; share reading the same estimate as the previous loop | ROS |
| `mpc_cost` | total objective of the last MPC solution (estimated state; diagnostic only) | — |
| `cpu_pct` | psutil `Process.cpu_percent()` of `online_estimator_node`, 0.5 s windows, % of ONE core (can exceed 100; max 100 × cores); whole pipeline incl. logger | process CPU time |
| `rss_mb`, `threads` | resident memory, threads of the same process | — |
| `rms_w*`, `std_w*`, `p95abs_w*` | raw gyro (x roll, y pitch, z yaw rate), resampled to 500 Hz on recorder arrival time, pieces split at gaps > 20 ms | recorder |
| `bp_rms_w*` | same after a zero-phase 4th-order Butterworth band-pass 0.5–30 Hz (common band, chosen from the spectra, Fig. S1) | recorder |
| `peak_hz_w*`, `psd_rms_*` | Welch PSD (1024 samples, length-weighted over pieces): peak in the band; RMS in fixed sub-bands 0.5–5, 5–15, 15–30 Hz (supplementary) | recorder |

Aggregation: every metric is computed per trial and scope; configurations are compared
on the trial values (median [min–max], plus mean ± sd in `summary_by_config.csv`;
`trials` lists the individual values). Sample-level boxes in the figures pool samples
for display only. Spearman ρ across trials is reported descriptively, no tests.

Unavailable in these recordings (reported, never filled with 0): MPC solver time and
status, objective components, deadline misses (no deadline defined), commands /
phase markers.

## Smoother window size

`smoother.window_size` = ws IMU propagation intervals; the window holds ws + 1
consecutive state frames, one per LowState sample (`InvariantSmoother::AllocateWindow`,
logged `window_frames` = ws + 1), i.e. ≈ ws × 2 ms at 500 Hz.
