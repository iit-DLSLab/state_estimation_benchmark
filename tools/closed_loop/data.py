"""Experiment configuration, topic discovery and loading of one trial.

A trial is loaded either from a rosbag2 directory (rosbag2_py, needs the sourced
ROS 2 workspace for the custom messages) or from a folder written by
seb_online/scripts/export_bags_csv.py (no ROS needed). Both produce the same
pandas DataFrames, with the same column names and time bases:

  t_s      recorder arrival time of the message [s] since the first message of the
           bag (one common axis for all topics of a trial; used for phases only)
  *_ns     timestamps taken inside the nodes (ROS clock, or steady clock for
           *_steady_ns); never mixed with t_s
"""

import glob
import os
from dataclasses import dataclass, field

import numpy as np
import pandas as pd
import yaml

# Role of each topic, discovered from the message type; types used by several topics
# (Float64, TimeReference) are resolved with the topic name.
ROLE_BY_TYPE = {
    "seb_interfaces/msg/EstimatorTiming": "timing",
    "seb_interfaces/msg/ResourceUsage": "resources",
    "seb_interfaces/msg/EstimatorState": "state",
    "unitree_go/msg/LowState": "lowstate",
    "diagnostic_msgs/msg/DiagnosticArray": "diagnostics",
    "dls2_interface/msg/TimeDebug": "mpc_timing",
}
ROLE_BY_NAME = {"mpc_state_consumption": "consumption", "mpc_cost": "mpc_cost"}
CSV_FILES = {  # role -> file of export_bags_csv.py
    "timing": "estimator_timing.csv", "resources": "resources.csv", "state": "estimator_state.csv",
    "lowstate": "lowstate.csv", "diagnostics": "diagnostics.csv", "consumption": "mpc_state_consumption.csv",
    "mpc_cost": "mpc_cost.csv", "mpc_timing": "mpc_timing.csv",
}
LOWSTATE_COLUMNS = ["t_s", "tick", "imu_gyro_x", "imu_gyro_y", "imu_gyro_z", "m1_q"] + [f"foot_force_{i}" for i in range(4)]
SAMPLES_COLUMNS = ["sample_id", "status", "receive_steady_ns", "complete_steady_ns", "release_steady_ns",
                   "est_start_steady_ns", "est_end_steady_ns", "publish_steady_ns"]
CONFIG_LABELS = {("muse", None): "MUSE", ("iekf", None): "IEKF", ("smoother", 1): "IS ws=1",
                 ("smoother", 3): "IS ws=3", ("smoother", 6): "IS ws=6"}
CONFIG_ORDER = ["MUSE", "IEKF", "IS ws=1", "IS ws=3", "IS ws=6"]


@dataclass
class Trial:
    estimator: str
    trial: int
    ws: int | None
    source: str                      # bag or csv directory
    start_s: float | None = None     # analysis window on t_s
    end_s: float | None = None
    warmup_s: float = 0.0
    phases: list = field(default_factory=list)   # manual phases [{name, start, end}]
    topics: dict = field(default_factory=dict)   # role -> topic overrides
    run_log: str | None = None       # seb_online run directory (samples.csv, events.csv, run_config.yaml)
    notes: str = ""
    data: dict = field(default_factory=dict)     # role -> DataFrame
    discovered: dict = field(default_factory=dict)

    @property
    def config(self):
        return CONFIG_LABELS.get((self.estimator, self.ws), f"{self.estimator} ws={self.ws}")

    @property
    def name(self):
        return f"{self.config} #{self.trial}"


def load_config(path):
    """Reads the YAML experiment configuration; relative paths are relative to it."""
    with open(path) as f:
        cfg = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(path))
    resolve = lambda p: p if p is None or os.path.isabs(os.path.expanduser(p)) else os.path.normpath(os.path.join(base, p))
    defaults = cfg.get("defaults", {})
    trials = []
    for e in cfg["experiments"]:
        e = {**defaults, **e}
        estimator = {"is": "smoother", "invariant_smoother": "smoother"}.get(e["estimator"], e["estimator"])
        trials.append(Trial(
            estimator=estimator, trial=int(e["trial"]), ws=e.get("ws"),
            source=os.path.expanduser(resolve(e.get("csv") or e.get("bag"))),
            start_s=e.get("start_s"), end_s=e.get("end_s"), warmup_s=float(e.get("warmup_s", 0.0)),
            phases=e.get("phases", []), topics={**cfg.get("topics", {}), **e.get("topics", {})},
            run_log=os.path.expanduser(resolve(e["run_log"])) if e.get("run_log") else None, notes=e.get("notes", "")))
    return cfg, trials


def discover_topics(metadata_path, overrides=None):
    """role -> topic name, from the bag metadata (types and names), then overrides."""
    with open(metadata_path) as f:
        meta = yaml.safe_load(f)["rosbag2_bagfile_information"]
    roles = {}
    for t in meta["topics_with_message_count"]:
        name, typ = t["topic_metadata"]["name"], t["topic_metadata"]["type"]
        role = next((r for key, r in ROLE_BY_NAME.items() if key in name), None) or ROLE_BY_TYPE.get(typ)
        if role and role not in roles:
            roles[role] = name
    roles.update(overrides or {})
    return roles


# ---------------------------------------------------------------- rosbag2 reading
def _ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def _rows(role, m):
    if role == "timing":
        return [dict(publish_ns=_ns(m.header.stamp), estimator=m.estimator, sample_id=m.sample_id,
                     lowstate_tick=m.lowstate_tick, status={0: "INITIALIZING", 1: "VALID", 2: "DEGRADED", 3: "INVALID"}.get(m.status),
                     estimator_ms=m.estimator_ms, kinematics_ms=m.kinematics_ms, contact_ms=m.contact_ms,
                     receive_to_publish_ms=m.receive_to_publish_ms)]
    if role == "resources":
        return [dict(stamp_ns=_ns(m.header.stamp), pid=m.pid, cpu_percent_one_core=m.cpu_percent,
                     num_cpus=m.num_cpus, rss_mb=m.rss_mb, num_threads=m.num_threads)]
    if role == "consumption":
        c, s = _ns(m.header.stamp), _ns(m.time_ref)
        return [dict(consume_ns=c, state_stamp_ns=s, sequence_id=int(m.source), state_age_ms=(c - s) * 1e-6)]
    if role == "mpc_cost":
        return [dict(mpc_cost=m.data)]
    if role == "mpc_timing":
        return [dict(time_mpc_s=m.time_mpc, time_wbc_s=m.time_wbc)]
    if role == "lowstate":
        r = dict(tick=m.tick, m1_q=m.motor_state[1].q)
        r.update({f"imu_gyro_{a}": m.imu_state.gyroscope[i] for i, a in enumerate("xyz")})
        r.update({f"foot_force_{i}": m.foot_force[i] for i in range(4)})
        return [r]
    return []


def read_bag(path, roles):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id=""), ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    role_of = {topic: role for role, topic in roles.items() if topic in types}
    rows = {role: [] for role in role_of.values()}
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        role = role_of.get(topic)
        if role:
            for r in _rows(role, deserialize_message(data, types[topic])):
                rows[role].append({"bag_ns": bag_ns, **r})
    frames = {k: pd.DataFrame(v) for k, v in rows.items() if v}
    t0 = min(df.bag_ns.min() for df in frames.values())
    for df in frames.values():
        df["t_s"] = (df.bag_ns - t0) * 1e-9
    return frames


def read_csv_dir(path):
    frames = {}
    for role, fname in CSV_FILES.items():
        f = os.path.join(path, fname)
        if not os.path.exists(f):
            continue
        cols = LOWSTATE_COLUMNS if role == "lowstate" else None
        frames[role] = pd.read_csv(f, usecols=cols) if cols else pd.read_csv(f)
    return frames


def load_trial(trial):
    """Fills trial.data; missing topics are simply absent (reported as unavailable)."""
    is_bag = os.path.exists(os.path.join(trial.source, "metadata.yaml")) and not glob.glob(os.path.join(trial.source, "*.csv"))
    if is_bag:
        trial.discovered = discover_topics(os.path.join(trial.source, "metadata.yaml"), trial.topics)
        trial.data = read_bag(trial.source, trial.discovered)
    else:
        trial.data = read_csv_dir(trial.source)
        trial.discovered = {r: f"(csv) {CSV_FILES[r]}" for r in trial.data}
    if "consumption" in trial.data and "mpc_cost" in trial.data and "mpc_cost" not in trial.data["consumption"]:
        c, k = trial.data["consumption"], trial.data["mpc_cost"]
        if len(c) == len(k):  # same control step, see export_bags_csv.py
            c["mpc_cost"] = k.mpc_cost.values

    # seb_online run log: per-sample steady-clock stamps (pre-processing, queue wait,
    # monotonic publication time), events and parameters.
    log = trial.run_log or (trial.source if os.path.exists(os.path.join(trial.source, "samples.csv")) else None)
    if log and os.path.exists(os.path.join(log, "samples.csv")):
        s = pd.read_csv(os.path.join(log, "samples.csv"), usecols=SAMPLES_COLUMNS)
        if "timing" in trial.data:
            s = s[s.sample_id.isin(trial.data["timing"].sample_id)]
        trial.data["samples"] = s.reset_index(drop=True)
    if log and os.path.exists(os.path.join(log, "events.csv")):
        trial.data["events"] = pd.read_csv(os.path.join(log, "events.csv"))
    if log and os.path.exists(os.path.join(log, "run_config.yaml")):
        trial.data["run_config"] = open(os.path.join(log, "run_config.yaml")).read()

    # Analysis window on the common recorder time axis.
    for role, df in list(trial.data.items()):
        if isinstance(df, pd.DataFrame) and "t_s" in df:
            keep = np.ones(len(df), bool)
            if trial.start_s is not None:
                keep &= df.t_s >= trial.start_s
            if trial.end_s is not None:
                keep &= df.t_s <= trial.end_s
            trial.data[role] = df[keep].reset_index(drop=True)
    return trial


def run_parameter(trial, key):
    """Value of a seb_online parameter from run_config.yaml, or None."""
    for line in trial.data.get("run_config", "").splitlines():
        if line.strip().startswith(key + ":"):
            return line.split(":", 1)[1].strip()
    return None
