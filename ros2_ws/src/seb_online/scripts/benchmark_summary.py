#!/usr/bin/env python3
"""Summary of a rosbag recorded with the benchmark topics (see docs/ROS2_ONLINE.md).

  ros2 run seb_online benchmark_summary.py <bag_dir> [--csv out_prefix]

Reads /benchmark/estimator_timing, /benchmark/resources, /diagnostics and, if
present, /benchmark/mpc_state_consumption. Prints median / p95 / p99 / max and
optionally writes one CSV per topic. Topics absent from the bag are skipped.
"""

import argparse

import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def read(bag):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=""), ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    timing, resources, consumption, rate = [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            continue
        m = deserialize_message(data, types[topic])
        if topic == "/benchmark/estimator_timing":
            timing.append(dict(publish_ns=ns(m.header.stamp), estimator=m.estimator, sample_id=m.sample_id,
                               lowstate_tick=m.lowstate_tick, status=m.status, estimator_ms=m.estimator_ms,
                               kinematics_ms=m.kinematics_ms, contact_ms=m.contact_ms,
                               receive_to_publish_ms=m.receive_to_publish_ms))
        elif topic == "/benchmark/resources":
            resources.append(dict(stamp_ns=ns(m.header.stamp), pid=m.pid, cpu_percent=m.cpu_percent,
                                  num_cpus=m.num_cpus, rss_mb=m.rss_mb, num_threads=m.num_threads))
        elif topic == "/benchmark/mpc_state_consumption":
            consumption.append(dict(consume_ns=ns(m.header.stamp), state_ns=ns(m.time_ref),
                                    sequence_id=int(m.source)))
        elif topic == "/diagnostics":
            for st in m.status:
                values = {kv.key: kv.value for kv in st.values}
                if "updates_per_s" in values:
                    rate.append(float(values["updates_per_s"]))
    return pd.DataFrame(timing), pd.DataFrame(resources), pd.DataFrame(consumption), rate


def line(name, x, unit):
    x = pd.Series(x).dropna()
    if x.empty:
        print(f"  {name:34s} not measured")
        return
    print(f"  {name:34s} median {x.median():9.4f}  p95 {x.quantile(.95):9.4f}  p99 {x.quantile(.99):9.4f}"
          f"  max {x.max():9.4f} {unit}  (n={len(x)})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--csv", help="write <prefix>_timing.csv, _resources.csv, _consumption.csv")
    args = ap.parse_args()
    timing, resources, consumption, rate = read(args.bag)

    if not timing.empty:
        # INITIALIZING samples (first sample after every (re)initialisation) include
        # allocations, e.g. ~35 ms for a new smoother: reported apart, not in the statistics.
        init = timing.status == 0
        print(f"Estimator timing: {', '.join(timing.estimator.unique())} "
              f"({init.sum()} INITIALIZING samples excluded, max estimator_ms among them "
              f"{timing.estimator_ms[init].max() if init.any() else float('nan'):.3f})")
        for c in ["estimator_ms", "kinematics_ms", "contact_ms", "receive_to_publish_ms"]:
            line(c, timing.loc[~init, c], "ms")
        dt = np.diff(timing.publish_ns.values) * 1e-9
        print(f"  output rate (publication stamps)   mean {1.0 / dt.mean():.1f} Hz, "
              f"median interval {np.median(dt) * 1e3:.3f} ms, max interval {dt.max() * 1e3:.3f} ms")
    if rate:
        line("output rate (/diagnostics)", rate, "Hz")
    if not resources.empty:
        print(f"Resources of pid {resources.pid.unique().tolist()} "
              f"(cpu_percent: % of one core, up to {100 * resources.num_cpus.iloc[0]})")
        line("cpu_percent", resources.cpu_percent, "%")
        line("rss_mb", resources.rss_mb, "MiB")
        line("num_threads", resources.num_threads, "")
    if not consumption.empty:
        age = (consumption.consume_ns - consumption.state_ns) * 1e-6
        print("MPC consumption")
        line("state age at consumption", age, "ms")
        if not timing.empty:
            joined = consumption.merge(timing.assign(sequence_id=timing.sample_id % 2**32)[["sequence_id", "publish_ns"]],
                                       on="sequence_id", how="left")
            line("publication -> consumption", (joined.consume_ns - joined.publish_ns) * 1e-6, "ms")
        print(f"  same state used by consecutive control steps: {(consumption.sequence_id.diff() == 0).mean() * 100:.1f}%")

    if args.csv:
        for name, df in [("timing", timing), ("resources", resources), ("consumption", consumption)]:
            if not df.empty:
                df.to_csv(f"{args.csv}_{name}.csv", index=False)


if __name__ == "__main__":
    main()
