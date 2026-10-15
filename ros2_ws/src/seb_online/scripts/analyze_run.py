#!/usr/bin/env python3
"""Runtime / synchronisation statistics of seb_online runs.

Usage:
  analyze_run.py RUN_DIR [RUN_DIR ...] [--json out.json] [--skip-warmup N]

RUN_DIR is a directory written by online_estimator_node or seb_replay
(samples.csv, events.csv, run_config.yaml). With several runs a comparison table
is printed (e.g. one run per smoother window size).

Definitions (all durations from the monotonic clock of the processing machine):
  estimator_exec      estimator update only (est_end - est_start)
  kinematics_exec     Pinocchio worker computation of the sample
  contact_exec        contact worker computation of the sample
  sync_wait           first worker result ready -> bundle released in order
  queue_wait          bundle released -> dequeued by the estimator thread
  latency_to_estimate LowState reception -> end of the estimator update
  e2e_latency         LowState reception -> publication of the state
  state_age           publication time - state timestamp (ROS clock); with the
                      receive_time time base it equals e2e_latency (both exclude
                      the unknown robot -> PC transport delay)
Estimator execution time is NOT latency: latency also contains queueing,
synchronisation and the other stages.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

LOSS_EVENTS = (
    "invalid_message", "out_of_order", "pipeline_full", "missing_kinematics", "missing_contact",
    "missing_kinematics_and_contact", "estimator_backlog_drop", "stale_input", "dt_too_small",
)


def read_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return reader.fieldnames or [], rows


def col(rows, name, dtype=float):
    out = np.empty(len(rows), dtype=dtype)
    for i, r in enumerate(rows):
        v = r[name]
        out[i] = dtype(v) if v != "" else (np.nan if dtype is float else 0)
    return out


def stats(x, scale=1.0):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)] * scale
    if x.size == 0:
        return {"n": 0}
    return {
        "n": int(x.size),
        "mean": float(np.mean(x)),
        "median": float(np.median(x)),
        "std": float(np.std(x)),
        "p95": float(np.percentile(x, 95)),
        "p99": float(np.percentile(x, 99)),
        "max": float(np.max(x)),
        "min": float(np.min(x)),
    }


def read_config(run_dir):
    cfg = {}
    path = os.path.join(run_dir, "run_config.yaml")
    if not os.path.exists(path):
        return cfg
    with open(path) as f:
        for line in f:
            line = line.strip()
            if ":" in line and not line.startswith("#"):
                k, v = line.split(":", 1)
                cfg[k.strip()] = v.strip()
    return cfg


def analyze(run_dir, skip_warmup=0):
    _, rows = read_csv(os.path.join(run_dir, "samples.csv"))
    _, events = read_csv(os.path.join(run_dir, "events.csv"))
    cfg = read_config(run_dir)
    if skip_warmup:
        rows = rows[skip_warmup:]
    if not rows:
        raise RuntimeError(f"{run_dir}: no processed samples")

    us = 1e6
    res = {"run_dir": run_dir, "estimator": rows[0]["estimator"], "mode": cfg.get("mode", "?")}
    res["config"] = {k: cfg[k] for k in cfg if k.startswith(("estimator.", "smoother.", "time.", "queues.", "sync.", "acquisition."))}
    for name in ("estimator_exec_s", "kinematics_exec_s", "contact_exec_s", "sync_wait_s", "estimator_queue_wait_s",
                 "latency_to_estimate_s", "e2e_latency_s", "state_age_at_publish_s"):
        res[name[:-2] + "_us"] = stats(col(rows, name), us)

    pub = col(rows, "publish_steady_ns")
    pub = pub[pub > 0]
    if pub.size > 1:
        duration = (pub[-1] - pub[0]) * 1e-9
        res["output_rate_hz"] = (pub.size - 1) / duration if duration > 0 else float("nan")
        res["output_interval_us"] = stats(np.diff(pub), 1e-3)
    recv = col(rows, "receive_steady_ns")
    res["dt_ms"] = stats(col(rows, "dt_s"), 1e3)
    res["estimator_queue_depth"] = stats(col(rows, "estimator_queue_depth"))
    res["sync_pending_at_dispatch"] = stats(col(rows, "sync_pending_at_dispatch"))
    res["worker_queue_at_dispatch"] = stats(np.maximum(col(rows, "kin_queue_at_dispatch"), col(rows, "contact_queue_at_dispatch")))

    status = {}
    for r in rows:
        status[r["status"]] = status.get(r["status"], 0) + 1
    res["status_counts"] = status

    ev_counts = {}
    for e in events:
        ev_counts[e["event"]] = ev_counts.get(e["event"], 0) + 1
    res["event_counts"] = ev_counts
    lost = sum(ev_counts.get(k, 0) for k in LOSS_EVENTS)
    processed = len(rows) + skip_warmup
    res["processed"] = processed
    res["lost"] = lost
    res["drop_rate"] = lost / (processed + lost) if processed + lost > 0 else 0.0
    res["decimated"] = ev_counts.get("decimated", 0)
    res["duplicates"] = ev_counts.get("duplicate", 0)
    res["lost_sample_ids"] = [int(e["sample_id"]) for e in events
                              if e["event"] in LOSS_EVENTS and e["sample_id"] != ""][:1000]

    if res["estimator"] == "invariant_smoother":
        res["smoother_window_size"] = cfg.get("smoother.window_size", "?")
        res["window_duration_ms"] = stats(col(rows, "window_duration_s"), 1e3)
        res["iterations"] = stats(col(rows, "iterations"))
    if recv.size > 1:
        res["input_duration_s"] = float((recv[-1] - recv[0]) * 1e-9)
    return res


def fmt_stats(s, unit):
    if not s or s.get("n", 0) == 0:
        return "n/a"
    return (f"mean {s['mean']:.1f} | median {s['median']:.1f} | p95 {s['p95']:.1f} | "
            f"p99 {s['p99']:.1f} | max {s['max']:.1f} {unit}")


def print_report(r):
    print(f"\n=== {r['run_dir']}")
    print(f"estimator: {r['estimator']}   mode: {r['mode']}   processed: {r['processed']}   "
          f"lost: {r['lost']} (drop rate {100 * r['drop_rate']:.3f} %)   duplicates dropped: {r['duplicates']}   "
          f"decimated: {r['decimated']}")
    if "smoother_window_size" in r:
        print(f"smoother window size: {r['smoother_window_size']}   window duration: {fmt_stats(r['window_duration_ms'], 'ms')}")
        print(f"smoother iterations: {fmt_stats(r['iterations'], '')}")
    print(f"estimator execution     : {fmt_stats(r['estimator_exec_us'], 'us')}")
    print(f"kinematics execution    : {fmt_stats(r['kinematics_exec_us'], 'us')}")
    print(f"contact execution       : {fmt_stats(r['contact_exec_us'], 'us')}")
    print(f"synchronisation wait    : {fmt_stats(r['sync_wait_us'], 'us')}")
    print(f"estimator queue wait    : {fmt_stats(r['estimator_queue_wait_us'], 'us')}")
    print(f"latency to estimate     : {fmt_stats(r['latency_to_estimate_us'], 'us')}")
    print(f"end-to-end latency      : {fmt_stats(r['e2e_latency_us'], 'us')}")
    print(f"state age at publication: {fmt_stats(r['state_age_at_publish_us'], 'us')}")
    if "output_rate_hz" in r:
        print(f"output rate             : {r['output_rate_hz']:.1f} Hz   interval {fmt_stats(r['output_interval_us'], 'us')}")
    print(f"dt                      : {fmt_stats(r['dt_ms'], 'ms')}")
    q = r["estimator_queue_depth"]
    print(f"estimator queue backlog : mean {q['mean']:.2f} | p99 {q['p99']:.0f} | max {q['max']:.0f}")
    print(f"status: {r['status_counts']}")
    print(f"events: {r['event_counts']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--json", help="write all results to this JSON file")
    ap.add_argument("--skip-warmup", type=int, default=0, help="ignore the first N processed samples")
    args = ap.parse_args()

    results = []
    for run in args.runs:
        try:
            r = analyze(run, args.skip_warmup)
        except Exception as exc:  # noqa: BLE001
            print(f"{run}: {exc}", file=sys.stderr)
            continue
        results.append(r)
        print_report(r)

    if len(results) > 1:
        print("\n=== comparison (execution time / latency in us)")
        hdr = f"{'run':40s} {'est':18s} {'win':>4s} {'exec mean':>10s} {'p99':>9s} {'max':>9s} {'e2e p99':>9s} {'rate Hz':>8s} {'drop %':>7s}"
        print(hdr)
        for r in results:
            e = r["estimator_exec_us"]
            l = r["e2e_latency_us"]
            print(f"{os.path.basename(os.path.normpath(r['run_dir']))[:40]:40s} {r['estimator']:18s} "
                  f"{str(r.get('smoother_window_size', '-')):>4s} {e['mean']:10.1f} {e['p99']:9.1f} {e['max']:9.1f} "
                  f"{l.get('p99', math.nan):9.1f} {r.get('output_rate_hz', math.nan):8.1f} {100 * r['drop_rate']:7.3f}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
