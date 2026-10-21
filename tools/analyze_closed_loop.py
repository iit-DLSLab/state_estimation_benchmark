#!/usr/bin/env python3
"""Analysis of the real-robot closed-loop experiments (seb_online + Quadruped-PyMPC on Go2).

  python3 tools/analyze_closed_loop.py --config tools/experiment_config.yaml --output results/analysis

Inputs: per trial a rosbag2 directory (needs the sourced ros2_ws for the custom
messages) or a folder exported by seb_online/scripts/export_bags_csv.py (no ROS
needed), see tools/README_closed_loop_analysis.md.

Outputs (in --output):
  tables/per_trial_metrics.csv     one row per trial and scope, every metric
  tables/summary_by_config.csv     per configuration and scope: n, median, min, max, mean, std, trial values
  tables/segments.csv              detected task segments of every trial
  tables/topic_mapping.csv         topics found / used per trial
  tables/*.csv|.tex                manuscript tables (median [min–max] over trials)
  figures/fig*.pdf|.png            figures 1-9 and S1
  analysis_report.md               report and manuscript-ready text
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
from scipy.stats import spearmanr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from closed_loop import data, metrics, phases, plots, report  # noqa: E402

DEFAULTS = dict(
    contact_threshold=15.0,     # LowState.foot_force units, as the estimators' contact detection
    margin_s=2.0,               # trimmed at both ends of every bout
    min_segment_s=1.0,          # shortest analysed bout after trimming
    yaw_straight=0.2,           # |yaw rate| below: straight [rad/s], 2 s average
    yaw_turn=0.3,               # |yaw rate| above: turning [rad/s]
    support_loss_s=1.0,
    band_hz=[0.5, 30.0],
    sub_bands_hz=[[0.5, 5.0], [5.0, 15.0], [15.0, 30.0]],
    welch_nperseg=1024,
    max_imu_gap_ms=20.0,
    min_piece_s=2.0,
    burst_ms=0.5,               # publication interval below: back-to-back (queue drain)
    long_gap_ms=10.0,
    timeseries_window_s=4.0,
    standup_window_s=4.0,       # stand-up segment length (from 1 s after four-foot contact)
)
SCOPES = ["all", "lying", "standup", "trot_straight", "trot_turn", "stand_final"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--output", help="output directory (default: 'output' of the config, else results/analysis)")
    args = ap.parse_args()

    cfg, trials = data.load_config(args.config)
    p = {**DEFAULTS, **cfg.get("analysis", {})}
    # --output is relative to the working directory, 'output' of the config to the config file.
    out = args.output or (os.path.join(os.path.dirname(os.path.abspath(args.config)), cfg["output"])
                          if cfg.get("output") else "results/analysis")
    out = os.path.normpath(out)
    tables_dir, figs_dir = os.path.join(out, "tables"), os.path.join(out, "figures")
    os.makedirs(tables_dir, exist_ok=True)

    rows, segs, topics = [], [], []
    for t in trials:
        print(f"[{t.name}] {t.source}", flush=True)
        if not os.path.exists(t.source):
            print("  missing input: skipped")
            continue
        data.load_trial(t)
        if "lowstate" not in t.data:
            print("  no LowState: phases and oscillation metrics unavailable")
        grid, events = phases.segment_trial(t, p) if "lowstate" in t.data else (None, dict(early_termination=False, support_loss_s=None, controller_stop_s=None))
        t.data["grid"] = grid
        ws_log = data.run_parameter(t, "smoother.window_size")
        if t.estimator == "smoother" and ws_log is not None and int(ws_log) != t.ws:
            print(f"  WARNING: config ws={t.ws} but run log smoother.window_size={ws_log}")
        rows += metrics.trial_metrics(t, grid, events, SCOPES if grid is not None else ["all"], p)
        if grid is not None:
            st = phases.segment_table(grid).assign(config=t.config, trial=t.trial)
            segs.append(st)
        topics += [dict(config=t.config, trial=t.trial, role=r, topic=n) for r, n in t.discovered.items()]
        t.data["phase_events"] = events
        if "state" in t.data:  # keep what Fig. 10 needs (orientation, velocity)
            t.data["state"] = t.data["state"][[c for c in ["t_s", "status", "q_wb_w", "q_wb_x", "q_wb_y", "q_wb_z",
                                                          "v_world_x", "v_world_y", "v_world_z"] if c in t.data["state"]]]

    per_trial = pd.DataFrame(rows)
    front = ["config", "estimator", "ws", "trial", "scope"]
    per_trial = per_trial[front + [c for c in per_trial.columns if c not in front]]
    summary = metrics.summarize(per_trial)
    segments = pd.concat(segs) if segs else pd.DataFrame(columns=["segment", "start_s", "end_s", "duration_s", "config", "trial"])
    per_trial.to_csv(os.path.join(tables_dir, "per_trial_metrics.csv"), index=False)
    summary.to_csv(os.path.join(tables_dir, "summary_by_config.csv"), index=False)
    segments.to_csv(os.path.join(tables_dir, "segments.csv"), index=False)
    pd.DataFrame(topics).to_csv(os.path.join(tables_dir, "topic_mapping.csv"), index=False)

    corr = {}  # (scope, timing metric) -> Spearman rho over trials (descriptive)
    for s in ["standup", "trot_straight"]:
        for m in [g[0] for g in plots.GAP_METRICS]:
            d = per_trial[per_trial.scope == s].dropna(subset=[m, "bp_rms_wxy"]) if m in per_trial else []
            if len(d) >= 5:
                corr[(s, m)] = float(spearmanr(d[m], d.bp_rms_wxy).statistic)

    configs = [c for c in data.CONFIG_ORDER if c in set(per_trial.config)]
    tables = report.write_tables(per_trial, summary, tables_dir, configs)
    loaded = [t for t in trials if t.data]
    plots.fig_runtime(loaded, per_trial, figs_dir)
    plots.fig_output_rate(per_trial, figs_dir)
    plots.fig_latency(loaded, per_trial, figs_dir)
    plots.fig_resources(per_trial, figs_dir)
    sel = plots.fig_timeseries(loaded, per_trial, p, figs_dir) if any("lowstate" in t.data for t in loaded) else []
    plots.fig_oscillation(per_trial, figs_dir)
    plots.fig_gap_vs_oscillation(per_trial, corr, figs_dir)
    plots.fig_window_sweep(per_trial, figs_dir)
    mpc_fig = plots.fig_mpc(loaded, per_trial, figs_dir)
    plots.fig_psd(loaded, p, figs_dir)
    plots.fig_base_state(loaded, p, figs_dir)
    plots.fig_base_state(loaded, p, figs_dir, t_before=1.0, t_after=10.0, only=["IS ws=1", "IS ws=3", "IS ws=6"],
                         name="fig10b_base_state_smoother", width=plots.COL2 * 0.75)
    report.write_report(os.path.join(out, "analysis_report.md"), loaded, per_trial, summary, segments, tables, corr, p,
                        sel, figs_dir, mpc_fig)
    print(f"wrote {out}/analysis_report.md, {len(per_trial)} trial-scope rows, figures in {figs_dir}")


if __name__ == "__main__":
    main()
