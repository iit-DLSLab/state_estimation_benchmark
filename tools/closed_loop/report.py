"""Tables (CSV, Markdown, LaTeX) and the Markdown report.

Aggregation across trials: median [min-max] of the per-trial values (n = number
of trials that have the metric). Every sentence with a number is generated from
the tables; statements that depend on a condition are only emitted when the data
satisfy it.
"""

import os

import numpy as np
import pandas as pd

from .data import CONFIG_ORDER

SCOPES_MAIN = ["all", "standup", "trot_straight"]


class Summary:
    def __init__(self, summary):
        self.s = summary.set_index(["config", "scope", "metric"])

    def get(self, config, scope, metric):
        try:
            return self.s.loc[(config, scope, metric)]
        except KeyError:
            return None

    def med(self, config, scope, metric):
        r = self.get(config, scope, metric)
        return None if r is None else float(r["median"])

    def fmt(self, config, scope, metric, d=2, scale=1.0):
        r = self.get(config, scope, metric)
        if r is None:
            return "n/a"
        f = lambda v: f"{v * scale:.{d}f}"
        n = int(r["n_trials"])
        return f"{f(r['median'])} [{f(r['min'])}–{f(r['max'])}]" + ("" if n == 3 else f" (n={n})")


def table(summary, configs, scope, columns):
    """columns: [(header, metric, digits)] -> DataFrame of 'median [min–max]' strings."""
    S = Summary(summary)
    rows = []
    for c in configs:
        rows.append({"configuration": c, **{h: S.fmt(c, scope, m, d) for h, m, d in columns}})
    return pd.DataFrame(rows)


def to_markdown(df):
    cols = list(df.columns)
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    lines += ["| " + " | ".join(str(v) for v in r) + " |" for r in df.itertuples(index=False)]
    return "\n".join(lines)


def to_latex(df, caption):
    esc = lambda s: str(s).replace("%", r"\%").replace("_", r"\_").replace("–", "--").replace("→", r"$\rightarrow$")
    cols = list(df.columns)
    out = [r"\begin{table*}[t]", r"\centering", rf"\caption{{{esc(caption)}}}", r"\footnotesize",
           r"\begin{tabular}{l" + "c" * (len(cols) - 1) + "}", r"\toprule", " & ".join(esc(c) for c in cols) + r" \\", r"\midrule"]
    out += [" & ".join(esc(v) for v in r) + r" \\" for r in df.itertuples(index=False)]
    out += [r"\bottomrule", r"\end{tabular}", r"\end{table*}"]
    return "\n".join(out)


TIMING_COLS = [("update time median [ms]", "est_ms_median", 3), ("update time p99 [ms]", "est_ms_p99", 3),
               ("update time max [ms]", "est_ms_max", 2), ("rx→pub median [ms]", "rx_to_pub_ms_median", 3),
               ("rx→pub p99 [ms]", "rx_to_pub_ms_p99", 2), ("mean rate [Hz]", "rate_mean_hz", 1),
               ("interval p99 [ms]", "interval_ms_p99", 2), ("max gap [ms]", "interval_ms_max", 1),
               ("missing samples [%]", "frac_missing", 2)]
RESOURCE_COLS = [("CPU mean [%core]", "cpu_pct_mean", 1), ("CPU median [%core]", "cpu_pct_median", 1),
                 ("CPU p95 [%core]", "cpu_pct_p95", 1), ("CPU peak [%core]", "cpu_pct_max", 1),
                 ("RSS mean [MiB]", "rss_mb_mean", 0), ("RSS peak [MiB]", "rss_mb_max", 0), ("threads", "threads_median", 0)]
PRE_COLS = [("kinematics median [ms]", "kin_ms_median", 4), ("contact median [ms]", "contact_ms_median", 4),
            ("pre-processing (sync-ready) median [ms]", "pre_ms_median", 3), ("pre-processing p99 [ms]", "pre_ms_p99", 3),
            ("queue wait median [ms]", "queue_ms_median", 3), ("queue wait p99 [ms]", "queue_ms_p99", 2)]
AGE_COLS = [("state age median [ms]", "state_age_ms_median", 2), ("state age p95 [ms]", "state_age_ms_p95", 2),
            ("state age p99 [ms]", "state_age_ms_p99", 2), ("control loop [Hz]", "control_loop_hz_mean", 0),
            ("state reused [%]", "frac_state_reused", 1), ("MPC objective median", "mpc_cost_median", 2),
            ("MPC objective p95", "mpc_cost_p95", 2)]
OSC_COLS = [("roll-rate RMS [rad/s]", "bp_rms_wx", 3), ("pitch-rate RMS [rad/s]", "bp_rms_wy", 3),
            ("roll/pitch RMS [rad/s]", "bp_rms_wxy", 3), ("raw roll/pitch RMS [rad/s]", "rms_wxy", 3),
            ("p95 |ω_x| [rad/s]", "p95abs_wx", 3), ("p95 |ω_y| [rad/s]", "p95abs_wy", 3),
            ("yaw-rate RMS [rad/s]", "rms_wz", 3), ("peak f roll [Hz]", "peak_hz_wx", 1), ("peak f pitch [Hz]", "peak_hz_wy", 1),
            ("analysed duration [s]", "imu_duration_s", 1)]


def pct(summary):
    """frac_* metrics shown in %."""
    s = summary.copy()
    for c in ["median", "min", "max", "mean", "std"]:
        s.loc[s.metric.str.startswith("frac_"), c] *= 100
    return s


def write_tables(per_trial, summary, out_dir, configs):
    os.makedirs(out_dir, exist_ok=True)
    s = pct(summary)
    specs = [("timing_all", "all", TIMING_COLS, "Estimator timing over the whole recording"),
             ("timing_stand", "standup", TIMING_COLS, "Estimator timing during stand-up (1–5 s after four-foot contact)"),
             ("timing_trot", "trot_straight", TIMING_COLS, "Estimator timing during straight trotting"),
             ("preprocessing_all", "all", PRE_COLS, "Pre-processing and queueing (whole recording)"),
             ("resources_all", "all", RESOURCE_COLS, "CPU (% of one core) and memory of the estimator process"),
             ("mpc_all", "all", AGE_COLS, "State age at MPC consumption and controller diagnostics (whole recording)"),
             ("mpc_stand", "standup", AGE_COLS, "State age and controller diagnostics, stand-up (1–5 s after four-foot contact)"),
             ("mpc_trot", "trot_straight", AGE_COLS, "State age and controller diagnostics, straight trotting"),
             ("oscillation_stand", "standup", OSC_COLS, "IMU body angular rates, stand-up (1–5 s after four-foot contact)"),
             ("oscillation_trot", "trot_straight", OSC_COLS, "IMU body angular rates, straight trotting"),
             ("oscillation_turn", "trot_turn", OSC_COLS, "IMU body angular rates, trotting while turning")]
    tables = {}
    for name, scope, cols, caption in specs:
        if scope not in set(per_trial.scope):
            continue
        df = table(s, configs, scope, cols)
        tables[name] = (df, caption)
        df.to_csv(os.path.join(out_dir, f"{name}.csv"), index=False)
        with open(os.path.join(out_dir, f"{name}.tex"), "w") as f:
            f.write(to_latex(df, caption + ". Median [min–max] over trials."))
    return tables


def unavailable(trials, per_trial):
    roles = {r for t in trials for r in t.data}
    notes = []
    if "mpc_timing" not in roles:
        notes.append("MPC solver execution time and its p99: not recorded (no dls2_interface/TimeDebug topic, e.g. /time_debug, in the bags).")
    notes.append("MPC solver success/failure: no solver-status topic is published by the controller; not available.")
    notes.append("Deadline misses: no scheduled deadline is defined for the estimator or the WBC loop; not computed.")
    notes.append("MPC objective components (velocity tracking, orientation, effort): only the total objective (/mpc_cost) is published.")
    notes.append("Commanded velocity / experiment phase markers: not recorded (no /commanded_base_velocity or /joy); phases detected from LowState foot forces and yaw rate.")
    if "samples" not in roles:
        notes.append("Pre-processing latency and queue wait: need the seb_online run log (samples.csv); not available.")
    if "consumption" not in roles:
        notes.append("State age at MPC consumption: no /benchmark/mpc_state_consumption topic.")
    return notes


def write_report(path, trials, per_trial, summary, segments, tables, corr, params, sel_ts, figs_dir, mpc_fig):
    S = Summary(summary)
    configs = [c for c in CONFIG_ORDER if c in set(per_trial.config)]
    allp = per_trial[per_trial.scope == "all"].set_index(["config", "trial"])
    L = []
    w = L.append
    w("# Real-robot closed-loop experiments: analysis report\n")
    w(f"Generated by `tools/analyze_closed_loop.py`. Values: median [min–max] over the trials of a configuration "
      f"(one value per trial; n = 3 unless stated). Figures: `{os.path.relpath(figs_dir, os.path.dirname(path))}/`.\n")

    # ---------------------------------------------------------------- summary
    w("## Experimental summary\n")
    rows = []
    for t in sorted(trials, key=lambda t: (CONFIG_ORDER.index(t.config), t.trial)):
        a = allp.loc[(t.config, t.trial)] if (t.config, t.trial) in allp.index else {}
        seg = segments[(segments.config == t.config) & (segments.trial == t.trial)]
        dur = lambda s: f"{seg[seg.segment == s].duration_s.sum():.1f}" if (seg.segment == s).any() else "0"
        rows.append(dict(configuration=t.config, trial=t.trial, source=os.path.basename(t.source.rstrip("/")),
                         estimates=int(a.get("est_ms_n", 0)),
                         **{"standing [s]": dur("standup"), "straight trot [s]": dur("trot_straight"),
                            "turning trot [s]": dur("trot_turn")},
                         termination=("early (support lost at %.1f s)" % a["support_loss_s"]) if a.get("early_termination") else "normal",
                         notes=t.notes))
    w(to_markdown(pd.DataFrame(rows)) + "\n")
    w(f"`standing` = stand-up segment: from 1 s to {1 + params['standup_window_s']:g} s after all four feet are first "
      "loaded (the same task event in every trial: goUp and the standing that follows), cut at the first stepping or "
      f"support loss. `straight/turning trot`: trot bouts (2 s trimmed at both ends) with |yaw rate| < "
      f"{params['yaw_straight']} / > {params['yaw_turn']} rad/s (2 s average of the raw gyro). Termination is *early* "
      "when, after standing, all four foot-force readings stayed below the contact threshold for "
      f"{params['support_loss_s']} s with the controller active in the 2 s before, and the controller then stopped "
      "before the end of the recording (robot down). Data after a support loss are excluded from all segments.\n")
    n_early = {c: int(per_trial[(per_trial.scope == "all") & (per_trial.config == c)].early_termination.fillna(False).astype(bool).sum()) for c in configs}

    # ---------------------------------------------------------------- computation
    w("## Computational performance\n")
    w("Estimator update time = duration of `EstimatorAdapter::step()` (the MUSE/IEKF/smoother update), steady clock. "
      "rx→pub = LowState subscription callback entry → state publication (software pipeline latency, not "
      "sensor-to-publication: LowState.tick is not a validated acquisition time). Rate and intervals from the "
      "steady-clock publication stamps of the estimator log. INITIALIZING samples excluded.\n")
    for key, title in [("timing_all", "Whole recording"), ("timing_stand", "Stand-up (1–5 s after four-foot contact)"), ("timing_trot", "Straight trotting")]:
        if key in tables:
            w(f"### {title}\n\n" + to_markdown(tables[key][0]) + "\n")
    if "preprocessing_all" in tables:
        w("### Pre-processing and queueing (whole recording)\n\nKinematics and contact run in parallel worker threads: "
          "their durations are not summed. Pre-processing latency = LowState reception → sample released by the "
          "synchronizer with both results (sync-ready); queue wait = release → start of the estimator update.\n\n"
          + to_markdown(tables["preprocessing_all"][0]) + "\n")
    if "resources_all" in tables:
        w("### CPU and memory\n\nCPU = psutil `Process.cpu_percent()` of the `online_estimator_node` process over 0.5 s "
          "windows, in % of one core (it can exceed 100 % on a multicore CPU; maximum 100 × number of cores). The process "
          "contains the whole estimation pipeline: acquisition, kinematics and contact workers, estimator, ROS 2 "
          "publication and the CSV run logger — not the estimator update alone. RSS = resident memory.\n\n"
          + to_markdown(tables["resources_all"][0]) + "\n")
    for key, title in [("mpc_all", "State age at MPC consumption — whole recording"),
                       ("mpc_stand", "State age at MPC consumption — stand-up (1–5 s after four-foot contact)"),
                       ("mpc_trot", "State age at MPC consumption — straight trotting")]:
        if key in tables:
            w(f"### {title}\n\n" + to_markdown(tables[key][0]) + "\n")
    w("State age = ROS time at which the controller's WBC loop read the base state − `BaseState.timestamp` of the "
      "estimate it read (= ROS time of the LowState reception of that sample). Receive-time-based, same host clock. "
      "`control loop` = rate of the controller loop that reads the state (the MPC solve runs asynchronously); "
      "`state reused` = loop iterations that read the same estimate as the previous one.\n")

    # ---------------------------------------------------------------- closed loop
    w("## Closed-loop behaviour\n")
    lo, hi = params["band_hz"]
    w(f"Raw LowState gyroscope (body axes: x roll rate, y pitch rate, z yaw rate), resampled to {params_fs()} Hz on the "
      f"recorder arrival time (LowState arrives at ≈500 Hz; pieces split at arrival gaps > {params['max_imu_gap_ms']} ms, "
      f"pieces < {params['min_piece_s']} s dropped). Band-limited RMS: zero-phase 4th-order Butterworth band-pass "
      f"{lo}–{hi} Hz. The band was chosen after inspecting the Welch spectra of all trials (Fig. S1): it contains the "
      f"body motion of every configuration and phase above the common noise floor (> {hi} Hz) and excludes slow posture "
      f"drift (< {lo} Hz). It is the same for all trials. Yaw rate is reported separately and turning segments are "
      "analysed apart.\n")
    for key, title in [("oscillation_stand", "Stand-up (1–5 s after four-foot contact)"), ("oscillation_trot", "Straight trotting"),
                       ("oscillation_turn", "Trotting while turning")]:
        if key in tables:
            w(f"### {title}\n\n" + to_markdown(tables[key][0]) + "\n")
    w("### Trial completion\n")
    w(to_markdown(pd.DataFrame([dict(configuration=c, trials=int((per_trial[(per_trial.scope == 'all')].config == c).sum()),
                                     early_termination=n_early[c]) for c in configs])) + "\n")
    w(f"Figure 5 shows, per configuration, the trial whose roll/pitch-rate RMS is the median of its configuration "
      f"in that segment (ties: lowest trial number), first {params['timeseries_window_s']} s of the first segment "
      "piece. Selected: " + ", ".join(f"{d['config']} #{d['trial']} ({d['scope']})" for d in sel_ts if d['trial']) + ".\n")
    w("Figures 10 / 10b show the estimated orientation and velocity (= `/base_state` sent to the MPC) of all trials, "
      "aligned on the first four-foot contact, with the raw IMU roll/pitch rate as estimator-independent reference. "
      "The estimate is plotted at its recorder arrival time, i.e. as received by the controller: with ws = 6 under "
      "four-foot support it arrives ≈100 ms after the LowState it describes. These curves come from each "
      "configuration's own estimator and are not an independent measurement.\n")
    if mpc_fig:
        w("MPC objective: `/mpc_cost`, the total objective of the last MPC solution, published once per controller loop. "
          "It is evaluated on the controller's *estimated* state and is a controller diagnostic, not an independent "
          "tracking-error measurement.\n")

    # ---------------------------------------------------------------- correlation
    w("## Update intervals vs body oscillation\n")
    w("One point per trial (Fig. 7). Spearman ρ between the per-trial timing metric and the per-trial roll/pitch-rate "
      "RMS of the same segment; descriptive, not a causal test.\n")
    for (s, m), rho in corr.items():
        n = int(per_trial[per_trial.scope == s].dropna(subset=[m, "bp_rms_wxy"]).shape[0])
        w(f"- {s}, {m}: ρ = {rho:.2f} (n = {n} trials)")
    i6, i1 = S.med("IS ws=6", "standup", "interval_ms_median"), S.med("IS ws=1", "standup", "interval_ms_median")
    p6, p1 = S.med("IS ws=6", "standup", "interval_ms_p99"), S.med("IS ws=1", "standup", "interval_ms_p99")
    if None not in (i6, i1, p6, p1):
        w(f"\nDuring stand-up the median publication interval was {i6:.2f} ms for ws = 6 and {i1:.2f} ms for ws = 1, and "
          f"the p99 interval {p6:.2f} vs {p1:.2f} ms: when overloaded, the smoother keeps publishing at its own update "
          "period and drops LowState samples instead of producing long gaps, so the overload shows in the output rate, "
          "the missing fraction and the state age rather than in the interval tail.\n")

    # ---------------------------------------------------------------- window size
    w("## Smoother window size\n")
    w("`smoother.window_size` = ws is the number of IMU propagation intervals in the sliding window: the window holds "
      "ws + 1 consecutive state frames, one per LowState sample (`InvariantSmoother::AllocateWindow`, n = ws + 1; logged "
      "`window_frames` = ws + 1). At ≈500 Hz it spans ≈ ws × 2 ms: 2, 6 and 12 ms for ws = 1, 3, 6.\n")
    lp = S.med("IS ws=1", "all", "interval_ms_median")
    for cfg in [c for c in configs if c.startswith("IS")]:
        bits = [f"update time median {S.fmt(cfg, 'all', 'est_ms_median', 3)} ms (p99 {S.fmt(cfg, 'all', 'est_ms_p99', 2)} ms)"]
        for sc, nm in [("standup", "stand-up (1–5 s after four-foot contact)"), ("trot_straight", "straight trotting")]:
            if S.get(cfg, sc, "est_ms_median") is not None:
                bits.append(f"{nm}: update {S.fmt(cfg, sc, 'est_ms_median', 2)} ms, rate {S.fmt(cfg, sc, 'rate_mean_hz', 0)} Hz, "
                            f"rx→pub p99 {S.fmt(cfg, sc, 'rx_to_pub_ms_p99', 1)} ms, state age p99 {S.fmt(cfg, sc, 'state_age_ms_p99', 1)} ms, "
                            f"missing {S.fmt(cfg, sc, 'frac_missing', 1, 100)} %")
        w(f"- **{cfg}**: " + "; ".join(bits) + f"; early terminations {n_early.get(cfg, 0)}/3.")
    w("")
    ws6 = "IS ws=6" in configs
    if ws6:
        st = S.med("IS ws=6", "standup", "est_ms_median")
        period = S.med("IS ws=6", "standup", "interval_ms_median")
        rate6, rate1 = S.med("IS ws=6", "standup", "rate_mean_hz"), S.med("IS ws=1", "standup", "rate_mean_hz")
        if st is not None and S.med("MUSE", "standup", "lowstate_arrival_rate_hz"):
            in_period = 1e3 / S.med("MUSE", "standup", "lowstate_arrival_rate_hz")
            if st > in_period:
                w(f"With ws = 6, during stand-up (1–5 s after four-foot contact) the median update time ({st:.2f} ms) exceeded the LowState "
                  f"period (≈{in_period:.2f} ms): the estimator could not keep up, its input queue filled, samples were "
                  f"dropped and the output rate fell to {rate6:.0f} Hz (median of trials; ws = 1: {rate1:.0f} Hz). "
                  "The records therefore support the observed reduction of the output rate, and they show that it occurs "
                  "under four-foot support, when the optimisation has the most contact factors.\n")
            else:
                w(f"With ws = 6 the median update time during stand-up (1–5 s after four-foot contact) ({st:.2f} ms) stayed below the LowState "
                  f"period (≈{in_period:.2f} ms).\n")
        a6 = S.med("IS ws=6", "standup", "state_age_ms_p99")
        if a6 is not None:
            w(f"The state the controller used was correspondingly older: p99 state age at consumption {a6:.1f} ms "
              f"during stand-up (1–5 s after four-foot contact) (ws = 1: {S.fmt('IS ws=1', 'standup', 'state_age_ms_p99', 1)} ms). "
              "This latency is bounded by the estimator input queue (capacity 50 samples × 2 ms), i.e. it is a property "
              "of the overload and of the queue size, not of the update time alone.\n")

    # ---------------------------------------------------------------- unavailable / limitations
    w("## Unavailable or unreliable metrics\n")
    for n in unavailable(trials, per_trial):
        w(f"- {n}")
    w("- `/diagnostics` hold/restart counters of these bags are unreliable (fixed later); event counts come from the run logs.")
    w("- Output rate and intervals use the steady-clock publication stamps of the run log; IMU and phase timing use recorder arrival times (jitter up to ≈10 ms, irrelevant at the analysed frequencies).")
    if "trot_straight" in set(per_trial.scope):
        n6 = int(((per_trial.scope == "trot_straight") & (per_trial.config == "IS ws=6")).sum())
        if n6 < 3:
            w(f"- IS ws=6 reached straight trotting in only {n6} of 3 trials: its trotting values rest on n = {n6}.")
    w("")
    w("## Limitations\n")
    for s in ["No external motion-capture ground truth was recorded: no estimation-accuracy metric is reported.",
              "IMU angular-rate RMS measures body motion variability (closed-loop smoothness), not state-estimation accuracy.",
              "The MPC objective is computed on the estimated state; it is not independent tracking ground truth.",
              "State age is receive-time-based (LowState reception on the estimator host), not physical acquisition age.",
              "Correlations between update intervals and oscillation are observational (n ≤ 15 trials, configurations differ in more than one respect); they do not establish causation.",
              "Commands were not recorded; segments are matched by detected gait and yaw rate, not by commanded velocity.",
              "Results apply to these five configurations on the tested Go2, controller (Quadruped-PyMPC) and onboard computer."]:
        w(f"- {s}")
    w("")

    # ---------------------------------------------------------------- manuscript text
    w("## Manuscript-ready text\n")
    w("**Results.** " + manuscript_results(S, configs, n_early, corr, per_trial) + "\n")
    w("**Response to the reviewer (onboard, closed-loop validation).** " + manuscript_reviewer(S, configs, trials, n_early, per_trial) + "\n")
    with open(path, "w") as f:
        f.write("\n".join(L))


def params_fs():
    from .phases import FS
    return int(FS)


def at_input_rate(S, configs, scope="all", tol_hz=2.0):
    """Configurations whose trial-median output rate is within tol_hz of 500 Hz."""
    return [c for c in configs if (S.med(c, scope, "rate_mean_hz") or 0) >= 500 - tol_hz]


def elevated(per_trial_scope, config, metric):
    """Trials of `config` above the maximum of all other configurations."""
    others = per_trial_scope[per_trial_scope.config != config][metric].max()
    v = per_trial_scope[per_trial_scope.config == config][metric].dropna()
    return int((v > others).sum()), int(v.size)


def manuscript_results(S, configs, n_early, corr, per_trial):
    parts = []
    m, i = "MUSE", "IEKF"
    if m in configs and i in configs:
        parts.append(f"On the onboard computer, the median estimator update took {S.fmt(m, 'all', 'est_ms_median', 3)} ms for MUSE "
                     f"and {S.fmt(i, 'all', 'est_ms_median', 3)} ms for the IEKF (median [range] over three closed-loop trials), "
                     f"with p99 values of {S.fmt(m, 'all', 'est_ms_p99', 2)} and {S.fmt(i, 'all', 'est_ms_p99', 2)} ms.")
    sm = [c for c in configs if c.startswith("IS")]
    if sm:
        parts.append("For the invariant smoother the median update time grew with the window size: " +
                     ", ".join(f"{S.fmt(c, 'all', 'est_ms_median', 2)} ms ({c.replace('IS ', '')})" for c in sm) + ".")
    ok = at_input_rate(S, configs, "standup")
    slow = [c for c in configs if c not in ok]
    if ok:
        parts.append(f"During stand-up, {', '.join(ok)} published an estimate for every LowState sample "
                     f"({', '.join(S.fmt(c, 'standup', 'rate_mean_hz', 0) + ' Hz' for c in ok)})"
                     + (f", whereas {', '.join(slow)} did not." if slow else "."))
    if "IS ws=6" in configs:
        st = S.med("IS ws=6", "standup", "est_ms_median")
        verb = "exceeded" if st and st > 2.0 else "stayed below"
        parts.append(f"With ws=6 the median update time during stand-up ({S.fmt('IS ws=6', 'standup', 'est_ms_median', 2)} ms) "
                     f"{verb} the 2 ms LowState period; the output rate was {S.fmt('IS ws=6', 'standup', 'rate_mean_hz', 0)} Hz, "
                     f"{S.fmt('IS ws=6', 'standup', 'frac_missing', 0, 100)} % of the samples were not estimated, and the p99 "
                     f"state age at controller consumption was {S.fmt('IS ws=6', 'standup', 'state_age_ms_p99', 0)} ms, compared "
                     f"with {S.fmt('IS ws=1', 'standup', 'state_age_ms_p99', 1)} ms for ws=1.")
        up = per_trial[per_trial.scope == "standup"]
        k, n = elevated(up, "IS ws=6", "bp_rms_wxy")
        others_early = sum(v for c, v in n_early.items() if c != "IS ws=6")
        n_others = 3 * (len(configs) - 1)
        parts.append(f"In the same segment the roll/pitch angular-rate RMS of the raw IMU was "
                     f"{S.fmt('IS ws=6', 'standup', 'bp_rms_wxy', 3)} rad/s for ws=6, against "
                     f"{S.fmt('IS ws=1', 'standup', 'bp_rms_wxy', 3)} rad/s for ws=1 and {S.fmt('MUSE', 'standup', 'bp_rms_wxy', 3)} "
                     f"rad/s for MUSE; {k} of {n} ws=6 trials exceeded every trial of the other configurations. "
                     f"{n_early.get('IS ws=6', 0)} of 3 ws=6 trials ended with a loss of support, against {others_early} of the "
                     f"{n_others} trials of the other configurations.")
    rho_rate, rho_age = corr.get(("standup", "rate_mean_hz")), corr.get(("standup", "state_age_ms_p99"))
    if rho_rate is not None and rho_age is not None:
        parts.append(f"Across the 15 trials, the stand-up roll/pitch-rate RMS was associated with the output rate "
                     f"(Spearman ρ = {rho_rate:.2f}) and with the p99 state age at consumption (ρ = {rho_age:.2f}) in the "
                     "same segment. Together with the timing records, this makes the reduced output rate and the larger "
                     "state age a plausible contributing factor to the oscillatory behaviour with ws=6; these observational "
                     "data do not establish causality.")
    parts.append(f"The estimator process used {S.fmt('MUSE', 'all', 'cpu_pct_median', 0)} % (MUSE) to "
                 f"{S.fmt(sm[-1] if sm else 'IEKF', 'all', 'cpu_pct_median', 0)} % ({sm[-1] if sm else 'IEKF'}) of one CPU core "
                 f"(median) and at most {max((S.med(c, 'all', 'rss_mb_max') or 0) for c in configs):.0f} MiB of resident memory.")
    return " ".join(parts)


def manuscript_reviewer(S, configs, trials, n_early, per_trial):
    n = len(trials)
    ok = [c for c in at_input_rate(S, configs, "standup") if n_early.get(c, 0) == 0]
    slow = [c for c in configs if c not in ok]
    txt = (f"To address the concern about onboard feasibility and closed-loop validation, we integrated the three estimators in "
           f"ROS 2 on the Unitree Go2 and used each of them as the only state feedback of the Quadruped-PyMPC controller in "
           f"{n} real-robot trials (three per configuration: MUSE, IEKF and the invariant smoother with window sizes 1, 3 and 6). "
           f"For every estimator update we recorded the update time, the reception-to-publication latency and the state actually "
           f"consumed by the controller, together with the CPU and memory usage of the estimation process and the raw IMU of the robot. ")
    if ok:
        txt += (f"{', '.join(ok)} kept up with the 500 Hz LowState stream (median update times "
                f"{', '.join(S.fmt(c, 'all', 'est_ms_median', 3) + ' ms' for c in ok)}) and completed all their trials. ")
    if "IS ws=6" in slow:
        k, nn = elevated(per_trial[per_trial.scope == "standup"], "IS ws=6", "bp_rms_wxy")
        txt += (f"The smoother with ws=6 exceeded the real-time budget when standing on four feet, which delayed the state used "
                f"by the controller (p99 state age {S.fmt('IS ws=6', 'standup', 'state_age_ms_p99', 0)} ms); this coincided with "
                f"increased body oscillation in {k} of {nn} trials and with a loss of support in {n_early.get('IS ws=6', 0)} of 3. ")
    txt += ("These measurements quantify the onboard cost of each estimator and identify the window size beyond which the "
            "smoother is not real-time on this platform.")
    return txt
