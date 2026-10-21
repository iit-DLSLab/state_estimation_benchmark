"""Publication figures (IEEE: 3.5 in single column, 7.16 in double column, 8 pt).

Trial-level values are drawn as individual markers (one per trial); the line
across them is the median of the trials of the configuration, the bar their range.
Sample-level distributions (boxes) pool the samples of the trials of one
configuration for display only; no statistics are computed on pooled samples.
"""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .data import CONFIG_ORDER
from .metrics import gyro_pieces
from .phases import FS, label

COLORS = {"MUSE": "#1b9e77", "IEKF": "#d95f02", "IS ws=1": "#7570b3", "IS ws=3": "#e7298a", "IS ws=6": "#66a61e"}
MARKERS = {"MUSE": "o", "IEKF": "s", "IS ws=1": "^", "IS ws=3": "D", "IS ws=6": "v"}
SCOPE_TITLE = {"all": "whole recording", "standup": "stand-up (1–5 s after 4-foot contact)", "trot_straight": "straight trotting",
               "trot_turn": "trotting while turning", "stand_final": "final standing", "lying": "lying"}
SHORT = {"standup": "Stand-up (1–5 s after four-foot contact)", "trot_straight": "Straight trotting"}
COL1, COL2 = 3.5, 7.16

plt.rcParams.update({
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
    "ytick.labelsize": 7, "lines.linewidth": 1.0, "axes.linewidth": 0.6, "grid.linewidth": 0.4,
    "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})


def maybe_log(ax, values, axis="y"):
    """Log scale only when the data span more than ~a decade (readable ticks otherwise)."""
    v = np.asarray([x for x in np.ravel(values) if np.isfinite(x) and x > 0])
    if v.size and v.max() / v.min() > 8:
        (ax.set_yscale if axis == "y" else ax.set_xscale)("log")


def save(fig, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(out_dir, f"{name}.{ext}"), dpi=300)
    plt.close(fig)


def configs_present(df):
    return [c for c in CONFIG_ORDER if c in set(df.config)]


def trial_points(ax, per_trial, scope, metric, ylabel, log=False, configs=None):
    """One marker per trial, median line and range bar per configuration."""
    d = per_trial[per_trial.scope == scope]
    configs = configs or configs_present(per_trial)
    for i, c in enumerate(configs):
        v = d.loc[d.config == c, metric].dropna().values if metric in d else np.array([])
        if v.size == 0:
            ax.text(i, 0.5, "n/a", transform=ax.get_xaxis_transform(), ha="center", va="center", fontsize=6, color="0.5")
            continue
        x = i + np.linspace(-0.12, 0.12, v.size) if v.size > 1 else np.array([i])
        ax.plot([i, i], [v.min(), v.max()], color=COLORS[c], lw=0.8, alpha=0.6)
        ax.plot([i - 0.25, i + 0.25], [np.median(v)] * 2, color=COLORS[c], lw=1.6)
        ax.scatter(x, v, s=14, marker=MARKERS[c], facecolor="white", edgecolor=COLORS[c], linewidth=0.9, zorder=3)
    ax.set_xticks(range(len(configs)))
    ax.set_xticklabels([c.replace("IS ", "IS\n") for c in configs])
    ax.set_xlim(-0.6, len(configs) - 0.4)
    ax.set_ylabel(ylabel)
    if log:
        maybe_log(ax, d[metric].values if metric in d else [])
    ax.grid(True, axis="y", which="both", alpha=0.3)


def pooled_box(ax, trials, column, role, ylabel, valid=None):
    configs = [c for c in CONFIG_ORDER if any(t.config == c for t in trials)]
    data = []
    for c in configs:
        v = np.concatenate([t.data[role][column].values[valid(t)] if valid else t.data[role][column].values
                            for t in trials if t.config == c and role in t.data] or [np.array([])])
        data.append(v[np.isfinite(v)])
    bp = ax.boxplot(data, positions=range(len(configs)), whis=(1, 99), showfliers=False, widths=0.5,
                    patch_artist=True, medianprops=dict(color="k", lw=1))
    for patch, c in zip(bp["boxes"], configs):
        patch.set_facecolor(COLORS[c])
        patch.set_alpha(0.35)
    ax.set_xticks(range(len(configs)))
    ax.set_xticklabels([c.replace("IS ", "IS\n") for c in configs])
    ax.set_yscale("log")
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", which="both", alpha=0.3)
    return configs


def overlay_p99(ax, per_trial, configs, metric, scope="all"):
    d = per_trial[per_trial.scope == scope]
    for i, c in enumerate(configs):
        v = d.loc[d.config == c, metric].dropna().values
        if v.size:
            ax.scatter(i + np.linspace(-0.12, 0.12, v.size) if v.size > 1 else [i], v, marker="_", s=60,
                       color="k", linewidth=1.2, zorder=4)


def running(t):
    return lambda tr: (tr.data["timing"].status.values != "INITIALIZING")


def fig_runtime(trials, per_trial, out):
    fig, axs = plt.subplots(1, 2, figsize=(COL2, 2.1))
    cfgs = pooled_box(axs[0], trials, "estimator_ms", "timing", "estimator update time [ms]", valid=running(None))
    overlay_p99(axs[0], per_trial, cfgs, "est_ms_p99")
    axs[0].set_title("(a) samples (box 25-75 %, whiskers 1-99 %); bars: trial p99")
    trial_points(axs[1], per_trial, "all", "est_ms_median", "estimator update time [ms]", log=True)
    for i, c in enumerate(configs_present(per_trial)):
        v = per_trial[(per_trial.scope == "all") & (per_trial.config == c)].est_ms_p99.dropna().values
        axs[1].scatter(i + np.linspace(-0.12, 0.12, v.size) if v.size > 1 else [i], v, marker="x", s=14, color=COLORS[c], zorder=3)
    axs[1].set_title("(b) per trial: median (open), p99 (x)")
    save(fig, out, "fig1_estimator_runtime")


def fig_output_rate(per_trial, out):
    scopes = [s for s in ["all", "standup", "trot_straight"] if s in set(per_trial.scope)]
    fig, axs = plt.subplots(len(scopes), 3, figsize=(COL2, 1.75 * len(scopes)), squeeze=False)
    for r, s in enumerate(scopes):
        trial_points(axs[r, 0], per_trial, s, "rate_mean_hz", "mean output rate [Hz]")
        trial_points(axs[r, 1], per_trial, s, "interval_ms_p99", "p99 update interval [ms]", log=True)
        trial_points(axs[r, 2], per_trial, s, "interval_ms_max", "max update gap [ms]", log=True)
        axs[r, 0].set_title(f"{SCOPE_TITLE[s]}", loc="left")
    fig.tight_layout()
    save(fig, out, "fig2_output_rate")


def fig_latency(trials, per_trial, out):
    fig, axs = plt.subplots(1, 2, figsize=(COL2, 2.1))
    cfgs = pooled_box(axs[0], trials, "receive_to_publish_ms", "timing", "reception→publication [ms]", valid=running(None))
    overlay_p99(axs[0], per_trial, cfgs, "rx_to_pub_ms_p99")
    axs[0].set_title("(a) LowState callback → state publication")
    if any("consumption" in t.data for t in trials):
        cfgs = pooled_box(axs[1], trials, "state_age_ms", "consumption", "state age at MPC consumption [ms]")
        overlay_p99(axs[1], per_trial, cfgs, "state_age_ms_p99")
        axs[1].set_title("(b) state age when the controller reads it")
    else:
        axs[1].text(0.5, 0.5, "MPC consumption not recorded", ha="center", transform=axs[1].transAxes)
    save(fig, out, "fig3_latency")


def fig_resources(per_trial, out):
    fig, axs = plt.subplots(1, 3, figsize=(COL2, 1.9))
    trial_points(axs[0], per_trial, "all", "cpu_pct_median", "CPU median [% core]")
    trial_points(axs[1], per_trial, "all", "cpu_pct_p95", "CPU p95 [% core]")
    trial_points(axs[2], per_trial, "all", "rss_mb_max", "peak RSS [MiB]")
    fig.tight_layout()
    save(fig, out, "fig4_cpu_memory")


def representative(per_trial, config, scope, metric="bp_rms_wxy"):
    """Trial whose metric is the median of its configuration (deterministic, not best/worst)."""
    d = per_trial[(per_trial.scope == scope) & (per_trial.config == config)].dropna(subset=[metric])
    if d.empty:
        return None
    d = d.assign(dist=(d[metric] - d[metric].median()).abs()).sort_values(["dist", "trial"])
    return int(d.trial.iloc[0])


def fig_timeseries(trials, per_trial, p, out):
    """Raw roll/pitch rates; per configuration the trial with the median RMS of the segment."""
    scopes = [s for s in ["standup", "trot_straight"] if s in set(per_trial.scope)]
    configs = configs_present(per_trial)
    win = p["timeseries_window_s"]
    fig = plt.figure(figsize=(COL2, 2.25 * len(scopes)))
    subs = np.atleast_1d(fig.subfigures(len(scopes), 1, hspace=0.04))
    sel = []
    for sf, s in zip(subs, scopes):
        sf.suptitle(SHORT[s], fontsize=8, fontweight="bold", x=0.01, y=0.995, ha="left")
        axs = sf.subplots(2, len(configs), sharex=True, sharey="row", squeeze=False)
        sf.subplots_adjust(top=0.84, bottom=0.17, left=0.09, right=0.99, wspace=0.12, hspace=0.12)
        for c, cfg in enumerate(configs):
            k = representative(per_trial, cfg, s)
            tr = next((t for t in trials if t.config == cfg and t.trial == k), None)
            axs[0, c].set_title(f"{cfg} (trial {k})" if tr else cfg, fontsize=7)
            for a in (0, 1):
                ax = axs[a, c]
                ax.grid(True, alpha=0.3)
                if tr is None:
                    ax.text(0.5, 0.5, "no segment", ha="center", va="center", transform=ax.transAxes, fontsize=6)
                    continue
                x = gyro_pieces(tr, tr.data["grid"], s, p)[0][: int(win * FS), a]  # first window of the first piece
                ax.plot(np.arange(x.size) / FS, x, color=COLORS[cfg], lw=0.5)
            sel.append(dict(scope=s, config=cfg, trial=k))
        axs[0, 0].set_ylabel("roll rate ω_x\n[rad/s]")
        axs[1, 0].set_ylabel("pitch rate ω_y\n[rad/s]")
        for ax in axs[1]:
            ax.set_xlabel("time in segment [s]")
        for r in range(2):
            lim = max(abs(l) for ax in axs[r] for l in ax.get_ylim())
            axs[r, 0].set_ylim(-lim, lim)
    save(fig, out, "fig5_imu_timeseries")
    return sel


def fig_oscillation(per_trial, out):
    scopes = [s for s in ["standup", "trot_straight"] if s in set(per_trial.scope)]
    fig, axs = plt.subplots(len(scopes), 3, figsize=(COL2, 1.75 * len(scopes)), squeeze=False)
    for r, s in enumerate(scopes):
        trial_points(axs[r, 0], per_trial, s, "bp_rms_wx", "roll-rate RMS [rad/s]")
        trial_points(axs[r, 1], per_trial, s, "bp_rms_wy", "pitch-rate RMS [rad/s]")
        trial_points(axs[r, 2], per_trial, s, "bp_rms_wxy", "roll/pitch-rate RMS [rad/s]")
        axs[r, 0].set_title(f"{SCOPE_TITLE[s]}", loc="left")
    fig.tight_layout()
    save(fig, out, "fig6_oscillation_rms")


GAP_METRICS = [("interval_ms_p99", "p99 update interval [ms]", True), ("rate_mean_hz", "mean output rate [Hz]", False),
               ("state_age_ms_p99", "p99 state age at MPC [ms]", True)]


def fig_gap_vs_oscillation(per_trial, corr, out):
    """One point per trial; x: estimator timing in the same segment, y: IMU roll/pitch-rate RMS."""
    scopes = [s for s in ["standup", "trot_straight"] if s in set(per_trial.scope)]
    fig, axs = plt.subplots(len(scopes), 3, figsize=(COL2, 2.0 * len(scopes)), squeeze=False)
    for r, s in enumerate(scopes):
        for k, (m, xl, log) in enumerate(GAP_METRICS):
            ax = axs[r, k]
            d = per_trial[per_trial.scope == s].dropna(subset=[m, "bp_rms_wxy"]) if m in per_trial else per_trial.iloc[:0]
            for c in configs_present(per_trial):
                g = d[d.config == c]
                ax.scatter(g[m], g.bp_rms_wxy, marker=MARKERS[c], s=20, facecolor="white", edgecolor=COLORS[c],
                           linewidth=1.0, label=c)
            if log:
                maybe_log(ax, d[m].values, axis="x")
            ax.set_xlabel(xl)
            if k == 0:
                ax.set_ylabel(f"roll/pitch-rate RMS [rad/s]\n{SCOPE_TITLE[s]}")
            rho = corr.get((s, m))
            ax.set_title(f"n={len(d)} trials" + (f", Spearman ρ={rho:.2f}" if rho is not None else ""))
            ax.grid(True, which="both", alpha=0.3)
    axs[0, 0].legend(frameon=False, fontsize=6)
    fig.tight_layout()
    save(fig, out, "fig7_gap_vs_oscillation")


def fig_window_sweep(per_trial, out):
    d = per_trial[per_trial.estimator == "smoother"]
    if d.empty:
        return
    panels = [("all", "est_ms_p99", "update time p99 [ms]"), ("all", "rate_mean_hz", "mean output rate [Hz]"),
              ("all", "interval_ms_p99", "update interval p99 [ms]"), ("all", "rx_to_pub_ms_p99", "rx→pub latency p99 [ms]"),
              ("standup", "est_ms_median", "update time median,\nstand-up [ms]"),
              ("standup", "bp_rms_wxy", "roll/pitch-rate RMS,\nstand-up [rad/s]")]
    fig, axs = plt.subplots(2, 3, figsize=(COL2, 3.4))
    for ax, (s, m, yl) in zip(axs.flat, panels):
        g = d[d.scope == s]
        ws = sorted(g.ws.dropna().unique())
        med = []
        for w in ws:
            v = g.loc[g.ws == w, m].dropna().values
            cfg = f"IS ws={int(w)}"
            ax.scatter(np.full(v.size, w), v, marker=MARKERS.get(cfg, "o"), s=16, facecolor="white",
                       edgecolor=COLORS.get(cfg, "k"), zorder=3)
            med.append(np.median(v) if v.size else np.nan)
        ax.plot(ws, med, color="0.3", lw=1)
        ax.set_xticks(ws)
        ax.set_xlabel("window size ws [–]")
        ax.set_ylabel(yl)
        if "p99" in m or "latency" in yl:
            maybe_log(ax, g[m].values)
        ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    save(fig, out, "fig8_window_size_sweep")


def fig_mpc(trials, per_trial, out):
    if not any("consumption" in t.data and "mpc_cost" in t.data["consumption"] for t in trials):
        return False
    fig, axs = plt.subplots(1, 2, figsize=(COL2, 2.7))
    for cfg in configs_present(per_trial):
        k = representative(per_trial, cfg, "all", "mpc_cost_median")
        tr = next((t for t in trials if t.config == cfg and t.trial == k), None)
        if tr is None:
            continue
        C = tr.data["consumption"]
        axs[0].plot(C.t_s, C.mpc_cost.rolling(200, min_periods=1).median(), color=COLORS[cfg], lw=0.8, label=f"{cfg} #{k}")
    axs[0].set_yscale("log")
    axs[0].set_xlabel("time since bag start [s]")
    axs[0].set_ylabel("MPC objective [–]\n(0.4 s running median)")
    axs[0].legend(frameon=False, fontsize=6, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.32))
    axs[0].grid(True, which="both", alpha=0.3)
    trial_points(axs[1], per_trial, "trot_straight" if "trot_straight" in set(per_trial.scope) else "all",
                 "mpc_cost_median", "MPC objective, median [–]", log=True)
    axs[1].set_title("per trial, straight trotting")
    fig.tight_layout()
    save(fig, out, "fig9_mpc_cost")
    return True


def fig_psd(trials, p, out):
    """Supplementary: mean Welch PSD per configuration, the basis of the band choice."""
    scopes = ["standup", "trot_straight"]
    fig, axs = plt.subplots(2, 2, figsize=(COL2, 3.6), sharex=True)
    for r, s in enumerate(scopes):
        for a in (0, 1):
            ax = axs[r, a]
            for cfg in CONFIG_ORDER:
                ps = [t.data["psd"][s] for t in trials if t.config == cfg and s in t.data.get("psd", {})]
                if not ps:
                    continue
                f = ps[0][0]
                P = np.mean([np.interp(f, q[0], q[1][:, a]) for q in ps], axis=0)
                ax.semilogy(f, P, color=COLORS[cfg], lw=0.8, label=f"{cfg} (n={len(ps)})")
            ax.axvspan(*p["band_hz"], color="0.9", zorder=0)
            ax.set_xlim(0, 60)
            ax.set_title(f"{SCOPE_TITLE[s]}: {'roll' if a == 0 else 'pitch'} rate")
            ax.set_ylabel("PSD [(rad/s)²/Hz]")
            ax.grid(True, which="both", alpha=0.3)
    for ax in axs[-1]:
        ax.set_xlabel("frequency [Hz]")
    axs[0, 0].legend(frameon=False, fontsize=6)
    fig.tight_layout()
    save(fig, out, "figS1_gyro_psd")


def quat_to_roll_pitch(w, x, y, z):
    """Roll, pitch [deg] of the base->world quaternion (ZYX convention)."""
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


def fig_base_state(trials, p, out, t_before=2.0, t_after=20.0, only=None, name="fig10_base_state", width=COL2):
    """Estimated orientation and velocity (= /base_state sent to the MPC) around stand-up.

    All trials of each configuration, aligned on the first four-foot contact (t = 0);
    the last row is the raw IMU roll/pitch rate (estimator-independent reference).
    A cross marks the support loss; the curves stop there.
    """
    configs = [c for c in (only or CONFIG_ORDER) if any(t.config == c and "state" in t.data for t in trials)]
    if not configs:
        return
    rows = [("roll", "roll [deg]"), ("pitch", "pitch [deg]"), ("v_world_x", "v_x [m/s]"),
            ("v_world_y", "v_y [m/s]"), ("v_world_z", "v_z [m/s]"), ("gyro", "IMU |ω_xy| [rad/s]")]
    fig, axs = plt.subplots(len(rows), len(configs), figsize=(width, 6.2), sharex=True, sharey="row", squeeze=False)
    styles = ["-", "--", ":"]
    for c, cfg in enumerate(configs):
        axs[0, c].set_title(cfg, fontsize=7)
        for k, tr in enumerate(sorted([t for t in trials if t.config == cfg], key=lambda t: t.trial)):
            ev = tr.data.get("phase_events", {})
            t0 = ev.get("four_foot_onset_s")
            if t0 is None or "state" not in tr.data:
                continue
            tend = ev.get("support_loss_s") or np.inf
            S = tr.data["state"]
            m = (S.t_s >= t0 - t_before) & (S.t_s <= min(t0 + t_after, tend)) & (S.status != "INITIALIZING")
            ts = S.t_s[m].values - t0
            roll, pitch = quat_to_roll_pitch(S.q_wb_w[m].values, S.q_wb_x[m].values, S.q_wb_y[m].values, S.q_wb_z[m].values)
            series = {"roll": roll, "pitch": pitch, "v_world_x": S.v_world_x[m].values, "v_world_y": S.v_world_y[m].values,
                      "v_world_z": S.v_world_z[m].values}
            L = tr.data["lowstate"]
            ml = (L.t_s >= t0 - t_before) & (L.t_s <= min(t0 + t_after, tend))
            g = np.hypot(L.imu_gyro_x[ml].values, L.imu_gyro_y[ml].values)
            n = max(1, int(0.1 * FS))  # 0.1 s moving RMS for readability
            series["gyro"] = np.sqrt(np.convolve(g ** 2, np.ones(n) / n, mode="same"))
            for r, (key, _) in enumerate(rows):
                x = L.t_s[ml].values - t0 if key == "gyro" else ts
                y = series[key]
                if key != "gyro" and y.size > 20000:  # thin to ~250 Hz for the vector output
                    x, y = x[::2], y[::2]
                axs[r, c].plot(x, y, color=COLORS[cfg], lw=0.5, ls=styles[k % 3], label=f"trial {tr.trial}")
                if np.isfinite(tend) and tend - t0 <= t_after and x.size:
                    axs[r, c].plot(x[-1], y[-1], marker="x", color="k", ms=4, mew=1.0)
    for r, (_, yl) in enumerate(rows):
        axs[r, 0].set_ylabel(yl)
        for ax in axs[r]:
            ax.grid(True, alpha=0.3)
            ax.axvline(0, color="0.6", lw=0.5)
    axs[0, 0].legend(frameon=False, fontsize=5, loc="lower left")
    fig.tight_layout(rect=(0, 0.045, 1, 1), h_pad=0.2, w_pad=0.3)
    fig.supxlabel("time from four-foot contact [s] (recorder arrival time: what the controller received when)",
                  fontsize=7, y=0.022)
    fig.text(0.01, 0.002, "Rows 1–5: estimate of the configuration's own estimator (= /base_state sent to the MPC); "
             "row 6: raw IMU, 0.1 s moving RMS. ×: support loss (curves end).", fontsize=6)
    save(fig, out, name)
