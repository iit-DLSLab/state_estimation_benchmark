"""Per-trial metrics (one value per trial and scope; the trial is the unit of replication).

Scopes: 'all' (whole analysis window) and the task segments of phases.py.
Every metric is computed only from the data that defines it; when that data is
absent the metric is simply not produced (never 0), and the report lists it as
unavailable.

Clock domains are never mixed:
  steady clock (estimator process)  estimator/kinematics/contact durations,
                                    reception->publication, pre-processing, queue wait,
                                    publication intervals (samples.csv publish_steady_ns)
  ROS clock (same host)             state age at MPC consumption, MPC loop intervals,
                                    publication intervals when samples.csv is missing
  recorder arrival (t_s)            phase labels only, IMU resampling, LowState input rate
"""

import numpy as np
import pandas as pd
from scipy.signal import butter, sosfiltfilt, welch

from .phases import FS, label, runs


def stats(prefix, x, which=("mean", "median", "std", "p95", "p99", "max")):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {}
    f = {"mean": np.mean, "median": np.median, "std": lambda v: np.std(v, ddof=1) if v.size > 1 else np.nan,
         "p95": lambda v: np.percentile(v, 95), "p99": lambda v: np.percentile(v, 99), "max": np.max, "min": np.min}
    out = {f"{prefix}_{w}": float(f[w](x)) for w in which}
    out[f"{prefix}_n"] = int(x.size)
    return out


def in_scope(grid, t_s, scope):
    return np.ones(len(t_s), bool) if scope == "all" else label(grid, t_s) == scope


def timing_metrics(trial, grid, scope, p):
    T = trial.data.get("timing")
    if T is None or not len(T):
        return {}
    t_first = T.t_s.min()
    ok = (T.status != "INITIALIZING").values & (T.t_s.values >= t_first + trial.warmup_s)
    m = ok & in_scope(grid, T.t_s.values, scope)
    out = {}
    out.update(stats("est_ms", T.estimator_ms[m]))
    out.update(stats("kin_ms", T.kinematics_ms[m], ("median", "p99", "max")))
    out.update(stats("contact_ms", T.contact_ms[m], ("median", "p99", "max")))
    out.update(stats("rx_to_pub_ms", T.receive_to_publish_ms[m]))
    out["n_initializing"] = int((T.status == "INITIALIZING").sum()) if scope == "all" else np.nan
    out["frac_degraded"] = float((T.status[m] == "DEGRADED").mean()) if m.any() else np.nan

    # Publication intervals: monotonic publication stamps when the run log is available.
    S = trial.data.get("samples")
    if S is not None and len(S):
        J = T[["sample_id", "t_s", "status"]].merge(S, on="sample_id", suffixes=("", "_log"))
        pub = J.publish_steady_ns.values.astype(np.int64)
        out["interval_clock"] = "steady (publication point)"
        mm = (J.status != "INITIALIZING").values & in_scope(grid, J.t_s.values, scope)
        out.update(stats("pre_ms", ((J.release_steady_ns - J.receive_steady_ns) * 1e-6)[mm], ("median", "p95", "p99", "max")))
        out.update(stats("queue_ms", ((J.est_start_steady_ns - J.release_steady_ns) * 1e-6)[mm], ("median", "p95", "p99", "max")))
        sid, ts = J.sample_id.values, J.t_s.values
    else:
        pub = T.publish_ns.values.astype(np.int64)
        out["interval_clock"] = "ROS (publication point)"
        sid, ts = T.sample_id.values, T.t_s.values
    order = np.argsort(pub)
    pub, sid, ts = pub[order], sid[order], ts[order]
    sel = in_scope(grid, ts, scope)
    both = sel[1:] & sel[:-1]
    dt = np.diff(pub)[both] * 1e-6
    if dt.size:
        out["rate_mean_hz"] = float(1e3 / dt.mean())
        out.update(stats("interval_ms", dt, ("median", "p95", "p99", "max")))
        out["frac_burst_intervals"] = float((dt < p["burst_ms"]).mean())     # queue drained back-to-back
        out["n_long_gaps"] = int((dt > p["long_gap_ms"]).sum())
        gaps = np.diff(sid)[both]
        missing = int(np.clip(gaps - 1, 0, None).sum())
        out["missing_samples"] = missing                                     # LowState samples without estimate
        out["frac_missing"] = missing / float(missing + dt.size)
        out["scope_duration_s"] = float((pub[sel][-1] - pub[sel][0]) * 1e-9) if sel.sum() > 1 else np.nan
    L = trial.data.get("lowstate")
    if L is not None and len(L):
        lm = in_scope(grid, L.t_s.values, scope)
        if lm.sum() > 1:
            d = np.diff(L.t_s.values[lm])
            d = d[d < 0.1]  # inside the scope's runs
            out["lowstate_arrival_rate_hz"] = float(1.0 / d.mean()) if d.size else np.nan
    return out


def consumption_metrics(trial, grid, scope):
    C = trial.data.get("consumption")
    if C is None or not len(C):
        return {}
    m = in_scope(grid, C.t_s.values, scope)
    out = stats("state_age_ms", C.state_age_ms[m], ("mean", "median", "p95", "p99", "max"))
    c = C.consume_ns.values
    both = m[1:] & m[:-1]
    d = np.diff(c)[both] * 1e-6
    if d.size:
        out["control_loop_hz_mean"] = float(1e3 / d.mean())
        out.update(stats("control_interval_ms", d, ("median", "p99", "max")))
        out["frac_state_reused"] = float((np.diff(C.sequence_id.values)[both] == 0).mean())
    if "mpc_cost" in C:
        out.update(stats("mpc_cost", C.mpc_cost[m], ("mean", "median", "p95", "max")))
    return out


def mpc_timing_metrics(trial, grid, scope):
    M = trial.data.get("mpc_timing")
    if M is None or not len(M):
        return {}
    m = in_scope(grid, M.t_s.values, scope)
    return stats("mpc_solve_ms", M.time_mpc_s[m] * 1e3, ("mean", "median", "p99", "max"))


def resource_metrics(trial, grid, scope):
    R = trial.data.get("resources")
    if R is None or not len(R):
        return {}
    m = in_scope(grid, R.t_s.values, scope)
    out = stats("cpu_pct", R.cpu_percent_one_core[m], ("mean", "median", "p95", "max"))
    out.update(stats("rss_mb", R.rss_mb[m], ("mean", "max")))
    out.update(stats("threads", R.num_threads[m], ("median", "max")))
    if m.any():
        out["num_cpus"] = int(R.num_cpus[m].iloc[0])
        out["monitored_pids"] = int(R.pid[m].nunique())
    return out


# ---------------------------------------------------------------- IMU oscillation
def gyro_pieces(trial, grid, scope, p):
    """Raw gyro resampled on the 500 Hz grid, as contiguous pieces of the scope.

    Pieces are split where LowState arrivals are missing for > max_imu_gap_ms
    (never interpolated across) and kept if >= min_piece_s (filter edge effects).
    """
    L = trial.data["lowstate"]
    t = L.t_s.values
    g = L[["imu_gyro_x", "imu_gyro_y", "imu_gyro_z"]].values
    gap = np.diff(t) > p["max_imu_gap_ms"] * 1e-3
    tg = grid.t_s.values
    mask = (grid.segment.values == scope) if scope != "all" else (grid.phase.values != "post_failure")
    # grid points that fall inside an IMU gap
    idx = np.clip(np.searchsorted(t, tg) - 1, 0, len(t) - 2)
    mask &= ~gap[idx]
    pieces = []
    for a, b in runs(mask):
        if (b - a) / FS >= p["min_piece_s"]:
            seg = tg[a:b]
            pieces.append(np.stack([np.interp(seg, t, g[:, k]) for k in range(3)], 1))
    return pieces


def band_limited(x, band):
    sos = butter(4, band, btype="bandpass", fs=FS, output="sos")
    return sosfiltfilt(sos, x, axis=0)


def oscillation_metrics(trial, grid, scope, p):
    if "lowstate" not in trial.data:
        return {}
    pieces = gyro_pieces(trial, grid, scope, p)
    if not pieces:
        return {}
    raw = np.concatenate(pieces)
    bp = np.concatenate([band_limited(x, p["band_hz"]) for x in pieces])
    wx, wy, wz = raw.T
    out = dict(imu_duration_s=raw.shape[0] / FS, imu_pieces=len(pieces))
    rms = lambda v: float(np.sqrt(np.mean(v ** 2)))
    out.update(rms_wx=rms(wx), rms_wy=rms(wy), rms_wxy=float(np.sqrt(np.mean(wx ** 2 + wy ** 2))), rms_wz=rms(wz),
               std_wx=float(np.std(wx, ddof=1)), std_wy=float(np.std(wy, ddof=1)), std_wz=float(np.std(wz, ddof=1)),
               p95abs_wx=float(np.percentile(np.abs(wx), 95)), p95abs_wy=float(np.percentile(np.abs(wy), 95)))
    bx, by, _ = bp.T
    out.update(bp_rms_wx=rms(bx), bp_rms_wy=rms(by), bp_rms_wxy=float(np.sqrt(np.mean(bx ** 2 + by ** 2))))
    # Welch PSD per piece, averaged with weights = piece length; dominant frequency and
    # band powers in the common band and its fixed sub-bands (supplementary only).
    nper = int(min(p["welch_nperseg"], min(len(x) for x in pieces)))
    psd, wsum = 0.0, 0
    for x in pieces:
        f, P = welch(x[:, :2] - x[:, :2].mean(0), fs=FS, nperseg=nper, axis=0)
        psd, wsum = psd + P * len(x), wsum + len(x)
    psd /= wsum
    lo, hi = p["band_hz"]
    inb = (f >= lo) & (f <= hi)
    for k, ax in enumerate(["wx", "wy"]):
        out[f"peak_hz_{ax}"] = float(f[inb][np.argmax(psd[inb, k])])
        for a, b in p["sub_bands_hz"]:
            sb = (f >= a) & (f < b)
            out[f"psd_rms_{ax}_{a:g}_{b:g}hz"] = float(np.sqrt(np.trapz(psd[sb, k], f[sb])))
    trial.data.setdefault("psd", {})[scope] = (f, psd)
    return out


def trial_metrics(trial, grid, events, scopes, p):
    rows = []
    for scope in scopes:
        m = {}
        for fn in (timing_metrics, consumption_metrics, mpc_timing_metrics, resource_metrics, oscillation_metrics):
            args = (trial, grid, scope, p) if fn in (timing_metrics, oscillation_metrics) else (trial, grid, scope)
            m.update(fn(*args))
        if scope != "all" and not any(k.startswith(("est_ms", "rms_", "state_age")) for k in m):
            continue  # segment absent in this trial
        m.update(config=trial.config, estimator=trial.estimator, ws=trial.ws, trial=trial.trial, scope=scope)
        if scope == "all":
            m.update(early_termination=events["early_termination"], support_loss_s=events["support_loss_s"],
                     controller_stop_s=events["controller_stop_s"])
        rows.append(m)
    return rows


def summarize(per_trial):
    """Per configuration and scope: trial-level aggregation of every numeric metric."""
    keys = ["config", "scope"]
    num = [c for c in per_trial.columns if c not in keys + ["estimator", "ws", "trial"]
           and pd.api.types.is_numeric_dtype(per_trial[c])]
    rows = []
    for (cfg, scope), g in per_trial.groupby(keys, sort=False):
        for c in num:
            v = g[c].dropna().astype(float)
            if v.empty:
                continue
            rows.append(dict(config=cfg, scope=scope, metric=c, n_trials=len(v), median=v.median(), min=v.min(),
                             max=v.max(), mean=v.mean(), std=v.std(ddof=1) if len(v) > 1 else np.nan,
                             trials=";".join(f"{x:.6g}" for x in v)))
    return pd.DataFrame(rows)
