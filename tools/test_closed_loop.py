"""Unit tests of the analysis code on SYNTHETIC fixtures (code validation only, not results).

  python3 -m pytest -q tools/test_closed_loop.py
"""

import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_closed_loop import DEFAULTS  # noqa: E402
from closed_loop import metrics, phases  # noqa: E402
from closed_loop.data import Trial  # noqa: E402

FS = 500.0


def synthetic_trial(fall=True):
    """Lying 0-5 s, four-foot standing 5-15 s, trot 15-35 s, then (optionally) support loss."""
    end = 40.0 if fall else 35.0
    t = np.arange(0, end, 1 / FS)
    ff = np.zeros((t.size, 4))
    stand = (t >= 5) & (t < 15)
    trot = (t >= 15) & (t < 35)
    ff[stand] = 50
    phase_a = (np.floor((t - 15) * 3.0 * 2) % 2 == 0)  # diagonal pairs alternate every 1/6 s
    ff[trot & phase_a, 0] = ff[trot & phase_a, 3] = 60
    ff[trot & ~phase_a, 1] = ff[trot & ~phase_a, 2] = 60
    if not fall:
        pass
    gx = 0.05 * np.sin(2 * np.pi * 3.0 * t) * stand + 0.2 * np.sin(2 * np.pi * 10.0 * t) * trot
    L = pd.DataFrame(dict(t_s=t, tick=np.arange(t.size), imu_gyro_x=gx, imu_gyro_y=0.0 * t, imu_gyro_z=0.0 * t,
                          m1_q=0.9, **{f"foot_force_{i}": ff[:, i] for i in range(4)}))
    # controller steps until 36 s (stops 4 s before the end when falling)
    tc = np.arange(5, 36 if fall else end, 1 / 400.0)
    C = pd.DataFrame(dict(t_s=tc, consume_ns=(tc * 1e9).astype(np.int64), state_stamp_ns=(tc * 1e9 - 2e6).astype(np.int64),
                          sequence_id=np.arange(tc.size), state_age_ms=2.0, mpc_cost=1.0))
    # estimator: one estimate per LowState sample, sample 1000..1009 missing
    sid = np.arange(t.size)
    keep = (sid < 1000) | (sid >= 1010)
    T = pd.DataFrame(dict(t_s=t[keep], publish_ns=(t[keep] * 1e9).astype(np.int64), estimator="x", sample_id=sid[keep],
                          lowstate_tick=sid[keep], status=np.where(sid[keep] == 0, "INITIALIZING", "VALID"),
                          estimator_ms=0.1, kinematics_ms=0.01, contact_ms=0.001, receive_to_publish_ms=0.2))
    tr = Trial(estimator="muse", trial=1, ws=None, source="synthetic")
    tr.data = dict(lowstate=L, consumption=C, timing=T)
    return tr


def test_stats_and_band_rms():
    s = metrics.stats("x", [1, 2, 3, np.nan])
    assert s["x_median"] == 2 and s["x_n"] == 3 and s["x_max"] == 3
    assert metrics.stats("x", [np.nan]) == {}  # unavailable, never 0
    t = np.arange(0, 10, 1 / FS)
    x = 0.3 * np.sin(2 * np.pi * 5 * t) + 1.0  # offset outside the band
    y = metrics.band_limited(x[:, None], [0.5, 30.0])[:, 0]
    rms = np.sqrt(np.mean(y[500:-500] ** 2))
    assert abs(rms - 0.3 / np.sqrt(2)) < 0.01


def test_phases_and_failure():
    tr = synthetic_trial(fall=True)
    grid, ev = phases.segment_trial(tr, DEFAULTS | {"standup_window_s": 4.0})
    segs = phases.segment_table(grid)
    up = segs[segs.segment == "standup"].iloc[0]
    assert abs(up.start_s - 6.0) < 0.1 and abs(up.duration_s - 4.0) < 0.05
    trot = segs[segs.segment == "trot_straight"].duration_s.sum()
    assert 14 < trot <= 16  # 20 s bout minus 2 s margins
    assert ev["early_termination"] and 34.5 < ev["support_loss_s"] < 36.0
    grid2, ev2 = phases.segment_trial(synthetic_trial(fall=False), DEFAULTS | {"standup_window_s": 4.0})
    assert not ev2["early_termination"]


def test_timing_and_oscillation_metrics():
    tr = synthetic_trial(fall=True)
    p = DEFAULTS | {"standup_window_s": 4.0}
    grid, ev = phases.segment_trial(tr, p)
    m = metrics.timing_metrics(tr, grid, "all", p)
    assert m["missing_samples"] == 10 and m["interval_clock"].startswith("ROS")
    assert abs(m["interval_ms_median"] - 2.0) < 1e-6 and abs(m["interval_ms_max"] - 22.0) < 1e-6
    o = metrics.oscillation_metrics(tr, grid, "standup", p)
    assert abs(o["bp_rms_wx"] - 0.05 / np.sqrt(2)) < 0.005 and abs(o["peak_hz_wx"] - 3.0) < 0.5
    o = metrics.oscillation_metrics(tr, grid, "trot_straight", p)
    assert abs(o["bp_rms_wx"] - 0.2 / np.sqrt(2)) < 0.01 and abs(o["peak_hz_wx"] - 10.0) < 0.5
    c = metrics.consumption_metrics(tr, grid, "standup")
    assert abs(c["state_age_ms_median"] - 2.0) < 1e-9 and abs(c["control_loop_hz_mean"] - 400.0) < 1
