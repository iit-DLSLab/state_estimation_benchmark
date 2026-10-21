"""Task-phase segmentation from raw LowState (estimator-independent).

No command, joystick or experiment-status topic was recorded, so phases are
detected from the robot's own measurements:

  contact      foot_force > contact_threshold (raw Unitree units; 15 = threshold of
               the estimators' contact detection)
  switch rate  contact changes per leg per second (1 s moving window)

  lying         mean loaded fraction < 0.25 (1 s window)
  four_foot     mean loaded fraction > 0.95 (1 s window) and switch rate < 0.5 /leg/s
  trot          switch rate > 3 /leg/s
  transition    everything else

Analysis segments (used for the matched comparisons):
  standup        the same task event in every trial: from 1 s to 1 s + standup_window_s
                 after the first time all four feet are loaded for >= 1 s (goUp and the
                 standing that follows), cut at the first stepping (switch rate > 0.5
                 /leg/s) or support loss
  stand_final    four_foot bouts after the last trot bout (trimmed by margin_s)
  trot_straight  trot with |yaw rate| (2 s moving average of the raw gyro z) < yaw_straight
  trot_turn      trot with |yaw rate| > yaw_turn
  lying          lying bouts

Support loss: after the robot has stood, the mean loaded fraction falls below 0.1 for
`support_loss_s`, with the controller sending commands within the 2 s before. Everything
after it is labelled post_failure and excluded from every segment.
The other segments (trot_*, lying, stand_final) are bouts trimmed by margin_s at both ends.

Manual phase boundaries in the configuration (phases: [{name, start, end}] on t_s)
replace the automatic segments.
"""

import numpy as np
import pandas as pd

FS = 500.0  # uniform analysis grid [Hz] = nominal LowState rate


def moving_mean(x, n):
    return np.convolve(x.astype(float), np.ones(n) / n, mode="same")


def runs(mask):
    """[start_idx, end_idx) of the True runs of a boolean array."""
    d = np.diff(np.r_[0, mask.astype(int), 0])
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0]))


def segment_trial(trial, p):
    """Returns (grid DataFrame with a 'phase' and 'segment' label per 2 ms, events dict)."""
    L = trial.data["lowstate"]
    t = L.t_s.values
    tg = np.arange(t[0], t[-1], 1.0 / FS)
    ld = np.stack([np.interp(tg, t, (L[f"foot_force_{i}"].values > p["contact_threshold"]).astype(float)) > 0.5
                   for i in range(4)], 1)
    w1 = int(FS)
    switches = np.r_[0, np.abs(np.diff(ld.astype(int), axis=0)).sum(1)]
    rate = moving_mean(switches, w1) * FS / 4.0
    loaded = moving_mean(ld.mean(1), w1)
    all4 = ld.all(1)
    yaw = moving_mean(np.interp(tg, t, L.imu_gyro_z.values), int(2 * FS))

    phase = np.full(len(tg), "transition", dtype=object)
    phase[loaded < 0.25] = "lying"
    phase[(loaded > 0.95) & (rate < 0.5)] = "four_foot"
    phase[rate > 3.0] = "trot"

    events = {"support_loss_s": None, "controller_stop_s": None, "early_termination": False}
    C = trial.data.get("consumption")
    if C is not None and len(C):
        events["controller_stop_s"] = float(C.t_s.iloc[-1])
        last = np.searchsorted(C.t_s.values, tg, side="right") - 1  # last controller step before each time
        active = (last >= 0) & (tg - C.t_s.values[np.clip(last, 0, None)] < 2.0)
    else:
        active = np.ones(len(tg), bool)
    stood = np.cumsum(phase == "four_foot") > FS  # stood for > 1 s at some point before
    lost = moving_mean(ld.mean(1), int(p["support_loss_s"] * FS)) < 0.1
    loss = np.where(stood & lost & active)[0]
    if len(loss):
        i0 = loss[0] - int(p["support_loss_s"] * FS / 2)  # start of the window
        events["support_loss_s"] = float(tg[max(i0, 0)])
        phase[max(i0, 0):] = "post_failure"
        stop = events["controller_stop_s"]
        events["early_termination"] = stop is not None and stop < t[-1] - 1.0

    segment = np.full(len(tg), "", dtype=object)
    margin = int(p["margin_s"] * FS)
    trot_idx = np.where(phase == "trot")[0]
    first_trot = tg[trot_idx[0]] if len(trot_idx) else np.inf
    last_trot = tg[trot_idx[-1]] if len(trot_idx) else -np.inf
    # stand-up window: first run of all four feet loaded lasting >= 1 s
    onset = next((a for a, b in runs(all4) if b - a >= FS), None)
    events["four_foot_onset_s"] = float(tg[onset]) if onset is not None else None
    if onset is not None:
        a = onset + int(FS)
        b = min(a + int(p["standup_window_s"] * FS), len(tg))
        stop = np.where((rate[a:b] > 0.5) | (phase[a:b] == "post_failure"))[0]  # first stepping or failure
        b = a + stop[0] if len(stop) else b
        if b - a >= int(p["min_segment_s"] * FS):
            segment[a:b] = "standup"
    for name in ["four_foot", "trot", "lying"]:
        for a, b in runs(phase == name):
            if b - a <= 2 * margin + int(p["min_segment_s"] * FS):
                continue
            a2, b2 = a + margin, b - margin
            if name == "lying":
                segment[a2:b2] = "lying"
            elif name == "four_foot":
                if tg[a] >= last_trot:
                    segment[a2:b2] = "stand_final"
            else:
                y = np.abs(yaw[a2:b2])
                seg = np.where(y < p["yaw_straight"], "trot_straight", np.where(y > p["yaw_turn"], "trot_turn", ""))
                segment[a2:b2] = seg

    if trial.phases:  # manual boundaries replace the automatic segments
        segment[:] = ""
        for ph in trial.phases:
            segment[(tg >= ph["start"]) & (tg < ph["end"])] = ph["name"]
        if events["support_loss_s"] is not None:
            segment[tg >= events["support_loss_s"]] = ""

    grid = pd.DataFrame(dict(t_s=tg, phase=phase, segment=segment, switch_rate=rate, loaded=loaded, yaw_rate_lp=yaw))
    return grid, events


def label(grid, t_s):
    """Segment label of arbitrary times (nearest grid point; '' outside)."""
    idx = np.clip(np.round((np.asarray(t_s) - grid.t_s.values[0]) * FS).astype(int), 0, len(grid) - 1)
    out = grid.segment.values[idx]
    out[(np.asarray(t_s) < grid.t_s.values[0]) | (np.asarray(t_s) > grid.t_s.values[-1])] = ""
    return out


def segment_table(grid):
    rows = []
    for name in sorted(set(grid.segment) - {""}):
        for a, b in runs(grid.segment.values == name):
            rows.append(dict(segment=name, start_s=grid.t_s.values[a], end_s=grid.t_s.values[b - 1],
                             duration_s=(b - a) / FS))
    return pd.DataFrame(rows, columns=["segment", "start_s", "end_s", "duration_s"])
