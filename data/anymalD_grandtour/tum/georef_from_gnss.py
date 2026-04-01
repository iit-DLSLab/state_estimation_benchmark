#!/usr/bin/env python3
import argparse
import os
import numpy as np
import pandas as pd
import pyproj.datadir


# Ensure PROJ can find its database when running inside a conda environment
# without CONDA_PREFIX being set (e.g. calling python directly without conda activate).
_proj_data_candidates = [
    os.environ.get("PROJ_DATA", ""),
    os.path.join(os.path.dirname(pyproj.datadir.__file__), "proj_dir", "share", "proj"),
]
for _p in _proj_data_candidates:
    if _p and os.path.isfile(os.path.join(_p, "proj.db")):
        pyproj.datadir.set_data_dir(_p)
        break

from pyproj import Transformer


def find_col(df, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    raise ValueError(f"Missing one of columns: {candidates}. Available: {list(df.columns)}")

def load_gnss(csv_path):
    df = pd.read_csv(csv_path)
    lat_col = find_col(df, ["lat", "latitude"])
    lon_col = find_col(df, ["lon", "longitude"])
    alt_col = None
    for c in ["alt", "altitude", "height"]:
        if c in df.columns:
            alt_col = c
            break

    # time column (try common names)
    t_col = None
    for c in ["t", "time", "stamp", "timestamp", "t_sec", "secs"]:
        if c in df.columns:
            t_col = c
            break
    if t_col is None:
        raise ValueError("GNSS csv needs a time column, e.g. t/time/stamp/timestamp (seconds).")

    t = df[t_col].to_numpy(float)
    lat = df[lat_col].to_numpy(float)
    lon = df[lon_col].to_numpy(float)
    alt = df[alt_col].to_numpy(float) if alt_col else np.zeros_like(lat)

    # make time relative to start (helps)
    t = t - t[0]

    # remove NaNs
    m = np.isfinite(t) & np.isfinite(lat) & np.isfinite(lon) & np.isfinite(alt)
    return t[m], lat[m], lon[m], alt[m]

def utm_epsg_from_latlon(lat0, lon0):
    zone = int(np.floor((lon0 + 180.0) / 6.0) + 1)
    north = lat0 >= 0
    epsg = (32600 + zone) if north else (32700 + zone)
    return epsg, zone, north

def latlon_to_utm(lat, lon, epsg):
    tr = Transformer.from_crs("epsg:4326", f"epsg:{epsg}", always_xy=True)
    e, n = tr.transform(lon, lat)
    return np.asarray(e, float), np.asarray(n, float)

def load_tum(path):
    # accepts space or comma; header optional
    df = pd.read_csv(path, sep=None, engine="python", header=None)
    if df.shape[1] < 8:
        raise ValueError(f"{path} must have at least 8 columns: t x y z qx qy qz qw")
    arr = df.iloc[:, :8].to_numpy(float)
    t = arr[:, 0]
    p = arr[:, 1:4]
    q = arr[:, 4:8]  # xyzw
    # make time relative to start
    t = t - t[0]
    return t, p, q

def interp_gnss_to_traj(t_g, E_g, N_g, U_g, t_traj):
    E = np.interp(t_traj, t_g, E_g)
    N = np.interp(t_traj, t_g, N_g)
    U = np.interp(t_traj, t_g, U_g)
    return E, N, U

def fit_2d_rigid(xy_local, EN_utm):
    """
    Find R (2x2) and t (2,) minimizing || EN - (R*xy + t) ||.
    No scale. SVD solution.
    """
    X = xy_local
    Y = EN_utm
    mx = X.mean(axis=0)
    my = Y.mean(axis=0)
    Xc = X - mx
    Yc = Y - my
    H = Xc.T @ Yc
    U, S, Vt = np.linalg.svd(H)
    R2 = Vt.T @ U.T
    if np.linalg.det(R2) < 0:
        Vt[-1, :] *= -1
        R2 = Vt.T @ U.T
    t2 = my - R2 @ mx
    return R2, t2

def apply_georef(p_local, R2, t2, z_utm=None, use_z_from_gnss=True):
    out = p_local.copy()
    xy = out[:, :2].T
    EN = (R2 @ xy).T + t2
    out[:, 0] = EN[:, 0]
    out[:, 1] = EN[:, 1]
    if use_z_from_gnss and z_utm is not None:
        out[:, 2] = z_utm
    return out

def write_tum(path, t, p, q):
    # evo expects: t x y z qx qy qz qw, space-separated, no header
    data = np.column_stack([t, p, q])
    np.savetxt(path, data, fmt="%.9f")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gnss", required=True, help="GNSS csv with time, lat, lon (and optional alt)")
    ap.add_argument("--gt", required=True, help="GT trajectory in TUM (t x y z qx qy qz qw) local")
    ap.add_argument("--traj", nargs="+", required=True, help="Other trajectories in TUM to georeference (MUSE/IEKF/IS...)")
    ap.add_argument("--out_dir", default="georef_out", help="Output directory")
    ap.add_argument("--fit_seconds", type=float, default=60.0, help="Use first N seconds to fit yaw+translation")
    ap.add_argument("--use_gnss_z", action="store_true", help="Replace z with interpolated GNSS altitude")
    args = ap.parse_args()

    import os
    os.makedirs(args.out_dir, exist_ok=True)

    # GNSS -> UTM
    t_g, lat, lon, alt = load_gnss(args.gnss)
    epsg, zone, north = utm_epsg_from_latlon(lat[0], lon[0])
    E_g, N_g = latlon_to_utm(lat, lon, epsg)
    print(f"[GNSS] EPSG:32632 expected? computed epsg:{epsg} (zone {zone}, {'N' if north else 'S'})")

    # Load GT local and fit transform using GNSS interpolated on GT timeline
    t_gt, p_gt, q_gt = load_tum(args.gt)
    E_i, N_i, U_i = interp_gnss_to_traj(t_g, E_g, N_g, alt, t_gt)

    # select fit window
    m = (t_gt >= 0.0) & (t_gt <= args.fit_seconds)
    if m.sum() < 20:
        raise ValueError("Not enough samples in fit window; increase --fit_seconds or check timestamps.")

    xy_gt = p_gt[m, :2]
    EN_gnss = np.column_stack([E_i[m], N_i[m]])
    R2, t2 = fit_2d_rigid(xy_gt, EN_gnss)
    yaw = np.degrees(np.arctan2(R2[1,0], R2[0,0]))
    print(f"[fit] estimated yaw (deg): {yaw:.3f}")
    print(f"[fit] translation (E,N): {t2}")

    # Georeference GT
    p_gt_geo = apply_georef(p_gt, R2, t2, z_utm=U_i if args.use_gnss_z else None, use_z_from_gnss=args.use_gnss_z)
    gt_out = os.path.join(args.out_dir, "gt_georef.tum")
    write_tum(gt_out, t_gt, p_gt_geo, q_gt)

    # Georeference each estimator using SAME (R2,t2)
    for tr in args.traj:
        name = os.path.splitext(os.path.basename(tr))[0]
        t, p, q = load_tum(tr)
        # interpolate GNSS altitude on this timeline if requested
        E_t, N_t, U_t = interp_gnss_to_traj(t_g, E_g, N_g, alt, t)
        p_geo = apply_georef(p, R2, t2, z_utm=U_t if args.use_gnss_z else None, use_z_from_gnss=args.use_gnss_z)
        out = os.path.join(args.out_dir, f"{name}_georef.tum")
        write_tum(out, t, p_geo, q)

    print(f"\nDone. Output in: {args.out_dir}")
    print(f"Map tile EPSG to use: epsg:{epsg}")

if __name__ == "__main__":
    main()