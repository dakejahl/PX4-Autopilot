#!/usr/bin/env python3
"""Fit raw counts against GNSS translation plus gyro; export every raw frame."""
import argparse
import json
from pathlib import Path

import numpy as np


SCALE = 1 / (11.914 * 39.3701)


def yaw(degrees):
    c, s = np.cos(np.deg2rad(degrees)), np.sin(np.deg2rad(degrees))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def interpolate(t, source_t, values, valid, max_gap):
    """Reject missing/invalid brackets instead of bridging them or extrapolating."""
    source_t, index = np.unique(source_t, return_index=True)
    values, valid = values[index], valid[index]
    if len(source_t) < 2:
        raise ValueError("Reference topic needs at least two distinct timestamps")
    right = np.clip(np.searchsorted(source_t, t, side="right"), 1, len(source_t) - 1)
    left = right - 1
    gap = source_t[right] - source_t[left]
    weight = (t - source_t[left]) / gap
    result = values[left] + weight[:, None] * (values[right] - values[left])
    ok = ((t >= source_t[0]) & (t <= source_t[-1]) & (gap <= max_gap)
          & valid[left] & valid[right] & np.all(np.isfinite(result), axis=1))
    result[~ok] = np.nan
    return result


def rotation(q):
    q = q / np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q.T
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1 - 2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1 - 2*(x*x+y*y)],
    ]).transpose(2, 0, 1)


def fit_model(expected, counts, mask):
    x, y = expected[mask], counts[mask]
    if len(x) < 100 or np.linalg.matrix_rank(x) < 2:
        raise ValueError("Need at least 100 valid frames with motion on both axes")
    if np.linalg.cond(x) > 100:
        raise ValueError("Insufficient independent axis excitation for a 2x2 fit")
    return np.linalg.lstsq(x, y, rcond=None)[0].T


def rms(values):
    return float(np.sqrt(np.mean(np.sum(values**2, axis=1))))


def analyze(args):
    from pyulog import ULog
    names = ["flow_raw", "sensor_gps", "vehicle_attitude", "distance_sensor", "vehicle_land_detected"]
    log = ULog(str(args.log), names)
    data = {(d.name, d.multi_id): d.data for d in log.data_list}

    def topic(name, instance=0):
        if (name, instance) not in data:
            raise ValueError(f"Missing {name}[{instance}] in log")
        return data[name, instance]

    raw, gnss = topic("flow_raw", args.instance), topic("sensor_gps", args.gnss_instance)
    att, distance = topic("vehicle_attitude"), topic("distance_sensor", args.range_instance)
    origin = float(raw["timestamp_sample"][0])

    def seconds(values):
        return (np.asarray(values, dtype=float) - origin) * 1e-6

    def columns(d, fields):
        return np.column_stack([d[f] for f in fields]).astype(float)

    end = seconds(raw["timestamp_sample"])
    dt = raw["interval_us"].astype(float) * 1e-6
    middle = end - dt / 2
    counts = columns(raw, ["delta_x", "delta_y"])
    gyro = columns(raw, [f"gyro_integral[{i}]" for i in range(3)]) @ yaw(args.node_yaw_deg).T
    gyro_rate = np.divide(gyro, dt[:, None], out=np.full_like(gyro, np.nan), where=dt[:, None] > 0)
    lever_velocity = np.cross(gyro_rate, np.array(args.flow_minus_gnss))
    gnss_stamp = "timestamp" if args.gnss_receive_time else "timestamp_sample"
    tg = seconds(gnss[gnss_stamp]) - args.gnss_delay_ms * 1e-3
    vg = columns(gnss, ["vel_n_m_s", "vel_e_m_s", "vel_d_m_s"])
    gps_valid = ((gnss[gnss_stamp] > 0) & gnss["vel_ned_valid"].astype(bool)
                 & (gnss["fix_type"] >= 3) & (gnss["s_variance_m_s"] <= args.max_speed_error))
    q = columns(att, [f"q[{i}]" for i in range(4)])
    # q and -q represent the same orientation; unwrap before interpolation.
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0:
            q[i] *= -1
    ta = seconds(att["timestamp_sample"])
    td = seconds(distance["timestamp"])
    ranges = distance["current_distance"].astype(float)
    range_valid = ((distance["signal_quality"] != 0) & (distance["orientation"] == 25)
                   & (ranges >= distance["min_distance"]) & (ranges <= distance["max_distance"]))

    def reference(t):
        velocity = interpolate(t, tg, vg, gps_valid, args.max_gnss_gap)
        r = rotation(interpolate(t, ta, q, np.ones(len(q), dtype=bool), 0.05))
        body = np.einsum("nji,nj->ni", r, velocity)
        depth = interpolate(t, td, ranges[:, None], range_valid, args.max_range_gap)[:, 0]
        camera = body + lever_velocity
        translation = np.column_stack([-camera[:, 1], camera[:, 0]]) / depth[:, None]
        return translation, body, velocity, r, depth

    base = (raw["timestamp_sample_valid"].astype(bool) & (dt > 0) & (dt < 0.05)
            & (raw["gyro_samples"] > 0) & np.all(np.isfinite(gyro), axis=1)
            & ((raw["observation"] & 0x3f) == 0x3f) & ((raw["observation"] >> 6) < 3)
            & (raw["squal_raw"] >= args.squal_min))
    if not args.include_ground:
        landed = topic("vehicle_land_detected")
        tl = seconds(landed["timestamp"])
        i = np.clip(np.searchsorted(tl, middle, side="right") - 1, 0, len(tl)-1)
        base &= (middle >= tl[0]) & ~landed["landed"][i].astype(bool)

    # Reserve whole one-second blocks; never select timing on held-out residuals.
    training = (np.floor(middle - np.min(middle)).astype(int) % 3) != 2
    candidates = []
    common = base.copy()
    for delay_ms in np.arange(-args.scan_ms, args.scan_ms + 0.5, 1):
        shift = delay_ms * 1e-3
        a, *_ = reference(end - dt + shift)
        b, *_ = reference(middle + shift)
        c, *_ = reference(end + shift)
        expected = (a + 4*b + c) * (dt[:, None] / 6) + gyro[:, :2]
        common &= np.all(np.isfinite(expected), axis=1)
        candidates.append((delay_ms, expected))
    if np.count_nonzero(common & ~training) < 100:
        raise ValueError("Need at least 100 held-out frames with valid time, gyro, GNSS and range")
    fits = []
    for delay_ms, expected in candidates:
        model = fit_model(expected, counts, common & training)
        error = counts - expected @ model.T
        fits.append((rms(error[common & training]), delay_ms, model, expected))
    _, shift_ms, model, expected = min(fits, key=lambda candidate: candidate[0])
    if np.linalg.cond(model) > 100:
        raise ValueError("Fitted camera map is nearly singular; inspect raw residuals before velocity inversion")
    _, _, velocity, r, depth = reference(middle + shift_ms * 1e-3)
    baseline = yaw(args.raw_yaw_deg)[:2, :2] * SCALE
    fitted = np.linalg.inv(model)
    residual = counts - expected @ model.T

    def flow_velocity(counts_to_radians):
        translation = (counts @ counts_to_radians.T - gyro[:, :2]) / dt[:, None]
        xy = np.column_stack([translation[:, 1], -translation[:, 0]]) * depth[:, None]
        xy -= lever_velocity[:, :2]
        # Solve horizontal NED velocity with GNSS vertical velocity held fixed.
        rt = r.transpose(0, 2, 1)
        rhs = xy - rt[:, :2, 2] * velocity[:, 2, None]
        determinant = np.linalg.det(rt[:, :2, :2])
        result = np.full_like(xy, np.nan)
        ok = np.isfinite(determinant) & (np.abs(determinant) > 0.2)
        result[ok] = np.linalg.solve(rt[ok, :2, :2], rhs[ok, :, None])[:, :, 0]
        return result

    measured, corrected = flow_velocity(baseline), flow_velocity(fitted)
    held = common & ~training & np.all(np.isfinite(corrected), axis=1)
    if np.count_nonzero(held) < 100:
        raise ValueError("Too few held-out frames with usable attitude geometry")
    summary = {
        "frames": len(end), "used": int(common.sum()), "held_out": int(held.sum()),
        "timestamp_unmapped": int((~raw["timestamp_sample_valid"].astype(bool)).sum()),
        "gyro_incomplete": int((raw["gyro_samples"] == 0).sum()),
        "counter_discontinuities": int((np.diff(raw["frame_counter"].astype(np.int64)) != 1).sum()),
        "reference_shift_ms": float(shift_ms), "shift_at_search_boundary": bool(abs(shift_ms) == args.scan_ms),
        "counts_per_radian_matrix": model.tolist(), "radians_per_count_matrix": fitted.tolist(),
        "held_out_count_residual_rms": rms(residual[held]),
        "held_out_baseline_velocity_rms_m_s": rms((measured-velocity[:, :2])[held]),
        "held_out_fitted_velocity_rms_m_s": rms((corrected-velocity[:, :2])[held]),
        "held_out_fitted_north_east_rms_m_s": np.sqrt(np.mean((corrected-velocity[:, :2])[held]**2, axis=0)).tolist(),
        "settings": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
    }
    table = dict(time_s=middle, used=common, held_out=held, frame_counter=raw["frame_counter"],
                 interval_us=raw["interval_us"], shutter=raw["shutter"], squal=raw["squal_raw"],
                 raw_data_sum=raw["raw_data_sum"], mode=raw["observation"] >> 6,
                 challenging_surface=(raw["motion"] & 1) != 0,
                 nominal_expected_counts=np.linalg.norm(expected @ np.linalg.inv(baseline).T, axis=1),
                 delta_x=counts[:, 0], delta_y=counts[:, 1],
                 expected_x=(expected @ model.T)[:, 0], expected_y=(expected @ model.T)[:, 1],
                 residual_counts=np.linalg.norm(residual, axis=1),
                 gnss_north=velocity[:, 0], gnss_east=velocity[:, 1],
                 baseline_north=measured[:, 0], baseline_east=measured[:, 1],
                 fitted_north=corrected[:, 0], fitted_east=corrected[:, 1])
    for field, values in raw.items():
        table.setdefault(field, values)
    args.output.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output / "frames.csv", np.column_stack(list(table.values())),
               delimiter=",", header=",".join(table), comments="")
    (args.output / "fit.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    for axis, label in enumerate(("north", "east")):
        axes[0, axis].plot(middle[held], velocity[held, axis], label="GNSS", linewidth=1)
        axes[0, axis].plot(middle[held], measured[held, axis], label="baseline", alpha=.5, linewidth=.6)
        axes[0, axis].plot(middle[held], corrected[held, axis], label="fit", alpha=.7, linewidth=.6)
        axes[0, axis].set(xlabel="Time (s)", ylabel=f"{label} velocity (m/s)")
        axes[0, axis].legend()
    for ax, field in zip(axes[1:].flat, ("shutter", "squal", "raw_data_sum", "nominal_expected_counts")):
        ax.scatter(table[field][common], table["residual_counts"][common], s=2, alpha=.2,
                   c=table["challenging_surface"][common])
        ax.set(xlabel=field, ylabel="Count residual magnitude")
    fig.tight_layout()
    fig.savefig(args.output / "comparison.png", dpi=150)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--gnss-instance", type=int, default=0)
    parser.add_argument("--range-instance", type=int, default=0)
    parser.add_argument("--node-yaw-deg", type=float, required=True, help="Raw gyro board FRD to vehicle FRD")
    parser.add_argument("--raw-yaw-deg", type=float, required=True, help="Total raw counts to vehicle rotation in the baseline")
    parser.add_argument("--flow-minus-gnss", nargs=3, type=float, default=[0., 0., 0.], metavar=("X", "Y", "Z"), help="Flow minus antenna position in vehicle FRD metres")
    parser.add_argument("--gnss-receive-time", action="store_true", help="Use receive timestamp for logs without GNSS sample timing")
    parser.add_argument("--gnss-delay-ms", type=float, default=0., help="Subtract from selected GNSS timestamps")
    parser.add_argument("--scan-ms", type=int, default=30, help="Search GNSS/attitude/range reference shift, +/- milliseconds")
    parser.add_argument("--squal-min", type=int, default=1)
    parser.add_argument("--max-speed-error", type=float, default=.5)
    parser.add_argument("--max-gnss-gap", type=float, default=.3)
    parser.add_argument("--max-range-gap", type=float, default=.1)
    parser.add_argument("--include-ground", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.scan_ms <= 200:
        parser.error("--scan-ms must be between 0 and 200")
    try:
        analyze(args)
    except (ValueError, KeyError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
