#!/usr/bin/env python3
"""
Barometer thrust compensation calibration tool.

Identifies optimal EKF2_PCOEF_THR and EKF2_PCOEF_TTAU parameters by comparing
barometer altitude against a range sensor (ground truth) during hover flight.

The tool fits a first-order lag model to the thrust-induced baro error:

    baro_error = K * LPF(thrust, tau) + c

where K is the static gain (meters per unit thrust) and tau is the filter time
constant (seconds). The EKF applies: baro_alt += EKF2_PCOEF_THR * filtered_thrust
to cancel this error, so EKF2_PCOEF_THR = -K.

Requirements:
    - Flight log (.ulg) from a hover flight with a range sensor (lidar/sonar)
    - Python packages: pyulog, numpy, scipy, matplotlib

Usage:
    python3 baro_thrust_calibration.py <log.ulg> [--output-dir <dir>]

Outputs:
    - baro_calibration.pdf    Calibration plots
    - Console output with recommended EKF2_PCOEF_THR / EKF2_PCOEF_TTAU values
"""

import argparse
import os
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    print("Error: pyulog not installed. Run: pip install pyulog", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
except ImportError:
    print("Error: matplotlib not installed. Run: pip install matplotlib", file=sys.stderr)
    sys.exit(1)

try:
    from scipy import signal as scipy_signal  # noqa: F401 — available for future use
except ImportError:
    print("Error: scipy not installed. Run: pip install scipy", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# ULog helpers
# ---------------------------------------------------------------------------

def get_topic(ulog, topic_name, multi_id=0):
    """Return the first matching dataset for a topic name and multi_id."""
    for d in ulog.data_list:
        if d.name == topic_name and d.multi_id == multi_id:
            return d
    return None


def get_param(ulog, name, default=None):
    """Get an initial parameter value from the log."""
    return ulog.initial_parameters.get(name, default)


def us_to_seconds(ts_us, start_us):
    """Convert microsecond timestamps to seconds relative to log start."""
    return (ts_us.astype(np.int64) - np.int64(start_us)) / 1e6


# ---------------------------------------------------------------------------
# Flight phase detection
# ---------------------------------------------------------------------------

def detect_armed_period(ulog):
    """Detect armed start/end times from vehicle_status or actuator_motors."""
    start_us = ulog.start_timestamp

    vstatus = get_topic(ulog, "vehicle_status")
    if vstatus is not None and "arming_state" in vstatus.data:
        ts = us_to_seconds(vstatus.data["timestamp"], start_us)
        armed_idx = np.where(vstatus.data["arming_state"] == 2)[0]
        if len(armed_idx) > 0:
            return ts[armed_idx[0]], ts[armed_idx[-1]]

    # Fallback: look for motor activity
    motors = get_topic(ulog, "actuator_motors")
    if motors is not None:
        ts = us_to_seconds(motors.data["timestamp"], start_us)
        active = np.zeros(len(ts), dtype=bool)
        for i in range(12):
            key = f"control[{i}]"
            if key in motors.data:
                active |= (motors.data[key] > 0.05)
        active_idx = np.where(active)[0]
        if len(active_idx) > 0:
            return ts[active_idx[0]], ts[active_idx[-1]]

    # Last resort: full log duration
    return 0.0, (ulog.last_timestamp - start_us) / 1e6


def find_hover_segment(armed_start, armed_end):
    """Return middle 60% of armed time as the hover analysis window."""
    duration = armed_end - armed_start
    return armed_start + duration * 0.2, armed_end - duration * 0.2


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------

def extract_baro(ulog):
    """Extract barometer altitude from vehicle_air_data."""
    start_us = ulog.start_timestamp
    vad = get_topic(ulog, "vehicle_air_data")
    if vad is None:
        return None
    return {
        "time_s": us_to_seconds(vad.data["timestamp"], start_us),
        "alt_m": vad.data["baro_alt_meter"],
    }


def extract_range(ulog):
    """Extract range sensor distance from distance_sensor."""
    start_us = ulog.start_timestamp
    dist = get_topic(ulog, "distance_sensor")
    if dist is None:
        return None
    return {
        "time_s": us_to_seconds(dist.data["timestamp"], start_us),
        "distance_m": dist.data["current_distance"],
    }


def extract_thrust(ulog):
    """Extract vertical thrust setpoint from vehicle_thrust_setpoint."""
    start_us = ulog.start_timestamp
    thr = get_topic(ulog, "vehicle_thrust_setpoint")
    if thr is None:
        return None
    return {
        "time_s": us_to_seconds(thr.data["timestamp"], start_us),
        "thrust_z": thr.data["xyz[2]"],
    }


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def compute_baro_error(baro, range_data, armed_start):
    """Compute baro altitude error relative to range sensor.

    Both signals are aligned to a common time base (range sensor timestamps).
    Baro altitude is zeroed at arm time so it represents change from ground.

    Returns: dict with time_s, baro_error (baro - range, positive = baro reads
    higher than truth), and the interpolated baro/range arrays.
    """
    baro_t, baro_alt = baro["time_s"], baro["alt_m"]
    rng_t, rng_dist = range_data["time_s"], range_data["distance_m"]

    # Zero baro at arm time
    arm_baro = np.interp(armed_start, baro_t, baro_alt)
    baro_zeroed = baro_alt - arm_baro

    # Interpolate baro onto range sensor time base
    baro_interp = np.interp(rng_t, baro_t, baro_zeroed)
    error = baro_interp - rng_dist

    return {
        "time_s": rng_t,
        "error": error,
        "baro_alt": baro_interp,
        "range_alt": rng_dist,
        "baro_full_time_s": baro_t,
        "baro_full_alt": baro_zeroed,
    }


def first_order_lpf(signal, time_s, tau):
    """Apply a causal first-order low-pass filter with time constant tau.

    Matches the AlphaFilter used in the EKF: alpha = dt / (dt + tau).
    When tau=0, returns the input unchanged.
    """
    if tau <= 0 or len(signal) < 2:
        return signal.copy()

    out = np.empty_like(signal)
    out[0] = signal[0]
    for i in range(1, len(signal)):
        dt = time_s[i] - time_s[i - 1]
        if dt <= 0:
            out[i] = out[i - 1]
            continue
        alpha = dt / (dt + tau)
        out[i] = out[i - 1] + alpha * (signal[i] - out[i - 1])
    return out


def calibrate(baro_err, thrust_data, hover_start, hover_end):
    """System identification for thrust-based baro compensation.

    Fits: baro_error = K * LPF(thrust_magnitude, tau) + c

    Sweeps over tau candidates and picks the one maximizing R^2.
    Also computes cross-correlation to visualize the delay structure.

    Returns dict with identified K, tau, cross-correlation data, sweep
    results, and the recommended EKF parameters.
    """
    err_t = baro_err["time_s"]
    err = baro_err["error"]

    # Restrict to hover segment
    hov = (err_t >= hover_start) & (err_t <= hover_end)
    if hov.sum() < 20:
        return None

    err_t_hov = err_t[hov]
    err_hov = err[hov]

    # Interpolate thrust onto error timestamps and convert to upward magnitude
    thrust_raw = np.interp(err_t_hov, thrust_data["time_s"],
                           thrust_data["thrust_z"])
    thrust_mag = np.clip(-thrust_raw, 0, 1)

    # Detrend error (remove slow thermal / bias drift)
    err_detrended = err_hov - np.polyval(
        np.polyfit(err_t_hov, err_hov, 1), err_t_hov)
    thrust_detrended = thrust_mag - np.mean(thrust_mag)

    err_var = np.var(err_detrended)
    if err_var < 1e-10:
        return None

    result = {
        "hover_time": err_t_hov,
        "hover_error": err_hov,
        "thrust_mag": thrust_mag,
    }

    # --- Cross-correlation ---
    dt_median = np.median(np.diff(err_t_hov))
    if dt_median > 0:
        max_lag = min(int(2.0 / dt_median), len(err_detrended) // 2)
        if max_lag > 5:
            xcorr = np.correlate(err_detrended, thrust_detrended, "full")
            mid = len(thrust_detrended) - 1
            lags = (np.arange(len(xcorr)) - mid) * dt_median
            norm = np.sqrt(np.sum(err_detrended**2) *
                           np.sum(thrust_detrended**2))
            if norm > 0:
                xcorr = xcorr / norm
            window = (lags >= -2.0) & (lags <= 2.0)
            result["xcorr_lags_s"] = lags[window]
            result["xcorr_values"] = xcorr[window]
            peak_idx = np.argmax(np.abs(xcorr[window]))
            result["xcorr_peak_lag_s"] = float(lags[window][peak_idx])
            result["xcorr_peak_value"] = float(xcorr[window][peak_idx])

    # --- Grid search over time constants ---
    tau_candidates = np.concatenate([
        [0.0],
        np.arange(0.02, 0.2, 0.02),
        np.arange(0.2, 1.01, 0.05),
    ])

    sweep_tau, sweep_r2, sweep_K, sweep_rmse = [], [], [], []

    for tau in tau_candidates:
        thrust_filt = first_order_lpf(thrust_mag, err_t_hov, tau)
        thrust_filt_centered = thrust_filt - np.mean(thrust_filt)

        if np.var(thrust_filt_centered) < 1e-10:
            continue

        # Least-squares: err_detrended = K * thrust_filt_centered
        K = np.sum(err_detrended * thrust_filt_centered) / \
            np.sum(thrust_filt_centered**2)
        residual = err_detrended - K * thrust_filt_centered
        r2 = 1.0 - np.var(residual) / err_var

        sweep_tau.append(float(tau))
        sweep_r2.append(float(r2))
        sweep_K.append(float(K))
        sweep_rmse.append(float(np.sqrt(np.mean(residual**2))))

    if not sweep_tau:
        return None

    result["sweep_tau"] = np.array(sweep_tau)
    result["sweep_r2"] = np.array(sweep_r2)
    result["sweep_K"] = np.array(sweep_K)
    result["sweep_rmse"] = np.array(sweep_rmse)

    # Best fit
    best_idx = int(np.argmax(sweep_r2))
    result["best_tau"] = sweep_tau[best_idx]
    result["best_K"] = sweep_K[best_idx]
    result["best_r2"] = sweep_r2[best_idx]
    result["best_rmse"] = sweep_rmse[best_idx]

    # No-lag baseline for comparison
    result["nolag_K"] = sweep_K[0] if sweep_tau[0] == 0.0 else 0.0
    result["nolag_r2"] = sweep_r2[0] if sweep_tau[0] == 0.0 else 0.0

    # Compensated timeseries (for plotting)
    best_filt = first_order_lpf(thrust_mag, err_t_hov, result["best_tau"])
    result["compensated_error"] = err_hov - result["best_K"] * best_filt

    # Recommended EKF parameters:
    #   baro_error = K * thrust  =>  to cancel, apply correction = -K * thrust
    #   EKF does: baro_alt += pcoef_thr * thrust
    #   So pcoef_thr = -K
    result["recommended_pcoef_thr"] = -result["best_K"]
    result["recommended_pcoef_ttau"] = result["best_tau"]

    return result


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_overview(baro_err, thrust_data, armed_start, armed_end,
                  hover_start, hover_end):
    """Page 1: Baro vs range altitude and baro error with thrust overlay."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Baro Altitude vs Range Sensor", fontsize=14, fontweight="bold")

    t = baro_err["time_s"]
    err = baro_err["error"]

    # Panel 1: Altitude comparison
    ax = axes[0]
    ax.plot(t, baro_err["range_alt"], label="Range sensor (ground truth)",
            color="tab:green", linewidth=1.2)
    ax.plot(baro_err["baro_full_time_s"], baro_err["baro_full_alt"],
            label="Baro alt (zeroed at arm)", color="tab:red",
            linewidth=1.2, alpha=0.85)
    ax.axvspan(armed_start, armed_end, alpha=0.04, color="green", label="Armed")
    ax.axvspan(hover_start, hover_end, alpha=0.08, color="blue",
               label="Hover window")
    ax.set_ylabel("Altitude [m]")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 2: Baro error
    ax = axes[1]
    ax.plot(t, err, color="tab:red", linewidth=0.8, label="Baro error (baro - range)")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.axvspan(hover_start, hover_end, alpha=0.08, color="blue")
    ax.set_ylabel("Baro Error [m]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 3: Thrust
    ax = axes[2]
    if thrust_data is not None:
        thr_t = thrust_data["time_s"]
        thr_mag = np.clip(-thrust_data["thrust_z"], 0, 1)
        ax.plot(thr_t, thr_mag, color="tab:orange", linewidth=0.8,
                label="Thrust magnitude")
        ax.axvspan(hover_start, hover_end, alpha=0.08, color="blue")
    ax.set_ylabel("Thrust [0-1]")
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_calibration(calib):
    """Page 2: Calibration results — xcorr, tau sweep, before/after, params."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Thrust Compensation Calibration", fontsize=14,
                 fontweight="bold")

    # Top-left: Cross-correlation
    ax = axes[0, 0]
    if "xcorr_lags_s" in calib:
        ax.plot(calib["xcorr_lags_s"] * 1000, calib["xcorr_values"],
                color="tab:blue", linewidth=1.0)
        peak_lag = calib["xcorr_peak_lag_s"]
        peak_val = calib["xcorr_peak_value"]
        ax.axvline(peak_lag * 1000, color="tab:red", linestyle="--",
                   linewidth=0.8,
                   label=f"Peak: {peak_lag*1000:.0f} ms (r={peak_val:.2f})")
        ax.axvline(0, color="k", linewidth=0.5, linestyle=":")
        ax.legend(fontsize=9)
    ax.set_xlabel("Lag [ms] (positive = error lags thrust)")
    ax.set_ylabel("Cross-correlation")
    ax.set_title("Cross-Correlation: Thrust vs Baro Error")
    ax.grid(True, alpha=0.3)

    # Top-right: R^2 vs tau
    ax = axes[0, 1]
    if "sweep_tau" in calib:
        ax.plot(calib["sweep_tau"], calib["sweep_r2"],
                "o-", color="tab:green", markersize=3, linewidth=1.0)
        ax.axvline(calib["best_tau"], color="tab:red", linestyle="--",
                   linewidth=0.8,
                   label=f"Best: tau={calib['best_tau']:.2f}s "
                         f"(R\u00b2={calib['best_r2']:.3f})")
        ax.legend(fontsize=9)
    ax.set_xlabel("Time constant tau [s]")
    ax.set_ylabel("R\u00b2")
    ax.set_title("Model Fit vs Filter Time Constant")
    ax.grid(True, alpha=0.3)

    # Bottom-left: Before/after compensation
    ax = axes[1, 0]
    if "hover_time" in calib:
        t = calib["hover_time"]
        ax.plot(t, calib["hover_error"], color="tab:red", linewidth=0.8,
                alpha=0.7, label="Raw baro error")
        ax.plot(t, calib["compensated_error"], color="tab:blue", linewidth=0.8,
                label=f"Compensated (K={calib['best_K']:.2f}, "
                      f"tau={calib['best_tau']:.2f}s)")
        ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
        ax.legend(fontsize=9)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro Error [m]")
    ax.set_title("Compensation Effect (Hover Segment)")
    ax.grid(True, alpha=0.3)

    # Bottom-right: Recommended parameters
    ax = axes[1, 1]
    ax.axis("off")
    lines = [
        "Recommended Parameters",
        "",
        f"  EKF2_PCOEF_THR  = {calib['recommended_pcoef_thr']:+.2f}  m",
        f"  EKF2_PCOEF_TTAU = {calib['recommended_pcoef_ttau']:.2f}  s",
        "",
        f"  Identified gain K  = {calib['best_K']:.3f} m/unit",
        f"  Time constant tau  = {calib['best_tau']:.3f} s",
        f"  Model R\u00b2           = {calib['best_r2']:.3f}",
        f"  Residual RMSE      = {calib['best_rmse']:.3f} m",
        "",
        f"  No-lag model R\u00b2    = {calib['nolag_r2']:.3f}",
        f"  R\u00b2 improvement     = {calib['best_r2'] - calib['nolag_r2']:.3f}",
    ]
    if "xcorr_peak_lag_s" in calib:
        lines.append(
            f"  Cross-corr peak    = {calib['xcorr_peak_lag_s']*1000:.0f} ms")
    ax.text(0.1, 0.95, "\n".join(lines), transform=ax.transAxes,
            fontsize=11, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f0f0f0",
                      edgecolor="#cccccc"))

    plt.tight_layout()
    return fig


def plot_scatter(baro_err, thrust_data, calib, hover_start, hover_end):
    """Page 3: Error vs thrust scatter — raw and compensated."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Baro Error vs Thrust (Hover Segment)", fontsize=14,
                 fontweight="bold")

    t = baro_err["time_s"]
    err = baro_err["error"]
    hov = (t >= hover_start) & (t <= hover_end)

    thrust_interp = np.interp(t[hov], thrust_data["time_s"],
                              thrust_data["thrust_z"])
    err_hov = err[hov]

    # Raw
    ax = axes[0]
    ax.scatter(thrust_interp, err_hov, s=3, alpha=0.4, color="tab:orange")
    z = np.polyfit(thrust_interp, err_hov, 1)
    x_fit = np.linspace(thrust_interp.min(), thrust_interp.max(), 50)
    ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
    r_val = float(np.corrcoef(thrust_interp, err_hov)[0, 1])
    ax.set_xlabel("Thrust Z setpoint")
    ax.set_ylabel("Baro Error [m]")
    ax.set_title(f"Raw\nr = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
    ax.grid(True, alpha=0.3)

    # Compensated
    ax = axes[1]
    thrust_mag = np.clip(-thrust_interp, 0, 1)
    thrust_filt = first_order_lpf(thrust_mag, t[hov], calib["best_tau"])
    comp_err = err_hov - calib["best_K"] * thrust_filt
    ax.scatter(thrust_interp, comp_err, s=3, alpha=0.4, color="tab:blue")
    z = np.polyfit(thrust_interp, comp_err, 1)
    ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
    r_val = float(np.corrcoef(thrust_interp, comp_err)[0, 1])
    ax.set_xlabel("Thrust Z setpoint")
    ax.set_ylabel("Compensated Error [m]")
    ax.set_title(f"After Compensation "
                 f"(PCOEF_THR={calib['recommended_pcoef_thr']:+.2f})\n"
                 f"r = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate EKF2 barometer thrust compensation parameters")
    parser.add_argument("ulog_file", help="Path to .ulg flight log")
    parser.add_argument("--output-dir", "-o", default=None,
                        help="Output directory (default: same dir as log)")
    args = parser.parse_args()

    if not os.path.isfile(args.ulog_file):
        print(f"Error: file not found: {args.ulog_file}", file=sys.stderr)
        sys.exit(1)

    output_dir = args.output_dir or os.path.dirname(
        os.path.abspath(args.ulog_file))
    os.makedirs(output_dir, exist_ok=True)

    # Load log
    print(f"Loading {args.ulog_file} ...")
    ulog = ULog(args.ulog_file)
    duration = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    print(f"  Duration: {duration:.1f} s")

    # Print relevant parameters
    print("\nParameters:")
    for p in ["EKF2_HGT_REF", "EKF2_BARO_CTRL", "EKF2_BARO_NOISE",
              "EKF2_PCOEF_THR", "EKF2_PCOEF_TTAU"]:
        val = get_param(ulog, p)
        if val is not None:
            print(f"  {p:24s} = {val}")

    # Detect flight phases
    armed_start, armed_end = detect_armed_period(ulog)
    hover_start, hover_end = find_hover_segment(armed_start, armed_end)
    print(f"\nArmed:  {armed_start:.1f}s - {armed_end:.1f}s")
    print(f"Hover:  {hover_start:.1f}s - {hover_end:.1f}s")

    # Extract data
    baro = extract_baro(ulog)
    range_data = extract_range(ulog)
    thrust_data = extract_thrust(ulog)

    if baro is None:
        print("Error: no barometer data (vehicle_air_data) in log",
              file=sys.stderr)
        sys.exit(1)
    if range_data is None:
        print("Error: no range sensor data (distance_sensor) in log.\n"
              "A range sensor is required as ground truth for calibration.",
              file=sys.stderr)
        sys.exit(1)
    if thrust_data is None:
        print("Error: no thrust data (vehicle_thrust_setpoint) in log",
              file=sys.stderr)
        sys.exit(1)

    print(f"\n  Baro samples:   {len(baro['time_s'])}")
    print(f"  Range samples:  {len(range_data['time_s'])}")
    print(f"  Thrust samples: {len(thrust_data['time_s'])}")

    # Compute baro error
    baro_err = compute_baro_error(baro, range_data, armed_start)

    hov = ((baro_err["time_s"] >= hover_start) &
           (baro_err["time_s"] <= hover_end))
    if hov.sum() < 20:
        print("Error: not enough data in hover segment", file=sys.stderr)
        sys.exit(1)

    err_hov = baro_err["error"][hov]
    print(f"\nBaro error during hover:")
    print(f"  Mean:  {np.mean(err_hov):+.3f} m")
    print(f"  Std:   {np.std(err_hov):.3f} m")

    # Thrust correlation
    thrust_interp = np.interp(baro_err["time_s"][hov],
                              thrust_data["time_s"], thrust_data["thrust_z"])
    r_thrust = float(np.corrcoef(thrust_interp, err_hov)[0, 1])
    print(f"  Thrust correlation: r = {r_thrust:+.3f}"
          f"  ({'strong' if abs(r_thrust) > 0.6 else 'moderate' if abs(r_thrust) > 0.3 else 'weak'})")

    # Calibrate
    print("\nRunning calibration ...")
    calib = calibrate(baro_err, thrust_data, hover_start, hover_end)

    if calib is None:
        print("Calibration failed: insufficient data or thrust variation.",
              file=sys.stderr)
        sys.exit(1)

    print(f"\n  Identified gain K       = {calib['best_K']:.3f} m/unit thrust")
    print(f"  Time constant tau       = {calib['best_tau']:.3f} s")
    print(f"  Model R\u00b2               = {calib['best_r2']:.3f}")
    print(f"  Residual RMSE           = {calib['best_rmse']:.3f} m")
    if "xcorr_peak_lag_s" in calib:
        print(f"  Cross-correlation peak  = {calib['xcorr_peak_lag_s']*1000:.0f} ms")

    improvement = calib["best_r2"] - calib["nolag_r2"]
    print(f"  No-lag R\u00b2              = {calib['nolag_r2']:.3f}")
    print(f"  R\u00b2 improvement from lag = {improvement:+.3f}")

    print(f"\n{'='*50}")
    print(f"  RECOMMENDED PARAMETERS")
    print(f"{'='*50}")
    print(f"  EKF2_PCOEF_THR  = {calib['recommended_pcoef_thr']:+.2f}")
    print(f"  EKF2_PCOEF_TTAU = {calib['recommended_pcoef_ttau']:.2f}")
    print(f"{'='*50}")

    if calib["best_r2"] < 0.1:
        print(f"\nNote: Low R\u00b2 ({calib['best_r2']:.3f}) suggests thrust is not "
              "the dominant baro error source. Compensation may not help much.")
    if improvement < 0.01:
        print(f"\nNote: The lag filter provides negligible improvement over "
              "tau=0. You may set EKF2_PCOEF_TTAU = 0.")

    # Generate plots
    print("\nGenerating plots ...")
    figures = [
        plot_overview(baro_err, thrust_data, armed_start, armed_end,
                      hover_start, hover_end),
        plot_calibration(calib),
        plot_scatter(baro_err, thrust_data, calib, hover_start, hover_end),
    ]

    pdf_path = os.path.join(output_dir, "baro_calibration.pdf")
    with PdfPages(pdf_path) as pdf:
        for fig in figures:
            pdf.savefig(fig)
            plt.close(fig)
    print(f"  Saved: {pdf_path}")

    print("\nDone.")


if __name__ == "__main__":
    main()
