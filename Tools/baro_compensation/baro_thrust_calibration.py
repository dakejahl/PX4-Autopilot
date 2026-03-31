#!/usr/bin/env python3
"""
Barometer thrust compensation analysis tool.

Analyzes baro thrust compensation from flight logs. Three modes:

1. **Estimator review** (baro_thrust_estimate logged, no range sensor):
   Shows online estimator convergence, K identification, residual analysis.

2. **Full validation** (baro_thrust_estimate + range sensor):
   Cross-validates online estimator results against range-sensor ground truth.

3. **Standalone calibration** (range sensor, no estimator):
   Identifies SENS_BARO_PCOEF from baro vs range error.

The correction model: baro_alt += SENS_BARO_PCOEF * |thrust_z|

Requirements:
    - Flight log (.ulg)
    - Python packages: pyulog, numpy, matplotlib

Usage:
    python3 baro_thrust_calibration.py <log.ulg> [--output-dir <dir>]
        If --output-dir is not given, results are written to logs/<log_name>/ in the PX4 root.

Outputs:
    - <log_name>.pdf    Analysis plots (saved in the output directory)
    - Console output with parameters and diagnostics
"""

import argparse
import io
import os
import shutil
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


# ---------------------------------------------------------------------------
# Layout constants
# ---------------------------------------------------------------------------

_SUBTITLE_X = 0.5
_SUBTITLE_Y = 0.94
_LAYOUT_TOP = 0.93  # top of plot area, just below subtitle

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


def thrust_z_magnitude(thrust_sp_data):
    """Compute vertical thrust magnitude [0, 1] from vehicle_thrust_setpoint.

    Uses |xyz[2]| which is the Z body-axis component (negative = upward in FRD).
    This matches the firmware compensation signal in VehicleAirData.
    """
    z = thrust_sp_data.get("xyz[2]", None)
    if z is None:
        return np.zeros(len(next(iter(thrust_sp_data.values()))))
    return np.where(np.isfinite(z), np.abs(z), 0.0)


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
    """Return the full armed period as the analysis window.

    Range-based analysis filters by minimum altitude separately
    to exclude ground proximity, so no time trimming needed here.
    """
    return armed_start, armed_end


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------

def extract_baro(ulog):
    """Extract barometer altitude from vehicle_air_data.baro_alt_meter."""
    start_us = ulog.start_timestamp
    vad = get_topic(ulog, "vehicle_air_data")
    if vad is None:
        return None
    return {
        "time_s": us_to_seconds(vad.data["timestamp_sample"], start_us),
        "alt_m": vad.data["baro_alt_meter"],
    }


def extract_ekf_baro_obs(ulog):
    """Extract baro observation from estimator_aid_src_baro_hgt (NED)."""
    start_us = ulog.start_timestamp
    d = get_topic(ulog, "estimator_aid_src_baro_hgt")
    if d is None:
        return None
    return {
        "time_s": us_to_seconds(d.data["timestamp"], start_us),
        "observation": d.data["observation"],
    }


def extract_range(ulog):
    """Extract range sensor distance from distance_sensor.

    Filters out non-finite distances and samples with signal_quality == 0.
    """
    start_us = ulog.start_timestamp
    dist = get_topic(ulog, "distance_sensor")
    if dist is None:
        return None

    time_s = us_to_seconds(dist.data["timestamp"], start_us)
    distance_m = dist.data["current_distance"]

    valid = np.isfinite(time_s) & np.isfinite(distance_m)
    if "signal_quality" in dist.data.dtype.names:
        valid &= dist.data["signal_quality"] > 0

    return {
        "time_s": time_s[valid],
        "distance_m": distance_m[valid],
    }


def extract_thrust(ulog):
    """Extract vertical thrust magnitude from vehicle_thrust_setpoint."""
    start_us = ulog.start_timestamp
    thrust_sp = get_topic(ulog, "vehicle_thrust_setpoint")
    if thrust_sp is None:
        return None
    return {
        "time_s": us_to_seconds(thrust_sp.data["timestamp"], start_us),
        "thrust": thrust_z_magnitude(thrust_sp.data),
    }


def extract_estimator(ulog):
    """Extract online estimator state from baro_thrust_estimate."""
    start_us = ulog.start_timestamp
    est = get_topic(ulog, "baro_thrust_estimate")
    if est is None:
        return None
    result = {
        "time_s": us_to_seconds(est.data["timestamp"], start_us),
        "residual": est.data["residual"],
        "k_estimate": est.data["k_estimate"],
        "k_estimate_var": est.data["k_estimate_var"],
        "error_var": est.data.get("error_var",
                        np.zeros(len(est.data["timestamp"]))),
        "thrust_std": est.data["thrust_std"],
        "converged": est.data["converged"],
        "estimation_active": est.data["estimation_active"],
    }

    return result


def extract_ekf_z(ulog):
    """Extract EKF Z position from vehicle_local_position."""
    start_us = ulog.start_timestamp
    d = get_topic(ulog, "vehicle_local_position")
    if d is None:
        return None
    return {
        "time_s": us_to_seconds(d.data["timestamp"], start_us),
        "z": d.data["z"],
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


def calibrate(baro_err, thrust_data, hover_start, hover_end):
    """System identification for thrust-based baro compensation.

    Fits: baro_error = K * motor_thrust + c
    using 2-parameter least squares. The intercept c absorbs any
    constant baro/range offset.

    Returns dict with identified K, R^2, and recommended PCOEF.
    """
    err_t = baro_err["time_s"]
    err = baro_err["error"]

    # Restrict to analysis window, excluding low altitude (ground proximity)
    MIN_RANGE_M = 0.5
    hov = ((err_t >= hover_start) & (err_t <= hover_end)
           & (baro_err["range_alt"] > MIN_RANGE_M))
    if hov.sum() < 20:
        return None

    err_t_hov = err_t[hov]
    err_hov = err[hov]

    # Interpolate motor thrust onto error timestamps
    thrust_mag = np.interp(err_t_hov, thrust_data["time_s"],
                           thrust_data["thrust"])

    err_var = np.var(err_hov)
    if err_var < 1e-10:
        return None

    # 2-parameter least squares: err = K * thrust + c
    A = np.column_stack([thrust_mag, np.ones(len(thrust_mag))])
    coeffs, _, _, _ = np.linalg.lstsq(A, err_hov, rcond=None)
    K, c = float(coeffs[0]), float(coeffs[1])
    residual = err_hov - (K * thrust_mag + c)
    r2 = 1.0 - np.var(residual) / err_var
    rmse = float(np.sqrt(np.mean(residual**2)))

    result = {
        "hover_time": err_t_hov,
        "hover_error": err_hov,
        "thrust_mag": thrust_mag,
        "best_K": K,
        "best_r2": r2,
        "best_rmse": rmse,
        "recommended_pcoef": -K,
    }

    return result


def estimate_open_loop_delay(baro_err, thrust_data, hover_start, hover_end):
    """Estimate physical motor-to-baro delay, rejecting closed-loop feedback.

    The naive cross-correlation between baro error and motor output includes
    feedback artifacts: baro_error -> EKF -> controller -> motor.  This path
    operates below the altitude controller bandwidth (~1-2 Hz).

    By high-pass filtering both signals (subtracting a smoothed version),
    we isolate motor variation above the controller bandwidth — mostly
    pilot input and attitude controller, not altitude feedback.  The
    cross-correlation of these high-passed signals gives the true
    open-loop physical delay (motor -> pressure -> baro integration).

    Returns dict with open-loop delay estimate, or None if insufficient data.
    """
    err_t = baro_err["time_s"]
    err = baro_err["error"]

    MIN_RANGE_M = 0.5
    hov = ((err_t >= hover_start) & (err_t <= hover_end)
           & (baro_err["range_alt"] > MIN_RANGE_M))
    if hov.sum() < 100:
        return None

    err_t_hov = err_t[hov]
    err_hov = err[hov]
    dt_median = float(np.median(np.diff(err_t_hov)))

    if dt_median <= 0:
        return None

    # Interpolate motor to error timestamps
    thrust_hov = np.interp(err_t_hov, thrust_data["time_s"],
                           thrust_data["thrust"])

    # High-pass filter: subtract moving average to remove feedback loop.
    # 500ms window ≈ 2Hz cutoff, above altitude controller bandwidth.
    window = max(3, int(round(0.5 / dt_median)))
    if window % 2 == 0:
        window += 1  # odd for symmetric window

    def highpass(sig):
        # Centered moving average (causal padding to avoid edge shift)
        kernel = np.ones(window) / window
        smoothed = np.convolve(sig, kernel, mode="same")
        return sig - smoothed

    err_hp = highpass(err_hov)
    thr_hp = highpass(thrust_hov)

    # Check that high-pass signals have meaningful variance
    if np.std(err_hp) < 1e-6 or np.std(thr_hp) < 1e-6:
        return None

    result = {}

    # Cross-correlation of high-passed signals
    err_c = err_hp - np.mean(err_hp)
    thr_c = thr_hp - np.mean(thr_hp)
    xcorr = np.correlate(err_c, thr_c, "full")
    mid = len(thr_c) - 1
    lags = (np.arange(len(xcorr)) - mid) * dt_median
    norm = np.sqrt(np.sum(err_c**2) * np.sum(thr_c**2))
    if norm > 0:
        xcorr = xcorr / norm

    # Focus on ±500ms window
    window_mask = (lags >= -0.5) & (lags <= 0.5)
    lags_w = lags[window_mask]
    xcorr_w = xcorr[window_mask]
    peak_idx = np.argmax(np.abs(xcorr_w))

    result["lags_s"] = lags_w
    result["xcorr"] = xcorr_w
    result["peak_lag_s"] = float(lags_w[peak_idx])
    result["peak_r"] = float(xcorr_w[peak_idx])
    result["hp_window_ms"] = window * dt_median * 1000

    # Lag sweep R² on high-passed signals
    max_lag = 0.3
    lag_steps = np.arange(-max_lag, max_lag + dt_median, dt_median)
    lag_r2 = np.zeros(len(lag_steps))

    err_hp_var = np.var(err_hp)
    for i, lag in enumerate(lag_steps):
        thr_shifted = highpass(
            np.interp(err_t_hov + lag, thrust_data["time_s"],
                      thrust_data["thrust"]))
        A = np.column_stack([thr_shifted, np.ones(len(thr_shifted))])
        c, _, _, _ = np.linalg.lstsq(A, err_hp, rcond=None)
        resid = err_hp - A @ c
        lag_r2[i] = 1.0 - np.var(resid) / err_hp_var

    best_idx = np.argmax(lag_r2)
    result["lag_steps_s"] = lag_steps
    result["lag_r2"] = lag_r2
    result["best_lag_s"] = float(lag_steps[best_idx])
    result["best_lag_r2"] = float(lag_r2[best_idx])

    zero_idx = np.argmin(np.abs(lag_steps))
    result["zero_lag_r2"] = float(lag_r2[zero_idx])

    return result


def diagnose_residual(baro_err, thrust_data, hover_start, hover_end):
    """Diagnose residual thrust-baro correlation after linear compensation.

    Tests two hypotheses for why a linear K*thrust model leaves residual
    correlation:
      1. Time delay — propwash takes time to reach the baro sensor
      2. Nonlinearity — pressure scales with thrust^2 or similar

    Returns dict with cross-correlation, lag-sweep R², and quadratic fit
    results, or None if insufficient data.
    """
    err_t = baro_err["time_s"]
    err = baro_err["error"]

    MIN_RANGE_M = 0.5
    hov = ((err_t >= hover_start) & (err_t <= hover_end)
           & (baro_err["range_alt"] > MIN_RANGE_M))
    if hov.sum() < 50:
        return None

    err_t_hov = err_t[hov]
    err_hov = err[hov]
    thrust_mag = np.interp(err_t_hov, thrust_data["time_s"],
                           thrust_data["thrust"])

    err_var = np.var(err_hov)
    if err_var < 1e-10:
        return None

    result = {}

    # --- Cross-correlation ---
    err_centered = err_hov - np.mean(err_hov)
    thrust_centered = thrust_mag - np.mean(thrust_mag)
    dt_median = float(np.median(np.diff(err_t_hov)))

    if dt_median > 0 and len(err_centered) > 20:
        xcorr = np.correlate(err_centered, thrust_centered, "full")
        mid = len(thrust_centered) - 1
        lags = (np.arange(len(xcorr)) - mid) * dt_median
        norm = np.sqrt(np.sum(err_centered**2) * np.sum(thrust_centered**2))
        if norm > 0:
            xcorr = xcorr / norm
        window = (lags >= -2.0) & (lags <= 2.0)
        result["xcorr_lags_s"] = lags[window]
        result["xcorr_values"] = xcorr[window]
        peak_idx = np.argmax(np.abs(xcorr[window]))
        result["xcorr_peak_lag_s"] = float(lags[window][peak_idx])
        result["xcorr_peak_value"] = float(xcorr[window][peak_idx])

    # --- Lag sweep: fit K at various time shifts, track R² ---
    max_lag_s = 1.0
    lag_steps = np.arange(-max_lag_s, max_lag_s + dt_median, dt_median)
    lag_r2 = np.zeros(len(lag_steps))
    lag_K = np.zeros(len(lag_steps))

    for i, lag in enumerate(lag_steps):
        thrust_shifted = np.interp(err_t_hov + lag, thrust_data["time_s"],
                                   thrust_data["thrust"])
        A = np.column_stack([thrust_shifted, np.ones(len(thrust_shifted))])
        coeffs, _, _, _ = np.linalg.lstsq(A, err_hov, rcond=None)
        resid = err_hov - A @ coeffs
        lag_r2[i] = 1.0 - np.var(resid) / err_var
        lag_K[i] = float(coeffs[0])

    best_lag_idx = np.argmax(lag_r2)
    result["lag_steps_s"] = lag_steps
    result["lag_r2"] = lag_r2
    result["lag_K"] = lag_K
    result["best_lag_s"] = float(lag_steps[best_lag_idx])
    result["best_lag_r2"] = float(lag_r2[best_lag_idx])
    result["best_lag_K"] = float(lag_K[best_lag_idx])

    # Zero-lag R² for comparison
    zero_idx = np.argmin(np.abs(lag_steps))
    result["zero_lag_r2"] = float(lag_r2[zero_idx])

    # --- Quadratic fit: err = a*thrust^2 + b*thrust + c ---
    A_quad = np.column_stack([thrust_mag**2, thrust_mag, np.ones(len(thrust_mag))])
    coeffs_quad, _, _, _ = np.linalg.lstsq(A_quad, err_hov, rcond=None)
    resid_quad = err_hov - A_quad @ coeffs_quad
    result["quad_r2"] = float(1.0 - np.var(resid_quad) / err_var)
    result["quad_coeffs"] = [float(c) for c in coeffs_quad]  # [a, b, c]

    # Linear R² for comparison
    A_lin = np.column_stack([thrust_mag, np.ones(len(thrust_mag))])
    coeffs_lin, _, _, _ = np.linalg.lstsq(A_lin, err_hov, rcond=None)
    resid_lin = err_hov - A_lin @ coeffs_lin
    result["linear_r2"] = float(1.0 - np.var(resid_lin) / err_var)

    # --- Combined: lag + quadratic ---
    if result["best_lag_s"] != 0:
        thrust_best_lag = np.interp(err_t_hov + result["best_lag_s"],
                                    thrust_data["time_s"],
                                    thrust_data["thrust"])
        A_comb = np.column_stack([thrust_best_lag**2, thrust_best_lag,
                                  np.ones(len(thrust_best_lag))])
        coeffs_comb, _, _, _ = np.linalg.lstsq(A_comb, err_hov, rcond=None)
        resid_comb = err_hov - A_comb @ coeffs_comb
        result["combined_r2"] = float(1.0 - np.var(resid_comb) / err_var)
    else:
        result["combined_r2"] = result["quad_r2"]

    result["thrust_mag"] = thrust_mag
    result["err_hov"] = err_hov
    result["err_t_hov"] = err_t_hov

    return result


def plot_residual_diagnostics(diag, calib):
    """Diagnostic page: cross-correlation, lag sweep, and nonlinearity test."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Residual Correlation Diagnostics",
                 fontsize=14, fontweight="bold")
    fig.text(_SUBTITLE_X, _SUBTITLE_Y,
             "Why does residual correlation remain after linear compensation? "
             "Left: time delay analysis. Right: nonlinearity analysis.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    # Top-left: Cross-correlation
    ax = axes[0, 0]
    if "xcorr_lags_s" in diag:
        ax.plot(diag["xcorr_lags_s"] * 1000, diag["xcorr_values"],
                color="tab:blue", linewidth=1.0)
        peak_lag = diag["xcorr_peak_lag_s"]
        peak_val = diag["xcorr_peak_value"]
        ax.axvline(peak_lag * 1000, color="tab:red", linestyle="--",
                   linewidth=0.8,
                   label=f"Peak: {peak_lag*1000:+.0f}ms (r={peak_val:.3f})")
        ax.axvline(0, color="k", linewidth=0.5, linestyle=":")
    ax.set_xlabel("Lag [ms]  (positive = thrust leads error)")
    ax.set_ylabel("Normalized cross-correlation")
    ax.set_title("Cross-Correlation: Baro Error vs Thrust")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Bottom-left: Lag sweep R²
    ax = axes[1, 0]
    ax.plot(diag["lag_steps_s"] * 1000, diag["lag_r2"],
            color="tab:blue", linewidth=1.0)
    best_lag = diag["best_lag_s"]
    best_r2 = diag["best_lag_r2"]
    zero_r2 = diag["zero_lag_r2"]
    ax.axvline(best_lag * 1000, color="tab:red", linestyle="--", linewidth=0.8,
               label=f"Best lag: {best_lag*1000:+.0f}ms "
                     f"(R\u00b2={best_r2:.3f}, K={diag['best_lag_K']:.1f})")
    ax.axvline(0, color="k", linewidth=0.5, linestyle=":")
    ax.axhline(zero_r2, color="tab:gray", linestyle=":", linewidth=0.8,
               label=f"Zero lag R\u00b2={zero_r2:.3f}")
    ax.set_xlabel("Thrust time shift [ms]")
    ax.set_ylabel("R\u00b2")
    ax.set_title("Lag Sweep: R\u00b2 vs Thrust Time Shift")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Top-right: Linear vs quadratic scatter
    ax = axes[0, 1]
    thrust = diag["thrust_mag"]
    err = diag["err_hov"]
    ax.scatter(thrust, err, s=2, alpha=0.3, color="tab:orange")

    x_fit = np.linspace(thrust.min(), thrust.max(), 100)
    # Linear fit line
    lin_coeffs = np.polyfit(thrust, err, 1)
    ax.plot(x_fit, np.polyval(lin_coeffs, x_fit), "k--", linewidth=1.2,
            label=f"Linear (R\u00b2={diag['linear_r2']:.3f})")
    # Quadratic fit line
    a, b, c = diag["quad_coeffs"]
    ax.plot(x_fit, a * x_fit**2 + b * x_fit + c, "r-", linewidth=1.2,
            label=f"Quadratic (R\u00b2={diag['quad_r2']:.3f})")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Baro Error [m]")
    ax.set_title("Linear vs Quadratic Fit")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Bottom-right: Summary text
    ax = axes[1, 1]
    ax.axis("off")

    r2_gain_lag = diag["best_lag_r2"] - diag["zero_lag_r2"]
    r2_gain_quad = diag["quad_r2"] - diag["linear_r2"]
    r2_gain_comb = diag["combined_r2"] - diag["linear_r2"]

    lines = [
        "Residual Diagnostics",
        "",
        "  Model R\u00b2 comparison:",
        f"    Linear (zero lag):     {diag['linear_r2']:.3f}",
        f"    Linear (best lag):     {diag['best_lag_r2']:.3f}"
        f"  (+{r2_gain_lag:.3f})",  # noqa: intentional string concat
        f"    Quadratic (zero lag):  {diag['quad_r2']:.3f}"
        f"  (+{r2_gain_quad:.3f})",  # noqa: intentional string concat
        f"    Quadratic + best lag:  {diag['combined_r2']:.3f}"
        f"  (+{r2_gain_comb:.3f})",  # noqa: intentional string concat
        "",
        f"  Optimal lag: {diag['best_lag_s']*1000:+.0f}ms",
        "",
    ]

    # Interpretation
    if r2_gain_lag > 0.05 and r2_gain_quad < 0.02:
        lines.append("  >> Time delay is the dominant factor.")
        lines.append(f"     Shifting thrust by {diag['best_lag_s']*1000:+.0f}ms")
        lines.append(f"     improves R\u00b2 by {r2_gain_lag:.3f}.")
    elif r2_gain_quad > 0.05 and r2_gain_lag < 0.02:
        lines.append("  >> Nonlinearity is the dominant factor.")
        lines.append("     Consider a quadratic thrust model.")
    elif r2_gain_lag > 0.02 and r2_gain_quad > 0.02:
        lines.append("  >> Both delay and nonlinearity contribute.")
        lines.append(f"     Combined R\u00b2 gain: +{r2_gain_comb:.3f}")
    else:
        lines.append("  >> Neither delay nor nonlinearity explain")
        lines.append("     the residual. Other factors at play")
        lines.append("     (e.g., altitude-dependent effect,")
        lines.append("     wind, ground effect).")

    ax.text(0.05, 0.95, "\n".join(lines), transform=ax.transAxes,
            fontsize=11, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f0f0f0",
                      edgecolor="#cccccc"))

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_open_loop_delay(ol):
    """Plot open-loop delay analysis (feedback-rejected)."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Open-Loop Delay Analysis (Feedback-Rejected)",
                 fontsize=14, fontweight="bold")
    fig.text(_SUBTITLE_X, _SUBTITLE_Y,
             "High-pass filtered to remove altitude controller feedback. "
             "Shows true physical motor-to-baro delay.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    # Left: Cross-correlation
    ax = axes[0]
    ax.plot(ol["lags_s"] * 1000, ol["xcorr"],
            color="tab:blue", linewidth=1.0)
    peak_lag = ol["peak_lag_s"]
    peak_r = ol["peak_r"]
    ax.axvline(peak_lag * 1000, color="tab:red", linestyle="--", linewidth=0.8,
               label=f"Peak: {peak_lag*1000:+.0f}ms (r={peak_r:.3f})")
    ax.axvline(0, color="k", linewidth=0.5, linestyle=":")
    ax.set_xlabel("Lag [ms]  (positive = thrust leads error)")
    ax.set_ylabel("Normalized cross-correlation")
    ax.set_title("High-Pass Cross-Correlation")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Right: Lag sweep R²
    ax = axes[1]
    ax.plot(ol["lag_steps_s"] * 1000, ol["lag_r2"],
            color="tab:blue", linewidth=1.0)
    best_lag = ol["best_lag_s"]
    best_r2 = ol["best_lag_r2"]
    zero_r2 = ol["zero_lag_r2"]
    ax.axvline(best_lag * 1000, color="tab:red", linestyle="--", linewidth=0.8,
               label=f"Best lag: {best_lag*1000:+.0f}ms (R\u00b2={best_r2:.3f})")
    ax.axvline(0, color="k", linewidth=0.5, linestyle=":")
    ax.axhline(zero_r2, color="tab:gray", linestyle=":", linewidth=0.8,
               label=f"Zero lag R\u00b2={zero_r2:.3f}")
    ax.set_xlabel("Thrust time shift [ms]")
    ax.set_ylabel("R\u00b2")
    ax.set_title("High-Pass Lag Sweep")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_summary_text(text_lines):
    """Final page: full console output summary."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    ax.axis("off")
    fig.suptitle("Analysis Summary", fontsize=14, fontweight="bold")
    ax.text(0.02, 0.98, "\n".join(text_lines), transform=ax.transAxes,
            fontsize=10, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f8f8f8",
                      edgecolor="#cccccc"))
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


def reconstruct_raw_error(baro_err, thrust_data, pcoef):
    """Undo existing baro compensation to reconstruct the raw error.

    During flight the firmware applied at baro timestamps:
        baro_alt += pcoef * motor_at(baro_timestamp_sample)
    To preserve timing fidelity, undo at the original baro timestamps,
    then re-interpolate to the range sensor time base.
    """
    baro_t = baro_err["baro_full_time_s"]
    baro_alt = baro_err["baro_full_alt"]

    # Undo compensation at baro timestamps (matching firmware time-alignment)
    thrust_at_baro = np.interp(baro_t, thrust_data["time_s"],
                               thrust_data["thrust"])
    raw_baro = baro_alt - pcoef * thrust_at_baro

    # Re-interpolate raw baro to range sensor time base
    rng_t = baro_err["time_s"]
    raw_baro_interp = np.interp(rng_t, baro_t, raw_baro)
    raw_error = raw_baro_interp - baro_err["range_alt"]

    result = dict(baro_err)
    result["error"] = raw_error
    return result


# ---------------------------------------------------------------------------
# Plotting — overview and range-based pages
# ---------------------------------------------------------------------------

def plot_altitude_overview(ekf_baro_obs, ekf_z, range_data, thrust_data,
                          armed_start, armed_end):
    """Altitude overview — baro observation, EKF Z, and distance sensor."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Altitude Overview", fontsize=14, fontweight="bold")
    fig.text(_SUBTITLE_X, _SUBTITLE_Y,
             "Baro altitude vs EKF and range sensor. "
             "Gap between baro and range reveals propwash-induced pressure error.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    ax = axes[0]

    # Distance sensor (ground truth, if available)
    if range_data is not None:
        ax.plot(range_data["time_s"], range_data["distance_m"],
                label="Distance sensor", color="tab:blue", linewidth=1.2)

    # Baro observation (estimator_aid_src_baro_hgt, inverted NED, zeroed at arm)
    if ekf_baro_obs is not None:
        baro_t = ekf_baro_obs["time_s"]
        baro_alt = -ekf_baro_obs["observation"]  # NED to altitude-up
        arm_baro = np.interp(armed_start, baro_t, baro_alt)
        baro_zeroed = baro_alt - arm_baro
        ax.plot(baro_t, baro_zeroed,
                label="Baro observation", color="tab:red",
                linewidth=1.0, alpha=0.8)

    # EKF Z (inverted NED -> altitude up, zeroed at arm)
    if ekf_z is not None:
        ekf_t = ekf_z["time_s"]
        ekf_alt = -ekf_z["z"]  # NED to altitude-up
        arm_ekf = np.interp(armed_start, ekf_t, ekf_alt)
        ekf_zeroed = ekf_alt - arm_ekf
        ax.plot(ekf_t, ekf_zeroed,
                label="EKF altitude (-Z)", color="tab:green",
                linewidth=1.0, alpha=0.8)

    ax.axvspan(armed_start, armed_end, alpha=0.04, color="green", label="Armed")
    ax.set_ylabel("Altitude AGL [m]")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, alpha=0.3)

    # Thrust
    ax = axes[1]
    if thrust_data is not None:
        thr_t = thrust_data["time_s"]
        thr_mag = thrust_data["thrust"]
        ax.plot(thr_t, thr_mag, color="tab:orange", linewidth=0.8,
                label="Thrust magnitude")
    ax.set_ylabel("Thrust [0-1]")
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_compensation_comparison(baro_err, thrust_data, calib, online_k,
                                 armed_start, armed_end,
                                 existing_pcoef=0.0):
    """Side-by-side baro vs range: online-corrected vs offline-corrected."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), sharex=True)
    fig.suptitle("Compensation Comparison: Online vs Offline",
                 fontsize=14, fontweight="bold")
    fig.text(_SUBTITLE_X, _SUBTITLE_Y,
             "Left: baro corrected using online estimator K. "
             "Right: corrected using offline calibration (range-sensor ground truth).\n"
             "Closer baro-range agreement = better compensation.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    t = baro_err["time_s"]
    err = baro_err["error"]
    baro_t = baro_err["baro_full_time_s"]
    baro_alt = baro_err["baro_full_alt"]

    # Reconstruct raw error/altitude by undoing any existing compensation.
    # baro_alt and err reflect the as-flown data (with existing_pcoef applied).
    thr_mag = np.interp(t, thrust_data["time_s"], thrust_data["thrust"])
    raw_err = err - existing_pcoef * thr_mag

    thr_full = np.interp(baro_t, thrust_data["time_s"], thrust_data["thrust"])
    raw_baro = baro_alt - existing_pcoef * thr_full

    # Online: total PCOEF = existing + (-online_k)
    online_delta = -online_k
    online_total_pcoef = existing_pcoef + online_delta
    online_err = raw_err + online_total_pcoef * thr_mag
    baro_online = raw_baro + online_total_pcoef * thr_full
    baro_online -= np.interp(armed_start, baro_t, baro_online)

    # Offline: apply identified K (calibrate ran on raw error, so -K is total PCOEF)
    offline_total_pcoef = -calib["best_K"]
    offline_delta = offline_total_pcoef - existing_pcoef
    offline_err = raw_err + offline_total_pcoef * thr_mag
    baro_offline = raw_baro + offline_total_pcoef * thr_full
    baro_offline -= np.interp(armed_start, baro_t, baro_offline)

    armed = (baro_t >= armed_start) & (baro_t <= armed_end)
    armed_r = (t >= armed_start) & (t <= armed_end)

    # Top-left: Online-corrected altitude
    ax = axes[0, 0]
    ax.plot(t, baro_err["range_alt"], color="tab:green", linewidth=1.2,
            label="Range sensor")
    ax.plot(baro_t[armed], baro_online[armed], color="tab:blue", linewidth=1.0,
            alpha=0.85,
            label=f"Baro + online (PCOEF={existing_pcoef:+.1f}+({online_delta:+.1f})={online_total_pcoef:+.1f})")
    ax.set_ylabel("Altitude [m]")
    ax.legend(fontsize=10)
    ax.set_title("Online Estimator Correction")
    ax.grid(True, alpha=0.3)

    # Top-right: Offline-corrected altitude
    ax = axes[0, 1]
    ax.plot(t, baro_err["range_alt"], color="tab:green", linewidth=1.2,
            label="Range sensor")
    ax.plot(baro_t[armed], baro_offline[armed], color="tab:blue", linewidth=1.0,
            alpha=0.85,
            label=f"Baro + offline (PCOEF={existing_pcoef:+.1f}+({offline_delta:+.1f})={offline_total_pcoef:+.1f})")
    ax.set_ylabel("Altitude [m]")
    ax.legend(fontsize=10)
    ax.set_title("Offline Calibration Correction")
    ax.grid(True, alpha=0.3)

    # Sync Y-axis limits
    ylim = [min(axes[0, 0].get_ylim()[0], axes[0, 1].get_ylim()[0]),
            max(axes[0, 0].get_ylim()[1], axes[0, 1].get_ylim()[1])]
    axes[0, 0].set_ylim(ylim)
    axes[0, 1].set_ylim(ylim)

    # Bottom-left: Online residual error
    ax = axes[1, 0]
    ax.plot(t[armed_r], online_err[armed_r], color="tab:blue", linewidth=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    online_std = float(np.std(online_err[armed_r]))
    online_mean = float(np.mean(online_err[armed_r]))
    ax.set_title(f"Residual Error: mean={online_mean:+.2f}m, std={online_std:.2f}m")
    ax.set_ylabel("Baro Error [m]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    # Bottom-right: Offline residual error
    ax = axes[1, 1]
    ax.plot(t[armed_r], offline_err[armed_r], color="tab:blue", linewidth=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    offline_std = float(np.std(offline_err[armed_r]))
    offline_mean = float(np.mean(offline_err[armed_r]))
    ax.set_title(f"Residual Error: mean={offline_mean:+.2f}m, std={offline_std:.2f}m")
    ax.set_ylabel("Baro Error [m]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    # Sync bottom Y-axis limits
    ylim = [min(axes[1, 0].get_ylim()[0], axes[1, 1].get_ylim()[0]),
            max(axes[1, 0].get_ylim()[1], axes[1, 1].get_ylim()[1])]
    axes[1, 0].set_ylim(ylim)
    axes[1, 1].set_ylim(ylim)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_online_vs_offline(baro_err, thrust_data, calib, online_k,
                           armed_start, armed_end,
                           existing_pcoef=0.0):
    """Scatter comparison and summary: online vs offline calibration."""
    fig, axes = plt.subplots(1, 3, figsize=(16, 6.5))
    fig.suptitle("Online vs Offline: Error vs Thrust",
                 fontsize=14, fontweight="bold")
    fig.text(_SUBTITLE_X, _SUBTITLE_Y,
             "Scatter plots show baro-range error vs thrust. "
             "Good compensation removes the thrust correlation (flat scatter near zero).",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    t = baro_err["time_s"]
    err = baro_err["error"]
    armed = (t >= armed_start) & (t <= armed_end)
    t_a = t[armed]
    err_a = err[armed]

    # Reconstruct raw error by undoing existing compensation
    thr_mag = np.interp(t_a, thrust_data["time_s"], thrust_data["thrust"])
    x_fit = np.linspace(thr_mag.min(), thr_mag.max(), 50)

    raw_err = err_a - existing_pcoef * thr_mag

    # Left: Raw error
    ax = axes[0]
    ax.scatter(thr_mag, raw_err, s=2, alpha=0.3, color="tab:orange")
    z_lin = np.polyfit(thr_mag, raw_err, 1)
    z_quad = np.polyfit(thr_mag, raw_err, 2)
    ax.plot(x_fit, np.polyval(z_lin, x_fit), "k--", linewidth=1.2, label="Linear")
    ax.plot(x_fit, np.polyval(z_quad, x_fit), "r-", linewidth=1.0, alpha=0.7, label="Quadratic")
    r_val = _safe_corrcoef(thr_mag, raw_err)
    ax.set_title(f"Raw (no compensation)\nr={r_val:.3f}, slope={z_lin[0]:.1f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Baro Error [m]")
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    # Middle: Online-corrected (total PCOEF = existing + (-online_k))
    online_delta = -online_k
    online_total_pcoef = existing_pcoef + online_delta
    online_err = raw_err + online_total_pcoef * thr_mag
    ax = axes[1]
    ax.scatter(thr_mag, online_err, s=2, alpha=0.3, color="tab:blue")
    z_lin = np.polyfit(thr_mag, online_err, 1)
    z_quad = np.polyfit(thr_mag, online_err, 2)
    ax.plot(x_fit, np.polyval(z_lin, x_fit), "k--", linewidth=1.2, label="Linear")
    ax.plot(x_fit, np.polyval(z_quad, x_fit), "r-", linewidth=1.0, alpha=0.7, label="Quadratic")
    r_val = _safe_corrcoef(thr_mag, online_err)
    ax.set_title(f"Online: {existing_pcoef:+.1f} + ({online_delta:+.1f})"
                 f" = {online_total_pcoef:+.1f}\n"
                 f"r={r_val:.3f}, slope={z_lin[0]:.1f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    # Right: Offline-corrected (calibrate ran on raw error, -K is total PCOEF)
    offline_total = -calib["best_K"]
    offline_delta = offline_total - existing_pcoef
    offline_err = raw_err + offline_total * thr_mag
    ax = axes[2]
    ax.scatter(thr_mag, offline_err, s=2, alpha=0.3, color="tab:green")
    z_lin = np.polyfit(thr_mag, offline_err, 1)
    z_quad = np.polyfit(thr_mag, offline_err, 2)
    ax.plot(x_fit, np.polyval(z_lin, x_fit), "k--", linewidth=1.2, label="Linear")
    ax.plot(x_fit, np.polyval(z_quad, x_fit), "r-", linewidth=1.0, alpha=0.7, label="Quadratic")
    r_val = _safe_corrcoef(thr_mag, offline_err)
    ax.set_title(f"Offline: {existing_pcoef:+.1f} + ({offline_delta:+.1f})"
                 f" = {offline_total:+.1f}\n"
                 f"r={r_val:.3f}, slope={z_lin[0]:.1f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    # Sync Y-axis
    ylim = [min(ax.get_ylim()[0] for ax in axes),
            max(ax.get_ylim()[1] for ax in axes)]
    for ax in axes:
        ax.set_ylim(ylim)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig



# ---------------------------------------------------------------------------
# Plotting — online estimator review
# ---------------------------------------------------------------------------

def plot_estimator_convergence(est_data, armed_start, armed_end,
                               existing_pcoef=0.0):
    """Estimator convergence: K estimate, error variance, convergence status."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Online Estimator Convergence", fontsize=14, fontweight="bold")
    fig.text(_SUBTITLE_X, _SUBTITLE_Y,
             "RLS estimator progress. K should stabilize before convergence locks. "
             "Sufficient thrust excitation (std > 0.05) is required.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    t = est_data["time_s"]
    armed = (t >= armed_start) & (t <= armed_end)
    ta = t[armed]

    # Panel 1: K estimate with variance band
    ax = axes[0]
    k = est_data["k_estimate"][armed]
    k_std = np.sqrt(np.clip(est_data["k_estimate_var"][armed], 0, None))
    ax.plot(ta, k, color="tab:blue", linewidth=1.0, label="K estimate")
    ax.fill_between(ta, k - k_std, k + k_std, alpha=0.15, color="tab:blue",
                    label="\u00b11\u03c3")
    if existing_pcoef != 0.0:
        ax.axhline(-existing_pcoef, color="tab:red", linestyle="--",
                   linewidth=0.8, label=f"Saved PCOEF = {existing_pcoef:+.1f}")
    ax.set_ylabel("K [m/unit thrust]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 2: Prediction error variance + thrust excitation
    ax = axes[1]
    ax.plot(ta, est_data["error_var"][armed], color="tab:orange",
            linewidth=0.8, label="Error variance")
    ax.set_ylabel("Error Var / Thrust Std")
    ax2 = ax.twinx()
    ax2.plot(ta, est_data["thrust_std"][armed], color="tab:purple",
             linewidth=0.8, alpha=0.7, label="Thrust std")
    ax2.set_ylabel("Thrust Std", color="tab:purple")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 3: Convergence + estimation active flags
    ax = axes[2]
    conv = est_data["converged"][armed].astype(float)
    active = est_data["estimation_active"][armed].astype(float)
    ax.fill_between(ta, 0, active * 0.5, alpha=0.3, color="tab:blue",
                    step="post", label="Estimation active")
    ax.fill_between(ta, 0.5, 0.5 + conv * 0.5, alpha=0.4, color="tab:green",
                    step="post", label="Converged")
    ax.set_ylim(-0.05, 1.05)
    ax.set_yticks([0, 0.5, 1.0])
    ax.set_yticklabels(["", "", ""])
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_estimator_residual(est_data, thrust_data, armed_start, armed_end):
    """Estimator residual analysis: residual vs time and vs thrust."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("CF Residual Analysis (Online Estimator)",
                 fontsize=14, fontweight="bold")

    t = est_data["time_s"]
    armed = (t >= armed_start) & (t <= armed_end)

    residual = est_data["residual"][armed]
    ta = t[armed]

    # Left: Residual time series
    ax = axes[0]
    ax.plot(ta, residual, color="tab:red", linewidth=0.6, alpha=0.7)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("CF Residual [m]")
    ax.set_title("Residual (Baro - Accel Prediction)")
    ax.grid(True, alpha=0.3)

    # Right: Residual vs thrust scatter
    ax = axes[1]
    if thrust_data is not None:
        thr_interp = np.interp(ta, thrust_data["time_s"], thrust_data["thrust"])
        ax.scatter(thr_interp, residual, s=2, alpha=0.3, color="tab:orange")
        if np.std(thr_interp) > 1e-6:
            z = np.polyfit(thr_interp, residual, 1)
            x_fit = np.linspace(thr_interp.min(), thr_interp.max(), 50)
            ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
            r_val = _safe_corrcoef(thr_interp, residual)
            ax.set_title(f"Residual vs Thrust\n"
                         f"r = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
        else:
            ax.set_title("Residual vs Thrust")
    ax.set_xlabel("Thrust magnitude [0-1]")
    ax.set_ylabel("CF Residual [m]")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_compensation_effect(est_data, thrust_data, armed_start, armed_end):
    """Compensation effect: CF residual before/after applying estimated K."""
    t = est_data["time_s"]
    armed = (t >= armed_start) & (t <= armed_end)
    ta = t[armed]
    residual = est_data["residual"][armed]

    # Use converged K, or last value if not converged
    conv_mask = (est_data["converged"][armed] > 0)
    if conv_mask.any():
        idx = np.where(conv_mask)[0][0]
        final_k = float(est_data["k_estimate"][armed][idx])
    else:
        final_k = float(est_data["k_estimate"][armed][-1])

    thrust_mag = np.interp(ta, thrust_data["time_s"], thrust_data["thrust"])
    corrected = residual - final_k * thrust_mag

    # Remove DC bias for visualization. The RLS model is residual = K*thrust + bias;
    # the firmware only corrects K*thrust (EKF absorbs the constant offset), so the
    # corrected signal retains the bias. Subtracting it here lets us visually compare
    # the thrust-correlated variation before vs after.
    bias = float(np.mean(corrected))
    corrected_centered = corrected - bias

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Estimated Compensation Effect", fontsize=14, fontweight="bold")
    fig.text(_SUBTITLE_X, _SUBTITLE_Y,
             "CF residual (baro minus accel-predicted altitude) before and after "
             "applying the online K estimate. Corrected residual should be flat with "
             "no thrust correlation. DC bias removed for comparison.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    # Top-left: Residual before/after
    ax = axes[0, 0]
    ax.plot(ta, residual, color="tab:red", linewidth=0.6, alpha=0.7,
            label="Raw CF residual")
    ax.plot(ta, corrected_centered, color="tab:blue", linewidth=0.6, alpha=0.7,
            label=f"Corrected (K={final_k:.2f}, bias={bias:+.1f}m removed)")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Residual [m]")
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=9)
    ax.set_title("CF Residual Before/After Compensation")
    ax.grid(True, alpha=0.3)

    # Top-right: Scatter — raw residual vs thrust
    ax = axes[0, 1]
    ax.scatter(thrust_mag, residual, s=2, alpha=0.3, color="tab:orange")
    if np.std(thrust_mag) > 1e-6:
        z = np.polyfit(thrust_mag, residual, 1)
        x_fit = np.linspace(thrust_mag.min(), thrust_mag.max(), 50)
        ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
        r_val = _safe_corrcoef(thrust_mag, residual)
        ax.set_title(f"Raw Residual vs Thrust\nr = {r_val:.3f}")
    else:
        ax.set_title("Raw Residual vs Thrust")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("CF Residual [m]")
    ax.grid(True, alpha=0.3)

    # Bottom-left: Scatter — corrected residual vs thrust
    ax = axes[1, 0]
    ax.scatter(thrust_mag, corrected_centered, s=2, alpha=0.3, color="tab:blue")
    if np.std(thrust_mag) > 1e-6:
        z = np.polyfit(thrust_mag, corrected_centered, 1)
        x_fit = np.linspace(thrust_mag.min(), thrust_mag.max(), 50)
        ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
        r_val = _safe_corrcoef(thrust_mag, corrected_centered)
        ax.set_title(f"Corrected Residual vs Thrust\nr = {r_val:.3f}")
    else:
        ax.set_title("Corrected Residual vs Thrust")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Corrected Residual [m]")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

    # Bottom-right: Statistics summary
    ax = axes[1, 1]
    ax.axis("off")
    raw_std = float(np.std(residual))
    corr_std = float(np.std(corrected_centered))
    raw_r = _safe_corrcoef(thrust_mag, residual)
    corr_r = _safe_corrcoef(thrust_mag, corrected_centered)
    var_reduction = ((1.0 - corr_std**2 / raw_std**2) * 100
                     if raw_std > 0 else 0)

    lines = [
        "Compensation Summary",
        "",
        f"  Estimated K        = {final_k:.3f} m/unit thrust",
        f"  SENS_BARO_PCOEF    = {-final_k:+.2f}",
        "",
        "  Residual std:",
        f"    Raw:         {raw_std:.3f} m",
        f"    Corrected:   {corr_std:.3f} m",
        f"    Reduction:   {var_reduction:.1f}%",
        "",
        "  Thrust correlation |r|:",
        f"    Raw:         {abs(raw_r):.3f} ({_correlation_quality(raw_r)})",
        f"    Corrected:   {abs(corr_r):.3f} ({_correlation_quality(corr_r)})",
    ]
    ax.text(0.05, 0.95, "\n".join(lines), transform=ax.transAxes,
            fontsize=11, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f0f0f0",
                      edgecolor="#cccccc"))

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig



# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _safe_corrcoef(x, y):
    """Pearson correlation, returning 0 if either input has zero variance."""
    if len(x) < 2 or np.std(x) < 1e-10 or np.std(y) < 1e-10:
        return 0.0
    return float(np.corrcoef(x, y)[0, 1])


def _correlation_quality(r):
    """Return a label for correlation magnitude."""
    ar = abs(r)
    if ar > 0.6:
        return "strong"
    if ar > 0.3:
        return "moderate"
    return "weak"


def _print_open_loop_delay(ol):
    """Print open-loop delay estimate."""
    print(f"\n--- Open-Loop Delay (feedback-rejected, "
          f"HP cutoff ~{ol['hp_window_ms']:.0f}ms) ---")
    peak = ol["peak_lag_s"] * 1000
    print(f"  Cross-corr peak:  {peak:+.0f}ms (r={ol['peak_r']:.3f})")
    best = ol["best_lag_s"] * 1000
    print(f"  Best lag R\u00b2:      {ol['best_lag_r2']:.3f} "
          f"(lag={best:+.0f}ms)")
    print(f"  Zero lag R\u00b2:      {ol['zero_lag_r2']:.3f}")
    delta = ol["best_lag_r2"] - ol["zero_lag_r2"]
    if delta < 0.01:
        print(f"  -> No significant delay detected")
    else:
        print(f"  -> Physical delay ~{abs(best):.0f}ms "
              f"(R\u00b2 gain: +{delta:.3f})")


def _print_diagnostics(diag):
    """Print residual diagnostic results to console."""
    print(f"\n--- Residual Diagnostics ---")
    print(f"  Linear R\u00b2 (zero lag):    {diag['linear_r2']:.3f}")
    print(f"  Linear R\u00b2 (best lag):    {diag['best_lag_r2']:.3f}"
          f"  (lag={diag['best_lag_s']*1000:+.0f}ms)")
    print(f"  Quadratic R\u00b2 (zero lag): {diag['quad_r2']:.3f}")
    print(f"  Combined R\u00b2 (lag+quad):  {diag['combined_r2']:.3f}")
    if "xcorr_peak_lag_s" in diag:
        print(f"  Cross-corr peak:         {diag['xcorr_peak_lag_s']*1000:+.0f}ms"
              f" (r={diag['xcorr_peak_value']:.3f})")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Analyze barometer thrust compensation from flight logs")
    parser.add_argument("ulog_file", help="Path to .ulg flight log")
    # Default output: logs/<log_name>/ relative to PX4 root (script location)
    px4_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    default_log_dir = os.path.join(px4_root, "logs")
    parser.add_argument("--output-dir", "-o", default=default_log_dir,
                        help="Output base directory (default: <PX4_ROOT>/logs/)")
    args = parser.parse_args()

    if not os.path.isfile(args.ulog_file):
        print(f"Error: file not found: {args.ulog_file}", file=sys.stderr)
        sys.exit(1)

    # Create per-log output directory: <default_base>/<log_name>/ when using
    # the default, or use --output-dir directly when explicitly provided.
    log_name = os.path.splitext(os.path.basename(args.ulog_file))[0]

    if args.output_dir == default_log_dir:
        output_dir = os.path.join(args.output_dir, log_name)
    else:
        output_dir = args.output_dir

    os.makedirs(output_dir, exist_ok=True)

    # Copy ULG into output dir for co-location
    ulg_dest = os.path.join(output_dir, os.path.basename(args.ulog_file))
    if not os.path.exists(ulg_dest):
        shutil.copy2(args.ulog_file, ulg_dest)

    # Capture console output for summary page
    _log_buf = io.StringIO()
    _orig_stdout = sys.stdout

    class _Tee:
        def __init__(self, *streams):
            self.streams = streams
        def write(self, data):
            for s in self.streams:
                s.write(data)
        def flush(self):
            for s in self.streams:
                s.flush()

    sys.stdout = _Tee(_orig_stdout, _log_buf)

    # Load log
    print(f"Loading {args.ulog_file} ...")
    ulog = ULog(args.ulog_file)
    duration = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    print(f"  Duration: {duration:.1f} s")

    # Print relevant parameters
    print("\nParameters:")
    for p in ["EKF2_HGT_REF", "EKF2_BARO_CTRL", "EKF2_BARO_NOISE",
              "SENS_BARO_PCOEF", "SENS_BAR_AUTOCAL"]:
        val = get_param(ulog, p)
        if val is not None:
            print(f"  {p:24s} = {val}")

    # Detect what data is available
    existing_pcoef = float(get_param(ulog, 'SENS_BARO_PCOEF', 0.0))
    is_calibrated = (existing_pcoef != 0.0)

    armed_start, armed_end = detect_armed_period(ulog)
    hover_start, hover_end = find_hover_segment(armed_start, armed_end)
    print(f"\nArmed:  {armed_start:.1f}s - {armed_end:.1f}s")
    print(f"Hover:  {hover_start:.1f}s - {hover_end:.1f}s")

    # Extract data
    baro = extract_baro(ulog)
    range_data = extract_range(ulog)
    thrust_data = extract_thrust(ulog)
    est_data = extract_estimator(ulog)
    ekf_z = extract_ekf_z(ulog)
    ekf_baro_obs = extract_ekf_baro_obs(ulog)

    if baro is None:
        print("Error: no barometer data (vehicle_air_data) in log",
              file=sys.stderr)
        sys.exit(1)
    if thrust_data is None:
        print("Error: no thrust data (vehicle_thrust_setpoint) in log",
              file=sys.stderr)
        sys.exit(1)

    has_range = range_data is not None
    has_estimator = est_data is not None

    print(f"\n  Baro samples:      {len(baro['time_s'])}")
    print(f"  Thrust samples:    {len(thrust_data['time_s'])}")
    print(f"  Range sensor:      {'yes' if has_range else 'no'}")
    print(f"  Online estimator:  {'yes' if has_estimator else 'no'}")
    if has_range:
        print(f"  Range samples:     {len(range_data['time_s'])}")
    if has_estimator:
        print(f"  Estimator samples: {len(est_data['time_s'])}")

    figures = []

    # Altitude overview — always first page
    figures.append(plot_altitude_overview(
        ekf_baro_obs, ekf_z, range_data, thrust_data, armed_start, armed_end))

    # ----- Online estimator review (always, when available) -----
    if has_estimator:
        print("\n--- Online Estimator Review ---")

        # Find convergence point
        armed_mask = ((est_data["time_s"] >= armed_start)
                      & (est_data["time_s"] <= armed_end))
        conv_mask = armed_mask & (est_data["converged"] > 0)

        if conv_mask.any():
            conv_idx = np.where(conv_mask)[0][0]
            final_k = float(est_data["k_estimate"][conv_idx])
            conv_time = float(est_data["time_s"][conv_idx]) - armed_start
            print(f"  Converged at {conv_time:.0f}s after arm")
            print(f"  Final K = {final_k:.3f}")
            print(f"  Implied PCOEF update: {existing_pcoef:.1f} - {final_k:.2f}"
                  f" = {existing_pcoef - final_k:+.2f}")
        else:
            final_k = float(est_data["k_estimate"][armed_mask][-1]) if armed_mask.any() else 0
            print("  Did NOT converge during this flight")
            print(f"  Last K = {final_k:.3f}")

        figures.append(plot_estimator_convergence(
            est_data, armed_start, armed_end, existing_pcoef))
        figures.append(plot_compensation_effect(
            est_data, thrust_data, armed_start, armed_end))

    # ----- Range-sensor-based analysis (when range available) -----
    if has_range:
        baro_err = compute_baro_error(baro, range_data, armed_start)
        hov = ((baro_err["time_s"] >= hover_start)
               & (baro_err["time_s"] <= hover_end))

        if hov.sum() < 20:
            print("\nWarning: not enough data in hover segment for "
                  "range-based analysis", file=sys.stderr)

        else:
            err_hov = baro_err["error"][hov]

            if is_calibrated:
                # --- VALIDATION MODE ---
                print(f"\n--- Range-Based Validation ---")
                print(f"  Compensation was ACTIVE (PCOEF={existing_pcoef:+.1f})")

                print(f"  Observed error:  mean={np.mean(err_hov):+.3f}m,"
                      f" std={np.std(err_hov):.3f}m")

                raw_baro_err = reconstruct_raw_error(
                    baro_err, thrust_data, existing_pcoef)
                raw_hov = raw_baro_err["error"][hov]
                print(f"  Reconstructed raw: mean={np.mean(raw_hov):+.3f}m,"
                      f" std={np.std(raw_hov):.3f}m")

                thrust_interp_mag = np.interp(baro_err["time_s"][hov],
                              thrust_data["time_s"], thrust_data["thrust"])
                r_raw = _safe_corrcoef(thrust_interp_mag, raw_hov)
                r_comp = _safe_corrcoef(thrust_interp_mag, err_hov)
                print(f"  Thrust |r|: raw={abs(r_raw):.3f} "
                      f"({_correlation_quality(r_raw)}), "
                      f"compensated={abs(r_comp):.3f} "
                      f"({_correlation_quality(r_comp)})")

                calib = calibrate(raw_baro_err, thrust_data,
                                  hover_start, hover_end)
                if calib is not None:
                    # calibrate() ran on reconstructed raw error, so
                    # recommended_pcoef is the TOTAL value, not a delta.
                    offline_new = calib['recommended_pcoef']
                    offline_delta = offline_new - existing_pcoef
                    print(f"  Re-identified: K={calib['best_K']:.3f},"
                          f" R\u00b2={calib['best_r2']:.3f}")
                    print(f"\n  SENS_BARO_PCOEF = {existing_pcoef:+.2f}"
                          f" + ({offline_delta:+.2f})"
                          f" = {offline_new:+.2f}")

                    # Cross-validate with online estimator
                    if has_estimator and conv_mask.any():
                        # Online K is the residual after existing compensation
                        online_delta = -final_k
                        online_total = existing_pcoef + online_delta
                        print(f"\n--- Cross-Validation ---")
                        print(f"  Online:  PCOEF = {existing_pcoef:+.2f}"
                              f" + ({online_delta:+.2f})"
                              f" = {online_total:+.2f}")
                        print(f"  Offline: PCOEF = {existing_pcoef:+.2f}"
                              f" + ({offline_delta:+.2f})"
                              f" = {offline_new:+.2f}")
                        pcoef_diff = abs(online_total - offline_new)
                        print(f"  Agreement: {pcoef_diff:.2f}m"
                              f" ({'good' if pcoef_diff < 2.0 else 'notable'})")

                    if has_estimator:
                        figures.append(plot_compensation_comparison(
                            baro_err, thrust_data, calib, final_k,
                            armed_start, armed_end,
                            existing_pcoef))
                        figures.append(plot_online_vs_offline(
                            baro_err, thrust_data, calib, final_k,
                            armed_start, armed_end,
                            existing_pcoef))

                    # Residual diagnostics on reconstructed raw error
                    diag = diagnose_residual(raw_baro_err, thrust_data,
                                            hover_start, hover_end)
                    if diag is not None:
                        _print_diagnostics(diag)
                        figures.append(plot_residual_diagnostics(diag, calib))

                    # Open-loop delay (feedback-rejected)
                    ol_delay = estimate_open_loop_delay(
                        raw_baro_err, thrust_data,
                        hover_start, hover_end)
                    if ol_delay is not None:
                        _print_open_loop_delay(ol_delay)
                        figures.append(plot_open_loop_delay(ol_delay))

            else:
                # --- CALIBRATION MODE ---
                print(f"\n--- Range-Based Calibration ---")
                print(f"  No compensation active")
                print(f"  Baro error: mean={np.mean(err_hov):+.3f}m,"
                      f" std={np.std(err_hov):.3f}m")

                thrust_interp = np.interp(
                    baro_err["time_s"][hov], thrust_data["time_s"],
                    thrust_data["thrust"])
                r_thrust = _safe_corrcoef(thrust_interp, err_hov)
                print(f"  Thrust correlation: r={r_thrust:+.3f}"
                      f" ({_correlation_quality(r_thrust)})")

                calib = calibrate(baro_err, thrust_data,
                                  hover_start, hover_end)
                if calib is not None:
                    print(f"\n  Identified K  = {calib['best_K']:.3f}")
                    print(f"  Model R\u00b2     = {calib['best_r2']:.3f}")
                    print(f"  Residual RMSE = {calib['best_rmse']:.3f}m")

                    new_pcoef = calib['recommended_pcoef']
                    print(f"\n{'='*50}")
                    print(f"  RECOMMENDED PARAMETERS")
                    print(f"{'='*50}")
                    print(f"  SENS_BARO_PCOEF = {new_pcoef:+.2f}")
                    print(f"{'='*50}")

                    # Cross-validate with online estimator
                    if has_estimator and conv_mask.any():
                        print(f"\n--- Cross-Validation ---")
                        print(f"  Online:  K={final_k:.3f},"
                              f" PCOEF={-final_k:+.2f}")
                        print(f"  Offline: K={calib['best_K']:.3f},"
                              f" PCOEF={calib['recommended_pcoef']:+.2f}")
                        k_diff = abs(final_k - calib['best_K'])
                        print(f"  K agreement: {k_diff:.2f}m"
                              f" ({'good' if k_diff < 2.0 else 'notable'})")

                    if has_estimator:
                        figures.append(plot_compensation_comparison(
                            baro_err, thrust_data, calib, final_k,
                            armed_start, armed_end))
                        figures.append(plot_online_vs_offline(
                            baro_err, thrust_data, calib, final_k,
                            armed_start, armed_end))

                    # Residual diagnostics
                    diag = diagnose_residual(baro_err, thrust_data,
                                            hover_start, hover_end)
                    if diag is not None:
                        _print_diagnostics(diag)
                        figures.append(plot_residual_diagnostics(diag, calib))

                    # Open-loop delay (feedback-rejected)
                    ol_delay = estimate_open_loop_delay(
                        baro_err, thrust_data,
                        hover_start, hover_end)
                    if ol_delay is not None:
                        _print_open_loop_delay(ol_delay)
                        figures.append(plot_open_loop_delay(ol_delay))
                else:
                    print("  Calibration failed: insufficient data or "
                          "thrust variation.")

    elif not has_estimator:
        print("\nError: no range sensor and no online estimator data.\n"
              "Need at least one of: distance_sensor or baro_thrust_estimate",
              file=sys.stderr)
        sys.exit(1)

    # Restore stdout and grab captured text
    sys.stdout = _orig_stdout
    summary_text = _log_buf.getvalue()

    # Write PDF with log name in filename
    if figures:
        # Add summary page as final page
        summary_lines = summary_text.strip().split("\n")
        figures.append(plot_summary_text(summary_lines))

        pdf_path = os.path.join(output_dir, f"{log_name}.pdf")

        with PdfPages(pdf_path) as pdf:
            for fig in figures:
                pdf.savefig(fig)
                plt.close(fig)
        print(f"\n  Saved: {pdf_path}")
    else:
        print("\nNo plots generated.")

    print("\nDone.")


if __name__ == "__main__":
    main()
