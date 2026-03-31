#!/usr/bin/env python3
"""
Motor command → ESC RPM latency analysis.

Measures the delay between actuator_motors commands and ESC RPM telemetry
response using cross-correlation and step-response analysis.

Note on telemetry delay: DShot serial telemetry has ~1ms UART round-trip
per motor, polled round-robin. With N motors, each motor's RPM updates
every N cycles. The measured command→RPM delay includes this telemetry
latency. Subtract ~N ms (N = motor count) for the true electromechanical
response time.

Usage:
    python3 motor_latency_analysis.py <log.ulg> [--output-dir <dir>]
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


def get_topic(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d.data
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="ULog file path")
    parser.add_argument("--output-dir", default="/tmp/baro_analysis",
                        help="Output directory (default: /tmp/baro_analysis)")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    print(f"Loading {args.log} ...")
    ulog = ULog(args.log)
    start_us = ulog.start_timestamp

    motors = get_topic(ulog, 'actuator_motors')
    esc = get_topic(ulog, 'esc_status')

    if motors is None:
        print("ERROR: actuator_motors not found in log", file=sys.stderr)
        sys.exit(1)
    if esc is None:
        print("ERROR: esc_status not found in log", file=sys.stderr)
        sys.exit(1)

    esc_count = int(esc['esc_count'][0])
    print(f"  ESC count: {esc_count}")

    # Time arrays (seconds from log start)
    t_m = (motors['timestamp'].astype(np.int64) - np.int64(start_us)) / 1e6
    t_e = (esc['timestamp'].astype(np.int64) - np.int64(start_us)) / 1e6

    dur_m = t_m[-1] - t_m[0]
    dur_e = t_e[-1] - t_e[0]
    print(f"  actuator_motors: {len(t_m)} samples, {dur_m:.1f}s, ~{len(t_m)/dur_m:.1f}Hz")
    print(f"  esc_status:      {len(t_e)} samples, {dur_e:.1f}s, ~{len(t_e)/dur_e:.1f}Hz")

    # --- Per-ESC telemetry timestamp analysis ---
    print(f"\n--- Per-ESC Telemetry Timestamps ---")
    for i in range(esc_count):
        ts_key = f"esc[{i}].timestamp"
        if ts_key in esc:
            esc_ts = esc[ts_key].astype(np.int64)
            main_ts = esc['timestamp'].astype(np.int64)
            # Delta between per-ESC timestamp and main esc_status timestamp
            dt = (main_ts - esc_ts) / 1e3  # ms
            valid = dt > 0
            if valid.sum() > 0:
                print(f"  ESC[{i}] telemetry age at publish: "
                      f"mean={dt[valid].mean():.1f}ms, "
                      f"median={np.median(dt[valid]):.1f}ms, "
                      f"max={dt[valid].max():.1f}ms")

    # --- Mean motor command ---
    motor_chs = []
    for i in range(esc_count):
        key = f"control[{i}]"
        if key in motors:
            ch = motors[key].copy()
            ch[~np.isfinite(ch)] = 0
            motor_chs.append(np.clip(ch, 0, 1))
    mean_cmd = np.mean(motor_chs, axis=0)

    # --- Mean ESC RPM ---
    rpm_chs = []
    for i in range(esc_count):
        key = f"esc[{i}].esc_rpm"
        if key in esc:
            rpm_chs.append(esc[key].astype(float))
    mean_rpm = np.mean(rpm_chs, axis=0)

    print(f"\n  RPM range: {mean_rpm.min():.0f} - {mean_rpm.max():.0f}")
    print(f"  Motor cmd range: {mean_cmd.min():.3f} - {mean_cmd.max():.3f}")

    # --- Interpolate to common 1kHz time base ---
    t_start = max(t_m[0], t_e[0])
    t_end = min(t_m[-1], t_e[-1])
    t_common = np.arange(t_start, t_end, 0.001)
    cmd_interp = np.interp(t_common, t_m, mean_cmd)
    rpm_interp = np.interp(t_common, t_e, mean_rpm)

    # Normalize for cross-correlation
    cmd_norm = cmd_interp - cmd_interp.mean()
    rpm_norm = rpm_interp - rpm_interp.mean()
    if cmd_norm.std() > 0:
        cmd_norm /= cmd_norm.std()
    if rpm_norm.std() > 0:
        rpm_norm /= rpm_norm.std()

    # --- Cross-correlation ---
    max_lag_ms = 200
    lags = np.arange(-max_lag_ms, max_lag_ms + 1)
    margin = max_lag_ms
    n = len(cmd_norm) - 2 * margin
    if n < 100:
        print("ERROR: not enough overlapping data for cross-correlation", file=sys.stderr)
        sys.exit(1)

    xcorr = np.array([
        np.corrcoef(cmd_norm[margin:margin + n],
                     rpm_norm[margin + l:margin + n + l])[0, 1]
        for l in lags
    ])

    peak_idx = np.argmax(xcorr)
    peak_lag = lags[peak_idx]
    peak_r = xcorr[peak_idx]

    print(f"\n--- Cross-Correlation: Command → RPM ---")
    print(f"  Peak lag: {peak_lag} ms (r = {peak_r:.4f})")
    print(f"  (positive = RPM lags command)")

    telem_delay_est = esc_count  # ~1ms per motor in round-robin
    print(f"\n  Estimated telemetry round-robin delay: ~{telem_delay_est} ms")
    print(f"  Corrected electromechanical delay: ~{peak_lag - telem_delay_est} ms")

    # --- Step response analysis ---
    dt_s = 0.001
    cmd_deriv = np.diff(cmd_interp) / dt_s
    threshold = np.percentile(np.abs(cmd_deriv), 99) * 0.3
    step_indices = np.where(np.abs(cmd_deriv) > threshold)[0]

    delays = []
    if len(step_indices) > 0:
        # Cluster nearby indices
        clusters = []
        current = [step_indices[0]]
        for idx in step_indices[1:]:
            if idx - current[-1] < 50:
                current.append(idx)
            else:
                clusters.append(current)
                current = [idx]
        clusters.append(current)

        for cluster in clusters[:30]:
            cmd_idx = cluster[0]
            if cmd_idx < 100 or cmd_idx > len(t_common) - 100:
                continue

            cmd_delta = (cmd_interp[min(cmd_idx + 20, len(cmd_interp) - 1)]
                         - cmd_interp[max(cmd_idx - 5, 0)])
            if abs(cmd_delta) < 0.02:
                continue

            rpm_before = rpm_interp[max(cmd_idx - 10, 0):cmd_idx].mean()
            rpm_window = rpm_interp[cmd_idx:min(cmd_idx + 100, len(rpm_interp))]
            if len(rpm_window) < 50:
                continue

            rpm_after = rpm_window[-1]
            rpm_change = rpm_after - rpm_before
            if abs(rpm_change) < 100:
                continue

            thresh_10 = rpm_before + 0.1 * rpm_change
            thresh_50 = rpm_before + 0.5 * rpm_change

            if rpm_change > 0:
                cross_10 = np.where(rpm_window >= thresh_10)[0]
                cross_50 = np.where(rpm_window >= thresh_50)[0]
            else:
                cross_10 = np.where(rpm_window <= thresh_10)[0]
                cross_50 = np.where(rpm_window <= thresh_50)[0]

            if len(cross_10) > 0 and len(cross_50) > 0:
                delays.append((cross_10[0], cross_50[0], t_common[cmd_idx]))

    if delays:
        d10 = [d[0] for d in delays]
        d50 = [d[1] for d in delays]
        print(f"\n--- Step Response ({len(delays)} events) ---")
        print(f"  10% RPM: mean={np.mean(d10):.1f}ms, median={np.median(d10):.1f}ms")
        print(f"  50% RPM: mean={np.mean(d50):.1f}ms, median={np.median(d50):.1f}ms")
        print(f"  (includes ~{telem_delay_est}ms telemetry delay)")

    # --- Plot ---
    pdf_path = os.path.join(args.output_dir, "motor_latency.pdf")
    with PdfPages(pdf_path) as pdf:
        fig, axes = plt.subplots(3, 1, figsize=(14, 12))

        # Full time series
        ax = axes[0]
        ax.set_title("Motor Command vs ESC RPM")
        ax.plot(t_common, cmd_interp, 'b-', alpha=0.7, label='Motor command (mean)')
        ax.set_ylabel("Command [0-1]", color='b')
        ax.set_xlabel("Time [s]")
        axr = ax.twinx()
        axr.plot(t_common, rpm_interp, 'r-', alpha=0.7, label='ESC RPM (mean)')
        axr.set_ylabel("RPM", color='r')
        ax.legend(loc='upper left')
        axr.legend(loc='upper right')

        # Zoomed view around high-activity region
        ax = axes[1]
        cmd_var = np.convolve(np.abs(cmd_deriv), np.ones(500) / 500, mode='same')
        zc = np.argmax(cmd_var)
        zs = max(0, zc - 1000)
        ze = min(len(t_common), zc + 1000)
        ax.set_title(f"Zoomed: t={t_common[zs]:.1f}-{t_common[ze]:.1f}s")
        ax.plot(t_common[zs:ze], cmd_interp[zs:ze], 'b-', label='Command')
        ax.set_ylabel("Command [0-1]", color='b')
        axr = ax.twinx()
        axr.plot(t_common[zs:ze], rpm_interp[zs:ze], 'r-', label='RPM')
        axr.set_ylabel("RPM", color='r')
        ax.legend(loc='upper left')
        axr.legend(loc='upper right')

        # Cross-correlation
        ax = axes[2]
        ax.set_title(f"Cross-Correlation: Command → RPM "
                     f"(peak={peak_lag}ms, r={peak_r:.3f}, "
                     f"corrected≈{peak_lag - telem_delay_est}ms)")
        ax.plot(lags, xcorr, 'b-')
        ax.axvline(peak_lag, color='r', linestyle='--',
                   label=f'Measured: {peak_lag}ms')
        ax.axvline(peak_lag - telem_delay_est, color='orange', linestyle='--',
                   label=f'Corrected: ~{peak_lag - telem_delay_est}ms')
        ax.axvline(0, color='gray', linestyle=':', alpha=0.5)
        ax.set_xlabel("Lag [ms] (positive = RPM lags command)")
        ax.set_ylabel("Normalized cross-correlation")
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        pdf.savefig(fig)
        plt.close()

    print(f"\n  Saved: {pdf_path}")


if __name__ == "__main__":
    main()
