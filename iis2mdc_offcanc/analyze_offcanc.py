#!/usr/bin/env python3
"""Compare IIS2MDC CFG_REG_B filter configurations from a PX4 ulog.

The driver reports the active CFG_REG_B value in debug_key_value under the key
"mdc_cfgb". This splits sensor_mag on those transitions and runs every analysis
that the data supports:

  timing    actual sample interval and dropouts (validates the 2x oversample)
  noise     per-axis RMS and spectral shape while stationary
  tempco    paired differential bias drift against the on-die temperature
  recovery  baseline restoration after a magnet perturbation
  esc       in-band interference and current coupling while flying
  sphere    sphere-fit offset, radius and residual (ArduPilot's "fitness")
  rotation  norm error vs rotation rate, and group delay against the gyro

Usage:
    ./analyze_offcanc.py log.ulg [--plot out.png]
"""

import argparse
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit("pyulog not found: pip install pyulog")

# CFG_REG_B bit values, see AN5080 table 3
CFG_NONE, CFG_LPF, CFG_OFF_CANC = 0x00, 0x01, 0x02
CFG_LABEL = {
    CFG_NONE: "none",
    CFG_LPF: "LPF only",
    CFG_OFF_CANC: "OFF_CANC",
}
CFG_ORDER = [CFG_NONE, CFG_OFF_CANC, CFG_LPF]

DEVTYPE_IIS2MDC = 0x0E
G_TO_MG = 1000.0
DEG = np.pi / 180.0


# ---------------------------------------------------------------- log loading

def get_dataset(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d
    return None


def list_instances(ulog, name):
    return [d for d in ulog.data_list if d.name == name]


def decode_key(data, i):
    """debug_key_value.key is char[10]; pyulog may expose it split per index."""
    if "key" in data.data:
        raw = data.data["key"][i]
        return raw.decode() if isinstance(raw, bytes) else str(raw)
    chars = []
    for k in range(10):
        field = "key[%d]" % k
        if field not in data.data:
            break
        c = int(data.data[field][i])
        if c == 0:
            break
        chars.append(chr(c))
    return "".join(chars)


def pick_mag(ulog, requested):
    """Return the sensor_mag instance for the IIS2MDC, or the requested one."""
    instances = list_instances(ulog, "sensor_mag")
    if not instances:
        sys.exit("no sensor_mag in log")

    print("sensor_mag instances:")
    chosen = None
    for d in instances:
        devid = int(d.data["device_id"][0])
        devtype = (devid >> 16) & 0xFF
        bus = (devid >> 3) & 0x1F
        addr = (devid >> 8) & 0xFF
        tag = " <- IIS2MDC" if devtype == DEVTYPE_IIS2MDC else ""
        print("  [%d] device_id=%d devtype=0x%02X bus=%d addr=0x%02X n=%d%s"
              % (d.multi_id, devid, devtype, bus, addr, len(d.data["timestamp"]), tag))
        if devtype == DEVTYPE_IIS2MDC and chosen is None:
            chosen = d

    if requested is not None:
        chosen = get_dataset(ulog, "sensor_mag", requested)
        if chosen is None:
            sys.exit("no sensor_mag instance %d" % requested)
    elif chosen is None:
        chosen = instances[0]
        print("  ! no instance reports devtype 0x%02X, falling back to instance %d"
              % (DEVTYPE_IIS2MDC, chosen.multi_id))
    print("  using instance %d\n" % chosen.multi_id)
    return chosen


def config_timeline(ulog):
    """[(t_us, cfg_reg_b)] from debug_key_value, falling back to the parameter."""
    dkv = get_dataset(ulog, "debug_key_value")
    points = []
    if dkv is not None:
        t = dkv.data["timestamp"]
        v = dkv.data["value"]
        for i in range(len(t)):
            if decode_key(dkv, i) == "mdc_cfgb":
                points.append((int(t[i]), int(round(float(v[i])))))
    if points:
        return points, "debug_key_value"

    # Fall back to IIS2MDC_FILT, which covers manual toggling without cycle mode.
    mode_to_cfg = {0: CFG_NONE, 1: CFG_OFF_CANC, 2: CFG_LPF}
    initial = ulog.initial_parameters.get("IIS2MDC_FILT")
    if initial is not None and int(initial) in mode_to_cfg:
        points.append((ulog.start_timestamp, mode_to_cfg[int(initial)]))
    for t, name, value in ulog.changed_parameters:
        if name == "IIS2MDC_FILT" and int(value) in mode_to_cfg:
            points.append((int(t), mode_to_cfg[int(value)]))
    if points:
        return points, "IIS2MDC_FILT parameter"

    return [], None


def infer_timeline(t_us, xyz, slot_s, guard_s=0.4, step_s=0.02):
    """Recover the cycle phase from the data when the timeline was not logged.

    The second difference of the samples is the discriminator. For white noise its
    variance is 6*sigma^2; for a two-tap average it is 1*sigma^2, a 6:1 separation.
    Smooth rotation barely registers in it (a 500 mG field turning at 90 deg/s
    contributes ~0.12 mG against ~4.5 mG of noise), so this works while moving.

    It cannot tell OFF_CANC from LPF; both average and look identical, which is
    exactly why LPF is the control arm. It finds the un-averaged 'none' slot and
    labels the rest from the driver's fixed order, none -> OFF_CANC -> LPF.
    """
    t = (t_us - t_us[0]) / 1e6
    d2 = xyz[2:] - 2.0 * xyz[1:-1] + xyz[:-2]
    power = np.sum(d2 * d2, axis=1)
    tp = t[1:-1]
    period = 3.0 * slot_s

    best = None
    for phi in np.arange(0.0, period, step_s):
        offset = (tp - phi) % period
        within = offset % slot_s
        ok = (within > guard_s) & (within < slot_s - guard_s)
        slot = np.floor(offset / slot_s).astype(int)
        med = []
        for k in range(3):
            m = ok & (slot == k)
            med.append(float(np.median(power[m])) if m.sum() > 20 else np.nan)
        med = np.array(med)
        if not np.all(np.isfinite(med)):
            continue
        hi = int(np.argmax(med))
        others = float(np.mean([med[i] for i in range(3) if i != hi]))
        if others <= 0:
            continue
        score = med[hi] / others
        if best is None or score > best[0]:
            best = (score, float(phi), hi, med)
    if best is None:
        return None, None

    score, phi, hi, med = best
    # Slot hi is un-averaged; the driver advances none -> OFF_CANC -> LPF.
    label = {hi: CFG_NONE, (hi + 1) % 3: CFG_OFF_CANC, (hi + 2) % 3: CFG_LPF}
    points, k = [], 0
    while True:
        edge = phi + k * slot_s
        if edge > t[-1]:
            break
        if edge >= 0:
            points.append((int(t_us[0] + edge * 1e6), label[k % 3]))
        k += 1
    return points, (score, phi, med, label)


def build_segments(points, t_start, t_end, settle_us):
    """Collapse the timeline into [(t0, t1, cfg)] with a post-transition settle."""
    points = sorted(points)
    merged = []
    for t, cfg in points:
        if merged and merged[-1][1] == cfg:
            continue
        merged.append((t, cfg))

    segments = []
    for i, (t, cfg) in enumerate(merged):
        t1 = merged[i + 1][0] if i + 1 < len(merged) else t_end
        t0 = max(t, t_start) + settle_us
        if t1 > t0:
            segments.append((t0, t1, cfg))
    return segments


# ------------------------------------------------------------------ utilities

def ols(x, y):
    """Least-squares slope with its standard error."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    n = len(x)
    if n < 3 or np.ptp(x) == 0:
        return None
    X = np.vstack([x, np.ones(n)]).T
    beta = np.linalg.lstsq(X, y, rcond=None)[0]
    resid = y - X @ beta
    dof = n - 2
    s2 = float(resid @ resid) / dof
    cov = s2 * np.linalg.inv(X.T @ X)
    return float(beta[0]), float(beta[1]), float(np.sqrt(cov[0, 0]))


def fit_sphere(p):
    """Linear-form sphere fit. Returns centre, radius, signed residuals."""
    A = np.hstack([2.0 * p, np.ones((len(p), 1))])
    b = np.sum(p * p, axis=1)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    c = sol[:3]
    r = float(np.sqrt(sol[3] + c @ c))
    resid = np.linalg.norm(p - c, axis=1) - r
    return c, r, resid


def coverage(p, c):
    """1.0 means directions are spread evenly over the sphere."""
    u = p - c
    n = np.linalg.norm(u, axis=1)
    u = u[n > 0] / n[n > 0, None]
    if len(u) < 4:
        return 0.0
    return float(3.0 * np.linalg.eigvalsh(u.T @ u / len(u))[0])


def detrend(xyz, t):
    """Remove a linear trend so slow ambient and thermal drift do not count as noise."""
    tt = t - t[0]
    trend = np.polyfit(tt, xyz, 1)
    return xyz - np.column_stack([np.polyval(trend[:, k], tt) for k in range(3)])


def quiescent(seg, min_static=0.8, floor_mg=30.0):
    """True if a segment is fit for the stationary analyses.

    Rejects segments that are mostly rotating, and segments holding a gross
    excursion such as a magnet poke, which would otherwise be reported as noise
    or as a bias shift.
    """
    if float(np.mean(seg["static"])) < min_static:
        return False
    resid = detrend(seg["xyz"] * G_TO_MG, seg["t"] / 1e6)
    dev = np.linalg.norm(resid - np.median(resid, axis=0), axis=1)
    scale = 1.4826 * float(np.median(np.abs(dev - np.median(dev))))
    return float(np.max(dev)) < max(8.0 * scale, floor_mg)


def clean_windows(seg, min_len, guard=0.5, floor_mg=30.0):
    """Contiguous stretches inside a segment that are still and undisturbed.

    A pinned run is one long segment, so whole-segment rejection would throw away
    a whole magnet-poke log. This keeps the quiet stretches between the pokes,
    with a guard band either side of each excursion to drop the approach ramp.
    """
    t = seg["t"] / 1e6
    resid = detrend(seg["xyz"] * G_TO_MG, t)
    dev = np.linalg.norm(resid - np.median(resid, axis=0), axis=1)
    scale = 1.4826 * float(np.median(np.abs(dev - np.median(dev))))
    bad = dev > max(8.0 * scale, floor_mg)

    widened = bad
    if bad.any():
        # Widen each excursion by the guard band, in samples. The kernel must stay
        # no longer than the signal or convolve('same') returns the kernel length.
        span = max(1, int(round(guard / max(float(np.median(np.diff(t))), 1e-6))))
        span = min(span, (len(bad) - 1) // 2)
        if span > 0:
            widened = np.convolve(bad.astype(int), np.ones(2 * span + 1), mode="same") > 0

    ok = seg["static"] & ~widened
    out, start = [], None
    for i, good in enumerate(ok):
        if good and start is None:
            start = i
        elif not good and start is not None:
            if i - start >= min_len:
                out.append(slice(start, i))
            start = None
    if start is not None and len(ok) - start >= min_len:
        out.append(slice(start, len(ok)))
    return out


def fit_ellipsoid(p):
    """Full quadric fit. Returns centre, mean radius, residuals in field units.

    A sphere fit cannot absorb soft iron or per-axis scale error, so a large
    sphere residual next to a small ellipsoid residual points at the airframe,
    not at the sensor configuration.
    """
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z,
                         2 * x, 2 * y, 2 * z])
    try:
        q = np.linalg.lstsq(D, np.ones(len(p)), rcond=None)[0]
    except np.linalg.LinAlgError:
        return None
    A = np.array([[q[0], q[3], q[4]], [q[3], q[1], q[5]], [q[4], q[5], q[2]]])
    if np.any(np.linalg.eigvalsh(A) <= 0):
        return None
    c = -np.linalg.solve(A, q[6:9])
    k = 1.0 + float(c @ A @ c)
    if k <= 0:
        return None
    evals, evecs = np.linalg.eigh(A / k)
    if np.any(evals <= 0):
        return None
    m = evecs @ np.diag(np.sqrt(evals)) @ evecs.T
    n = np.linalg.norm((p - c) @ m.T, axis=1)
    radius = float(np.mean(np.linalg.norm(p - c, axis=1)))
    return c, radius, (n / np.mean(n) - 1.0) * radius


def robust_sigma(resid):
    """MAD-based per-axis scale, insensitive to the odd outlier."""
    med = np.median(resid, axis=0)
    return 1.4826 * np.median(np.abs(resid - med), axis=0)


def welch(x, fs, nperseg):
    """One-sided PSD, Hann windowed, 50% overlap."""
    if len(x) < nperseg:
        return None, None
    win = np.hanning(nperseg)
    scale = 1.0 / (fs * np.sum(win ** 2))
    acc = []
    for i in range(0, len(x) - nperseg + 1, nperseg // 2):
        s = x[i:i + nperseg]
        spec = np.abs(np.fft.rfft((s - s.mean()) * win)) ** 2 * scale
        spec[1:-1] *= 2.0
        acc.append(spec)
    return np.fft.rfftfreq(nperseg, 1.0 / fs), np.mean(acc, axis=0)


def cross_accumulate(a, g, fs, nfft, acc):
    """Accumulate windowed cross-spectra of a against g for a delay estimate."""
    if len(a) < nfft:
        return 0
    win = np.hanning(nfft)
    n = 0
    for i in range(0, len(a) - nfft + 1, nfft // 2):
        A = np.fft.rfft((a[i:i + nfft] - a[i:i + nfft].mean()) * win)
        G = np.fft.rfft((g[i:i + nfft] - g[i:i + nfft].mean()) * win)
        acc.append((np.fft.rfftfreq(nfft, 1.0 / fs), A * np.conj(G), np.abs(A) * np.abs(G)))
        n += 1
    return n


def cross_delay(acc, fmin=0.15, fmax_frac=0.25, frac=0.05):
    """Delay from the phase slope of the averaged cross-spectrum, seconds.

    A single correlation peak is far too flat to resolve half a sample, and
    zero-padded correlation pins the peak at zero lag; averaging the cross
    spectrum over many windows and fitting phase against frequency does resolve
    it. Positive means a lags g.
    """
    if not acc:
        return None
    f = acc[0][0]
    S = sum(x[1] for x in acc)
    mag = sum(x[2] for x in acc)
    band = (f >= fmin) & (f <= fmax_frac * 2.0 * f[-1])
    if not band.any():
        return None
    # Keep only bins that actually carry signal, or noise phase dominates the fit.
    band &= mag > frac * mag[band].max()
    if band.sum() < 3:
        return None
    phi = np.unwrap(np.angle(S[band]))
    w, fb = mag[band], f[band]
    den = float(np.sum(w * fb ** 2))
    return None if den <= 0 else -float(np.sum(w * fb * phi)) / (2 * np.pi * den)


def interp_current(ulog, t_mag):
    """Battery current interpolated onto the mag timestamps, amps."""
    d = get_dataset(ulog, "battery_status")
    if d is None or "current_a" not in d.data:
        return None
    return np.interp(t_mag, d.data["timestamp"], d.data["current_a"])


def interp_reference_temp(ulog, t_mag):
    """Board temperature from a sensor other than the magnetometer.

    The IIS2MDC's own temperature output turns out to depend on CFG_REG_B, so it
    cannot be the regressor when the point is to compare configurations.
    """
    for name in ("sensor_baro", "sensor_accel", "sensor_gyro"):
        d = get_dataset(ulog, name)
        if d is not None and "temperature" in d.data:
            return np.interp(t_mag, d.data["timestamp"], d.data["temperature"]), name
    return None, None


def interp_armed(ulog, t_mag):
    """Armed flag on the mag timestamps. Fallback when the FC has no current sensing."""
    d = get_dataset(ulog, "actuator_armed")
    if d is None or "armed" not in d.data:
        return None
    a = np.asarray(d.data["armed"], dtype=float)
    return np.interp(t_mag, d.data["timestamp"], a) > 0.5


def interp_gyro(ulog, t_mag):
    """Angular velocity interpolated onto the mag timestamps, rad/s."""
    d = get_dataset(ulog, "vehicle_angular_velocity")
    if d is not None:
        t = d.data["timestamp"]
        cols = [d.data["xyz[%d]" % i] for i in range(3)]
    else:
        d = get_dataset(ulog, "sensor_gyro")
        if d is None:
            return None
        t = d.data["timestamp"]
        cols = [d.data[a] for a in ("x", "y", "z")]
    return np.column_stack([np.interp(t_mag, t, c) for c in cols])


def fmt(v, digits=3):
    return "n/a" if v is None or (isinstance(v, float) and not np.isfinite(v)) \
        else ("%.*f" % (digits, v))


def header(title):
    print("\n" + title)
    print("-" * len(title))


# ------------------------------------------------------------------- analyses

def analyse_timing(by_cfg):
    header("Timing (mag sample interval)")
    print("  %-10s %8s %8s %8s %8s" % ("config", "rate Hz", "med ms", "jit ms", "gaps %"))
    for cfg in CFG_ORDER:
        segs = by_cfg.get(cfg)
        if not segs:
            continue
        dt = np.concatenate([np.diff(s["t"]) for s in segs if len(s["t"]) > 1]) / 1e6
        if len(dt) == 0:
            continue
        med = float(np.median(dt))
        jit = float(np.std(dt))
        gaps = 100.0 * float(np.mean(dt > 1.5 * med))
        print("  %-10s %8s %8s %8s %8s" % (CFG_LABEL[cfg], fmt(1.0 / med, 1),
                                           fmt(med * 1e3, 2), fmt(jit * 1e3, 2), fmt(gaps, 1)))
    print("\n  Expect ~100 Hz. Jitter above ~2 ms smears the spectra below.")


def analyse_noise(by_cfg, nperseg):
    header("Noise while stationary (mG)")
    print("  Predicted by AN5080 table 9 at LP=0: none 4.5, OFF_CANC/LPF 3.0")
    print("  %-10s %7s %7s %7s %9s %8s" % ("config", "x", "y", "z", "hi/lo PSD", "segs"))
    psds = {}
    for cfg in CFG_ORDER:
        windows = [(s, w) for s in by_cfg.get(cfg, [])
                   for w in clean_windows(s, nperseg)]
        if not windows:
            continue
        rms, spectra, fs_used = [], [], None
        for s, w in windows:
            xyz = s["xyz"][w] * G_TO_MG
            t = s["t"][w] / 1e6
            resid = detrend(xyz, t)
            rms.append(robust_sigma(resid))
            fs_used = 1.0 / float(np.median(np.diff(t)))
            for k in range(3):
                f, p = welch(resid[:, k], fs_used, nperseg)
                if f is not None:
                    spectra.append(p)
        segs = windows
        rms = np.sqrt(np.mean(np.square(rms), axis=0))
        ratio = None
        if spectra:
            p = np.mean(spectra, axis=0)
            f = np.fft.rfftfreq(nperseg, 1.0 / fs_used)
            psds[cfg] = (f, p)
            # Bands kept clear of DC and of the Nyquist bin, which is not doubled
            # in the one-sided convention and would bias the ratio low.
            lo = p[(f >= 2) & (f <= 12)].mean()
            hi = p[(f >= 0.33 * fs_used) & (f <= 0.45 * fs_used)].mean()
            ratio = hi / lo if lo > 0 else None
        print("  %-10s %7s %7s %7s %9s %8d" % (CFG_LABEL[cfg], fmt(rms[0], 2), fmt(rms[1], 2),
                                               fmt(rms[2], 2), fmt(ratio, 3), len(segs)))
    print("\n  hi/lo PSD is band power over 33-45% of the rate against 2-12 Hz.")
    print("  A two-tap average has a zero at ODR/2, giving ~0.13 there; flat noise")
    print("  gives ~1.0. If OFF_CANC is not attenuated the set/reset path is not")
    print("  working (check the 220 nF cap on pin 5, AN5080 table 1).")
    return psds


def analyse_bias(by_cfg, segments_ordered):
    """Absolute field per configuration while still.

    Offset cancellation exists to remove the AMR's intrinsic offset, so with the
    board untouched the difference between configurations is that offset. This is
    the largest effect the part can show and it is a plain DC comparison.
    """
    header("Static bias by configuration (mG)")
    stats = []
    for s in segments_ordered:
        m = s["static"]
        if m.sum() < 20 or not quiescent(s):
            continue
        stats.append((s["cfg"], np.mean(s["xyz"][m], axis=0) * G_TO_MG))
    if not stats:
        print("  no stationary segments")
        return

    print("  %-10s %9s %9s %9s %9s %6s" % ("config", "x", "y", "z", "|B|", "segs"))
    for cfg in CFG_ORDER:
        sel = [v for c, v in stats if c == cfg]
        if not sel:
            continue
        v = np.mean(sel, axis=0)
        print("  %-10s %9s %9s %9s %9s %6d"
              % (CFG_LABEL[cfg], fmt(v[0], 1), fmt(v[1], 1), fmt(v[2], 1),
                 fmt(float(np.linalg.norm(v)), 1), len(sel)))

    # Adjacent pairs only, so a drifting ambient field cannot masquerade as offset.
    print("\n  Paired difference between adjacent segments:")
    print("  %-22s %9s %9s %9s %9s %6s"
          % ("pair", "dx", "dy", "dz", "|delta|", "n"))
    for a, b in ((CFG_NONE, CFG_OFF_CANC), (CFG_LPF, CFG_OFF_CANC), (CFG_NONE, CFG_LPF)):
        deltas = []
        for i in range(len(stats) - 1):
            c0, v0 = stats[i]
            c1, v1 = stats[i + 1]
            if {c0, c1} != {a, b}:
                continue
            deltas.append((v0 - v1) if c0 == a else (v1 - v0))
        if len(deltas) < 2:
            continue
        d = np.mean(deltas, axis=0)
        print("  %-22s %9s %9s %9s %9s %6d"
              % ("%s - %s" % (CFG_LABEL[a], CFG_LABEL[b]), fmt(d[0], 1), fmt(d[1], 1),
                 fmt(d[2], 1), fmt(float(np.linalg.norm(d)), 1), len(deltas)))
    have_ref = any(s["reftemp"] is not None for s in segments_ordered)
    if have_ref:
        print("\n  Temperature the magnetometer itself reports, against an independent")
        print("  sensor on the same board:")
        print("  %-10s %12s %12s %6s" % ("config", "mag temp C", "board temp C", "segs"))
        for cfg in CFG_ORDER:
            pairs = [(float(np.mean(s["temp"][s["static"]])),
                      float(np.mean(s["reftemp"][s["static"]])))
                     for s in segments_ordered
                     if s["cfg"] == cfg and s["static"].sum() >= 20 and quiescent(s)
                     and s["reftemp"] is not None]
            if len(pairs) < 3:
                continue
            print("  %-10s %12s %12s %6d"
                  % (CFG_LABEL[cfg], fmt(float(np.mean([a for a, _ in pairs])), 2),
                     fmt(float(np.mean([b for _, b in pairs])), 2), len(pairs)))

    print("\n  A large 'none - OFF_CANC' delta is the intrinsic AMR offset that the")
    print("  set/reset pair removes. Calibration can absorb a constant offset, but")
    print("  it eats headroom and it is the term that walks with temperature.")


def analyse_tempco(by_cfg, segments_ordered, temp_source="magnetometer"):
    header("Bias vs temperature")
    print("  temperature reference: %s" % temp_source)
    stats = []
    for s in segments_ordered:
        m = s["static"]
        if m.sum() < 20 or not quiescent(s):
            continue
        ref = s["reftemp"] if s["reftemp"] is not None else s["temp"]
        stats.append((s["cfg"], float(np.mean(ref[m])),
                      np.mean(s["xyz"][m], axis=0) * G_TO_MG))

    if len(stats) < 4:
        print("  not enough stationary segments")
        return
    temps = np.array([s[1] for s in stats])
    print("  temperature span: %.1f to %.1f C over %d segments"
          % (temps.min(), temps.max(), len(stats)))
    if temps.max() - temps.min() < 3.0:
        print("  ! span under 3 C, tempco estimates will be meaningless")

    print("\n  Per-config drift (mG/C), confounded with any ambient drift:")
    print("  %-10s %10s %10s %10s" % ("config", "x", "y", "z"))
    for cfg in CFG_ORDER:
        sel = [s for s in stats if s[0] == cfg]
        if len(sel) < 3:
            continue
        t = [s[1] for s in sel]
        out = []
        for k in range(3):
            fit = ols(t, [s[2][k] for s in sel])
            out.append("%s" % fmt(fit[0], 2) if fit else "n/a")
        print("  %-10s %10s %10s %10s" % (CFG_LABEL[cfg], *out))

    # Paired differences between adjacent segments cancel any drift the two
    # configurations share, which is the whole point of interleaving them.
    print("\n  Paired differential drift (mG/C), ambient cancelled:")
    print("  %-22s %10s %10s %10s %6s" % ("pair", "x", "y", "z", "n"))
    for a, b in ((CFG_NONE, CFG_OFF_CANC), (CFG_LPF, CFG_OFF_CANC), (CFG_NONE, CFG_LPF)):
        pair_t, pair_d = [], []
        for i in range(len(stats) - 1):
            c0, t0, m0 = stats[i]
            c1, t1, m1 = stats[i + 1]
            if {c0, c1} != {a, b}:
                continue
            delta = (m0 - m1) if c0 == a else (m1 - m0)
            pair_t.append(0.5 * (t0 + t1))
            pair_d.append(delta)
        if len(pair_d) < 3:
            continue
        pair_d = np.array(pair_d)
        out = []
        for k in range(3):
            fit = ols(pair_t, pair_d[:, k])
            out.append("%s+-%s" % (fmt(fit[0], 2), fmt(fit[2], 2)) if fit else "n/a")
        label = "%s - %s" % (CFG_LABEL[a], CFG_LABEL[b])
        print("  %-22s %10s %10s %10s %6d" % (label, *out, len(pair_d)))
    print("\n  A non-zero 'none - OFF_CANC' slope is offset cancellation earning its")
    print("  keep: that drift is uncalibratable in flight. 'LPF only - OFF_CANC'")
    print("  isolates it from the bandwidth change.")


def analyse_recovery(by_cfg, threshold_mg, tol_mg):
    header("Magnet perturbation recovery")
    bins = [(0.02, 0.08), (0.08, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 2.0)]
    found_any = False
    print("  %-10s %6s %s" % ("config", "events", "  ".join("%.0f-%.0fms" % (a * 1e3, b * 1e3)
                                                            for a, b in bins)))
    for cfg in CFG_ORDER:
        residuals, events = [], 0
        for s in by_cfg.get(cfg, []):
            if float(np.mean(s["static"])) < 0.5:
                continue
            t = s["t"] / 1e6
            xyz = s["xyz"] * G_TO_MG
            norm = np.linalg.norm(xyz, axis=1)
            if len(t) < 100:
                continue
            base = np.median(norm)
            hot = np.abs(norm - base) > threshold_mg
            if not hot.any():
                continue
            # Contiguous excursions, merging gaps under 100 ms.
            idx = np.flatnonzero(hot)
            groups, start = [], idx[0]
            for a, b in zip(idx[:-1], idx[1:]):
                if t[b] - t[a] > 0.1:
                    groups.append((start, a))
                    start = b
            groups.append((start, idx[-1]))

            for g0, g1 in groups:
                pre = (t >= t[g0] - 1.0) & (t <= t[g0] - 0.1)
                if pre.sum() < 20 or t[-1] - t[g1] < bins[-1][1]:
                    continue
                ref = np.median(xyz[pre], axis=0)
                row = []
                for a, b in bins:
                    m = (t >= t[g1] + a) & (t < t[g1] + b)
                    row.append(float(np.linalg.norm(np.mean(xyz[m], axis=0) - ref))
                               if m.sum() >= 2 else np.nan)
                residuals.append(row)
                events += 1
        if events:
            found_any = True
            med = np.nanmedian(np.array(residuals), axis=0)
            print("  %-10s %6d %s" % (CFG_LABEL[cfg], events,
                                      "  ".join("%9s" % fmt(v, 1) for v in med)))
    if not found_any:
        print("  no qualifying events (need a >%.0f mG excursion with 1 s before and"
              % threshold_mg)
        print("  2 s after inside one config segment; run this with IIS2MDC_FILT")
        print("  pinned to 0 then 1 rather than in cycle mode)")
        return
    print("\n  Residual offset in mG after the magnet leaves, tolerance %.0f mG."
          % tol_mg)
    print("  OFF_CANC re-sets the AMR every sample. With it off the set pulse only")
    print("  fires every 63 ODR (630 ms at 100 Hz, AN5080 section 8), so expect a")
    print("  residual that persists and then steps away near the 500-1000 ms bin.")


def band_rms(f, p, lo, hi):
    """RMS in a band from a one-sided PSD."""
    m = (f >= lo) & (f <= hi)
    if not m.any():
        return None
    return float(np.sqrt(np.sum(p[m]) * (f[1] - f[0])))


def analyse_interference(by_cfg, threshold_a, nperseg):
    header("ESC interference with motors running")
    def running(s):
        if s["current"] is not None and float(np.median(s["current"])) > threshold_a:
            return True
        return s["armed"] is not None and float(np.mean(s["armed"])) > 0.5

    live = {cfg: [s for s in segs if running(s)] for cfg, segs in by_cfg.items()}
    if not any(live.values()):
        print("  no segments above %.1f A and none armed; this needs the flight log"
              % threshold_a)
        return
    has_current = any(s["current"] is not None and float(np.median(s["current"])) > threshold_a
                      for segs in live.values() for s in segs)
    if not has_current:
        print("  ! no usable battery current; selecting on the armed flag instead and")
        print("    skipping the current-coupling column")

    print("  In-band is what the estimator actually fuses; the high band is mostly")
    print("  the filter shape and is only a consistency check.")
    print("  %-10s %11s %11s %11s %8s"
          % ("config", "0.5-5 Hz", "5-45 Hz", "mG per A", "segs"))
    for cfg in CFG_ORDER:
        segs = live.get(cfg, [])
        if not segs:
            continue
        spectra, cur, mag_all, fs_used = [], [], [], None
        for s in segs:
            xyz = s["xyz"] * G_TO_MG
            t = s["t"] / 1e6
            if len(t) < nperseg:
                continue
            resid = detrend(xyz, t)
            fs_used = 1.0 / float(np.median(np.diff(t)))
            for k in range(3):
                f, p = welch(resid[:, k], fs_used, nperseg)
                if f is not None:
                    spectra.append(p)
            cur.append(s["current"] if s["current"] is not None else np.zeros(len(t)))
            mag_all.append(xyz)
        if not spectra:
            continue
        p = np.mean(spectra, axis=0)
        f = np.fft.rfftfreq(nperseg, 1.0 / fs_used)
        # Slope of field against current, which the filter should not touch.
        cur = np.concatenate(cur)
        mag_all = np.vstack(mag_all)
        slopes = [ols(cur, mag_all[:, k]) for k in range(3)] if has_current else []
        worst = max((abs(s[0]) for s in slopes if s), default=None)
        print("  %-10s %11s %11s %11s %8d"
              % (CFG_LABEL[cfg], fmt(band_rms(f, p, 0.5, 5.0), 2),
                 fmt(band_rms(f, p, 5.0, 0.45 * fs_used), 2),
                 fmt(worst, 2), len(segs)))
    print("\n  Motor and PWM harmonics sit above Nyquist and alias down. Aliasing")
    print("  happens at the ADC, before the two-tap average, so both configs should")
    print("  inherit it equally: expect the 0.5-5 Hz column to match. A real")
    print("  difference there is a genuine finding either way.")
    print("  The 5-45 Hz column should be lower for OFF_CANC and LPF purely from")
    print("  the cos-shaped roll-off; that is not interference rejection.")
    print("  'mG per A' is the DC coupling to current and should be identical")
    print("  across configs; a mismatch means the filter is affecting the response.")
    print("  Vehicle motion also lands in band and contaminates both configs")
    print("  equally, so only differences between configs mean anything here.")


def analyse_sphere(by_cfg):
    header("Sphere fit over rotation")
    print("  %-10s %9s %9s %9s %9s %8s %7s %8s %7s"
          % ("config", "off x", "off y", "off z", "|offset|", "radius", "resid",
             "ellipsd", "cover"))
    fits = {}
    for cfg in CFG_ORDER:
        pts = [s["xyz"][s["rotating"]] for s in by_cfg.get(cfg, [])]
        pts = [p for p in pts if len(p)]
        if not pts:
            continue
        p = np.vstack(pts) * G_TO_MG
        if len(p) < 200:
            continue
        c, r, resid = fit_sphere(p)
        fits[cfg] = (c, r)
        ell = fit_ellipsoid(p)
        ell_rms = float(np.sqrt(np.mean(ell[2] ** 2))) if ell else None
        print("  %-10s %9s %9s %9s %9s %8s %7s %8s %7s"
              % (CFG_LABEL[cfg], fmt(c[0], 1), fmt(c[1], 1), fmt(c[2], 1),
                 fmt(float(np.linalg.norm(c)), 1), fmt(r, 1),
                 fmt(float(np.sqrt(np.mean(resid ** 2))), 2), fmt(ell_rms, 2),
                 fmt(coverage(p, c), 2)))
    if not fits:
        print("  no rotating data")
    else:
        print("\n  'resid' is the analogue of ArduPilot's calibration fitness, in mG.")
        print("  Offset cancellation removes the sensor's intrinsic offset, so with it")
        print("  on |offset| should be smaller (only real hard iron left) while resid")
        print("  should be unchanged. 'cover' below ~0.3 means poor orientation spread.")
        print("  'ellipsd' is the same fit allowing soft iron and per-axis scale. If it")
        print("  collapses relative to 'resid', the scatter is airframe geometry, not")
        print("  the sensor configuration.")
    return fits


def analyse_rotation(by_cfg, fits, odr_hz, nfft):
    header("Rotation-rate effects")
    predicted = 1.0 / (8.0 * odr_hz ** 2)
    print("  Norm shrink from averaging two samples across rotation:")
    print("  predicted d|B|/|B| = w^2/(8*ODR^2) = %.2e per (rad/s)^2" % predicted)
    print("  %-10s %14s %14s %8s" % ("config", "measured", "std err", "n"))
    for cfg in CFG_ORDER:
        if cfg not in fits:
            continue
        c, r = fits[cfg]
        w2, dn = [], []
        for s in by_cfg.get(cfg, []):
            m = s["rotating"]
            if s["omega"] is None or m.sum() < 10:
                continue
            p = s["xyz"][m] * G_TO_MG
            w = np.linalg.norm(s["omega"][m], axis=1)
            w2.append(w ** 2)
            dn.append(1.0 - np.linalg.norm(p - c, axis=1) / r)
        if not w2:
            continue
        w2 = np.concatenate(w2)
        dn = np.concatenate(dn)
        fit = ols(w2, dn)
        if fit:
            print("  %-10s %14.2e %14.2e %8d" % (CFG_LABEL[cfg], fit[0], fit[2], len(w2)))
    print("\n  At 90 deg/s in a 500 mG field this is 0.015 mG, about 200x under the")
    print("  noise floor, so expect a std err that straddles both zero and the")
    print("  prediction. A large positive slope for OFF_CANC only would be the")
    print("  first real evidence against it.")

    header("Group delay against the gyro")
    # Hand rotation lives near 0.3-1 Hz, so the window has to be long enough to
    # put real bins under it. Size it from the longest rotating stretch present.
    runs = [int(s["rotating"].sum()) for cfg in CFG_ORDER for s in by_cfg.get(cfg, [])
            if s["omega"] is not None]
    longest = max(runs) if runs else 0
    if longest < 64:
        print("  no rotating stretch long enough; raise IIS2MDC_CYCLE and rotate more")
        return
    nfft = int(np.clip(1 << int(np.floor(np.log2(longest))), 64, 512))
    print("  window %d samples (%.1f s), bin spacing %.2f Hz"
          % (nfft, nfft / odr_hz, odr_hz / nfft))
    print("  %-10s %10s %10s %8s" % ("config", "lag ms", "vs none", "windows"))
    measured = {}
    for cfg in CFG_ORDER:
        if cfg not in fits:
            continue
        c, _ = fits[cfg]
        acc, windows = [], 0
        for s in by_cfg.get(cfg, []):
            m = s["rotating"]
            if s["omega"] is None or m.sum() < nfft:
                continue
            t = s["t"][m] / 1e6
            p = s["xyz"][m] - c / G_TO_MG
            n = np.linalg.norm(p, axis=1)
            if np.any(n <= 0):
                continue
            b = p / n[:, None]
            dt = np.diff(t)
            if np.any(dt <= 0):
                continue
            # Field rotation only sees w perpendicular to B, but b carries the very
            # delay being measured, so the reference must come from the gyro alone.
            # |w| tracks the perpendicular rate up to a slowly varying geometric
            # factor, which does not shift the phase slope.
            mag_rate = np.linalg.norm(np.cross(b[:-1], b[1:]), axis=1) / dt
            gyro_rate = np.linalg.norm(s["omega"][m][:-1], axis=1)
            fs = 1.0 / float(np.median(dt))
            windows += cross_accumulate(mag_rate, gyro_rate, fs, nfft, acc)
        tau = cross_delay(acc)
        if tau is not None:
            measured[cfg] = (tau * 1e3, windows)
    if not measured:
        print("  no usable rotation content in band; rotate faster and for longer")
        return
    base = measured.get(CFG_NONE, (None, 0))[0]
    for cfg in CFG_ORDER:
        if cfg not in measured:
            continue
        lag, windows = measured[cfg]
        rel = "-" if base is None or cfg == CFG_NONE else fmt(lag - base, 2)
        print("  %-10s %10s %10s %8d" % (CFG_LABEL[cfg], fmt(lag, 2), rel, windows))
    print("\n  Only the 'vs none' column means anything. The absolute figure carries")
    print("  an arbitrary offset from the driver's read-time timestamp and from the")
    print("  rate estimator, and it moves around with rotation speed; the difference")
    print("  does not. A two-tap average costs 0.5/ODR, so expect about +5 ms for")
    print("  OFF_CANC and the same for LPF; against synthetic truth this estimator")
    print("  returns 3.7 to 4.9 ms for a real 5 ms, so treat it as +-25 percent.")
    print("  It confirms the filter shape, it does not discriminate OFF_CANC from")
    print("  LPF, which is exactly why LPF is the honest control arm.")
    print("  Needs sustained rotation: raise IIS2MDC_CYCLE to 5000 for this part,")
    print("  and treat a window count under ~10 as unreliable.")


def make_plot(psds, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not available, skipping plot")
        return
    if not psds:
        print("\nno spectra to plot")
        return
    fig, ax = plt.subplots(figsize=(8, 5))
    for cfg, (f, p) in psds.items():
        ax.semilogy(f, np.sqrt(p), label=CFG_LABEL[cfg])
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("ASD (mG/sqrt(Hz))")
    ax.set_title("IIS2MDC stationary noise by CFG_REG_B")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print("\nwrote %s" % path)


# ----------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--instance", type=int, default=None,
                    help="sensor_mag instance (default: auto-detect IIS2MDC)")
    ap.add_argument("--settle", type=float, default=200.0,
                    help="ms discarded after each config change (default 200)")
    ap.add_argument("--static-rate", type=float, default=3.0,
                    help="deg/s below which a sample counts as stationary")
    ap.add_argument("--rot-rate", type=float, default=20.0,
                    help="deg/s above which a sample counts as rotating")
    ap.add_argument("--event-threshold", type=float, default=150.0,
                    help="mG excursion that marks a magnet perturbation")
    ap.add_argument("--tolerance", type=float, default=10.0,
                    help="mG considered recovered")
    ap.add_argument("--nperseg", type=int, default=128, help="PSD window length")
    ap.add_argument("--current-threshold", type=float, default=3.0,
                    help="amps above which motors count as running")
    ap.add_argument("--odr", type=float, default=100.0, help="sensor ODR in Hz")
    ap.add_argument("--start", type=float, default=None,
                    help="seconds from log start to begin at")
    ap.add_argument("--end", type=float, default=None,
                    help="seconds from log start to stop at (trims a corrupt tail)")
    ap.add_argument("--plot", default=None, help="write a spectrum PNG here")
    args = ap.parse_args()

    ulog = ULog(args.log)
    mag = pick_mag(ulog, args.instance)

    t = np.asarray(mag.data["timestamp"], dtype=np.int64)
    xyz = np.column_stack([mag.data[a] for a in ("x", "y", "z")]).astype(float)
    temp = np.asarray(mag.data["temperature"], dtype=float)
    if args.start is not None or args.end is not None:
        rel = (t - t[0]) / 1e6
        keep = np.ones(len(t), bool)
        if args.start is not None:
            keep &= rel >= args.start
        if args.end is not None:
            keep &= rel <= args.end
        print("time range %.1f-%.1f s: keeping %d of %d samples\n"
              % (args.start or 0.0, args.end if args.end is not None else rel[-1],
                 keep.sum(), len(t)))
        t, xyz, temp = t[keep], xyz[keep], temp[keep]

    reftemp, temp_source = interp_reference_temp(ulog, t)
    omega = interp_gyro(ulog, t)
    current = interp_current(ulog, t)
    armed = interp_armed(ulog, t)
    if omega is None:
        print("! no gyro in log; motion classification and lag are unavailable")

    points, source = config_timeline(ulog)
    if not points:
        mode = ulog.initial_parameters.get("IIS2MDC_FILT")
        slot_ms = ulog.initial_parameters.get("IIS2MDC_CYCLE")
        if mode is not None and int(mode) == 3 and slot_ms:
            print("! debug_key_value missing (SDLOG_PROFILE needs bit 5, +32), but")
            print("  IIS2MDC_FILT=3 so the phase can be recovered from the data.")
            points, info = infer_timeline(t, xyz, float(slot_ms) / 1000.0)
            if points is None:
                sys.exit("  phase recovery failed; re-record with SDLOG_PROFILE=2081")
            score, phi, med, label = info
            print("  slot %.1f s, phase %.2f s, separation %.1fx (expect ~6x)"
                  % (float(slot_ms) / 1000.0, phi, score))
            print("  second-difference power per slot: %s"
                  % "  ".join("%d=%.3g[%s]" % (k, med[k], CFG_LABEL[label[k]])
                              for k in range(3)))
            if score < 2.5:
                print("  ! separation too weak to trust these labels")
            source = "inferred from second-difference power"
        else:
            print("! no config timeline found, treating the whole log as one config")
            points = [(int(t[0]), CFG_OFF_CANC)]
            source = "assumed"
    print("config timeline from %s, %d transitions" % (source, len(points)))

    segments = build_segments(points, int(t[0]), int(t[-1]), args.settle * 1000)
    by_cfg, ordered = {}, []
    for t0, t1, cfg in segments:
        m = (t >= t0) & (t < t1)
        if m.sum() < 10:
            continue
        w = omega[m] if omega is not None else None
        wn = np.linalg.norm(w, axis=1) if w is not None else None
        seg = {
            "cfg": cfg, "t": t[m], "xyz": xyz[m], "temp": temp[m], "omega": w,
            "current": current[m] if current is not None else None,
            "armed": armed[m] if armed is not None else None,
            "reftemp": reftemp[m] if reftemp is not None else None,
            "static": (wn < args.static_rate * DEG) if wn is not None
                      else np.ones(int(m.sum()), bool),
            "rotating": (wn > args.rot_rate * DEG) if wn is not None
                        else np.zeros(int(m.sum()), bool),
        }
        by_cfg.setdefault(cfg, []).append(seg)
        ordered.append(seg)

    if not ordered:
        sys.exit("no usable segments")

    print("segments: " + ", ".join("%s=%d" % (CFG_LABEL.get(c, hex(c)), len(v))
                                   for c, v in sorted(by_cfg.items())))
    total = sum(len(s["t"]) for s in ordered)
    stat = sum(int(s["static"].sum()) for s in ordered)
    rot = sum(int(s["rotating"].sum()) for s in ordered)
    print("samples: %d total, %d stationary, %d rotating" % (total, stat, rot))

    analyse_timing(by_cfg)
    psds = analyse_noise(by_cfg, args.nperseg)
    analyse_bias(by_cfg, ordered)
    analyse_tempco(by_cfg, ordered, temp_source or "magnetometer")
    analyse_recovery(by_cfg, args.event_threshold, args.tolerance)
    analyse_interference(by_cfg, args.current_threshold, args.nperseg)
    fits = analyse_sphere(by_cfg)
    if fits and omega is not None:
        analyse_rotation(by_cfg, fits, args.odr, args.nperseg)
    if args.plot:
        make_plot(psds, args.plot)


if __name__ == "__main__":
    main()
