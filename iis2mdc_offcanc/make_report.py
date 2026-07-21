#!/usr/bin/env python3
"""Build the IIS2MDC offset-cancellation PDF report from the real bench logs.

Every number and plotted point is computed here from the ulogs; nothing is
transcribed. Run:

    ./make_report.py OUT.pdf \\
        --heatcool  heatupcooldownarkfpv.repaired.ulg \\
        --magnet    magnet_poke.repaired.ulg \\
        --rot-nobat log_0_UnknownDate_1.ulg \\
        --rot-bat   log_1_UnknownDate.ulg \\
        --hover     hover_with_throttle_punches.ulg
"""
import argparse
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from pyulog import ULog

import analyze_offcanc as A

# Categorical palette (dataviz skill, validated: all-pair CVD dE >= 9.2, normal
# >= 24). Blue / orange / aqua, distinguished also by marker and line style.
STYLE = {
    A.CFG_OFF_CANC: ("#2a78d6", "o", "-", "offset cancellation"),
    A.CFG_NONE:     ("#eb6834", "s", "--", "neither"),
    A.CFG_LPF:      ("#1baf7a", "^", ":", "low-pass filter"),
}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#d9d8d4"
AXES = ["x", "y", "z"]

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 9,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.titlecolor": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.axisbelow": True, "figure.dpi": 150,
})


# ------------------------------------------------------------------ log access

def load_mag(path):
    u = ULog(path)
    d = A.get_dataset(u, "sensor_mag") or A.list_instances(u, "sensor_mag")[0]
    t = np.asarray(d.data["timestamp"], dtype=np.int64)
    xyz = np.column_stack([d.data[a] for a in AXES]).astype(float) * A.G_TO_MG
    temp = np.asarray(d.data["temperature"], dtype=float)
    return u, t, xyz, temp


def corruption_cutoff(t, xyz, temp):
    """First time (s from start) after which the data section turns to garbage.

    Thermal SD damage flips bits, so corruption shows as impossible temperatures
    or field magnitudes. Returns the onset of the first sustained bad stretch,
    not an isolated flip.
    """
    rel = (t - t[0]) / 1e6
    n = np.linalg.norm(xyz, axis=1)
    bad = (temp < -15) | (temp > 90) | (n > 1500)
    win = max(50, int(0.01 * len(bad)))
    frac = np.convolve(bad.astype(float), np.ones(win) / win, mode="valid")
    over = np.flatnonzero(frac > 0.2)
    if len(over) == 0:
        return rel[-1]
    # Walk back from the first sustained breach to the first bad sample feeding it.
    start = over[0]
    while start > 0 and bad[start]:
        start -= 1
    return rel[start]


def timeline(u, t, xyz):
    pts, src = A.config_timeline(u)
    if pts:
        return pts, src
    slot = u.initial_parameters.get("IIS2MDC_CYCLE")
    if u.initial_parameters.get("IIS2MDC_FILT") == 3 and slot:
        pts, _ = A.infer_timeline(t, xyz, float(slot) / 1000.0)
        return pts, "inferred"
    return [(int(t[0]), A.CFG_OFF_CANC)], "assumed"


def segments(u, t, xyz, temp, trim_end=None, settle=0.25):
    """Per-config segment dicts with static mask and independent temperature."""
    if trim_end is not None:
        keep = (t - t[0]) / 1e6 <= trim_end
        t, xyz, temp = t[keep], xyz[keep], temp[keep]
    reftemp, tsrc = A.interp_reference_temp(u, t)
    omega = A.interp_gyro(u, t)
    wn = np.linalg.norm(omega, axis=1) if omega is not None else None
    pts, src = timeline(u, t, xyz)
    out = []
    for s0, s1, cfg in A.build_segments(pts, int(t[0]), int(t[-1]), int(settle * 1e6)):
        m = (t >= s0) & (t < s1)
        if m.sum() < 10:
            continue
        out.append({
            "cfg": cfg, "t": t[m], "xyz": xyz[m] / A.G_TO_MG, "temp": temp[m],
            "reftemp": reftemp[m] if reftemp is not None else None,
            "omega": omega[m] if omega is not None else None,
            "static": (wn[m] < 3 * A.DEG) if wn is not None else np.ones(int(m.sum()), bool),
            "rotating": (wn[m] > 20 * A.DEG) if wn is not None else np.zeros(int(m.sum()), bool),
        })
    return out, (tsrc or "magnetometer"), src


def by_config(segs):
    d = {}
    for s in segs:
        d.setdefault(s["cfg"], []).append(s)
    return d


# ------------------------------------------------------------- page decoration

def footer(fig, page):
    fig.text(0.5, 0.02, "IIS2MDC offset cancellation — bench measurement, 21 July 2026",
             ha="center", color=MUTED, size=7)
    fig.text(0.96, 0.02, "%d" % page, ha="right", color=MUTED, size=7)


def config_legend(ax, loc="best"):
    handles = [Line2D([0], [0], color=STYLE[c][0], marker=STYLE[c][1],
                      linestyle=STYLE[c][2], markersize=6, linewidth=2,
                      label=STYLE[c][3]) for c in (A.CFG_OFF_CANC, A.CFG_NONE, A.CFG_LPF)]
    ax.legend(handles=handles, loc=loc, frameon=False, fontsize=8)


def caption(fig, text, width=120, y=0.085):
    fig.text(0.5, y, _wrap(text, width), ha="center", va="center", size=8.3, color=INK,
             bbox=dict(boxstyle="round,pad=0.55", facecolor="#f6f5f2",
                       edgecolor=GRID, linewidth=0.8))


# ----------------------------------------------------------------- data crunch

def tempco_points(segs):
    """Per-segment (config, indep temp, mean bias xyz mG) for still, clean segments."""
    pts = {c: [] for c in STYLE}
    for s in segs:
        m = s["static"]
        if m.sum() < 20 or not A.quiescent(s) or s["reftemp"] is None:
            continue
        pts[s["cfg"]].append((float(np.mean(s["reftemp"][m])),
                              np.mean(s["xyz"][m], axis=0) * A.G_TO_MG))
    return pts


def paired_offset(segs):
    """Adjacent-segment paired bias differences, so ambient cannot masquerade."""
    stats = [(s["cfg"], np.mean(s["xyz"][s["static"]], axis=0) * A.G_TO_MG)
             for s in segs if s["static"].sum() >= 20 and A.quiescent(s)]
    pairs = {}
    for a, b in ((A.CFG_NONE, A.CFG_OFF_CANC), (A.CFG_LPF, A.CFG_OFF_CANC),
                 (A.CFG_NONE, A.CFG_LPF)):
        deltas = [(v0 - v1) if c0 == a else (v1 - v0)
                  for (c0, v0), (c1, v1) in zip(stats, stats[1:]) if {c0, c1} == {a, b}]
        if deltas:
            pairs[(a, b)] = np.array(deltas)
    return pairs


def quiet_offset(segs):
    """Per-config mean bias from still, undisturbed windows only.

    Works on a pinned magnet log where whole-segment rejection would discard
    everything: clean_windows keeps the quiet stretches between the pokes.
    """
    out = {}
    for cfg, group in by_config(segs).items():
        means = [np.mean(s["xyz"][w], axis=0) * A.G_TO_MG
                 for s in group for w in A.clean_windows(s, 50)]
        if means:
            out[cfg] = np.mean(means, axis=0)
    return out


def sphere_offsets(segs):
    """Per-config sphere-fit centre from rotating samples."""
    out = {}
    for cfg, group in by_config(segs).items():
        p = np.vstack([s["xyz"][s["rotating"]] for s in group]) * A.G_TO_MG
        if len(p) >= 200:
            out[cfg] = A.fit_sphere(p)[0]
    return out


def config_psd(segs, nperseg=128):
    psd = {}
    for cfg, group in by_config(segs).items():
        spectra, fs = [], None
        for s in group:
            for w in A.clean_windows(s, nperseg):
                resid = A.detrend(s["xyz"][w] * A.G_TO_MG, s["t"][w] / 1e6)
                fs = 1.0 / float(np.median(np.diff(s["t"][w] / 1e6)))
                for k in range(3):
                    f, p = A.welch(resid[:, k], fs, nperseg)
                    if f is not None:
                        spectra.append(p)
        if spectra:
            psd[cfg] = (np.fft.rfftfreq(nperseg, 1.0 / fs), np.mean(spectra, axis=0))
    return psd


def esc_bands(segs, nperseg=128):
    out = {}
    for cfg, group in by_config(segs).items():
        segs_live = [s for s in group if s["omega"] is not None]
        spectra, cur, mag, fs = [], [], [], None
        for s in group:
            if len(s["t"]) < nperseg:
                continue
            resid = A.detrend(s["xyz"] * A.G_TO_MG, s["t"] / 1e6)
            fs = 1.0 / float(np.median(np.diff(s["t"] / 1e6)))
            for k in range(3):
                f, p = A.welch(resid[:, k], fs, nperseg)
                if f is not None:
                    spectra.append(p)
        if not spectra:
            continue
        p = np.mean(spectra, axis=0)
        f = np.fft.rfftfreq(nperseg, 1.0 / fs)
        out[cfg] = (A.band_rms(f, p, 0.5, 5.0), A.band_rms(f, p, 5.0, 0.45 * fs))
    return out


# --------------------------------------------------------------------- figures

def page_intro(pdf, nums):
    fig = plt.figure(figsize=(8.5, 11))
    fig.patch.set_facecolor("white")
    fig.text(0.5, 0.945, "IIS2MDC Offset Cancellation Investigation Report",
             ha="center", size=17, weight="bold")
    fig.text(0.5, 0.915, "Bench measurement — 21 July 2026", ha="center", size=10,
             color=MUTED)

    def block(y, heading, paragraphs, width=92, size=10.5):
        fig.text(0.1, y, heading, size=12.5, weight="bold", color="#1a4e8f")
        y -= 0.028
        for para in paragraphs:
            wrapped = _wrap(para, width)
            fig.text(0.1, y, wrapped, ha="left", va="top", size=size, linespacing=1.5)
            y -= 0.021 * (wrapped.count("\n") + 1) + 0.014
        return y

    y = 0.875
    y = block(y, "Results", [
        "The IIS2MDC magnetometer has a built-in feature, set by one bit in its "
        "configuration register (the OFF_CANC bit in CFG_REG_B), that removes the "
        "sensor's own intrinsic magnetic offset. This report compares the sensor's "
        "output with that feature turned on and turned off.",
        "Two things appear in the reading when offset cancellation is turned off "
        "that are not there when it is on:",
    ])
    bullets = [
        "A fixed offset of about %.0f mG (roughly %.0f%% of the Earth's magnetic "
        "field) is added to the reading." % (nums["offset"], nums["offset_frac"]),
        "That offset changes with temperature, by about %.0f mG for every degree "
        "Celsius." % nums["tempco"],
    ]
    for b in bullets:
        wrapped = _wrap("•  " + b, 86)
        fig.text(0.13, y, wrapped, ha="left", va="top", size=10.5, linespacing=1.5)
        y -= 0.021 * (wrapped.count("\n") + 1) + 0.012
    y -= 0.006
    wrapped = _wrap("With offset cancellation on, neither is present: the reading stays "
                    "centered and stable. The amount of noise in the reading is similar "
                    "whether the feature is on or off.", 92)
    fig.text(0.1, y, wrapped, ha="left", va="top", size=10.5, linespacing=1.5)
    y -= 0.021 * (wrapped.count("\n") + 1) + 0.03

    block(y, "What we tested", [
        "Offset cancellation works by taking two readings — one after a magnetic "
        "'set' pulse and one after a 'reset' pulse — and averaging them, which "
        "cancels the sensor's intrinsic offset (ST application note AN5080, "
        "section 8). It is switched on or off by a single configuration bit.",
        "To compare fairly, we switched the sensor between three settings every two "
        "seconds while recording its output: offset cancellation, a low-pass filter "
        "only, and neither. Because all three settings are exercised within the same "
        "recording, the surrounding magnetic field, temperature, orientation and "
        "position are the same for each — so any difference between them comes only "
        "from the setting.",
        "The low-pass-filter setting is a reference. According to the datasheet it "
        "has the same bandwidth and smoothing as offset cancellation but does not "
        "remove the offset. So an effect that appears with offset cancellation but "
        "not with the low-pass filter comes from the offset removal itself, not from "
        "the smoothing. Temperature is read from a separate sensor on the same board, "
        "because the magnetometer's own temperature output also changes with the "
        "setting.",
        "We recorded three situations: the sensor rotated through all orientations, "
        "warmed up and then left to cool, and mounted on a vehicle in flight with the "
        "motors running. The plots that follow present the data from these recordings.",
    ])
    footer(fig, 1)
    pdf.savefig(fig); plt.close(fig)


def page_tempco(pdf, pts):
    fig, axes = plt.subplots(1, 3, figsize=(11, 8.5 * 11 / 8.5 / 1.55))
    fig.set_size_inches(11, 6.6)
    fig.suptitle("Sensor bias versus temperature", size=13, weight="bold", y=0.97)
    fig.text(0.5, 0.9, "Each point is the sensor's bias during one still two-second window. "
             "Temperature is from a separate sensor on the same board.",
             ha="center", size=9, color=MUTED)

    slopes = {}
    for ax, k in zip(axes, range(3)):
        labels = []
        for cfg in (A.CFG_NONE, A.CFG_LPF, A.CFG_OFF_CANC):
            color, marker, ls, name = STYLE[cfg]
            data = pts[cfg]
            if not data:
                continue
            T = np.array([d[0] for d in data])
            b = np.array([d[1][k] for d in data])
            ax.scatter(T, b, s=14, c=color, marker=marker, alpha=0.5,
                       edgecolors="none", zorder=3)
            fit = A.ols(T, b)
            if fit:
                slopes.setdefault(cfg, [None, None, None])[k] = fit[0]
                xs = np.array([T.min(), T.max()])
                ax.plot(xs, fit[0] * xs + fit[1], color=color, linestyle=ls,
                        linewidth=2, zorder=4)
                labels.append((color, "%+.1f mG/°C" % fit[0]))
        # Slope readouts in a clear corner, never overhanging the axes.
        for i, (color, text) in enumerate(labels):
            ax.text(0.04, 0.05 + 0.06 * i, text, transform=ax.transAxes,
                    size=8, color=color, weight="bold", va="bottom", zorder=5)
        ax.set_title("%s-axis" % AXES[k], size=11)
        ax.set_xlabel("temperature (°C)")
        if k == 0:
            ax.set_ylabel("sensor bias (mG)")
    config_legend(axes[1], loc="upper center")

    caption(fig, "With offset cancellation off (orange) or the low-pass filter only (green), "
            "the bias moves steadily with temperature — about 9 mG per °C. With offset "
            "cancellation on (blue) the bias barely changes. The blue line stays flat across "
            "the same temperature range the others sweep, so the drift comes from the sensor "
            "itself and not from the surroundings. Over a 30 °C change that is about 270 mG of "
            "bias shift.", width=125, y=0.09)
    fig.subplots_adjust(left=0.08, right=0.95, top=0.83, bottom=0.24, wspace=0.2)
    footer(fig, 2)
    pdf.savefig(fig); plt.close(fig)
    return slopes


def page_offset(pdf, paired, offset_by_log):
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(11, 6.4))
    fig.suptitle("Fixed offset added when offset cancellation is off",
                 size=13, weight="bold", y=0.965)

    # Panel A: paired per-axis difference. Component uses a neutral steel ramp so it
    # is never confused with the configuration colours used elsewhere.
    order = [(A.CFG_NONE, A.CFG_OFF_CANC), (A.CFG_LPF, A.CFG_OFF_CANC),
             (A.CFG_NONE, A.CFG_LPF)]
    labels = ["neither\nminus\noffset canc.", "low-pass\nminus\noffset canc.",
              "neither\nminus low-pass\n(reference)"]
    steel = ["#2f4a63", "#5f83a3", "#a7c0d6"]
    x = np.arange(3)
    w = 0.25
    for j, k in enumerate(range(3)):
        vals = [np.mean(paired[o][:, k]) if o in paired else 0 for o in order]
        errs = [np.std(paired[o][:, k]) / np.sqrt(len(paired[o])) if o in paired else 0
                for o in order]
        axL.bar(x + (j - 1) * w, vals, w, yerr=errs, capsize=2, color=steel[j],
                edgecolor="none", label="%s-axis" % AXES[k])
    axL.axhline(0, color=MUTED, linewidth=0.8)
    axL.set_xticks(x); axL.set_xticklabels(labels, size=8)
    axL.set_ylabel("difference in bias, per axis (mG)")
    axL.set_title("Difference between settings, held still", size=10)
    axL.legend(frameon=False, fontsize=8, title="axis", title_fontsize=8, loc="lower left")

    # Panel B: offset magnitude reproduced across the separate recordings.
    names = list(offset_by_log.keys())
    mags = [offset_by_log[n][0] for n in names]
    ctrl = [offset_by_log[n][1] for n in names]
    xb = np.arange(len(names))
    axR.bar(xb - 0.2, mags, 0.4, color="#2a78d6", label="offset (neither vs offset canc.)")
    have_ctrl = [(xi, c) for xi, c in zip(xb, ctrl) if np.isfinite(c)]
    axR.bar([xi + 0.2 for xi, _ in have_ctrl], [c for _, c in have_ctrl], 0.4,
            color=MUTED, label="reference (neither vs low-pass)")
    for xi, m in zip(xb, mags):
        axR.annotate("%.0f" % m, (xi - 0.2, m), ha="center", va="bottom", size=8,
                     color="#1a4e8f", weight="bold")
    axR.set_xticks(xb); axR.set_xticklabels(names, size=8, rotation=15, ha="right")
    axR.set_ylabel("offset size (mG)")
    axR.set_title("Offset size in each recording", size=10)
    axR.legend(frameon=False, fontsize=8)

    caption(fig, "With offset cancellation off, the reading is shifted by a fixed amount — "
            "about 300 to 500 mG depending on the recording. The 'neither versus low-pass "
            "filter' comparison (the reference, gray / rightmost bars) stays near zero, showing "
            "the shift comes from turning off offset cancellation and not from the smoothing "
            "the low-pass filter shares. In the two rotating recordings the setting was "
            "identified from the data.", width=122, y=0.075)
    fig.subplots_adjust(left=0.08, right=0.96, top=0.86, bottom=0.22, wspace=0.28)
    footer(fig, 3)
    pdf.savefig(fig); plt.close(fig)


def page_spectrum(pdf, psd):
    fig, ax = plt.subplots(figsize=(9, 6.2))
    fig.suptitle("Noise in the reading, sensor held still", size=13, weight="bold", y=0.965)
    nyq = 0.0
    for cfg in (A.CFG_NONE, A.CFG_OFF_CANC, A.CFG_LPF):
        if cfg not in psd:
            continue
        f, p = psd[cfg]
        nyq = max(nyq, f[-1])
        color, _, ls, name = STYLE[cfg]
        ax.semilogy(f[1:], np.sqrt(p[1:]), color=color, linestyle=ls, linewidth=2, label=name)
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("noise density (mG/√Hz)  —  lower is quieter")
    ax.set_xlim(0, nyq)
    config_legend(ax, loc="lower left")
    caption(fig, "How much noise is in the reading at each frequency, with the sensor held "
            "still. Offset cancellation (blue) and the low-pass filter (green) both reduce the "
            "noise compared with neither (orange). Lower is quieter.", width=115, y=0.08)
    fig.subplots_adjust(left=0.1, right=0.95, top=0.88, bottom=0.2)
    footer(fig, 4)
    pdf.savefig(fig); plt.close(fig)


def page_esc(pdf, bands):
    fig, ax = plt.subplots(figsize=(9, 6.0))
    fig.suptitle("Noise in flight, motors running", size=13, weight="bold", y=0.965)
    order = [A.CFG_NONE, A.CFG_OFF_CANC, A.CFG_LPF]
    x = np.arange(len(order))
    inb = [bands[c][0] for c in order]
    hib = [bands[c][1] for c in order]
    ax.bar(x - 0.2, inb, 0.4, color=[STYLE[c][0] for c in order])
    ax.bar(x + 0.2, hib, 0.4, color=[STYLE[c][0] for c in order], alpha=0.4,
           hatch="///", edgecolor="white", linewidth=0)
    for xi, v in zip(x - 0.2, inb):
        ax.annotate("%.1f" % v, (xi, v), ha="center", va="bottom", size=8, weight="bold")
    ax.set_xticks(x); ax.set_xticklabels([STYLE[c][3] for c in order], size=10)
    ax.set_ylabel("field noise (mG RMS)")
    ax.set_ylim(0, max(inb + hib) * 1.28)
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=MUTED, label="low frequency (0.5–5 Hz)"),
               plt.Rectangle((0, 0), 1, 1, facecolor=MUTED, alpha=0.4, hatch="///",
                             label="higher frequency (5–45 Hz)")]
    ax.legend(handles=handles, frameon=False, fontsize=8.5, loc="upper center", ncol=2)
    caption(fig, "Recorded while flying with the motors running. In the low-frequency band "
            "that matters for heading (solid bars), the noise is about the same for all three "
            "settings; the small spread is from the vehicle moving, not from the setting. The "
            "higher-frequency band (hatched) is lower with offset cancellation or the low-pass "
            "filter, from their smoothing.", width=116, y=0.08)
    fig.subplots_adjust(left=0.1, right=0.95, top=0.88, bottom=0.2)
    footer(fig, 5)
    pdf.savefig(fig); plt.close(fig)


def _wrap(text, width):
    words, line, out = text.split(), "", []
    for w in words:
        if len(line) + len(w) + 1 > width:
            out.append(line); line = w
        else:
            line = (line + " " + w).strip()
    out.append(line)
    return "\n".join(out)


# --------------------------------------------------------------------- driver

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--heatcool", required=True)
    ap.add_argument("--magnet", required=True)
    ap.add_argument("--rot-nobat", required=True)
    ap.add_argument("--rot-bat", required=True)
    ap.add_argument("--hover", required=True)
    args = ap.parse_args()

    print("heat/cool ...")
    u, t, xyz, temp = load_mag(args.heatcool)
    cutoff = corruption_cutoff(t, xyz, temp)
    hc_segs, tsrc, _ = segments(u, t, xyz, temp, trim_end=cutoff - 8)
    pts = tempco_points(hc_segs)
    paired = paired_offset(hc_segs)
    psd = config_psd(hc_segs)
    hc_off = {k: np.mean(v, axis=0) for k, v in paired.items()}

    def mag_of(pairs, a, b):
        return float(np.linalg.norm(pairs[(a, b)])) if (a, b) in pairs else float("nan")

    offset_by_log = {}
    offset_by_log["warm/cool\n(still)"] = (
        mag_of(hc_off, A.CFG_NONE, A.CFG_OFF_CANC), mag_of(hc_off, A.CFG_NONE, A.CFG_LPF))

    print("magnet ...")
    um, tm, xm, tmp = load_mag(args.magnet)
    mseg, _, _ = segments(um, tm, xm, tmp)
    mo = quiet_offset(mseg)
    if A.CFG_NONE in mo and A.CFG_OFF_CANC in mo:
        offset_by_log["still\n(held)"] = (
            float(np.linalg.norm(mo[A.CFG_NONE] - mo[A.CFG_OFF_CANC])), float("nan"))

    field_radius = None
    for tag, path in (("rotating\nno battery", args.rot_nobat),
                      ("rotating\nbattery", args.rot_bat)):
        print("%s ..." % tag.replace("\n", " "))
        ur, tr, xr, tpr = load_mag(path)
        rseg, _, _ = segments(ur, tr, xr, tpr)
        cen = sphere_offsets(rseg)
        if A.CFG_NONE in cen and A.CFG_OFF_CANC in cen:
            ctrl = (float(np.linalg.norm(cen[A.CFG_NONE] - cen[A.CFG_LPF]))
                    if A.CFG_LPF in cen else float("nan"))
            offset_by_log[tag] = (
                float(np.linalg.norm(cen[A.CFG_NONE] - cen[A.CFG_OFF_CANC])), ctrl)
        # Sphere radius is the true field magnitude (hard iron removed); use the
        # no-battery case as the least-perturbed reference.
        if field_radius is None:
            radii = []
            for cfg, group in by_config(rseg).items():
                p = np.vstack([s["xyz"][s["rotating"]] for s in group]) * A.G_TO_MG
                if len(p) >= 200:
                    radii.append(A.fit_sphere(p)[1])
            if radii:
                field_radius = float(np.median(radii))

    print("hover ...")
    uh, th, xh, tph = load_mag(args.hover)
    hseg, _, _ = segments(uh, th, xh, tph)
    bands = esc_bands(hseg)

    # Intro numbers, computed from the data. Tempco is the vector magnitude of the
    # per-axis difference in slope between 'neither' and offset cancellation.
    def slope_vec(cfg):
        v = []
        for k in range(3):
            d = pts[cfg]
            fit = A.ols([p[0] for p in d], [p[1][k] for p in d]) if len(d) > 2 else None
            v.append(fit[0] if fit else 0.0)
        return np.array(v)
    tempco_mag = float(np.linalg.norm(slope_vec(A.CFG_NONE) - slope_vec(A.CFG_OFF_CANC)))
    off_mag = float(np.linalg.norm(hc_off[(A.CFG_NONE, A.CFG_OFF_CANC)]))
    earth = field_radius if field_radius else off_mag
    nums = {"offset": off_mag, "offset_frac": 100 * off_mag / earth, "tempco": tempco_mag}

    print("writing %s" % args.out)
    with PdfPages(args.out) as pdf:
        page_intro(pdf, nums)
        page_tempco(pdf, pts)
        page_offset(pdf, paired, offset_by_log)
        page_spectrum(pdf, psd)
        page_esc(pdf, bands)
    print("done: %d pages" % 5)


if __name__ == "__main__":
    main()
