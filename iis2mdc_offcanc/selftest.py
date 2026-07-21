#!/usr/bin/env python3
"""Feed analyze_offcanc synthetic data with known truth and check it recovers it.

Models CFG_REG_B = OFF_CANC as what AN5080 section 8 says it is: a two-tap
average of consecutive set/reset measurements. Ground truth injected here:

  noise      4.5 mG white before the average, so ~3.2 mG after it
  spectrum   flat for 'none', cos-shaped with a null at ODR/2 for OFF_CANC
  tempco     3.0 mG/C on x for 'none' only, which is the effect being hunted
  lag        5 ms (0.5/ODR) of extra group delay for OFF_CANC
  sphere     hard iron [80, -40, 25] mG, field 500 mG

The magnet block only exercises the recovery code path; it does not model
the AMR failing to re-set, so equal residuals there are expected and are
not evidence about the real part.
"""

import numpy as np

import analyze_offcanc as A

RNG = np.random.default_rng(7)
FS = 100.0
FIELD_MG = 500.0
HARD_IRON_MG = np.array([80.0, -40.0, 25.0])
NOISE_MG = 4.5
TEMPCO_MG_PER_C = 3.0
MG = 1e-3  # sensor_mag is in Gauss


def boxcar(v):
    """The two-tap average the part applies when OFF_CANC is set."""
    out = v.copy()
    out[1:] = 0.5 * (v[1:] + v[:-1])
    return out


def make_segment(cfg, t0, duration, temp, rotating=False, magnet_at=None):
    n = int(duration * FS)
    t = t0 + np.arange(n) / FS

    if rotating:
        # Yaw sweeps back and forth so the rate varies across the segment.
        yaw = 1.6 * np.sin(2 * np.pi * 0.6 * (t - t0))
        rate = 1.6 * 2 * np.pi * 0.6 * np.cos(2 * np.pi * 0.6 * (t - t0))
        pitch = 0.9 * np.sin(2 * np.pi * 0.11 * (t - t0) + 1.0)
        earth = np.column_stack([
            FIELD_MG * np.cos(pitch) * np.cos(yaw),
            FIELD_MG * np.cos(pitch) * np.sin(yaw),
            FIELD_MG * np.sin(pitch),
        ])
        omega = np.column_stack([np.zeros(n), np.zeros(n), rate])
    else:
        earth = np.tile([FIELD_MG * 0.6, FIELD_MG * 0.2, -FIELD_MG * 0.77], (n, 1))
        omega = RNG.normal(0, 0.002, (n, 3))

    field = earth + HARD_IRON_MG
    if magnet_at is not None:
        hit = (t - t0 >= magnet_at) & (t - t0 < magnet_at + 1.0)
        field = field + np.outer(hit, [900.0, 300.0, -200.0])

    # Intrinsic offset drifts with temperature; offset cancellation removes it.
    field = field + np.array([TEMPCO_MG_PER_C * (temp - 25.0), 0.0, 0.0])
    field = field + RNG.normal(0, NOISE_MG, (n, 3))

    if cfg == A.CFG_OFF_CANC:
        field = boxcar(field)
        field = field - np.array([TEMPCO_MG_PER_C * (temp - 25.0), 0.0, 0.0])

    wn = np.linalg.norm(omega, axis=1)
    return {
        "cfg": cfg,
        "t": (t * 1e6).astype(np.int64),
        "xyz": field * MG,
        "temp": np.full(n, temp),
        "omega": omega,
        "current": np.zeros(n),
        "armed": np.zeros(n, bool),
        "reftemp": np.full(n, temp),
        "static": wn < 3.0 * A.DEG,
        "rotating": wn > 20.0 * A.DEG,
    }


def make_hover_segment(cfg, t0, duration):
    """Motors running: above-Nyquist interference aliased at the ADC, filtered after."""
    n = int(duration * FS)
    t = t0 + np.arange(n) / FS
    current = 15.0 + 3.0 * np.sin(2 * np.pi * 0.3 * (t - t0))

    # Slow attitude wobble, the real field change a hovering vehicle produces.
    wobble = 15.0 * np.column_stack([np.sin(2 * np.pi * 1.1 * (t - t0)),
                                     np.cos(2 * np.pi * 0.8 * (t - t0)),
                                     np.sin(2 * np.pi * 1.4 * (t - t0) + 0.7)])

    # Motor harmonics live above Nyquist. Build them at 10x and decimate, so the
    # aliasing happens before any filtering, exactly as it does in the part.
    hi_n = n * 10
    hi_t = t0 + np.arange(hi_n) / (FS * 10)
    hf = np.zeros((hi_n, 3))
    for freq, amp in ((180.0, 8.0), (350.0, 5.0)):
        for k in range(3):
            hf[:, k] += amp * np.sin(2 * np.pi * freq * (hi_t - t0) + k)
    hf = hf[::10]

    field = (np.tile([FIELD_MG * 0.6, FIELD_MG * 0.2, -FIELD_MG * 0.77], (n, 1))
             + HARD_IRON_MG + wobble + hf
             + np.outer(current, [2.0, 1.0, -0.5])
             + RNG.normal(0, NOISE_MG, (n, 3)))

    if cfg == A.CFG_OFF_CANC:
        field = boxcar(field)

    omega = 0.2 * np.column_stack([np.sin(2 * np.pi * 1.1 * (t - t0)),
                                   np.cos(2 * np.pi * 0.8 * (t - t0)),
                                   np.zeros(n)])
    wn = np.linalg.norm(omega, axis=1)
    return {
        "cfg": cfg,
        "t": (t * 1e6).astype(np.int64),
        "xyz": field * MG,
        "temp": np.full(n, 35.0),
        "omega": omega,
        "current": current,
        "armed": np.ones(n, bool),
        "reftemp": np.full(n, 35.0),
        "static": wn < 3.0 * A.DEG,
        "rotating": wn > 20.0 * A.DEG,
    }


def build():
    ordered, t0 = [], 0.0
    # Interleaved static block over a cooling ramp, the tempco experiment.
    for i in range(40):
        cfg = A.CFG_NONE if i % 2 == 0 else A.CFG_OFF_CANC
        temp = 60.0 - 0.7 * i
        ordered.append(make_segment(cfg, t0, 1.8, temp))
        t0 += 2.0
    # Rotation block for the sphere fit and group delay.
    for i in range(12):
        cfg = A.CFG_NONE if i % 2 == 0 else A.CFG_OFF_CANC
        ordered.append(make_segment(cfg, t0, 4.8, 25.0, rotating=True))
        t0 += 5.0
    # Magnet pokes, held inside a single config segment as the report advises.
    for i in range(8):
        cfg = A.CFG_NONE if i % 2 == 0 else A.CFG_OFF_CANC
        ordered.append(make_segment(cfg, t0, 5.0, 25.0, magnet_at=1.5))
        t0 += 5.2

    # Hover block for the ESC interference analysis.
    for i in range(16):
        cfg = A.CFG_NONE if i % 2 == 0 else A.CFG_OFF_CANC
        ordered.append(make_hover_segment(cfg, t0, 4.8))
        t0 += 5.0

    by_cfg = {}
    for s in ordered:
        by_cfg.setdefault(s["cfg"], []).append(s)
    return by_cfg, ordered


def main():
    by_cfg, ordered = build()
    print("Synthetic truth: noise %.1f mG, tempco %.1f mG/C on x (none only),"
          % (NOISE_MG, TEMPCO_MG_PER_C))
    print("hard iron %s mG, field %.0f mG, OFF_CANC lag +%.0f ms"
          % (HARD_IRON_MG.tolist(), FIELD_MG, 500.0 / FS))

    A.analyse_timing(by_cfg)
    A.analyse_noise(by_cfg, 128)
    A.analyse_bias(by_cfg, ordered)
    A.analyse_tempco(by_cfg, ordered)
    A.analyse_recovery(by_cfg, 150.0, 10.0)
    A.analyse_interference(by_cfg, 3.0, 128)
    fits = A.analyse_sphere(by_cfg)
    A.analyse_rotation(by_cfg, fits, FS, 128)

    print("\n" + "=" * 70)
    print("Check: noise 4.5 vs ~3.2 | hi/lo ~1 vs <<1 | paired 'none - OFF_CANC'")
    print("x slope ~%.1f, y and z ~0 | offset ~%s | lag difference ~%.0f ms"
          % (TEMPCO_MG_PER_C, HARD_IRON_MG.tolist(), 500.0 / FS))


if __name__ == "__main__":
    main()
