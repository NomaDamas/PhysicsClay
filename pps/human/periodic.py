"""Periodic hand motion: is a hand path a weave (an oscillation across a path that travels), and what are its travel
speed, width, frequency and shape? Pure numpy; used by the live activity reader (activity.py, short windows) and by
skill extraction (pps/skills, a whole demonstration).

    a = analyze(ts, ps)            # ts (N,), ps (N,3) world metres
    ok, why = is_weave(a)          # the gates below

The method, in order:
1. Travel. A least-squares line p(t) = p0 + v t gives the travel velocity (a zero-mean oscillation does not bias it);
   `r2` is how much of the motion along v the straight travel explains, `disp` how far it got. PCA of the path gives the
   dominant direction too (`pca`), used when the travel is too slow to have a direction of its own.
2. The oscillation is the residual's principal axis across the travel (`axis`, unit xyz); `x` is the lateral offset.
3. Frequency: the peak of a Hann-windowed, zero-padded FFT of `x` inside `band`, refined by parabolic interpolation.
4. Width: the signal is band-limited to the fundamental and its harmonics (a zigzag is a triangle wave), split into
   cycles at the fundamental's zero crossings, and `amp` is the median half peak-to-peak of the cycles.
5. Periodicity strength: `peak` = fraction of the lateral power (0.3-8 Hz) in the fundamental and its harmonics, and
   `acf` = the autocorrelation at one period. A regular weave scores ~0.6-1 on both, jitter ~0-0.3.
6. Shape: the residual along the travel at f (in quadrature) makes it a circle, at 2f a figure 8, else a zigzag.

Why the gates (real-camera finding, PR #40): MediaPipe's world landmarks make a still, hanging hand jitter 1-3 cm at
~0.9 Hz, which passed the old weave rule (>= 4 mm, >= 4 zero crossings). A weave now needs a hand that travels
(speed, straightness and distance), a frequency in 1-5 Hz, a width in 3 mm-6 cm and a strong periodicity.
"""
import math

import numpy as np

UP = np.array([0.0, 0.0, 1.0])
WEAVE = {"speed_min": 0.003,       # m/s along the path (welding travel is 3-10 mm/s; a still hand's jitter 1-3 mm/s)
         "r2_min": 0.55,           # the travel is a straight-ish line (jitter wanders: r2 ~ 0-0.4)
         "disp_min": 0.012,        # m travelled inside the window
         "f_band": (1.0, 5.0),     # Hz
         "amp_band": (0.0025, 0.06),  # m, peak
         "peak_min": 0.30,         # spectral peak ratio
         "acf_min": 0.30}          # autocorrelation at one period


def _unit(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros_like(v)


def uniform(ts, ps, fs=None):
    """Resample a timed path onto a uniform grid. -> (t, p, fs). fs defaults to the median frame rate (10-60 Hz)."""
    ts, ps = np.asarray(ts, float), np.asarray(ps, float)
    if fs is None:
        d = np.diff(ts)
        d = d[d > 1e-6]
        fs = float(np.clip(1.0 / np.median(d), 10.0, 60.0)) if len(d) else 30.0
    n = max(2, int(math.floor((ts[-1] - ts[0]) * fs)) + 1)
    t = ts[0] + np.arange(n) / fs
    p = np.stack([np.interp(t, ts, ps[:, c]) for c in range(ps.shape[1])], axis=1)
    return t, p, fs


def travel_fit(t, p):
    """Least-squares travel line. -> {vel, speed, dir, r2, disp, res (residual), pca (unit), pca_ext (m)}."""
    tc = t - t.mean()
    A = np.column_stack([np.ones_like(tc), tc])
    coef, *_ = np.linalg.lstsq(A, p, rcond=None)
    vel = coef[1]
    res = p - A @ coef
    sp = float(np.linalg.norm(vel))
    d = vel / sp if sp > 1e-9 else np.zeros(3)
    c = p - p.mean(axis=0)
    w, V = np.linalg.eigh(c.T @ c / max(1, len(c)))
    pca = V[:, -1]
    if np.dot(pca, d) < 0:
        pca = -pca
    along = c @ d if sp > 1e-9 else c @ pca
    var = float(np.var(along))
    r2 = 0.0 if var < 1e-12 or sp < 1e-9 else float(np.clip(1.0 - np.var(along - sp * tc) / var, 0.0, 1.0))
    ext = float(np.ptp(c @ pca)) if len(c) else 0.0
    return {"vel": vel, "speed": sp, "dir": d, "r2": r2, "disp": sp * float(t[-1] - t[0]), "res": res, "pca": pca, "pca_ext": ext}


def spectrum(x, fs, n_fft=None):
    x = np.asarray(x, float) - float(np.mean(x))
    w = np.hanning(len(x))
    n_fft = n_fft or max(1024, 1 << int(math.ceil(math.log2(max(8, len(x)) * 8))))
    X = np.fft.rfft(x * w, n_fft)
    f = np.fft.rfftfreq(n_fft, 1.0 / fs)
    return f, np.abs(X), float(np.sum(w))


def _band_power(f, P, lo, hi):
    m = (f >= lo) & (f <= hi)
    return float(P[m].sum())


def dominant(x, fs, band=(0.5, 6.0), n_fft=None):
    """-> (freq, peak ratio, magnitude spectrum) of the strongest oscillation of x inside band."""
    f, mag, _ = spectrum(x, fs, n_fft)
    P = mag ** 2
    m = (f >= band[0]) & (f <= band[1])
    if not m.any() or P[m].max() <= 0:
        return 0.0, 0.0, (f, mag)
    idx = np.nonzero(m)[0]
    k = int(idx[np.argmax(P[idx])])
    fk = f[k]
    if 0 < k < len(P) - 1:                         # parabolic peak interpolation on the log magnitude
        a, b, c = (math.log(max(v, 1e-30)) for v in mag[k - 1:k + 2])
        den = a - 2 * b + c
        if abs(den) > 1e-12:
            fk = f[k] + 0.5 * (a - c) / den * (f[1] - f[0])
    total = _band_power(f, P, 0.3, 8.0)
    on = sum(_band_power(f, P, h * fk * 0.8, h * fk * 1.2) for h in (1, 2, 3) if h * fk * 0.8 < 8.0)
    return float(fk), (on / total if total > 0 else 0.0), (f, mag)


def acf_at(x, lag):
    """Normalised autocorrelation of x at an integer lag (unbiased: the overlap's own variance)."""
    x = np.asarray(x, float) - float(np.mean(x))
    lag = int(lag)
    if lag <= 0 or lag >= len(x) - 3:
        return 0.0
    a, b = x[:-lag], x[lag:]
    den = math.sqrt(float(np.dot(a, a)) * float(np.dot(b, b)))
    return float(np.dot(a, b) / den) if den > 1e-15 else 0.0


def _harmonic(x, fs, f0, harmonics=(1, 3, 5), width=0.25, up=1):
    """x band-limited to f0 and its (odd, for triangle/sine waves) harmonics, optionally interpolated `up` times finer
    (band-limited interpolation: the peaks of a fast weave fall between frames)."""
    n = len(x)
    X = np.fft.rfft(x - np.mean(x))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    keep = np.zeros(len(f), bool)
    for h in harmonics:
        keep |= np.abs(f - h * f0) <= max(width * f0, 1.5 * fs / n)
    return np.fft.irfft(np.where(keep, X, 0), n * up) * up


def cycle_amp(x, fs, f0):
    """Median half peak-to-peak of the cycles of x at frequency f0 (robust to noise and to the waveform's shape).

    The odd harmonics up to the Nyquist frequency are kept. A zigzag (triangle wave) whose higher harmonics the frame
    rate cuts off reads a little narrow (only up to the 3rd harmonic: 90 %), so when the 3rd harmonic says "triangle"
    the missing part of the series is added back."""
    if f0 <= 0 or len(x) < 8:
        return float(np.ptp(x)) / 2 if len(x) else 0.0
    # harmonics that stand out of the noise floor (noise in a harmonic's band would widen every cycle)
    f, mag, _ = spectrum(x, fs)
    off = np.ones(len(f), bool)
    for h in range(1, 12):
        off &= np.abs(f - h * f0) > 0.3 * f0
    floor = float(np.median(mag[off & (f > 0.3) & (f < fs / 2)])) if np.any(off & (f > 0.3)) else 0.0
    peak_at = lambda fr: float(mag[np.argmin(np.abs(f - fr))])
    hs = (1,) + tuple(h for h in (3, 5, 7, 9) if h * f0 < 0.92 * fs / 2 and peak_at(h * f0) > 3.0 * floor)
    clean = _harmonic(x, fs, f0, hs, up=8)
    fund = _harmonic(x, fs, f0, (1,), up=8)
    up = np.nonzero((fund[:-1] < 0) & (fund[1:] >= 0))[0]
    amps = [float(np.ptp(clean[a:b + 1])) / 2 for a, b in zip(up[:-1], up[1:]) if b - a > 2]
    amp = float(np.median(amps)) if len(amps) >= 2 else float(np.percentile(clean, 95) - np.percentile(clean, 5)) / 2
    if 3 in hs:
        a1, a3 = abs(_component_at(x, fs, f0)), abs(_component_at(x, fs, 3 * f0))
        r = a3 / a1 if a1 > 1e-12 else 0.0
        tri = float(np.clip((r - 0.03) / (1 / 9 - 0.03), 0.0, 1.0)) if r < 0.2 else 0.0   # 1/9: a triangle wave
        trunc = (8 / math.pi ** 2) * sum(1.0 / h ** 2 for h in hs)
        amp *= 1.0 + tri * (1.0 / trunc - 1.0)
    return amp


def _component_at(x, fs, f):
    """Complex amplitude of x at frequency f (single-bin DFT on the window)."""
    t = np.arange(len(x)) / fs
    return 2.0 * complex(np.mean((x - np.mean(x)) * np.exp(-2j * math.pi * f * t)))


def analyze(ts, ps, band=(0.5, 6.0), fs=None, n_fft=None):
    """Hand path -> weave reading (see the module docstring), or None when the path is too short (< 0.8 s, < 8 frames)."""
    ts, ps = np.asarray(ts, float), np.asarray(ps, float)
    if len(ts) < 8 or ts[-1] - ts[0] < 0.8:
        return None
    t, p, fs = uniform(ts, ps, fs)
    tr = travel_fit(t, p)
    res = tr["res"]
    d = tr["dir"] if tr["speed"] > 1e-4 else np.zeros(3)
    # the oscillation: the residual's main axis across the travel (and, for a still hand, its main axis overall)
    r_lat = res - np.outer(res @ d, d) if np.any(d) else res
    w, V = np.linalg.eigh(r_lat.T @ r_lat / len(r_lat))
    axis = V[:, -1]
    if abs(axis[0]) >= abs(axis[1]):              # a stable sign for display: +x or +y
        axis = axis if axis[0] >= 0 else -axis
    else:
        axis = axis if axis[1] >= 0 else -axis
    x = res @ axis
    f0, peak, _ = dominant(x, fs, band, n_fft)
    period = fs / f0 if f0 > 0 else 0
    acf = acf_at(x, round(period)) if period else 0.0
    amp = cycle_amp(x, fs, f0)
    # shape: what the hand does along the travel at f (circle) or 2f (figure 8)
    kind = "zigzag"
    if np.any(d) and f0 > 0:
        along = res @ d
        lat1, al1, al2 = _component_at(x, fs, f0), _component_at(along, fs, f0), _component_at(along, fs, 2 * f0)
        a_lat = abs(lat1) or 1e-9
        quad = abs(math.sin(np.angle(al1) - np.angle(lat1)))
        if abs(al1) > 0.5 * a_lat and quad > 0.7:
            kind = "circle"
        elif abs(al2) > 0.25 * a_lat and abs(al2) > 1.5 * abs(al1):
            kind = "figure8"
    return {"amp": float(amp), "freq": float(f0), "speed": tr["speed"], "dir": d, "axis": axis, "r2": tr["r2"],
            "disp": tr["disp"], "peak": float(peak), "acf": float(acf), "kind": kind, "duration": float(t[-1] - t[0]),
            "pca": tr["pca"], "pca_ext": tr["pca_ext"], "x": x, "fs": fs, "t": t}


def is_weave(a, gates=None):
    """-> (ok, [reasons it is not]) for an analyze() reading."""
    if a is None:
        return False, ["too short"]
    g = dict(WEAVE, **(gates or {}))
    why = []
    if a["speed"] < g["speed_min"]:
        why.append(f"no travel ({a['speed'] * 1000:.1f} mm/s)")
    if a["r2"] < g["r2_min"]:
        why.append(f"no straight travel (r2 {a['r2']:.2f})")
    if a["disp"] < g["disp_min"]:
        why.append(f"travelled {a['disp'] * 100:.1f} cm")
    if not g["f_band"][0] <= a["freq"] <= g["f_band"][1]:
        why.append(f"{a['freq']:.2f} Hz outside {g['f_band'][0]:g}-{g['f_band'][1]:g} Hz")
    if not g["amp_band"][0] <= a["amp"] <= g["amp_band"][1]:
        why.append(f"width {a['amp'] * 1000:.1f} mm outside {g['amp_band'][0] * 1000:g}-{g['amp_band'][1] * 1000:g} mm")
    if a["peak"] < g["peak_min"]:
        why.append(f"weak periodicity (peak {a['peak']:.2f})")
    if a["acf"] < g["acf_min"]:
        why.append(f"irregular (acf {a['acf']:.2f})")
    return not why, why


def ema_gain(f, alpha, dt):
    """Amplitude gain at f (Hz) of the exponential smoothing y += alpha (x - y) applied every dt seconds."""
    if not alpha or alpha >= 1 or dt <= 0:
        return 1.0
    z = complex(math.cos(2 * math.pi * f * dt), -math.sin(2 * math.pi * f * dt))
    return float(alpha / abs(1 - (1 - alpha) * z))


def ema_gain_at(ts, f, alpha):
    """The same gain, measured on the actual (irregular) frame times: the server's ticks are not evenly spaced, and the
    smoothing runs once per tick. A unit sine at f is smoothed on those times and its amplitude read back."""
    ts = np.asarray(ts, float)
    if not alpha or alpha >= 1 or len(ts) < 8 or f <= 0:
        return 1.0
    x = np.sin(2 * math.pi * f * ts)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1 - alpha) * y[i - 1]
    k = len(ts) // 4                                   # skip the start-up transient
    c, s = np.cos(2 * math.pi * f * ts[k:]), np.sin(2 * math.pi * f * ts[k:])
    A = np.column_stack([s, c, np.ones_like(s)])
    coef, *_ = np.linalg.lstsq(A, y[k:], rcond=None)
    return float(np.clip(math.hypot(coef[0], coef[1]), 0.2, 1.0))


def best_segment(ts, ps, win=2.0, step=0.5, gates=None, band=(0.5, 6.0)):
    """The longest run of windows that pass the weave gates (skips a demonstration's lead-in and lead-out).
    -> (i0, i1) sample indices into ts, or None."""
    ts = np.asarray(ts, float)
    if len(ts) < 8 or ts[-1] - ts[0] < win:
        return None
    starts = np.arange(ts[0], ts[-1] - win + 1e-9, step)
    good = []
    for s in starts:
        m = (ts >= s) & (ts <= s + win)
        if m.sum() < 8:
            good.append(False)
            continue
        ok, _ = is_weave(analyze(ts[m], np.asarray(ps)[m], band), gates)
        good.append(ok)
    best, cur = None, None
    for k, g in enumerate(good):
        if g:
            cur = (cur[0], k) if cur else (k, k)
            if best is None or cur[1] - cur[0] > best[1] - best[0]:
                best = cur
        else:
            cur = None
    if best is None:
        return None
    t0, t1 = starts[best[0]], starts[best[1]] + win
    i0 = int(np.searchsorted(ts, t0, side="left"))
    i1 = int(np.searchsorted(ts, t1, side="right")) - 1
    return i0, max(i0 + 1, i1)
