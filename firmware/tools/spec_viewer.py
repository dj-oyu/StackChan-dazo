#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "pyserial",
#     "numpy",
#     "matplotlib",
# ]
# ///
"""
Stack-chan kick beat monitor (perceptible) + decimation sweep.

Firmware streams band-passed (40-75 Hz) kick energy at ~100 Hz: "[KICK]<m><hh>".
This tool DECIMATES that to a lower effective rate (DECIMATE) before onset detection,
to find how low we can go: lower rate -> smoother single peaks (easier thresholding,
fewer double-hits) but coarser phase. Watch the green BEAT flash vs the music and the
BPM lock, then bake the chosen rate into firmware.

  DECIMATE = 1 -> 100 fps, 2 -> 50, 3 -> ~33, 4 -> 25 ...

Usage:  uv run firmware/tools/spec_viewer.py --port COM3
"""
import argparse
import sys
import time
from collections import deque
import numpy as np

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing -> uv run handles this, or pip install pyserial")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

DECIMATE = 4                      # average this many 100 Hz samples per detection frame
NOMINAL_FPS = 100.0 / DECIMATE
HISTORY = max(200, round(6 * NOMINAL_FPS))   # ~6 s window

# Detector knobs (time-based so they hold across DECIMATE values).
FLUX_MEAN_S = 0.08      # local-mean baseline subtracted from the current sample
NOISE_S = 0.60          # adaptive noise estimate window
REFRACTORY_S = 0.18     # min time between onsets (-> <= ~330 BPM)
THRESH_K = 1.8
THRESH_FLOOR = 8.0

FLUX_MEAN_N = max(1, round(FLUX_MEAN_S * NOMINAL_FPS))
NOISE_N = max(4, round(NOISE_S * NOMINAL_FPS))
REFRACTORY = max(1, round(REFRACTORY_S * NOMINAL_FPS))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0)
    ser.reset_input_buffer()

    kick = np.zeros(HISTORY, dtype=np.float32)
    moving = np.zeros(HISTORY, dtype=np.float32)
    onset = np.zeros(HISTORY, dtype=np.float32)
    arrivals = deque(maxlen=200)
    onset_gaps = deque(maxlen=16)
    acc = []           # accumulating 100 Hz samples for decimation
    acc_mv = []
    state = {"frames": 0, "last_onset": -999}

    fig, (ax_wave, ax_servo, ax_beat) = plt.subplots(
        3, 1, figsize=(11, 6.5), gridspec_kw={"height_ratios": [3, 0.3, 0.7]})

    wave_line, = ax_wave.plot(np.zeros(HISTORY), color="k", lw=1.4, label="kick 40-75 Hz")
    onset_mk, = ax_wave.plot([], [], "o", color="limegreen", ms=7, label="onset")
    ax_wave.set_ylim(0, 255); ax_wave.set_xlim(0, HISTORY)
    ax_wave.set_ylabel("kick energy"); ax_wave.legend(loc="upper left", fontsize=8)
    ax_wave.set_xticks([])

    servo_im = ax_servo.imshow(moving.reshape(1, HISTORY), aspect="auto", vmin=0, vmax=1,
                               extent=[0, HISTORY, 0, 1], cmap="Reds")
    ax_servo.set_yticks([]); ax_servo.set_xticks([])
    ax_servo.set_ylabel("servo", rotation=0, ha="right", va="center", fontsize=8)

    beat_im = ax_beat.imshow(onset.reshape(1, HISTORY), aspect="auto", vmin=0, vmax=1,
                             extent=[0, HISTORY, 0, 1], cmap="Greens")
    ax_beat.set_yticks([]); ax_beat.set_xticks([])
    ax_beat.set_ylabel("BEAT", rotation=0, ha="right", va="center", fontsize=10)
    ax_beat.set_xlabel(f"frames @ {NOMINAL_FPS:.0f} fps (DECIMATE={DECIMATE}); newest at right")

    buf = bytearray()

    def detect_onset():
        if moving[-1] > 0.5:
            return False
        base = kick[-1 - FLUX_MEAN_N:-1].mean() if state["frames"] > FLUX_MEAN_N else kick[-1]
        flux = kick[-1] - base
        noise = kick[-NOISE_N:].std() if state["frames"] > NOISE_N else 0.0
        thr = max(THRESH_FLOOR, THRESH_K * noise)
        if flux > thr and (state["frames"] - state["last_onset"]) >= REFRACTORY:
            if state["last_onset"] > -999:
                onset_gaps.append(state["frames"] - state["last_onset"])
            state["last_onset"] = state["frames"]
            return True
        return False

    def push_frame(v, mv):
        kick[:-1] = kick[1:]; kick[-1] = v
        moving[:-1] = moving[1:]; moving[-1] = mv
        arrivals.append(time.time())
        state["frames"] += 1
        onset[:-1] = onset[1:]
        onset[-1] = 1.0 if detect_onset() else 0.0

    def read_frames():
        nonlocal buf
        try:
            buf += ser.read(8192)
        except Exception:
            return 0
        n = 0
        while b"\n" in buf:
            line, _, buf = buf.partition(b"\n")
            tag = line.find(b"[KICK]")
            if tag < 0:
                continue
            payload = line[tag + 6:]
            if len(payload) < 3:
                continue
            mv = 1.0 if payload[0:1] == b"1" else 0.0
            try:
                v = float(int(payload[1:3], 16))
            except ValueError:
                continue
            acc.append(v); acc_mv.append(mv)
            if len(acc) >= DECIMATE:
                push_frame(float(np.mean(acc)), 1.0 if any(acc_mv) else 0.0)
                acc.clear(); acc_mv.clear()
                n += 1
        return n

    def update(_):
        if read_frames() == 0:
            return
        wave_line.set_ydata(kick)
        idx = np.nonzero(onset > 0.5)[0]
        onset_mk.set_data(idx, kick[idx])
        servo_im.set_data(moving.reshape(1, HISTORY))
        beat_im.set_data(onset.reshape(1, HISTORY))

        fps = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 30 else 0.0
        bpm_txt = "BPM --"
        if len(onset_gaps) >= 4 and fps > 0:
            raw = np.array([60.0 * fps / g for g in onset_gaps if g > 0])
            ref = float(np.median(raw))
            folded = []
            for b in raw:
                while b < ref / 1.5:
                    b *= 2.0
                while b > ref * 1.5:
                    b /= 2.0
                folded.append(b)
            folded = np.array(folded)
            bpm_txt = f"BPM {np.median(folded):.0f}  (+/-{folded.std():.0f}, raw {ref:.0f})"
        fig.suptitle(f"kick beat monitor  {state['frames']}f  {fps:.0f} fps "
                     f"(DECIMATE={DECIMATE})  |  green = detected kick  |  {bpm_txt}")
        return

    anim = FuncAnimation(fig, update, interval=40, cache_frame_data=False)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
    del anim


if __name__ == "__main__":
    main()
