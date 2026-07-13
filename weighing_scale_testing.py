#!/usr/bin/env python3
"""
Dual HX711 load cell reader for Raspberry Pi Compute Module 3+, using
lgpio instead of pigpio.

WHY THE SWITCH FROM pigpio:
pigpio's Python module talks to the pigpiod daemon over a network
socket, even for local calls. Bit-banging the HX711's 24-bit clock
train through 25+ socket round trips is too slow to keep each SCK
pulse under the chip's ~60us limit, so the HX711 powers down mid-read
and you get raw values stuck at -1 (0xFFFFFF, i.e. "all bits read as
1") no matter what's on the scale - which is exactly what you saw.

lgpio talks to the kernel's GPIO character device directly, in the
same process - no socket, no separate daemon - so per-pulse latency
is far lower and far more consistent. This is also what gpiozero and
current maintained HX711 libraries use now that RPi.GPIO/pigpio are
unreliable on newer OS releases.

SETUP:
    pip3 install lgpio --break-system-packages
Run with sudo for real-time scheduling to take effect:
    sudo python3 dual_hx711_lgpio.py
"""

import lgpio
import time
import sys
import os
import select
import statistics

# ---------------- Pin configuration (BCM numbering) ----------------
DOUT_1 = 5
SCK_1 = 6

DOUT_2 = 13
SCK_2 = 19

# Calibration factors carried over from your Arduino testing
CAL_FACTOR_1 = 396.66
CAL_FACTOR_2 = 379.83

# Distance between the centers of the two load cells (in meters)
CELL_DISTANCE = 6.0

# gpiochip number: 0 for CM3+/Pi 3 and earlier (old-style GPIO).
# Only newer Pi 5 boards need chip 4 - not relevant to your CM3+.
GPIOCHIP = 0

# How many raw samples to combine (via median) into one printed reading.
SAMPLES_PER_READING = 5

# Reject a reading if it jumps more than this many grams from the last
# accepted value in a single update (catches any still-corrupted reads).
MAX_JUMP_G = 500.0

READ_TIMEOUT_S = 1.0

# ----------------------------------------------------------------------

h = lgpio.gpiochip_open(GPIOCHIP)

scale1_active = True
scale2_active = True


def setup_channel(dout, sck):
    lgpio.gpio_claim_output(h, sck, 0)
    lgpio.gpio_claim_input(h, dout)


def wait_ready(dout, timeout=READ_TIMEOUT_S):
    """Block until DOUT goes low (HX711 signals a fresh conversion is ready)."""
    start = time.time()
    while lgpio.gpio_read(h, dout) == 1:
        if time.time() - start > timeout:
            return False
    return True


def read_raw(dout, sck):
    if not wait_ready(dout):
        return None

    val = 0
    for _ in range(24):
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        val = (val << 1) | lgpio.gpio_read(h, dout)

    # 25th pulse selects channel A / gain 128 for the *next* conversion,
    # matching your original code.
    lgpio.gpio_write(h, sck, 1)
    lgpio.gpio_write(h, sck, 0)

    if val & 0x800000:
        val -= 1 << 24
    return val


def read_pair_filtered(n=SAMPLES_PER_READING):
    """Interleave n reads across both cells, return (median_1, median_2)."""
    vals1, vals2 = [], []
    for _ in range(n):
        if scale1_active:
            v1 = read_raw(DOUT_1, SCK_1)
            if v1 is not None:
                vals1.append(v1)
        if scale2_active:
            v2 = read_raw(DOUT_2, SCK_2)
            if v2 is not None:
                vals2.append(v2)

    raw1 = statistics.median(vals1) if vals1 else None
    raw2 = statistics.median(vals2) if vals2 else None
    return raw1, raw2


def tare(dout, sck, n=15):
    print(f"[INFO] Taring scale on DOUT={dout}... Keep platform empty.", flush=True)
    vals = []
    for _ in range(n):
        v = read_raw(dout, sck)
        if v is not None:
            vals.append(v)
        time.sleep(0.02)

    if len(vals) < max(3, n // 2):
        print(f"[WARNING] Tare failed on DOUT={dout}! Scale not responding reliably.", file=sys.stderr, flush=True)
        return None

    offset = statistics.median(vals)
    print(f"[INFO] Tare complete. Offset: {offset:.1f} ({len(vals)}/{n} good samples)", flush=True)

    # Sanity check: a real unloaded HX711 offset is essentially never
    # exactly -1 (0xFFFFFF). If we see that, the chip is powering down
    # mid-read - flag it loudly instead of silently treating it as valid.
    if offset == -1:
        print(f"[WARNING] Offset of exactly -1 on DOUT={dout} usually means the "
              f"HX711 is powering down mid-read (timing issue), not a real zero "
              f"reading. Check wiring/power if this persists.", file=sys.stderr, flush=True)

    return offset


def try_realtime_priority():
    """Best-effort: ask the OS for real-time scheduling to reduce preemption jitter (needs root)."""
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
        print("[INFO] Real-time (SCHED_FIFO) scheduling enabled.", flush=True)
    except PermissionError:
        print("[INFO] Could not set real-time priority - run with sudo for best stability.", flush=True)
    except Exception as e:
        print(f"[INFO] Real-time priority not set: {e}", flush=True)


def main():
    global scale1_active, scale2_active

    print("=" * 60)
    print("  Dual Load Cell Weight & CoG Measurement System (lgpio)")
    print("=" * 60)
    print(f"Scale 1: DOUT={DOUT_1}, SCK={SCK_1} | Cal Factor={CAL_FACTOR_1}")
    print(f"Scale 2: DOUT={DOUT_2}, SCK={SCK_2} | Cal Factor={CAL_FACTOR_2}")
    print(f"Distance between cells: {CELL_DISTANCE} m")
    print(f"Samples per printed reading (median filter): {SAMPLES_PER_READING}")
    print("=" * 60)

    try_realtime_priority()

    try:
        setup_channel(DOUT_1, SCK_1)
        setup_channel(DOUT_2, SCK_2)

        offset_1 = tare(DOUT_1, SCK_1)
        offset_2 = tare(DOUT_2, SCK_2)

        scale1_active = offset_1 is not None
        scale2_active = offset_2 is not None
        offset_1 = offset_1 if offset_1 is not None else 0.0
        offset_2 = offset_2 if offset_2 is not None else 0.0

        if not scale1_active:
            print("[WARNING] Scale 1 marked INACTIVE due to tare timeout.", file=sys.stderr, flush=True)
        if not scale2_active:
            print("[WARNING] Scale 2 marked INACTIVE due to tare timeout.", file=sys.stderr, flush=True)

        print("\n[INFO] Starting data capture. Ctrl+C to exit. Type 't' + Enter to re-tare.\n", flush=True)

        last_w1, last_w2 = 0.0, 0.0

        while True:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                user_input = sys.stdin.readline().strip()
                if user_input.lower() == 't':
                    print("\n[INFO] Re-taring scales...", flush=True)
                    if scale1_active:
                        o = tare(DOUT_1, SCK_1)
                        if o is not None:
                            offset_1 = o
                    if scale2_active:
                        o = tare(DOUT_2, SCK_2)
                        if o is not None:
                            offset_2 = o
                    print("[INFO] Resuming readings...\n", flush=True)

            raw_1, raw_2 = read_pair_filtered()

            w1 = (raw_1 - offset_1) / CAL_FACTOR_1 if raw_1 is not None else last_w1
            w2 = (raw_2 - offset_2) / CAL_FACTOR_2 if raw_2 is not None else last_w2

            # Reject implausible single-update jumps (leftover corrupted reads)
            if abs(w1 - last_w1) > MAX_JUMP_G:
                w1 = last_w1
            if abs(w2 - last_w2) > MAX_JUMP_G:
                w2 = last_w2

            last_w1, last_w2 = w1, w2
            total_weight = w1 + w2

            cog_distance = CELL_DISTANCE / 2.0
            if total_weight > 5.0:
                cog_distance = (w2 * CELL_DISTANCE) / total_weight

            print(f"W1: {w1:.1f}g | W2: {w2:.1f}g | Total: {total_weight:.1f}g | "
                  f"CoG: {cog_distance:.2f}m from Cell 1", flush=True)

    except KeyboardInterrupt:
        print("\n[INFO] Exiting program...", flush=True)
    except Exception as e:
        print(f"\n[CRASH] Unexpected error: {e}", file=sys.stderr, flush=True)
    finally:
        print("[INFO] Cleaning up...", flush=True)
        lgpio.gpiochip_close(h)
        print("[INFO] Done.", flush=True)


if __name__ == "__main__":
    main()