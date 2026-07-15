"""
Weight sensor module - wraps your working dual-HX711 (lgpio) reading
logic in a background thread so main.py can just call get_weight()
any time without blocking.
"""

import lgpio
import time
import threading
import statistics

DOUT_1 = 5
SCK_1 = 6
DOUT_2 = 13
SCK_2 = 19

CAL_FACTOR_1 = 396.66
CAL_FACTOR_2 = 379.83

GPIOCHIP = 0  # 0 for CM3+/Pi3/Pi4, 4 for Pi 5
SAMPLES_PER_READING = 5
READ_TIMEOUT_S = 1.0        # used during normal operation - tolerant of real jitter
TARE_READ_TIMEOUT_S = 0.3   # used only during startup tare - fail fast if disconnected
TARE_FAIL_FAST_AFTER = 3    # give up on a scale after this many consecutive misses


CELL_DISTANCE = 6.0  # Distance between load cell 1 and 2 in cm

class WeightSensor:
    def __init__(self):
        self.h = lgpio.gpiochip_open(GPIOCHIP)
        self._setup_channel(DOUT_1, SCK_1)
        self._setup_channel(DOUT_2, SCK_2)
        self.offset_1 = 0.0
        self.offset_2 = 0.0
        self.scale1_active = True
        self.scale2_active = True
        self._current_weight = 0.0
        self._current_cog = 3.0
        self._current_w1 = 0.0
        self._current_w2 = 0.0
        self._lock = threading.Lock()
        self._hw_lock = threading.Lock()
        self.reading_count = 0
        self._running = False
        self._thread = None

    def _setup_channel(self, dout, sck):
        lgpio.gpio_claim_output(self.h, sck, 0)
        lgpio.gpio_claim_input(self.h, dout)

    def _wait_ready(self, dout, timeout=READ_TIMEOUT_S):
        start = time.time()
        while lgpio.gpio_read(self.h, dout) == 1:
            if time.time() - start > timeout:
                return False
        return True

    def _read_raw(self, dout, sck, timeout=READ_TIMEOUT_S):
        with self._hw_lock:
            if not self._wait_ready(dout, timeout=timeout):
                return None
            val = 0
            for _ in range(24):
                lgpio.gpio_write(self.h, sck, 1)
                lgpio.gpio_write(self.h, sck, 0)
                val = (val << 1) | lgpio.gpio_read(self.h, dout)
            lgpio.gpio_write(self.h, sck, 1)
            lgpio.gpio_write(self.h, sck, 0)
            if val & 0x800000:
                val -= 1 << 24
            return val

    def tare(self, n=15):
        print("[WeightSensor] Taring... keep platform empty.")
        # Tare Scale 1
        vals1 = []
        for i in range(n):
            v = self._read_raw(DOUT_1, SCK_1, timeout=TARE_READ_TIMEOUT_S)
            if v is not None:
                vals1.append(v)
            else:
                if i == 0:
                    self.scale1_active = False
                    print("[WeightSensor] WARNING: Scale 1 (DOUT=5) is offline/inactive.")
                    break
            time.sleep(0.02)
        if vals1 and self.scale1_active:
            self.offset_1 = statistics.median(vals1)
            print(f"[WeightSensor] offset_1 = {self.offset_1:.1f}")

        # Tare Scale 2
        vals2 = []
        for i in range(n):
            v = self._read_raw(DOUT_2, SCK_2, timeout=TARE_READ_TIMEOUT_S)
            if v is not None:
                vals2.append(v)
            else:
                if i == 0:
                    self.scale2_active = False
                    print("[WeightSensor] WARNING: Scale 2 (DOUT=13) is offline/inactive.")
                    break
            time.sleep(0.02)
        if vals2 and self.scale2_active:
            self.offset_2 = statistics.median(vals2)
            print(f"[WeightSensor] offset_2 = {self.offset_2:.1f}")

    def _loop(self):
        while self._running:
            # Only read from active channels to prevent floating pin static noise
            vals1, vals2 = [], []
            for _ in range(SAMPLES_PER_READING):
                if self.scale1_active:
                    v1 = self._read_raw(DOUT_1, SCK_1)
                    if v1 is not None:
                        vals1.append(v1)
                if self.scale2_active:
                    v2 = self._read_raw(DOUT_2, SCK_2)
                    if v2 is not None:
                        vals2.append(v2)

            w1, w2 = 0.0, 0.0
            if self.scale1_active and vals1:
                raw1 = statistics.median(vals1)
                w1 = (raw1 - self.offset_1) / CAL_FACTOR_1
            if self.scale2_active and vals2:
                raw2 = statistics.median(vals2)
                w2 = (raw2 - self.offset_2) / CAL_FACTOR_2

            total_weight = w1 + w2
            cog_distance = (w2 * CELL_DISTANCE) / total_weight if total_weight > 20.0 else 3.0
            
            with self._lock:
                self._current_weight = total_weight
                self._current_cog = cog_distance
                self._current_w1 = w1
                self._current_w2 = w2
                self.reading_count += 1
            
            time.sleep(0.05)

    def start(self):
        self.tare()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[WeightSensor] Started.")

    def get_weight(self):
        with self._lock:
            return self._current_weight

    def get_w1(self):
        with self._lock:
            return self._current_w1

    def get_w2(self):
        with self._lock:
            return self._current_w2

    def get_cog(self):
        with self._lock:
            return self._current_cog

    def get_reading_count(self):
        with self._lock:
            return self.reading_count

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        lgpio.gpiochip_close(self.h)


if __name__ == "__main__":
    print("=" * 60)
    print("       Standalone Weight Sensor & CoG Test Mode")
    print("=" * 60)
    # Enable real-time scheduling priority (SCHED_FIFO)
    import os
    try:
        pid = os.getpid()
        param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
        os.sched_setscheduler(pid, os.SCHED_FIFO, param)
        print("[INFO] Real-time (SCHED_FIFO) scheduling enabled.", flush=True)
    except PermissionError:
        print("[WARNING] Could not enable real-time scheduling. Run with sudo.", flush=True)

    sensor = WeightSensor()
    sensor.start()
    try:
        while True:
            w = sensor.get_weight()
            cog = sensor.get_cog()
            # Carriage return \r updates the current line dynamically
            print(f"\r[Live Sensor Data] Weight: {w:.1f} g | CoG: {cog:.2f} cm (from Cell 1)      ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[+] Exiting Weight Standalone Test.")
    finally:
        sensor.stop()
