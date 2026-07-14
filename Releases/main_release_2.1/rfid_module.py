"""
RFID module - continuous version of your tested standalone hidraw
reader. Same parsing logic (HID_MAP, last_code de-dup, Enter=code 40,
tag.upper()), just wrapped in a background thread so it keeps
listening for every scan instead of exiting after the first one, and
exposes the most recent tag to main.py.
"""

import select
import threading
import time

RFID_HIDRAW_PATH = "/dev/hidraw0"

# USB HID Keyboard scan codes to ASCII character mapping
HID_MAP = {
    4: 'a', 5: 'b', 6: 'c', 7: 'd', 8: 'e', 9: 'f', 10: 'g', 11: 'h', 12: 'i',
    13: 'j', 14: 'k', 15: 'l', 16: 'm', 17: 'n', 18: 'o', 19: 'p', 20: 'q',
    21: 'r', 22: 's', 23: 't', 24: 'u', 25: 'v', 26: 'w', 27: 'x', 28: 'y',
    29: 'z', 30: '1', 31: '2', 32: '3', 33: '4', 34: '5', 35: '6', 36: '7',
    37: '8', 38: '9', 39: '0', 40: '\n', 43: '\t', 44: ' '
}


class RFIDReader:
    def __init__(self, device_path=RFID_HIDRAW_PATH):
        self.device_path = device_path
        self._last_tag = None
        self._last_tag_time = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def _loop(self):
        print(f"[RFIDReader] Opening raw device: {self.device_path}...")
        fp = None
        while self._running and fp is None:
            try:
                fp = open(self.device_path, 'rb')
                print(f"[RFIDReader] Connected to {self.device_path}")
            except (PermissionError, FileNotFoundError):
                # Wait for device to be plugged in or permission granted
                time.sleep(2.0)

        if fp is None:
            return

        tag = ""
        last_code = 0

        while self._running:
            # Non-blocking wait using select
            r, _, _ = select.select([fp], [], [], 0.5)
            if r:
                try:
                    report = fp.read(8)
                    if not report or len(report) < 3:
                        continue
                    code = report[2]
                    if code != 0:
                        if code != last_code:
                            last_code = code
                            if code == 40:  # Enter key (end of scan)
                                scanned_tag = tag.upper().strip()
                                if scanned_tag:
                                    with self._lock:
                                        self._last_tag = scanned_tag
                                        self._last_tag_time = time.time()
                                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                                    print(f"[RFIDReader] [{timestamp}] Tag scanned: {scanned_tag}")
                                tag = ""
                            else:
                                char = HID_MAP.get(code, '')
                                tag += char
                    else:
                        last_code = 0
                except Exception as e:
                    print(f"[RFIDReader] Connection lost: {e}. Reconnecting...")
                    try:
                        fp.close()
                    except Exception:
                        pass
                    fp = None
                    while self._running and fp is None:
                        try:
                            fp = open(self.device_path, 'rb')
                            print(f"[RFIDReader] Reconnected to {self.device_path}")
                        except Exception:
                            time.sleep(2.0)
                    last_code = 0
                    tag = ""

        if fp is not None:
            try:
                fp.close()
            except Exception:
                pass

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def get_last_tag(self, max_age=None):
        """
        Returns (tag_id, age_seconds) for the most recent scan.
        If max_age is given and the scan is older than that, returns (None, age).
        """
        with self._lock:
            if self._last_tag is None:
                return None, None
            age = time.time() - self._last_tag_time
            if max_age is not None and age > max_age:
                return None, age
            return self._last_tag, age

    def clear(self):
        with self._lock:
            self._last_tag = None
            self._last_tag_time = 0.0

    def stop(self):
        self._running = False


if __name__ == "__main__":
    print("=" * 60)
    print("       Standalone RFID Module Test Mode")
    print("=" * 60)
    reader = RFIDReader()
    reader.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[+] Exiting RFID Standalone Test.")
    finally:
        reader.stop()
