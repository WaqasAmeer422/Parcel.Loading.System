"""
Main orchestrator: weight and RFID run continuously in background
threads. The camera's own detection state machine is the real
trigger - when it completes a capture (parcel placed, settled,
measured), that callback grabs the current weight and the most
recently scanned RFID tag, assembles one payload, and displays it.
"""

import time
import json
import sys
from datetime import datetime

from weight_module import WeightSensor
from rfid_module import RFIDReader
from camera_module import CameraDimensioner

# Ignore a scanned tag if it's older than this when a capture fires
RFID_MAX_AGE_S = 120.0


def display_on_lcd(payload):
    # TODO: replace with your actual LCD driver call
    print("[LCD] " + json.dumps(payload))


def send_payload(payload):
    # TODO: swap this for an MQTT publish later
    print("\n=== PARCEL PAYLOAD ===")
    print(json.dumps(payload, indent=2))
    print("=======================\n")


def make_on_capture(weight_sensor, rfid_reader):
    def on_capture(camera_result):
        print("\n[System] Camera dimensioning complete. Syncing sensors...", flush=True)
        
        tag_id = None
        weight = 0.0
        weight_history = []
        
        # Sync Loop: waits for RFID tag, weight >= 10g, and weight stability
        while True:
            tag_id, _ = rfid_reader.get_last_tag(max_age=RFID_MAX_AGE_S)
            weight = weight_sensor.get_weight()
            w1 = weight_sensor.get_w1()
            w2 = weight_sensor.get_w2()
            
            # Track weight history for stability detection
            weight_history.append(weight)
            weight_history = weight_history[-5:]  # Keep the last 5 samples (approx 1 second)
            
            is_stable = False
            if len(weight_history) >= 5:
                # Stable if difference between max and min weight in the window is less than 3.0g
                diff = max(weight_history) - min(weight_history)
                if diff < 3.0:
                    is_stable = True
            
            # Print real-time status dynamically on the same console line
            status = (f"\r[Sensor Sync] Cell1: {w1:.1f}g | Cell2: {w2:.1f}g | Total: {weight:.1f}g "
                      f"({'STABLE' if is_stable else 'STABILIZING...'}) | "
                      f"RFID: {tag_id if tag_id else 'WAITING FOR TAG...'}")
            sys.stdout.write(status)
            sys.stdout.flush()
            
            # Finalize payload when tag is scanned, weight is at least 10g, and weight is stable
            if tag_id is not None and weight >= 10.0 and is_stable:
                # Clear the status line
                sys.stdout.write("\r" + " " * 120 + "\r")
                sys.stdout.flush()
                break
                
            time.sleep(0.2)

        payload = {
            "timestamp": datetime.now().isoformat(),
            "tag_id": tag_id,
            "weight_g": round(weight, 1),
            "cog_offset_cm": round(weight_sensor.get_cog(), 2),
            **camera_result,
        }

        display_on_lcd(payload)
        send_payload(payload)

        rfid_reader.clear()  # clear the tag immediately

        # Wait for the parcel to be completely removed from the scale
        print("[System] Please remove the parcel from the platform...", flush=True)
        while weight_sensor.get_weight() >= 10.0:
            time.sleep(0.2)

        # Flush any scans that happened while the user was lifting the box
        rfid_reader.clear()
        print("[System] Platform cleared. Ready for next scan.\n", flush=True)

    return on_capture


def main():
    # Enable real-time scheduling priority (SCHED_FIFO)
    import os
    try:
        pid = os.getpid()
        param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
        os.sched_setscheduler(pid, os.SCHED_FIFO, param)
        print("[INFO] Real-time (SCHED_FIFO) scheduling enabled.", flush=True)
    except PermissionError:
        print("[WARNING] Could not enable real-time scheduling. Run with sudo.", flush=True)

    weight_sensor = WeightSensor()
    weight_sensor.start()

    rfid_reader = RFIDReader()
    rfid_reader.start()

    camera = CameraDimensioner(on_capture=make_on_capture(weight_sensor, rfid_reader))
    camera.start()

    print("[INFO] System ready. Waiting for parcels...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
    finally:
        camera.stop()
        rfid_reader.stop()
        weight_sensor.stop()


if __name__ == "__main__":
    main()
