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
import paho.mqtt.client as mqtt

from weight_module import WeightSensor
from rfid_module import RFIDReader
from camera_module import CameraDimensioner

# Ignore a scanned tag if it's older than this when a capture fires
RFID_MAX_AGE_S = 120.0

# MQTT / RabbitMQ Configuration
MQTT_BROKER = "192.168.18.19"  # Replace with your RabbitMQ server IP
MQTT_PORT = 1883
MQTT_USER = "guest"             # Default RabbitMQ username
MQTT_PASS = "guest"             # Default RabbitMQ password
MQTT_TOPIC = "cargoInfo"


def display_on_lcd(payload):
    # TODO: replace with your actual LCD driver call
    #print("[LCD] " + json.dumps(payload))
    pass


def send_payload(payload):
    print("\n=== PARCEL PAYLOAD ===")
    print(json.dumps(payload, indent=2))
    print("=======================\n")
    
    # Publish payload to MQTT Broker
    try:
        # Compatibility check for both paho-mqtt v1.x and v2.x
        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        except AttributeError:
            client = mqtt.Client()
            
        # Set RabbitMQ credentials
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.publish(MQTT_TOPIC, json.dumps(payload))
        client.disconnect()
        print(f"[MQTT] Payload successfully published to {MQTT_BROKER} on topic '{MQTT_TOPIC}'", flush=True)
    except Exception as e:
        print(f"[MQTT ERROR] Failed to publish payload: {e}", file=sys.stderr, flush=True)


def make_on_capture(weight_sensor, rfid_reader):
    def on_capture(camera_result):
        print("\n[System] Camera dimensioning complete...", flush=True)
        
        tag_id = None
        weight = 0.0
        weight_history = []
        last_reading_count = -1
        
        # Sync Loop: waits for RFID tag, weight >= 10g, and weight stability
        while True:
            tag_id, _ = rfid_reader.get_last_tag(max_age=RFID_MAX_AGE_S)
            
            # Retrieve the current scale reading version counter
            current_count = weight_sensor.get_reading_count()
            if current_count != last_reading_count:
                last_reading_count = current_count
                weight = weight_sensor.get_weight()
                w1 = weight_sensor.get_w1()
                w2 = weight_sensor.get_w2()
                
                # Append only unique, new readings to the history
                weight_history.append(weight)
                weight_history = weight_history[-3:]  # Keep the last 3 unique samples (approx 3 seconds)
                
                is_stable = False
                if len(weight_history) >= 3:
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
                
            time.sleep(0.05)

        # Calculate CoG X-deviation in mm (assuming platform center is at 3.0 cm)
        cog_offset = weight_sensor.get_cog()
        x_offset_mm = (cog_offset - 3.0) * 10
        
        # Estimate CoG Z-offset (vertical) as half of the physical height of the box in mm
        z_offset_mm = (camera_result['height_cm'] * 10) / 2
        
        # Dynamically set emergency level and ETA based on weight
        if weight > 1000.0:
            emergency_level = "High"
            required_eta = "Within 15 minutes"
        elif weight > 500.0:
            emergency_level = "Medium"
            required_eta = "Within 30 minutes"
        else:
            emergency_level = "Low"
            required_eta = "Within 2 hours"
            
        # Map material class to specific types requested
        mat_class = camera_result["material_class"]
        obj_type = camera_result["object_type"]
        
        if obj_type == "Box":
            material_type = "Hard (cardboard box)"
        elif mat_class == "Hard":
            material_type = "Hard (cardboard box)"
        elif mat_class == "Soft":
            material_type = "Soft (bag / fabric)"
        else:
            material_type = "Mixed (soft + hard)"

        payload = {
            "timestamp": datetime.now().isoformat(),
            "rfid_id": tag_id,
            "weight": f"{weight:.0f} g",
            "dimensions": f"{camera_result['length_cm']:.1f} x {camera_result['width_cm']:.1f} x {camera_result['height_cm']:.1f} cm",
            "center_of_gravity": f"X:{x_offset_mm:+.0f} mm / Y:+0 mm / Z:+{z_offset_mm:.0f} mm",
            "material_type": material_type,
            "emergency_level": emergency_level,
            "required_eta": required_eta,
            "image_path": camera_result["image_path"]
        }

        display_on_lcd(payload)
        send_payload(payload)

        rfid_reader.clear()  # clear the tag immediately

        # Wait for the parcel to be completely removed from the scale
        print("[System] Please remove the parcel...", flush=True)
        while weight_sensor.get_weight() >= 10.0:
            time.sleep(0.2)

        # Flush any scans that happened while the user was lifting the box
        rfid_reader.clear()
        print("[System] Ready for next scan.\n", flush=True)

    return on_capture


import threading

def watch_keyboard_input(weight_sensor):
    print("[System] Type 't' and press Enter to manually re-tare the scale.", flush=True)
    while True:
        try:
            line = sys.stdin.readline().strip().lower()
            if line == 't':
                print("\n[System] Manual tare triggered. Keep platform empty...", flush=True)
                weight_sensor.tare()
                print("[System] Manual tare complete. Ready.\n", flush=True)
        except Exception:
            time.sleep(1)


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

    # Start keyboard listener thread for manual tare
    kbd_thread = threading.Thread(target=watch_keyboard_input, args=(weight_sensor,), daemon=True)
    kbd_thread.start()

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
