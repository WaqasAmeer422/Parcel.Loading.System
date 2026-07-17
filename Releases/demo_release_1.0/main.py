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
import lgpio
import threading
from datetime import datetime
import paho.mqtt.client as mqtt

from weight_module import WeightSensor
from rfid_module import RFIDReader
from camera_module import CameraDimensioner

# Global continuous sorting state
active_parcels = []
weighing_in_progress = False
last_dispatch_time = 0.0
active_parcels_lock = threading.Lock()

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


# Motor Control Configuration
MOTOR_PIN = 24            # GPIO 24 (LPWM on the IBT-2 for reverse direction)
MOTOR_START_SPEED = 60    # Jerk start speed (60% PWM for 0.3s)
MOTOR_RUN_SPEED = 40      # Normal run speed (40% PWM)
MOTOR_RUN_DURATION = 10.0 # Total run time in seconds


def compile_payload(parcel, tag_id):
    camera_result = parcel["dimensions"]
    locked_weight = parcel["weight"]
    locked_cog = parcel["cog"]
    
    h_cm = camera_result.get('height_cm')
    height_str = f"{h_cm:.1f} cm" if h_cm is not None else "N/A"
    
    cog_offset = locked_cog if locked_cog is not None else 3.0
    x_offset_mm = (cog_offset - 3.0) * 10
    
    if h_cm is not None:
        z_offset_mm = (h_cm * 10) / 2
    else:
        z_offset_mm = 0.0

    if locked_weight > 1000.0:
        emergency_level = "High"
        required_eta = "Within 15 minutes"
    elif locked_weight > 500.0:
        emergency_level = "Medium"
        required_eta = "Within 30 minutes"
    else:
        emergency_level = "Low"
        required_eta = "Within 2 hours"
        
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

    return {
        "timestamp": datetime.now().isoformat(),
        "rfid_id": tag_id,
        "weight": f"{locked_weight:.0f} g",
        "dimensions": f"{camera_result['length_cm']:.1f} x {camera_result['width_cm']:.1f} x {height_str}",
        "center_of_gravity": f"X:{x_offset_mm:+.0f} mm / Y:+0 mm / Z:+{z_offset_mm:.0f} mm",
        "material_type": material_type,
        "emergency_level": emergency_level,
        "required_eta": required_eta,
        "image_path": camera_result["image_path"]
    }


def make_on_capture(weight_sensor, rfid_reader, reset_event, h_motor=None, motor_pin=MOTOR_PIN):
    def process_weighing(camera_result):
        global weighing_in_progress, active_parcels, last_dispatch_time
        weighing_in_progress = True
        
        # Flush reset
        reset_event.clear()
        
        h_cm = camera_result.get('height_cm')
        height_str = f"{h_cm:.1f} cm" if h_cm is not None else "N/A"
        
        weight = 0.0
        weight_history = []
        last_reading_count = -1
        
        print(f"\n[Sensor Sync] Dimensioning complete ({camera_result['length_cm']:.1f}x{camera_result['width_cm']:.1f}cm). Weighing parcel...", flush=True)
        
        # Sync Loop Part 1: waits ONLY for weight stability on scale
        while True:
            if reset_event.is_set():
                reset_event.clear()
                print("\n[System] Scan canceled by user. Resetting state.", flush=True)
                weighing_in_progress = False
                return
                
            current_count = weight_sensor.get_reading_count()
            if current_count != last_reading_count:
                last_reading_count = current_count
                weight = weight_sensor.get_weight()
                
                weight_history.append(weight)
                weight_history = weight_history[-3:]
                
                is_stable = False
                if len(weight_history) >= 3:
                    diff = max(weight_history) - min(weight_history)
                    if diff < 3.0:
                        is_stable = True
                        
                # Lock weight and CoG once stable
                if weight >= 10.0 and is_stable:
                    locked_weight = weight
                    locked_cog = weight_sensor.get_cog()
                    print(f"\n[Sensor Sync] Weight & CoG Captured and Locked: {locked_weight:.1f}g", flush=True)
                    
                    with active_parcels_lock:
                        active_parcels.append({
                            "dimensions": camera_result,
                            "weight": locked_weight,
                            "cog": locked_cog,
                            "timestamp": time.time()
                        })
                    
                    weighing_in_progress = False
                    last_dispatch_time = time.time()
                    break
                
                dim_status = f"{camera_result['length_cm']:.1f}x{camera_result['width_cm']:.1f}x{height_str}"
                sys.stdout.write(f"\r[Sensor Sync] Dimensions: CAPTURED ({dim_status}) | Weight: WAITING FOR WEIGHT...")
                sys.stdout.flush()
                
            time.sleep(0.05)

    def on_capture(camera_result):
        # Spawn thread to handle weighing without blocking the camera main loop
        threading.Thread(target=process_weighing, args=(camera_result,), daemon=True).start()

    return on_capture


def rfid_sync_loop(rfid_reader):
    global active_parcels
    while True:
        try:
            # 1. Check for timeout of active parcels on the conveyor (10 seconds)
            now = time.time()
            timeout_parcel = None
            with active_parcels_lock:
                if len(active_parcels) > 0 and (now - active_parcels[0]["timestamp"] > 10.0):
                    timeout_parcel = active_parcels.pop(0)
            
            if timeout_parcel is not None:
                print("\n[System] WARNING: RFID downstream scan timed out (10s limit).", flush=True)
                payload = compile_payload(timeout_parcel, "N/A")
                display_on_lcd(payload)
                send_payload(payload)
                rfid_reader.clear()

            # 2. Check for scanned RFID tag downstream
            tag_id, _ = rfid_reader.get_last_tag(max_age=RFID_MAX_AGE_S)
            if tag_id is not None:
                scanned_parcel = None
                with active_parcels_lock:
                    if len(active_parcels) > 0:
                        scanned_parcel = active_parcels.pop(0)
                
                if scanned_parcel is not None:
                    print(f"\n[RFID Reader] Tag Scanned Downstream: {tag_id}. Pairing with parcel.", flush=True)
                    payload = compile_payload(scanned_parcel, tag_id)
                    display_on_lcd(payload)
                    send_payload(payload)
                    rfid_reader.clear()
        except Exception as e:
            print(f"[RFID Sync Thread ERROR]: {e}", flush=True)
            
        time.sleep(0.05)


def motor_control_loop(h_motor, motor_pin, weight_sensor, reset_event):
    global active_parcels, weighing_in_progress, last_dispatch_time
    motor_running = False
    while True:
        try:
            if h_motor is not None:
                # -------------------------------------------------------------
                # AUTOMATIC INSTANT STOP COMMENTED OUT PER USER REQUEST:
                # current_weight = weight_sensor.get_weight()
                # should_stop = (
                #     weighing_in_progress and
                #     (time.time() - last_dispatch_time >= 4.0) and
                #     (current_weight >= 10.0)
                # )
                # if should_stop:
                #     if motor_running:
                #         try:
                #             lgpio.tx_pwm(h_motor, motor_pin, 0, 0)
                #         except Exception:
                #             pass
                #         lgpio.gpio_write(h_motor, motor_pin, 0)
                #         motor_running = False
                #         print("[Conveyor] Stop belt (weighing new parcel - verified weight & 4s cooldown)", flush=True)
                # -------------------------------------------------------------

                # Manual Reset Button / Event Stop: Stop instantly when button is pressed
                if reset_event.is_set():
                    if motor_running:
                        try:
                            lgpio.tx_pwm(h_motor, motor_pin, 0, 0)
                        except Exception:
                            pass
                        lgpio.gpio_write(h_motor, motor_pin, 0)
                        motor_running = False
                        print("[Conveyor] Stop belt instantly (manual stop button pressed)", flush=True)
                
                # Otherwise, run the belt if we have active moving parcels or within 6s of last dispatch
                elif not reset_event.is_set() and (len(active_parcels) > 0 or (time.time() - last_dispatch_time < 6.0)):
                    if not motor_running:
                        print("[Conveyor] Start belt (jerk start)", flush=True)
                        try:
                            # Jerk start: 60% speed for 0.3s to kickstart
                            lgpio.tx_pwm(h_motor, motor_pin, 1000, MOTOR_START_SPEED)
                            time.sleep(0.3)
                            # Normal run speed
                            lgpio.tx_pwm(h_motor, motor_pin, 1000, MOTOR_RUN_SPEED)
                        except Exception:
                            lgpio.gpio_write(h_motor, motor_pin, 1)
                        motor_running = True
                
                # If line is clear and 6 seconds have passed, stop motor
                else:
                    if motor_running:
                        print("[Conveyor] Stop belt (line clear / idle)", flush=True)
                        try:
                            lgpio.tx_pwm(h_motor, motor_pin, 0, 0)
                        except Exception:
                            pass
                        lgpio.gpio_write(h_motor, motor_pin, 0)
                        motor_running = False
        except Exception as e:
            print(f"[Motor Loop Thread ERROR]: {e}", flush=True)
            
        time.sleep(0.1)


import threading

def watch_keyboard_input(weight_sensor, reset_event, camera):
    print("[System] Controls: 't' = Tare Scale | 'r' = Reset Current Scan | 'p' = Pause/Resume Scanner", flush=True)
    while True:
        try:
            line = sys.stdin.readline().strip().lower()
            if line == 't':
                print("\n[System] Manual tare triggered. Keep platform empty...", flush=True)
                weight_sensor.tare()
                print("[System] Manual tare complete. Ready.\n", flush=True)
            elif line == 'r':
                print("\n[System] Manual RESET triggered. Canceling current scan...", flush=True)
                reset_event.set()
            elif line == 'p':
                if camera.paused:
                    camera.resume()
                else:
                    camera.pause()
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

    # Initialize conveyor motor GPIO pin
    from weight_module import GPIOCHIP
    h_motor = None
    try:
        h_motor = lgpio.gpiochip_open(GPIOCHIP)
        lgpio.gpio_claim_output(h_motor, MOTOR_PIN)
        print(f"[INFO] Conveyor motor driver initialized on GPIO {MOTOR_PIN}.", flush=True)
    except Exception as e:
        print(f"[WARNING] Could not initialize conveyor motor on GPIO {MOTOR_PIN}: {e}", flush=True)

    reset_event = threading.Event()

    def on_motion():
        global weighing_in_progress
        weighing_in_progress = True
        print("\n[System] Motion detected by camera. Waiting for weight & cooldown verification...", flush=True)

    def on_reset():
        global weighing_in_progress
        weighing_in_progress = False
        print("[System] Platform clear / scan reset.", flush=True)

    rfid_reader = RFIDReader()
    rfid_reader.start()

    weight_sensor = WeightSensor()
    weight_sensor.start()

    camera = CameraDimensioner(
        on_capture=make_on_capture(weight_sensor, rfid_reader, reset_event, h_motor, MOTOR_PIN),
        on_motion=on_motion,
        on_reset=on_reset
    )
    camera.start()

    # Start downstream RFID syncer and motor control manager threads
    rfid_sync_thread = threading.Thread(target=rfid_sync_loop, args=(rfid_reader,), daemon=True)
    rfid_sync_thread.start()
    
    motor_control_thread = threading.Thread(target=motor_control_loop, args=(h_motor, MOTOR_PIN, weight_sensor, reset_event), daemon=True)
    motor_control_thread.start()

    # Start keyboard listener thread for manual tare, reset, and pause
    kbd_thread = threading.Thread(target=watch_keyboard_input, args=(weight_sensor, reset_event, camera), daemon=True)
    kbd_thread.start()

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
        if h_motor is not None:
            print("[INFO] Shutting down conveyor motor...", flush=True)
            try:
                lgpio.tx_pwm(h_motor, MOTOR_PIN, 0, 0)
            except Exception:
                pass
            lgpio.gpio_write(h_motor, MOTOR_PIN, 0)
            lgpio.gpiochip_close(h_motor)


if __name__ == "__main__":
    main()
