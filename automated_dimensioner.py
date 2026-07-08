import cv2
import numpy as np
import time
import os

# --- CONFIGURATION ---
CAMERA_INDEX = 1
# Reference calibration scale: 20.87 pixels/cm (change to your calibrated value)
PIXELS_PER_CM = 20.87
# Dilation offset correction in pixels
CALIBRATION_OFFSET_PX = 16.0

# Motion Detection Parameters
MOTION_THRESHOLD = 50000   # Min sum of changed pixels to register motion
STILL_DURATION_SEC = 1.0   # How long the parcel must be still to trigger scan
# ---------------------

# State Machine States
STATE_EMPTY = 0
STATE_MOVING = 1
STATE_STILL = 2
STATE_WAIT_FOR_EXIT = 3

def measure_parcel(frame):
    """Isolates the parcel and calculates dimensions using pre-processing."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Pre-processing: Edge detection and morphology to clean up contours
    edges = cv2.Canny(blurred, 30, 100)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    dilated = cv2.dilate(closed, kernel, iterations=1)

    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    max_area = 0
    parcel_contour = None
    
    for c in contours:
        area = cv2.contourArea(c)
        if area > 1000:  # Ignore small noise/hand movements
            if area > max_area:
                max_area = area
                parcel_contour = c
                
    if parcel_contour is None:
        return None, None, None

    # Rotated bounding box
    min_area_rect = cv2.minAreaRect(parcel_contour)
    box_points = cv2.boxPoints(min_area_rect)
    box_points = np.intp(box_points)

    (cx, cy), (width_px, height_px), angle = min_area_rect
    
    # Apply calibration offset correction
    calibrated_w_px = max(1.0, width_px - CALIBRATION_OFFSET_PX)
    calibrated_h_px = max(1.0, height_px - CALIBRATION_OFFSET_PX)

    dim1_cm = calibrated_w_px / PIXELS_PER_CM
    dim2_cm = calibrated_h_px / PIXELS_PER_CM
    
    length_cm = max(dim1_cm, dim2_cm)
    width_cm = min(dim1_cm, dim2_cm)
    
    return length_cm, width_cm, box_points

def main():
    print("--- Starting Automated Camera-Triggered Scanner ---")
    print("[+] Press 'q' to quit.")
    
    # Robust Camera Initialization
    cap = None
    prev_frame = None
    for idx in [CAMERA_INDEX, 0, 2]:
        print(f"[+] Probing camera index {idx}...")
        temp_cap = cv2.VideoCapture(idx)
        if temp_cap.isOpened():
            # Set resolution before test read
            temp_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            temp_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            # Test frame grab to verify it is not locked by another app
            ret, test_frame = temp_cap.read()
            if ret and test_frame is not None:
                cap = temp_cap
                prev_frame = test_frame
                print(f"[+] Connected to working camera index {idx}.")
                break
            temp_cap.release()
            
    if cap is None:
        print("[-] Error: Could not grab a valid frame from any camera index (1, 0, or 2).")
        print("    Please make sure no other program (like Skype, Chrome, or Windows Camera app) is using the webcam.")
        import sys
        sys.exit(1)

    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    prev_gray = cv2.GaussianBlur(prev_gray, (21, 21), 0)

    current_state = STATE_EMPTY
    still_start_time = 0
    
    cv2.namedWindow("Automated Scanner Feed", cv2.WINDOW_NORMAL)
    try:
        cv2.setWindowProperty("Automated Scanner Feed", cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (21, 21), 0)

        # 1. Motion Calculation (Frame Differencing)
        frame_diff = cv2.absdiff(prev_gray, gray_blur)
        thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
        motion_score = np.sum(thresh)

        # Update previous frame for next iteration
        prev_gray = gray_blur.copy()

        # 2. State Machine for Automatic Capture
        status_text = "Status: Empty"
        status_color = (255, 255, 255)  # White

        if current_state == STATE_EMPTY:
            if motion_score > MOTION_THRESHOLD:
                current_state = STATE_MOVING
                print("[+] Motion detected. Parcel entering...")
                
        elif current_state == STATE_MOVING:
            status_text = "Status: Moving..."
            status_color = (0, 165, 255)  # Orange
            
            if motion_score < MOTION_THRESHOLD:
                # Motion stopped. Start timing stillness
                current_state = STATE_STILL
                still_start_time = time.time()
                print("    - Motion stopped. Waiting for parcel to settle...")
                
        elif current_state == STATE_STILL:
            status_text = "Status: Settling..."
            status_color = (0, 255, 255)  # Yellow
            
            if motion_score > MOTION_THRESHOLD:
                # Interrupted by motion (e.g. hand adjustment)
                current_state = STATE_MOVING
            else:
                elapsed_still = time.time() - still_start_time
                if elapsed_still >= STILL_DURATION_SEC:
                    # TRIGGER SCAN!
                    print("[!] TRIGGER: Scanning parcel dimensions...")
                    length, width, box_points = measure_parcel(frame)
                    
                    if length is not None:
                        timestamp = int(time.time())
                        scratch_dir = os.path.dirname(os.path.abspath(__file__))
                        
                        # 1. Save RAW Reference Image (no drawings)
                        raw_path = os.path.join(scratch_dir, f"raw_reference_{timestamp}.png")
                        cv2.imwrite(raw_path, frame)
                        
                        # 2. Draw clean measurement overlays on a copy of the frame
                        annotated_frame = frame.copy()
                        cv2.drawContours(annotated_frame, [box_points], 0, (0, 255, 0), 3)
                        
                        # Draw points on all 4 corners
                        for pt in box_points:
                            cv2.circle(annotated_frame, (pt[0], pt[1]), 5, (0, 0, 255), -1)
                        
                        # Add a semi-transparent black background behind the text for readability
                        text_l = f"L: {length:.1f} cm"
                        text_w = f"W: {width:.1f} cm"
                        
                        # Draw Length text at box_points[0]
                        tx1, ty1 = int(box_points[0][0]), int(box_points[0][1] - 15)
                        cv2.rectangle(annotated_frame, (tx1 - 5, ty1 - 25), (tx1 + 160, ty1 + 5), (0, 0, 0), -1)
                        cv2.putText(annotated_frame, text_l, (tx1, ty1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                    
                        # Draw Width text at box_points[1]
                        tx2, ty2 = int(box_points[1][0]), int(box_points[1][1] - 15)
                        cv2.rectangle(annotated_frame, (tx2 - 5, ty2 - 25), (tx2 + 160, ty2 + 5), (0, 0, 0), -1)
                        cv2.putText(annotated_frame, text_w, (tx2, ty2 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                        # 3. Save ANNOTATED Result Image
                        measured_path = os.path.join(scratch_dir, f"measured_result_{timestamp}.png")
                        cv2.imwrite(measured_path, annotated_frame)
                        
                        print(f"\n=====================================")
                        print(f"    PARCEL SCAN RESULT")
                        print(f"    Length: {length:.1f} cm")
                        print(f"    Width:  {width:.1f} cm")
                        print(f"=====================================")
                        print(f"[+] Saved RAW reference image to: {raw_path}")
                        print(f"[+] Saved MEASURED image to:      {measured_path}")

                        # 4. Display result in a pop-up window for 3 seconds
                        cv2.imshow("Scan Result", annotated_frame)
                        try:
                            cv2.setWindowProperty("Scan Result", cv2.WND_PROP_TOPMOST, 1)
                        except Exception:
                            pass
                        cv2.waitKey(3000)  # Wait 3000ms (3 seconds)
                        cv2.destroyWindow("Scan Result")
                    else:
                        print("[-] Scan failed: Could not isolate parcel contour.")

                    current_state = STATE_WAIT_FOR_EXIT

        elif current_state == STATE_WAIT_FOR_EXIT:
            status_text = "Status: Scan Complete. Please remove parcel."
            status_color = (0, 255, 0)  # Green
            
            # Wait for exit motion
            if motion_score > MOTION_THRESHOLD:
                # Wait for motion to stop again (empty background)
                time.sleep(1.0)  # Give time for the hand/parcel to leave
                current_state = STATE_EMPTY
                print("[+] Parcel removed. Ready for next package.\n")

        # 3. Draw overlays on the live feed
        display_frame = frame.copy()
        cv2.putText(display_frame, status_text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(display_frame, f"Motion Score: {motion_score}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Automated Scanner Feed", display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
