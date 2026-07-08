import cv2
import numpy as np
import time
import os
import sys

# --- CONFIGURATION ---
CAMERA_INDEX = 1
# Physical size of the black mat in centimeters
MAT_WIDTH_CM = 22.0
MAT_HEIGHT_CM = 18.0

# Scale factor for the warped region (pixels per cm)
WARP_SCALE = 20.0  # 1 cm = 20 pixels in the flat warped view
# Dilation offset correction in centimeters (applied to final measurements)
CALIBRATION_OFFSET_CM = 0.5 

# Motion Detection Parameters (used to trigger automated scan)
MOTION_THRESHOLD = 50000   # Min sum of changed pixels to register motion
STILL_DURATION_SEC = 1.0   # How long the parcel must be still to trigger scan
# ---------------------

# State Machine States
STATE_EMPTY = 0
STATE_MOVING = 1
STATE_STILL = 2
STATE_WAIT_FOR_EXIT = 3

def sort_corners(pts):
    """Sorts 4 corners in clockwise order: Top-Left, Top-Right, Bottom-Right, Bottom-Left."""
    pts = pts.reshape(4, 2)
    rect = np.zeros((4, 2), dtype=np.float32)
    
    # Top-Left has minimum sum, Bottom-Right has maximum sum
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    
    # Top-Right has minimum difference, Bottom-Left has maximum difference
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    
    return rect

def detect_and_warp_mat(frame):
    """Detects the black mat, warps the image, and returns the flat view."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # Threshold to isolate the dark mat (mat is dark, wood table is bright)
    # Adjust threshold value (100) if mat detection is unstable under your lighting
    _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
    
    # Clean up threshold mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    mat_contour = None
    max_area = 0
    
    for c in contours:
        area = cv2.contourArea(c)
        if area > 10000:  # Mat should be relatively large
            if area > max_area:
                max_area = area
                mat_contour = c
                
    if mat_contour is None:
        return None, None, None

    # Approximate polygon to find 4 corners
    perimeter = cv2.arcLength(mat_contour, True)
    approx = cv2.approxPolyDP(mat_contour, 0.03 * perimeter, True)
    
    if len(approx) != 4:
        # If approximation fails to yield exactly 4 corners, fit a bounding box as fallback
        rect = cv2.minAreaRect(mat_contour)
        box_points = cv2.boxPoints(rect)
        approx = np.intp(box_points)
        
    # Sort corners in clockwise order
    sorted_pts = sort_corners(approx)
    
    # Destination points for flat view
    dest_w = int(MAT_WIDTH_CM * WARP_SCALE)
    dest_h = int(MAT_HEIGHT_CM * WARP_SCALE)
    
    dest_pts = np.float32([
        [0, 0],
        [dest_w - 1, 0],
        [dest_w - 1, dest_h - 1],
        [0, dest_h - 1]
    ])
    
    # Compute Homography Matrix
    M = cv2.getPerspectiveTransform(sorted_pts, dest_pts)
    warped = cv2.warpPerspective(frame, M, (dest_w, dest_h))
    
    return warped, sorted_pts, M

def measure_parcel_in_mat(warped_mat):
    """Detects and measures the parcel inside the warped (flat) black mat."""
    gray = cv2.cvtColor(warped_mat, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Since the mat is dark (values < 60) and the parcel is brighter:
    # Threshold isolates the bright parcel against the dark mat background
    _, thresh = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    dilated = cv2.dilate(closed, kernel, iterations=1)
    
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    parcel_contour = None
    max_area = 0
    
    for c in contours:
        area = cv2.contourArea(c)
        if area > 1000:  # Ignore tiny noise
            if area > max_area:
                max_area = area
                parcel_contour = c
                
    if parcel_contour is None:
        return None, None, None
        
    # Rotated bounding box in warped frame coordinates
    min_area_rect = cv2.minAreaRect(parcel_contour)
    box_points_warped = cv2.boxPoints(min_area_rect)
    box_points_warped = np.float32(box_points_warped)
    
    (cx, cy), (w_px, h_px), angle = min_area_rect
    
    # Convert pixels to cm
    w_cm = w_px / WARP_SCALE
    h_cm = h_px / WARP_SCALE
    
    # Subtract morphology dilation offset
    length_cm = max(w_cm, h_cm) - CALIBRATION_OFFSET_CM
    width_cm = min(w_cm, h_cm) - CALIBRATION_OFFSET_CM
    
    return length_cm, width_cm, box_points_warped

def main():
    print("--- Starting Mat-Tracking Automated Scanner ---")
    print(f"Tracking Mat: {MAT_WIDTH_CM} cm x {MAT_HEIGHT_CM} cm")
    print("[+] Press 'q' to quit.")
    
    # Camera Initialization
    cap = None
    prev_frame = None
    for idx in [CAMERA_INDEX, 0, 2]:
        print(f"[+] Probing camera index {idx}...")
        temp_cap = cv2.VideoCapture(idx)
        if temp_cap.isOpened():
            temp_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            temp_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            ret, test_frame = temp_cap.read()
            if ret and test_frame is not None:
                cap = temp_cap
                prev_frame = test_frame
                print(f"[+] Connected to working camera index {idx}.")
                break
            temp_cap.release()
            
    if cap is None:
        print("[-] Error: Could not grab a valid frame from any camera index.")
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

        # 1. Motion Calculation (Frame Differencing)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (21, 21), 0)
        frame_diff = cv2.absdiff(prev_gray, gray_blur)
        thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
        motion_score = np.sum(thresh)
        prev_gray = gray_blur.copy()

        # 2. Mat Detection and Perspective Warp
        warped_mat, mat_corners, M = detect_and_warp_mat(frame)
        
        display_frame = frame.copy()
        length, width, box_points_original = None, None, None
        
        if warped_mat is not None:
            # Draw blue outline around the tracked mat
            cv2.polylines(display_frame, [mat_corners.astype(int)], True, (255, 0, 0), 2)
            cv2.putText(display_frame, f"Tracked Mat ({MAT_WIDTH_CM}x{MAT_HEIGHT_CM}cm)", 
                        (int(mat_corners[0][0]), int(mat_corners[0][1] - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Measure parcel inside the warped mat
            length, width, box_points_warped = measure_parcel_in_mat(warped_mat)
            
            if length is not None:
                # Project the box corners from warped space back to the original camera perspective!
                M_inv = np.linalg.inv(M)
                pts_warped = box_points_warped.reshape(-1, 1, 2)
                pts_original = cv2.perspectiveTransform(pts_warped, M_inv)
                box_points_original = np.intp(pts_original.reshape(-1, 2))
                
                # Draw live green measurement box on the camera feed
                cv2.drawContours(display_frame, [box_points_original], 0, (0, 255, 0), 2)
                cv2.putText(display_frame, f"{length:.1f}x{width:.1f} cm", 
                            (box_points_original[0][0], box_points_original[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 3. State Machine for Automatic Capture
        status_text = "Status: No Mat Detected"
        status_color = (0, 0, 255)  # Red
        
        if warped_mat is not None:
            if current_state == STATE_EMPTY:
                status_text = "Status: Mat Tracked (Empty)"
                status_color = (255, 0, 0)  # Blue
                if motion_score > MOTION_THRESHOLD and length is not None:
                    current_state = STATE_MOVING
                    print("[+] Motion detected. Parcel entering mat...")
                    
            elif current_state == STATE_MOVING:
                status_text = "Status: Scanning..."
                status_color = (0, 165, 255)  # Orange
                if motion_score < MOTION_THRESHOLD:
                    current_state = STATE_STILL
                    still_start_time = time.time()
                    print("    - Motion stopped. Waiting for parcel to settle...")
                    
            elif current_state == STATE_STILL:
                status_text = "Status: Settling..."
                status_color = (0, 255, 255)  # Yellow
                if motion_score > MOTION_THRESHOLD:
                    current_state = STATE_MOVING
                else:
                    elapsed = time.time() - still_start_time
                    if elapsed >= STILL_DURATION_SEC:
                        print("[!] TRIGGER: Capturing parcel dimensions...")
                        if length is not None:
                            timestamp = int(time.time())
                            repo_dir = os.path.dirname(os.path.abspath(__file__))
                            
                            # Save RAW Reference Frame (un-annotated)
                            raw_path = os.path.join(repo_dir, f"raw_reference_{timestamp}.png")
                            cv2.imwrite(raw_path, frame)
                            
                            # Draw clean, high-contrast overlay on the captured frame
                            annotated = frame.copy()
                            cv2.polylines(annotated, [mat_corners.astype(int)], True, (255, 0, 0), 2)
                            cv2.drawContours(annotated, [box_points_original], 0, (0, 255, 0), 3)
                            
                            # Text labels with black backgrounds for visibility
                            text_l = f"L: {length:.1f} cm"
                            text_w = f"W: {width:.1f} cm"
                            
                            tx1, ty1 = int(box_points_original[0][0]), int(box_points_original[0][1] - 15)
                            cv2.rectangle(annotated, (tx1 - 5, ty1 - 25), (tx1 + 160, ty1 + 5), (0, 0, 0), -1)
                            cv2.putText(annotated, text_l, (tx1, ty1 - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                        
                            tx2, ty2 = int(box_points_original[1][0]), int(box_points_original[1][1] - 15)
                            cv2.rectangle(annotated, (tx2 - 5, ty2 - 25), (tx2 + 160, ty2 + 5), (0, 0, 0), -1)
                            cv2.putText(annotated, text_w, (tx2, ty2 - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                        
                            measured_path = os.path.join(repo_dir, f"measured_result_{timestamp}.png")
                            cv2.imwrite(measured_path, annotated)
                            
                            print(f"\n=====================================")
                            print(f"    AUTOMATED DISPATCH RESULT")
                            print(f"    Length: {length:.1f} cm")
                            print(f"    Width:  {width:.1f} cm")
                            print(f"=====================================")
                            print(f"[+] Saved RAW reference to: {raw_path}")
                            print(f"[+] Saved MEASURED result to: {measured_path}")
                            
                            # Display result in topmost popup for 3 seconds
                            cv2.imshow("Scan Result", annotated)
                            try:
                                cv2.setWindowProperty("Scan Result", cv2.WND_PROP_TOPMOST, 1)
                            except Exception:
                                pass
                            cv2.waitKey(3000)
                            cv2.destroyWindow("Scan Result")
                        else:
                            print("[-] Scan failed: Parcel was moved or lost.")
                            
                        current_state = STATE_WAIT_FOR_EXIT
                        
            elif current_state == STATE_WAIT_FOR_EXIT:
                status_text = "Status: Scan Complete. Please remove parcel."
                status_color = (0, 255, 0)  # Green
                if motion_score > MOTION_THRESHOLD:
                    time.sleep(1.0)
                    current_state = STATE_EMPTY
                    print("[+] Parcel removed. Ready for next scan.\n")

        # Display Live Scanner Window
        cv2.putText(display_frame, status_text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.imshow("Automated Scanner Feed", display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
