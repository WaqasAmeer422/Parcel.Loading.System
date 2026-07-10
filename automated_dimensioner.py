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

# Path where captured raw and measured images will be saved
OUTPUT_DIR = r"D:\123\Cargo Info Capturing Unit\Parcel.Loading.System\Parcel Capture Images"
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
    
    # Destination points for flat view (with a 0.5 cm margin to allow corner placement)
    dest_w = int(MAT_WIDTH_CM * WARP_SCALE)
    dest_h = int(MAT_HEIGHT_CM * WARP_SCALE)
    margin_px = int(0.5 * WARP_SCALE)  # 0.5 cm crop margin
    
    dest_pts = np.float32([
        [margin_px, margin_px],
        [dest_w - 1 - margin_px, margin_px],
        [dest_w - 1 - margin_px, dest_h - 1 - margin_px],
        [margin_px, dest_h - 1 - margin_px]
    ])
    
    # Compute Homography Matrix
    M = cv2.getPerspectiveTransform(sorted_pts, dest_pts)
    warped = cv2.warpPerspective(frame, M, (dest_w, dest_h))
    
    return warped, sorted_pts, M

def measure_parcel_in_mat(warped_mat):
    """Detects and measures the parcel inside the warped (flat) black mat."""
    gray = cv2.cvtColor(warped_mat, cv2.COLOR_BGR2GRAY)
    
    # 1. Standard Deviation Check: If the image has low contrast variance, the mat is empty!
    std_dev = np.std(gray)
    if std_dev < 15:  # Flat image (no parcel present)
        return None, None, None, None, None, False
        
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 2. Otsu's automatic thresholding (no manual tuning needed, dim/bright room safe!)
    otsu_thresh, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # If the optimal Otsu threshold is below 85, it means the whole image is dark (no bright parcel)
    if otsu_thresh < 85:
        return None, None, None, None, None, False
        
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    dilated = cv2.dilate(closed, kernel, iterations=1)
    
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    parcel_contour = None
    max_area = 0
    
    # Warped image dimensions
    w_h, w_w = warped_mat.shape[:2]
    warped_area = w_h * w_w
    
    for c in contours:
        area = cv2.contourArea(c)
        # Skip small noise, and skip contours > 75% of mat area (indicates mat border leakage)
        if 1500 < area < (0.75 * warped_area):
            if area > max_area:
                max_area = area
                parcel_contour = c
                
    if parcel_contour is None:
        return None, None, None, None, None, False
        
    # 2. Find top face edges by polygon approximation of the parcel contour
    perimeter = cv2.arcLength(parcel_contour, True)
    approx = cv2.approxPolyDP(parcel_contour, 0.04 * perimeter, True)
    
    # If the contour has 4 corners, it's a perfect top-down quad face!
    if len(approx) == 4:
        box_points_warped = approx.reshape(4, 2).astype(np.float32)
    else:
        # Fallback to minimum bounding rectangle if not exactly 4 corners
        min_area_rect = cv2.minAreaRect(parcel_contour)
        box_points_warped = cv2.boxPoints(min_area_rect)
        box_points_warped = np.float32(box_points_warped)
        
    # Sort the corners in clockwise order
    box_points_warped = sort_corners(box_points_warped)
    
    # Check if any corner of the box touches/crosses the mat borders (0.5 cm margin in warped pixels)
    # This prevents false positives on border leakage and warns if parcel is placed off-mat!
    border_margin_px = int(0.5 * WARP_SCALE)
    is_on_boundary = False
    for pt in box_points_warped:
        x, y = pt[0], pt[1]
        if (x <= border_margin_px or x >= (w_w - 1 - border_margin_px) or
            y <= border_margin_px or y >= (w_h - 1 - border_margin_px)):
            is_on_boundary = True
            break
            
    # Calculate Length and Width of the Top Face in centimeters using Euclidean distance
    side1 = np.linalg.norm(box_points_warped[0] - box_points_warped[1]) / WARP_SCALE
    side2 = np.linalg.norm(box_points_warped[0] - box_points_warped[3]) / WARP_SCALE
    
    length_cm = max(side1, side2) - CALIBRATION_OFFSET_CM
    width_cm = min(side1, side2) - CALIBRATION_OFFSET_CM

    # 3. Dynamic Height Estimation (3rd Dimension)
    # Estimates height by dividing the perspective side projection area by the box length
    min_area_rect = cv2.minAreaRect(parcel_contour)
    outer_area = min_area_rect[1][0] * min_area_rect[1][1]
    top_face_area = cv2.contourArea(parcel_contour)
    
    side_area = max(0.0, outer_area - top_face_area)
    box_length_px = max(min_area_rect[1][0], min_area_rect[1][1])
    side_thickness_px = side_area / max(1.0, box_length_px)
    
    # For a 35-degree tilted camera, height_cm = (side_thickness_px / scale) / sin(35 degrees)
    height_cm = (side_thickness_px / WARP_SCALE) / 0.57
    height_cm = float(np.clip(height_cm, 2.0, 15.0)) # Clamp to realistic values

    # 4. Material Type Classification using HSV color & texture properties
    material = "Cardboard Box"
    x_start = int(max(0, np.min(box_points_warped[:, 0])))
    x_end = int(min(w_w, np.max(box_points_warped[:, 0])))
    y_start = int(max(0, np.min(box_points_warped[:, 1])))
    y_end = int(min(w_h, np.max(box_points_warped[:, 1])))
    parcel_roi = warped_mat[y_start:y_end, x_start:x_end]
    
    if parcel_roi.size > 0:
        hsv_roi = cv2.cvtColor(parcel_roi, cv2.COLOR_BGR2HSV)
        h_channel, s_channel, v_channel = cv2.split(hsv_roi)
        avg_h = np.mean(h_channel)
        avg_s = np.mean(s_channel)
        avg_v = np.mean(v_channel)
        std_v = np.std(v_channel)
        
        # Cardboard: Brown/tan hue range (10-25) and moderate saturation (30-150)
        if 10 <= avg_h <= 25 and 30 <= avg_s <= 150:
            material = "Cardboard Box"
        # Plastic mailer: Low saturation (white/grey) and high value (bright) with high reflection variance
        elif avg_s < 30 and avg_v > 150:
            if std_v > 35:
                material = "Plastic Mailer"
            else:
                material = "Paper Envelope"
        # Colorful plastic packaging
        else:
            material = "Packaging (Plastic)"
            
    return length_cm, width_cm, height_cm, material, box_points_warped, is_on_boundary

def main():
    # Ensure capture folder exists
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    print("--- Starting Mat-Tracking Automated Scanner ---")
    print(f"Tracking Mat: {MAT_WIDTH_CM} cm x {MAT_HEIGHT_CM} cm")
    print(f"Saving Images to: {OUTPUT_DIR}")
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

    try:
        while True:
            # Check if the stream window was closed by the user clicking the "X" button
            try:
                if cv2.getWindowProperty("Automated Scanner Feed", cv2.WND_PROP_VISIBLE) < 1:
                    print("[+] Stream window closed by user.")
                    break
            except Exception:
                print("[+] Stream window closed.")
                break

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
            length, width, height, material, box_points_original = None, None, None, None, None
            is_on_boundary = False
            
            if warped_mat is not None:
                # Draw blue outline around the tracked mat
                cv2.polylines(display_frame, [mat_corners.astype(int)], True, (255, 0, 0), 2)
                cv2.putText(display_frame, f"Tracked Mat ({MAT_WIDTH_CM}x{MAT_HEIGHT_CM}cm)", 
                            (int(mat_corners[0][0]), int(mat_corners[0][1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
                # Measure parcel inside the warped mat
                length, width, height, material, box_points_warped, is_on_boundary = measure_parcel_in_mat(warped_mat)
                
                if length is not None:
                    try:
                        # Project the box corners from warped space back to the original camera perspective!
                        M_inv = np.linalg.inv(M)
                        pts_warped = box_points_warped.reshape(-1, 1, 2)
                        pts_original = cv2.perspectiveTransform(pts_warped, M_inv)
                        box_points_original = np.intp(pts_original.reshape(-1, 2))
                        
                        if is_on_boundary:
                            # Draw live RED warning box on the camera feed
                            cv2.drawContours(display_frame, [box_points_original], 0, (0, 0, 255), 2)
                            cv2.putText(display_frame, "OFF-MAT WARNING", 
                                        (box_points_original[0][0], box_points_original[0][1] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                            # Reject measurements for the state machine to prevent auto-scans
                            length, width = None, None
                        else:
                            # Draw live green measurement box on the camera feed
                            cv2.drawContours(display_frame, [box_points_original], 0, (0, 255, 0), 2)
                            cv2.putText(display_frame, f"{length:.1f}x{width:.1f}x{height:.1f} cm ({material})", 
                                        (box_points_original[0][0], box_points_original[0][1] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    except np.linalg.LinAlgError:
                        # Mat corners were collinear/singular; skip mapping this frame
                        length, width, box_points_original = None, None, None

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
                                
                                # Save RAW Reference Frame (un-annotated)
                                raw_path = os.path.join(OUTPUT_DIR, f"raw_reference_{timestamp}.png")
                                cv2.imwrite(raw_path, frame)
                                
                                # Draw clean, high-contrast overlay on the captured frame
                                annotated = frame.copy()
                                cv2.polylines(annotated, [mat_corners.astype(int)], True, (255, 0, 0), 2)
                                cv2.drawContours(annotated, [box_points_original], 0, (0, 255, 0), 3)
                                
                                # Text labels with black backgrounds for visibility
                                text_l = f"L: {length:.1f} cm"
                                text_w = f"W: {width:.1f} cm"
                                text_h = f"H: {height:.1f} cm ({material})"
                                
                                tx1, ty1 = int(box_points_original[0][0]), int(box_points_original[0][1] - 15)
                                cv2.rectangle(annotated, (tx1 - 5, ty1 - 25), (tx1 + 160, ty1 + 5), (0, 0, 0), -1)
                                cv2.putText(annotated, text_l, (tx1, ty1 - 5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                            
                                tx2, ty2 = int(box_points_original[1][0]), int(box_points_original[1][1] - 15)
                                cv2.rectangle(annotated, (tx2 - 5, ty2 - 25), (tx2 + 160, ty2 + 5), (0, 0, 0), -1)
                                cv2.putText(annotated, text_w, (tx2, ty2 - 5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                                tx3, ty3 = int(box_points_original[2][0]), int(box_points_original[2][1] - 15)
                                cv2.rectangle(annotated, (tx3 - 5, ty3 - 25), (tx3 + 300, ty3 + 5), (0, 0, 0), -1)
                                cv2.putText(annotated, text_h, (tx3, ty3 - 5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                            
                                measured_path = os.path.join(OUTPUT_DIR, f"measured_result_{timestamp}.png")
                                cv2.imwrite(measured_path, annotated)
                                
                                print(f"\n=====================================")
                                print(f"    AUTOMATED DISPATCH RESULT")
                                print(f"    Length:   {length:.1f} cm")
                                print(f"    Width:    {width:.1f} cm")
                                print(f"    Height:   {height:.1f} cm")
                                print(f"    Material: {material}")
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

            if is_on_boundary and warped_mat is not None:
                status_text = "WARNING: Place parcel fully inside mat borders!"
                status_color = (0, 0, 255)  # Red

            # Display Live Scanner Window
            cv2.putText(display_frame, status_text, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv2.imshow("Automated Scanner Feed", display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("\n[+] Exiting safely via Ctrl+C...")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[+] Camera and windows closed safely.")

if __name__ == "__main__":
    main()
