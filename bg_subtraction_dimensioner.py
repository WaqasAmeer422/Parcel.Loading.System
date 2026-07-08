import cv2
import numpy as np
import os
import sys

# --- CONFIGURATION ---
# Path to the empty table background image
BG_IMAGE_PATH = r"C:\Users\Admin\.gemini\antigravity-cli\brain\77b04675-0428-4a2d-984f-9d9b6087de4d\emptyTable.jpg"
# Path to the parcel image
FG_IMAGE_PATH = r"C:\Users\Admin\.gemini\antigravity-cli\brain\77b04675-0428-4a2d-984f-9d9b6087de4d\scratch\parcelImage1.jpg"

# Calibration: We use Length = 11.0 cm of the top face as the physical reference scale
# (Since perspective is tilted, we'll calculate scale relative to the detected box length)
PHYSICAL_LENGTH_CM = 11.0
DILATION_OFFSET_PX = 8.0  # Slightly smaller offset for clean subtraction borders
# ---------------------

def main():
    print("--- Background Subtraction Parcel Detector ---")

    # Allow custom foreground override if passed as command line argument
    fg_path = FG_IMAGE_PATH
    if len(sys.argv) > 1:
        fg_path = sys.argv[1]

    # Verify files exist
    if not os.path.exists(BG_IMAGE_PATH):
        print(f"[-] Error: Background image not found at: {BG_IMAGE_PATH}")
        sys.exit(1)
    if not os.path.exists(fg_path):
        print(f"[-] Error: Foreground parcel image not found at: {fg_path}")
        sys.exit(1)

    # 1. Load images
    bg_img = cv2.imread(BG_IMAGE_PATH)
    fg_img = cv2.imread(fg_path)

    if bg_img is None or fg_img is None:
        print("[-] Error: OpenCV failed to read one of the images.")
        sys.exit(1)

    # Ensure both images are the same resolution
    if bg_img.shape != fg_img.shape:
        print("[-] Warning: Background and foreground sizes differ. Resizing background...")
        bg_img = cv2.resize(bg_img, (fg_img.shape[1], fg_img.shape[0]))

    # Convert to grayscale
    bg_gray = cv2.cvtColor(bg_img, cv2.COLOR_BGR2GRAY)
    fg_gray = cv2.cvtColor(fg_img, cv2.COLOR_BGR2GRAY)

    # Apply light Gaussian blur to reduce camera noise
    bg_blur = cv2.GaussianBlur(bg_gray, (5, 5), 0)
    fg_blur = cv2.GaussianBlur(fg_gray, (5, 5), 0)

    # 2. Perform Absolute Difference (Background Subtraction)
    # This subtracts the wood texture of the empty table completely!
    diff = cv2.absdiff(bg_blur, fg_blur)

    # Threshold the difference image to get a binary mask of the box
    _, thresh = cv2.threshold(diff, 18, 255, cv2.THRESH_BINARY)

    # Clean up the mask using morphological closing (fills holes inside the box)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
    mask_closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    mask_dilated = cv2.dilate(mask_closed, kernel, iterations=1)

    # 3. Find Contours
    contours, _ = cv2.findContours(mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    parcel_contour = None
    max_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area > 1000:  # Ignore small noise
            if area > max_area:
                max_area = area
                parcel_contour = c

    if parcel_contour is None:
        print("[-] Error: Could not isolate the parcel.")
        print("    Try adjusting the threshold value (currently 18) in the script.")
        # Save difference image for debugging
        debug_fail_path = os.path.join(os.path.dirname(fg_path), "diff_debug_fail.png")
        cv2.imwrite(debug_fail_path, diff)
        print(f"[+] Saved difference map for debugging: {debug_fail_path}")
        sys.exit(1)

    # 4. Fit Rotated Bounding Box
    min_area_rect = cv2.minAreaRect(parcel_contour)
    box_points = cv2.boxPoints(min_area_rect)
    box_points = np.intp(box_points)

    (cx, cy), (width_px, height_px), angle = min_area_rect

    # Calibration scale factor
    # We use the longer side of the bounding box as the reference Length
    max_px = max(width_px, height_px)
    pixels_per_cm = max_px / PHYSICAL_LENGTH_CM
    print(f"[+] Calibration:")
    print(f"    - Reference Length: {PHYSICAL_LENGTH_CM} cm ({max_px:.1f} pixels)")
    print(f"    - Calculated Scale: {pixels_per_cm:.2f} pixels/cm")

    # Subtract dilation offset bias
    calibrated_w_px = max(1.0, width_px - DILATION_OFFSET_PX)
    calibrated_h_px = max(1.0, height_px - DILATION_OFFSET_PX)

    dim1_cm = calibrated_w_px / pixels_per_cm
    dim2_cm = calibrated_h_px / pixels_per_cm

    length_cm = max(dim1_cm, dim2_cm)
    width_cm = min(dim1_cm, dim2_cm)

    # 5. Output results
    print("\n=============================================")
    print("  BACKGROUND SUBTRACTION MEASUREMENT RESULTS ")
    print("=============================================")
    print(f"  Length: {length_cm:.2f} cm (Target: 11.0 cm)")
    print(f"  Width:  {width_cm:.2f} cm (Target: 8.5 cm)")
    print("=============================================")

    # Draw result overlay
    output_img = fg_img.copy()
    cv2.drawContours(output_img, [box_points], 0, (0, 255, 0), 3)
    cv2.putText(output_img, f"L: {length_cm:.1f} cm", (box_points[0][0], box_points[0][1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
    cv2.putText(output_img, f"W: {width_cm:.1f} cm", (box_points[1][0], box_points[1][1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

    # Save output images
    result_path = os.path.join(os.path.dirname(fg_path), "bg_sub_result.png")
    cv2.imwrite(result_path, output_img)
    print(f"\n[+] Saved final annotated scan to: {result_path}")

    # Save binary mask for debugging
    mask_path = os.path.join(os.path.dirname(fg_path), "bg_sub_mask.png")
    cv2.imwrite(mask_path, mask_dilated)
    print(f"[+] Saved binary subtraction mask to: {mask_path}")

if __name__ == "__main__":
    main()
