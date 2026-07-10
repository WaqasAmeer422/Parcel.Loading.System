import os
import sys
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple

import cv2
import numpy as np

# ============================== CONFIGURATION ===============================
CAMERA_INDEX = 1
CAMERA_FALLBACK_INDICES = (0, 2)
FRAME_WIDTH, FRAME_HEIGHT = 1280, 720

MAT_WIDTH_CM, MAT_HEIGHT_CM = 22.0, 18.0
WARP_SCALE = 20.0                       # pixels per cm in the flat warped view
WARP_MARGIN_CM = 0.5                    # crop margin around the warped mat
CALIBRATION_OFFSET_CM = 0.5             # corrects dilation growth on L/W

# Height is estimated with NO depth sensor, from a single tilted RGB camera.
# The camera looks at the mat from a known fixed tilt angle, so a vertical
# side face of the parcel becomes visible next to its bright top face. That
# visible side band, converted to mat-plane cm, is the projection of the
# real height onto the mat plane: apparent_cm = height_cm * sin(tilt).
# So: height_cm = apparent_cm / sin(tilt).
CAMERA_HEIGHT_CM = 45.0                 # camera height above the mat surface
CAMERA_TILT_DEG = 35.0                  # camera tilt angle from vertical
HEIGHT_CALIBRATION_OFFSET_CM = 0.5
HEIGHT_MIN_CM, HEIGHT_MAX_CM = 2.0, 15.0
_TILT_SIN = np.sin(np.radians(CAMERA_TILT_DEG))

MOTION_THRESHOLD = 50000                # min changed-pixel sum to register motion
STILL_DURATION_SEC = 1.0                # settle time before an auto scan fires

MIN_MAT_AREA_PX = 10000
MAT_EMPTY_STD_DEV = 15                  # below this, warped view is just flat mat
MIN_OTSU_THRESH = 85                    # below this, no bright object present
MIN_PARCEL_AREA_PX = 1500
MAX_PARCEL_AREA_RATIO = 0.75            # ignore contours that leak onto the mat edge
MIN_HOLE_AREA_PX = 100
MIN_PARCEL_SOLIDITY = 0.65              # rejects background-leaked/broken blobs
MAX_CANDIDATE_CONTOURS = 5              # how many area-ranked contours to test

# A box's top face should be a clean bright rectangle. If the locally-detected
# top face is too small a fraction of its own bounding region, the detection is
# unreliable (dark object, poor contrast) and we fall back to the outer contour.
MIN_TOP_FACE_AREA_RATIO = 0.25

CIRCULARITY_BOX = 0.76                  # round, rigid (e.g. shipping tube) -> Box
RECTANGULARITY_BOX = 0.85               # clean rectangle top face -> Box
BAG_SOLIDITY_MAX = 0.85                 # soft/wrinkled footprint, no holes -> Bag

# Material is judged by surface glossiness, NOT color, since parcel color can be
# anything. Matte cardboard/paper scatters light; plastic/wrap/tape reflects a
# tight bright "glint" (near-white, near-zero saturation).
SPECULAR_V_THRESH = 240
SPECULAR_S_THRESH = 30
SPECULAR_RATIO_SOFT = 0.10              # strong, broad glare -> Soft/glossy
SPECULAR_RATIO_MIXED = 0.03             # partial glare (label/tape) -> Mixed

OUTPUT_DIR = r"D:\123\Cargo Info Capturing Unit\Parcel.Loading.System\Parcel Capture Images"
# ==============================================================================


class ScanState(Enum):
    EMPTY = auto()
    MOVING = auto()
    STILL = auto()
    WAIT_FOR_EXIT = auto()


@dataclass
class ParcelResult:
    length_cm: float
    width_cm: float
    height_cm: float
    material_class: str      # "Hard" | "Soft" | "Mixed"
    material_detail: str
    object_type: str         # "Box" | "Bag" | "Object"
    box_points_warped: np.ndarray
    is_on_boundary: bool


# ------------------------------ shared geometry helpers ----------------------
def sort_corners(pts: np.ndarray) -> np.ndarray:
    """Sort 4 points clockwise: Top-Left, Top-Right, Bottom-Right, Bottom-Left."""
    pts = pts.reshape(4, 2)
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)]
    return rect


def contour_to_quad(contour: np.ndarray, epsilon_ratio: float) -> np.ndarray:
    """Approximate a contour to 4 sorted corners, falling back to a min-area rect."""
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
    if len(approx) != 4:
        approx = np.intp(cv2.boxPoints(cv2.minAreaRect(contour)))
    return sort_corners(approx.reshape(4, 2).astype(np.float32))


def contour_solidity(contour: np.ndarray, area: float) -> float:
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    return area / hull_area if hull_area > 0 else 0.0


def select_best_parcel_contour(contours, min_area: float, max_area: float):
    """
    Pick the best-looking parcel candidate: largest area first, but skip any
    candidate whose shape is too broken/non-convex (a sign that segmentation
    leaked into the mat/background) in favor of the next-largest clean one.
    """
    ranked = sorted(
        ((c, i, cv2.contourArea(c)) for i, c in enumerate(contours)),
        key=lambda t: t[2], reverse=True,
    )
    candidates = [t for t in ranked if min_area < t[2] < max_area][:MAX_CANDIDATE_CONTOURS]

    for contour, idx, area in candidates:
        if contour_solidity(contour, area) >= MIN_PARCEL_SOLIDITY:
            return contour, idx, area
    return None, -1, 0.0  # nothing looked like a clean, trustworthy object


def touches_mat_border(box_points: np.ndarray, h: int, w: int) -> bool:
    margin = int(WARP_MARGIN_CM * WARP_SCALE)
    xs, ys = box_points[:, 0], box_points[:, 1]
    return bool(np.any(xs <= margin) or np.any(xs >= w - 1 - margin) or
                np.any(ys <= margin) or np.any(ys >= h - 1 - margin))


def has_significant_holes(hierarchy, contours, parcel_idx: int) -> bool:
    if hierarchy is None:
        return False
    child_idx = hierarchy[0][parcel_idx][2]
    while child_idx != -1:
        if cv2.contourArea(contours[child_idx]) > MIN_HOLE_AREA_PX:
            return True
        child_idx = hierarchy[0][child_idx][0]
    return False


# ------------------------------ mat detection --------------------------------
def detect_and_warp_mat(frame: np.ndarray):
    """Detect the black mat (adaptive to lighting via Otsu) and warp it flat."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mat_contour, mat_idx, mat_area = select_best_parcel_contour(
        contours, MIN_MAT_AREA_PX, float("inf"))
    if mat_contour is None:
        return None, None, None

    sorted_pts = contour_to_quad(mat_contour, epsilon_ratio=0.03)

    dest_w, dest_h = int(MAT_WIDTH_CM * WARP_SCALE), int(MAT_HEIGHT_CM * WARP_SCALE)
    margin = int(WARP_MARGIN_CM * WARP_SCALE)
    dest_pts = np.float32([
        [margin, margin], [dest_w - 1 - margin, margin],
        [dest_w - 1 - margin, dest_h - 1 - margin], [margin, dest_h - 1 - margin],
    ])

    M = cv2.getPerspectiveTransform(sorted_pts, dest_pts)
    warped = cv2.warpPerspective(frame, M, (dest_w, dest_h))
    return warped, sorted_pts, M


# ------------------------------ top-face isolation ----------------------------
def find_top_face(roi: np.ndarray):
    """
    Split a parcel's ROI into its bright top face vs the darker side band that
    a tilted camera sees around it. Returns (top_face_contour_or_None, area).
    Used for BOTH clean L/W-and-shape measurement and height estimation, so the
    (relatively expensive) Otsu pass only runs once per frame.
    """
    if roi.size == 0:
        return None, 0.0
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0
    top_contour = max(contours, key=cv2.contourArea)
    return top_contour, cv2.contourArea(top_contour)


def estimate_height_cm(roi_area: float, top_face_area: float, box_px_span: float) -> float:
    """Real height from the visible side band left over around the top face."""
    side_area = max(0.0, roi_area - top_face_area)
    side_thickness_px = side_area / max(1.0, box_px_span)
    apparent_cm = side_thickness_px / WARP_SCALE
    height_cm = (apparent_cm / _TILT_SIN) - HEIGHT_CALIBRATION_OFFSET_CM
    return float(np.clip(height_cm, HEIGHT_MIN_CM, HEIGHT_MAX_CM))


# ------------------------------ shape / type / material ------------------------
def classify_object_type(circularity: float, rectangularity: float,
                          outer_solidity: float, has_holes: bool) -> str:
    if circularity > CIRCULARITY_BOX or rectangularity > RECTANGULARITY_BOX:
        return "Box"
    if not has_holes and outer_solidity < BAG_SOLIDITY_MAX:
        return "Bag"
    return "Object"


def classify_material(sample_roi: np.ndarray, object_type: str) -> Tuple[str, str]:
    """Color-independent Hard/Soft/Mixed classification via surface glossiness."""
    if object_type == "Bag":
        return "Soft", "Fabric / Poly Bag"
    if sample_roi is None or sample_roi.size == 0:
        return "Hard", "Unknown"

    hsv = cv2.cvtColor(sample_roi, cv2.COLOR_BGR2HSV)
    v, s = hsv[:, :, 2], hsv[:, :, 1]
    glossy_ratio = float(np.mean((v > SPECULAR_V_THRESH) & (s < SPECULAR_S_THRESH)))

    if glossy_ratio > SPECULAR_RATIO_SOFT:
        return "Soft", "Plastic / Glossy Wrap"
    if glossy_ratio > SPECULAR_RATIO_MIXED:
        return "Mixed", "Cardboard + Glossy Wrap/Label"
    return "Hard", "Cardboard / Paper"


# ------------------------------ parcel measurement ----------------------------
def measure_parcel_in_mat(warped_mat: np.ndarray) -> Optional[ParcelResult]:
    gray = cv2.cvtColor(warped_mat, cv2.COLOR_BGR2GRAY)
    if np.std(gray) < MAT_EMPTY_STD_DEV:
        return None  # flat image, mat is empty

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    otsu_thresh, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if otsu_thresh < MIN_OTSU_THRESH:
        return None  # no bright object present

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    dilated = cv2.dilate(cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel), kernel, iterations=1)
    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    warped_area = warped_mat.shape[0] * warped_mat.shape[1]
    parcel_contour, parcel_idx, area = select_best_parcel_contour(
        contours, MIN_PARCEL_AREA_PX, MAX_PARCEL_AREA_RATIO * warped_area)
    if parcel_contour is None:
        return None  # nothing solid/trustworthy found (e.g. background leak)

    has_holes = has_significant_holes(hierarchy, contours, parcel_idx)
    outer_solidity = contour_solidity(parcel_contour, area)
    outer_quad = contour_to_quad(parcel_contour, epsilon_ratio=0.03)
    is_on_boundary = touches_mat_border(outer_quad, *warped_mat.shape[:2])

    x0, x1 = int(max(0, outer_quad[:, 0].min())), int(min(warped_mat.shape[1], outer_quad[:, 0].max()))
    y0, y1 = int(max(0, outer_quad[:, 1].min())), int(min(warped_mat.shape[0], outer_quad[:, 1].max()))
    roi = warped_mat[y0:y1, x0:x1]

    top_contour, top_face_area = find_top_face(roi)
    roi_area = roi.shape[0] * roi.shape[1] if roi.size else 1
    top_face_valid = top_contour is not None and top_face_area >= MIN_TOP_FACE_AREA_RATIO * roi_area

    # Outer-footprint rectangularity: a real box's footprint stays a clean
    # rectangle even when a printed label fragments the top-face brightness
    # mask, so this is used as a second, more robust vote for "Box".
    outer_rect_w, outer_rect_h = cv2.minAreaRect(parcel_contour)[1]
    outer_rectangularity = area / (outer_rect_w * outer_rect_h) if outer_rect_w * outer_rect_h > 0 else 0.0
    outer_perimeter = cv2.arcLength(parcel_contour, True)
    outer_circularity = (4 * np.pi * area) / (outer_perimeter ** 2) if outer_perimeter > 0 else 0.0

    if top_face_valid:
        # Measure from the clean isolated top face (fixes tilt-merged blobs).
        dim_quad = contour_to_quad(top_contour, epsilon_ratio=0.04) + [x0, y0]
        top_perimeter = cv2.arcLength(top_contour, True)
        top_circularity = (4 * np.pi * top_face_area) / (top_perimeter ** 2) if top_perimeter > 0 else 0.0
        top_rect_w, top_rect_h = cv2.minAreaRect(top_contour)[1]
        top_rectangularity = top_face_area / (top_rect_w * top_rect_h) if top_rect_w * top_rect_h > 0 else 0.0
    else:
        # Fallback: dark/low-contrast object, use the outer footprint instead.
        dim_quad = outer_quad
        top_circularity, top_rectangularity = outer_circularity, outer_rectangularity

    circularity = max(top_circularity, outer_circularity)
    rectangularity = max(top_rectangularity, outer_rectangularity)

    side_a = np.linalg.norm(dim_quad[0] - dim_quad[1]) / WARP_SCALE
    side_b = np.linalg.norm(dim_quad[0] - dim_quad[3]) / WARP_SCALE
    length_cm = max(side_a, side_b) - CALIBRATION_OFFSET_CM
    width_cm = min(side_a, side_b) - CALIBRATION_OFFSET_CM

    object_type = classify_object_type(circularity, rectangularity, outer_solidity, has_holes)
    height_cm = estimate_height_cm(roi_area, top_face_area if top_face_valid else 0.0,
                                    max(roi.shape[:2]) if roi.size else 1)
    # Whole footprint (top + visible sides) gives a more representative gloss
    # reading than just the top-face crop, which can over-sample a shiny label.
    material_class, material_detail = classify_material(roi, object_type)

    return ParcelResult(length_cm, width_cm, height_cm, material_class, material_detail,
                         object_type, dim_quad, is_on_boundary)


# ------------------------------ camera / capture I-O ---------------------------
def open_camera():
    for idx in (CAMERA_INDEX, *CAMERA_FALLBACK_INDICES):
        print(f"[+] Probing camera index {idx}...")
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        ok, frame = cap.read()
        if ok and frame is not None:
            print(f"[+] Connected to working camera index {idx}.")
            return cap, frame
        cap.release()
    return None, None


def compute_motion_score(prev_gray: np.ndarray, frame: np.ndarray):
    gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (21, 21), 0)
    diff = cv2.absdiff(prev_gray, gray)
    score = int(np.sum(cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]))
    return score, gray


def draw_label(img: np.ndarray, text: str, x: int, y: int, color, box_w: int = 260):
    cv2.rectangle(img, (x - 5, y - 25), (x + box_w, y + 5), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def save_capture(frame, mat_corners, result: ParcelResult, box_points_original):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    timestamp = int(time.time())

    raw_path = os.path.join(OUTPUT_DIR, f"raw_reference_{timestamp}.png")
    cv2.imwrite(raw_path, frame)

    annotated = frame.copy()
    cv2.polylines(annotated, [mat_corners.astype(int)], True, (255, 0, 0), 2)
    cv2.drawContours(annotated, [box_points_original], 0, (0, 255, 0), 3)

    labels = (
        (f"L: {result.length_cm:.1f} cm ({result.object_type})", box_points_original[0]),
        (f"W: {result.width_cm:.1f} cm", box_points_original[1]),
        (f"H: {result.height_cm:.1f} cm ({result.material_class} - {result.material_detail})",
         box_points_original[2]),
    )
    for text, pt in labels:
        draw_label(annotated, text, int(pt[0]), int(pt[1] - 15), (0, 255, 0))

    measured_path = os.path.join(OUTPUT_DIR, f"measured_result_{timestamp}.png")
    cv2.imwrite(measured_path, annotated)

    print("\n=====================================")
    print("    AUTOMATED DISPATCH RESULT")
    print(f"    Type:     {result.object_type}")
    print(f"    Class:    {result.material_class}")
    print(f"    Detail:   {result.material_detail}")
    print(f"    Length:   {result.length_cm:.1f} cm")
    print(f"    Width:    {result.width_cm:.1f} cm")
    print(f"    Height:   {result.height_cm:.1f} cm")
    print("=====================================")
    print(f"[+] Saved RAW reference to: {raw_path}")
    print(f"[+] Saved MEASURED result to: {measured_path}")

    cv2.imshow("Scan Result", annotated)
    try:
        cv2.setWindowProperty("Scan Result", cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass
    cv2.waitKey(3000)
    cv2.destroyWindow("Scan Result")


# ------------------------------ main loop -------------------------------------
def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print("--- Starting Mat-Tracking Automated Scanner ---")
    print(f"Tracking Mat: {MAT_WIDTH_CM} cm x {MAT_HEIGHT_CM} cm")
    print(f"Saving Images to: {OUTPUT_DIR}")
    print("[+] Press 'q' to quit.")

    cap, first_frame = open_camera()
    if cap is None:
        print("[-] Error: Could not grab a valid frame from any camera index.")
        sys.exit(1)

    prev_gray = cv2.GaussianBlur(cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY), (21, 21), 0)
    state = ScanState.EMPTY
    still_start_time = 0.0

    cv2.namedWindow("Automated Scanner Feed", cv2.WINDOW_NORMAL)
    try:
        cv2.setWindowProperty("Automated Scanner Feed", cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass

    try:
        while True:
            try:
                if cv2.getWindowProperty("Automated Scanner Feed", cv2.WND_PROP_VISIBLE) < 1:
                    print("[+] Stream window closed by user.")
                    break
            except Exception:
                print("[+] Stream window closed.")
                break

            ok, frame = cap.read()
            if not ok:
                break

            motion_score, prev_gray = compute_motion_score(prev_gray, frame)
            warped_mat, mat_corners, M = detect_and_warp_mat(frame)
            display_frame = frame.copy()
            result, box_points_original = None, None

            if warped_mat is not None:
                cv2.polylines(display_frame, [mat_corners.astype(int)], True, (255, 0, 0), 2)
                cv2.putText(display_frame, f"Tracked Mat ({MAT_WIDTH_CM}x{MAT_HEIGHT_CM}cm)",
                            (int(mat_corners[0][0]), int(mat_corners[0][1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                result = measure_parcel_in_mat(warped_mat)
                if result is not None:
                    try:
                        pts_original = cv2.perspectiveTransform(
                            result.box_points_warped.reshape(-1, 1, 2), np.linalg.inv(M))
                        box_points_original = np.intp(pts_original.reshape(-1, 2))
                    except np.linalg.LinAlgError:
                        result = None  # mat corners collinear/singular this frame

                if result is not None and result.is_on_boundary:
                    cv2.drawContours(display_frame, [box_points_original], 0, (0, 0, 255), 2)
                    cv2.putText(display_frame, "OFF-MAT WARNING",
                                (box_points_original[0][0], box_points_original[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                elif result is not None:
                    color = (0, 255, 0) if result.object_type == "Box" else (0, 165, 255)
                    cv2.drawContours(display_frame, [box_points_original], 0, color, 2)
                    cv2.putText(display_frame,
                                f"{result.length_cm:.1f}x{result.width_cm:.1f}x{result.height_cm:.1f} cm "
                                f"({result.object_type} | {result.material_class} | {result.material_detail})",
                                (box_points_original[0][0], box_points_original[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.putText(display_frame, f"Type: {result.object_type}", (20, 75),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            usable = result is not None and not result.is_on_boundary
            status_text, status_color = "Status: No Mat Detected", (0, 0, 255)

            if warped_mat is not None:
                if state == ScanState.EMPTY:
                    status_text, status_color = "Status: Mat Tracked (Empty)", (255, 0, 0)
                    if motion_score > MOTION_THRESHOLD and usable:
                        state = ScanState.MOVING
                        print("[+] Motion detected. Parcel entering mat...")

                elif state == ScanState.MOVING:
                    status_text, status_color = "Status: Scanning...", (0, 165, 255)
                    if motion_score < MOTION_THRESHOLD:
                        state, still_start_time = ScanState.STILL, time.time()
                        print("    - Motion stopped. Waiting for parcel to settle...")

                elif state == ScanState.STILL:
                    status_text, status_color = "Status: Settling...", (0, 255, 255)
                    if motion_score > MOTION_THRESHOLD:
                        state = ScanState.MOVING
                    elif time.time() - still_start_time >= STILL_DURATION_SEC:
                        print("[!] TRIGGER: Capturing parcel dimensions...")
                        if usable:
                            save_capture(frame, mat_corners, result, box_points_original)
                        else:
                            print("[-] Scan failed: Parcel was moved or lost.")
                        state = ScanState.WAIT_FOR_EXIT

                elif state == ScanState.WAIT_FOR_EXIT:
                    status_text, status_color = "Status: Scan Complete. Please remove parcel.", (0, 255, 0)
                    if motion_score > MOTION_THRESHOLD:
                        time.sleep(1.0)
                        state = ScanState.EMPTY
                        print("[+] Parcel removed. Ready for next scan.\n")

            if result is not None and result.is_on_boundary:
                status_text, status_color = "WARNING: Place parcel fully inside mat borders!", (0, 0, 255)

            cv2.putText(display_frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
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