"""
Camera dimensioning module - headless version of your tested
parcel_dimensioning_scrpt.py.

Everything about HOW parcels are detected and measured (mat tracking,
motion state machine, top-face isolation, height-from-tilt estimate,
material/type classification) is unchanged from your original script.

What changed:
  - No cv2.imshow / live window / 'q' to quit -> runs headless in a
    background thread, stopped via .stop() instead.
  - save_capture() still writes the annotated image to disk, but now
    also keeps only the newest MAX_IMAGES files (rolling folder), and
    calls on_capture(result_dict) instead of just printing.
  - OUTPUT_DIR defaults to a Pi-friendly path instead of the old
    Windows D:\\ path - change it below if you want it elsewhere.
"""

import os
import time
import json
import platform
import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple

import cv2
import numpy as np

# ============================== CONFIGURATION ===============================
CAMERA_INDEX = 1
CAMERA_FALLBACK_INDICES = (0, 2)
FRAME_WIDTH, FRAME_HEIGHT = 1280, 720

MAT_WIDTH_CM, MAT_HEIGHT_CM = 10.0, 10.0   # your actual L-bracket platform size
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "platform_calibration.json")
WARP_SCALE = 20.0                       # pixels per cm in the flat warped view
WARP_MARGIN_CM = 0.5                    # crop margin around the warped mat
CALIBRATION_OFFSET_CM = 0.5             # corrects dilation growth on L/W

# Height is estimated with NO depth sensor, from a single tilted RGB camera.
# The camera looks at the mat from a known fixed tilt angle, so a vertical
# side face of the parcel becomes visible next to its bright top face. That
# visible side band, converted to mat-plane cm, is the projection of the
# real height onto the mat plane: apparent_cm = height_cm * sin(tilt).
# So: height_cm = apparent_cm / sin(tilt).
CAMERA_HEIGHT_CM = 29.0                 # camera height above the mat surface
CAMERA_TILT_DEG = 35.0                  # camera tilt angle from vertical
HEIGHT_CALIBRATION_OFFSET_CM = 0.5
# NEEDS CALIBRATION: scales the apparent side-band measurement before the tilt
# correction. Not yet validated against a real ruler measurement.
HEIGHT_SCALE_FACTOR = 1.0
HEIGHT_MIN_CM, HEIGHT_MAX_CM = 2.0, 15.0
_TILT_SIN = np.sin(np.radians(CAMERA_TILT_DEG))

MOTION_THRESHOLD = 2000                 # calibrated threshold for 160x120 resolution
still_start_time = 0.0
STILL_DURATION_SEC = 1.0                # settle time before an auto scan fires

MIN_MAT_AREA_PX = 10000
MAT_EMPTY_STD_DEV = 15                  # below this, warped view is just flat mat
MIN_OTSU_THRESH = 85                    # below this, no bright object present
MIN_PARCEL_AREA_PX = 1500
MAX_PARCEL_AREA_RATIO = 0.75            # ignore contours that leak onto the mat edge
MIN_HOLE_AREA_PX = 100
MIN_PARCEL_SOLIDITY = 0.65              # rejects background-leaked/broken blobs
MAX_CANDIDATE_CONTOURS = 5              # how many area-ranked contours to test

MIN_TOP_FACE_AREA_RATIO = 0.25

CIRCULARITY_BOX = 0.76
RECTANGULARITY_BOX = 0.85
BAG_SOLIDITY_MAX = 0.85

SPECULAR_V_THRESH = 240
SPECULAR_S_THRESH = 30
SPECULAR_RATIO_SOFT = 0.10
SPECULAR_RATIO_MIXED = 0.03

# Pi-friendly path (change if you want captures elsewhere)
OUTPUT_DIR = "/home/pi/parcel_system/captures"
MAX_IMAGES = 20   # keep only the newest N annotated images
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
    material_class: str
    material_detail: str
    object_type: str
    box_points_warped: np.ndarray
    is_on_boundary: bool


# ------------------------------ shared geometry helpers ----------------------
def sort_corners(pts: np.ndarray) -> np.ndarray:
    pts = pts.reshape(4, 2)
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)]
    return rect


def contour_to_quad(contour: np.ndarray) -> np.ndarray:
    box = cv2.boxPoints(cv2.minAreaRect(contour))
    return sort_corners(box.astype(np.float32))


def contour_solidity(contour: np.ndarray, area: float) -> float:
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    return area / hull_area if hull_area > 0 else 0.0


def select_best_parcel_contour(contours, min_area: float, max_area: float):
    ranked = sorted(
        ((c, i, cv2.contourArea(c)) for i, c in enumerate(contours)),
        key=lambda t: t[2], reverse=True,
    )
    candidates = [t for t in ranked if min_area < t[2] < max_area][:MAX_CANDIDATE_CONTOURS]

    for contour, idx, area in candidates:
        if contour_solidity(contour, area) >= MIN_PARCEL_SOLIDITY:
            return contour, idx, area
    return None, -1, 0.0


def touches_mat_border(box_points: np.ndarray, h: int, w: int) -> bool:
    # Restricts placement to the conveyor belt (X between 10 and 245)
    # and ignores Top/Bottom borders (Y between 15 and h-15)
    for pt in box_points:
        x, y = pt[0], pt[1]
        if x <= 10 or x >= 245 or y <= 15 or y >= h - 15:
            return True
    return False


def has_significant_holes(hierarchy, contours, parcel_idx: int) -> bool:
    if hierarchy is None:
        return False
    child_idx = hierarchy[0][parcel_idx][2]
    while child_idx != -1:
        if cv2.contourArea(contours[child_idx]) > MIN_HOLE_AREA_PX:
            return True
        child_idx = hierarchy[0][child_idx][0]
    return False


# ------------------------------ platform (fixed calibration) ------------------
# The platform is a FIXED 10x10cm zone marked by the 4 L-brackets on the belt -
# it doesn't move relative to the camera, so we calibrate its 4 corner pixel
# positions ONCE (see calibrate_platform.py) instead of re-detecting it every
# frame. Trying to auto-detect it via thresholding (the old approach) picks up
# the whole dark belt as "the mat" since the belt and platform are the same
# color, which is exactly the bug you're seeing.
_PLATFORM_CORNERS = None  # cached after first load


def _load_platform_corners():
    global _PLATFORM_CORNERS
    if _PLATFORM_CORNERS is not None:
        return _PLATFORM_CORNERS

    if not os.path.exists(CALIBRATION_FILE):
        print(f"[camera_module] ERROR: no calibration file found at {CALIBRATION_FILE}")
        print("[camera_module] Run calibrate_platform.py once first to mark the 4 platform corners.")
        return None

    with open(CALIBRATION_FILE, "r") as f:
        pts = json.load(f)

    if len(pts) != 4:
        print(f"[camera_module] ERROR: {CALIBRATION_FILE} does not contain exactly 4 points.")
        return None

    _PLATFORM_CORNERS = sort_corners(np.array(pts, dtype=np.float32))
    return _PLATFORM_CORNERS


def detect_and_warp_mat(frame: np.ndarray):
    sorted_pts = _load_platform_corners()
    if sorted_pts is None:
        return None, None, None

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


def estimate_height_cm(footprint_area: float, top_face_area: float, box_px_span: float) -> float:
    side_area = max(0.0, footprint_area - top_face_area)
    side_thickness_px = side_area / max(1.0, box_px_span)
    apparent_cm = (side_thickness_px / WARP_SCALE) * HEIGHT_SCALE_FACTOR
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
    
    # Check if the conveyor belt is empty. Since the belt is uniform black rubber,
    # its standard deviation will be extremely low (<10.0) and its mean value will be dark (<45.0).
    # Inside the 10x10cm warped view (400x400px), the belt is X=3 to X=250.
    # We crop the belt check strictly inside the belt region (X=30 to 220, Y=50 to 350).
    belt_gray = gray[50:350, 30:220]
    if np.std(belt_gray) < 10.0 or np.mean(belt_gray) < 45.0:
        return None

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 1. First, check if the platform is empty using Otsu threshold on the conveyor belt.
    h, w = blurred.shape
    inner_mat = blurred[50:350, 30:220]
    
    otsu_val, _ = cv2.threshold(inner_mat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if otsu_val < MIN_OTSU_THRESH:
        return None
        
    # Use a band-pass filter (55 to 155) to isolate the brown cardboard box.
    thresh = cv2.inRange(blurred, 55, 155)
    
    # Clear borders of the 10x10cm platform.
    # Left edge gets 10px clear. Right side wood board (X >= 245) gets completely blacked out.
    # Top and bottom get 15px clear to ignore physical lines/L-brackets.
    thresh[0:15, :] = 0        # Top border
    thresh[h-15:h, :] = 0      # Bottom border
    thresh[:, 0:10] = 0        # Left border (ignores left edge)
    thresh[:, 245:w] = 0       # Right border (ignores right wooden plate completely)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    dilated = cv2.dilate(cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel), kernel, iterations=1)
    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    warped_area = warped_mat.shape[0] * warped_mat.shape[1]
    parcel_contour, parcel_idx, area = select_best_parcel_contour(
        contours, MIN_PARCEL_AREA_PX, MAX_PARCEL_AREA_RATIO * warped_area)
    if parcel_contour is None:
        return None

    has_holes = has_significant_holes(hierarchy, contours, parcel_idx)
    outer_solidity = contour_solidity(parcel_contour, area)
    outer_quad = contour_to_quad(parcel_contour)
    is_on_boundary = touches_mat_border(outer_quad, *warped_mat.shape[:2])

    x0, x1 = int(max(0, outer_quad[:, 0].min())), int(min(warped_mat.shape[1], outer_quad[:, 0].max()))
    y0, y1 = int(max(0, outer_quad[:, 1].min())), int(min(warped_mat.shape[0], outer_quad[:, 1].max()))
    roi = warped_mat[y0:y1, x0:x1]

    top_contour, top_face_area = find_top_face(roi)
    roi_area = roi.shape[0] * roi.shape[1] if roi.size else 1
    top_face_valid = top_contour is not None and top_face_area >= MIN_TOP_FACE_AREA_RATIO * roi_area

    outer_rect_w, outer_rect_h = cv2.minAreaRect(parcel_contour)[1]
    outer_rectangularity = area / (outer_rect_w * outer_rect_h) if outer_rect_w * outer_rect_h > 0 else 0.0
    outer_perimeter = cv2.arcLength(parcel_contour, True)
    outer_circularity = (4 * np.pi * area) / (outer_perimeter ** 2) if outer_perimeter > 0 else 0.0

    if top_face_valid:
        dim_quad = contour_to_quad(top_contour) + [x0, y0]
        top_perimeter = cv2.arcLength(top_contour, True)
        top_circularity = (4 * np.pi * top_face_area) / (top_perimeter ** 2) if top_perimeter > 0 else 0.0
        top_rect_w, top_rect_h = cv2.minAreaRect(top_contour)[1]
        top_rectangularity = top_face_area / (top_rect_w * top_rect_h) if top_rect_w * top_rect_h > 0 else 0.0
        rect_w, rect_h = top_rect_w, top_rect_h
    else:
        dim_quad = outer_quad
        top_circularity, top_rectangularity = outer_circularity, outer_rectangularity
        rect_w, rect_h = outer_rect_w, outer_rect_h

    circularity = max(top_circularity, outer_circularity)
    rectangularity = max(top_rectangularity, outer_rectangularity)

    length_cm = max(rect_w, rect_h) / WARP_SCALE - CALIBRATION_OFFSET_CM
    width_cm = min(rect_w, rect_h) / WARP_SCALE - CALIBRATION_OFFSET_CM

    object_type = classify_object_type(circularity, rectangularity, outer_solidity, has_holes)
    height_cm = estimate_height_cm(area, top_face_area if top_face_valid else 0.0, max(rect_w, rect_h))
    material_class, material_detail = classify_material(roi, object_type)

    return ParcelResult(length_cm, width_cm, height_cm, material_class, material_detail,
                         object_type, dim_quad, is_on_boundary)


# ------------------------------ camera / capture I-O ---------------------------
def get_backend():
    system = platform.system()
    if system == "Windows":
        return cv2.CAP_DSHOW
    elif system == "Linux":
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def open_camera():
    for idx in (CAMERA_INDEX, *CAMERA_FALLBACK_INDICES):
        print(f"[CameraDimensioner] Probing camera index {idx}...")
        # Force the right backend explicitly per OS - letting OpenCV
        # auto-pick on Linux commonly selects GStreamer, whose plugin set is
        # often incomplete for UVC cameras and throws "Internal data stream
        # error"; CAP_V4L2 doesn't exist on Windows at all, so it must be
        # conditional rather than hardcoded to one platform.
        cap = cv2.VideoCapture(idx, get_backend())
        if not cap.isOpened():
            continue
        # Request onboard MJPG compression BEFORE setting resolution. Without
        # this the camera sends raw YUYV frames (~1.8MB each at 720p), which
        # can exceed USB bandwidth on a CM3+ carrier board and trigger bus
        # resets / disconnects.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        ok, frame = cap.read()
        if ok and frame is not None:
            print(f"[CameraDimensioner] Connected to working camera index {idx}.")
            return cap, frame
        cap.release()
    return None, None


def compute_motion_score(prev_gray: np.ndarray, frame: np.ndarray):
    # Resize to 160x120 to filter out pixel noise and exposure fluctuations
    small_gray = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (160, 120))
    gray = cv2.GaussianBlur(small_gray, (9, 9), 0)
    diff = cv2.absdiff(prev_gray, gray)
    score = int(np.sum(cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]))
    return score, gray


def draw_label(img: np.ndarray, text: str, x: int, y: int, color, box_w: int = 260):
    cv2.rectangle(img, (x - 5, y - 25), (x + box_w, y + 5), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


# ------------------------------ headless orchestrator wrapper -----------------
class CameraDimensioner:
    def __init__(self, output_dir=OUTPUT_DIR, max_images=MAX_IMAGES, on_capture=None, conveyor=None, on_motion=None, on_reset=None):
        self.output_dir = output_dir
        self.max_images = max_images
        self.on_capture = on_capture  # callback: on_capture(result_dict)
        self.conveyor = conveyor      # optional ConveyorMotor - stopped while measuring
        self.on_motion = on_motion    # callback: on_motion() triggered when motion detected
        self.on_reset = on_reset      # callback: on_reset() triggered when platform cleared or scan fails
        self._running = False
        self._thread = None

    def _prune_old_images(self):
        files = sorted(
            (f for f in os.listdir(self.output_dir) if f.startswith("measured_result_") or f.startswith("failed_capture_")),
            key=lambda f: os.path.getmtime(os.path.join(self.output_dir, f))
        )
        while len(files) > self.max_images:
            oldest = files.pop(0)
            try:
                os.remove(os.path.join(self.output_dir, oldest))
            except OSError:
                pass

    def _save_failed_capture(self, frame, mat_corners):
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            timestamp = int(time.time())
            annotated = frame.copy()
            if mat_corners is not None:
                cv2.polylines(annotated, [mat_corners.astype(int)], True, (255, 0, 0), 2)
            draw_label(annotated, "SCAN FAILED: parcel moved or lost", 20, 40, (0, 0, 255), 450)
            
            image_path = os.path.join(self.output_dir, f"failed_capture_{timestamp}.png")
            cv2.imwrite(image_path, annotated)
            self._prune_old_images()
            print(f"[CameraDimensioner] Saved failed capture to: {image_path}")
        except Exception as e:
            print(f"[CameraDimensioner] WARNING: Failed to save failed scan image: {e}")

    def _save_capture(self, frame, mat_corners, result: ParcelResult, box_points_original):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = int(time.time())

        annotated = frame.copy()
        cv2.polylines(annotated, [mat_corners.astype(int)], True, (255, 0, 0), 2)
        cv2.drawContours(annotated, [box_points_original], 0, (0, 255, 0), 3)

        height_display = f"{result.height_cm:.1f} cm" if result.height_cm is not None else "N/A"
        labels = (
            (f"L: {result.length_cm:.1f} cm ({result.object_type})", box_points_original[0]),
            (f"W: {result.width_cm:.1f} cm", box_points_original[1]),
            (f"H: {height_display} ({result.material_class} - {result.material_detail})",
             box_points_original[2]),
        )
        for text, pt in labels:
            draw_label(annotated, text, int(pt[0]), int(pt[1] - 15), (0, 255, 0))

        image_path = os.path.join(self.output_dir, f"measured_result_{timestamp}.png")
        cv2.imwrite(image_path, annotated)
        self._prune_old_images()

        result_dict = {
            "length_cm": round(result.length_cm, 1),
            "width_cm": round(result.width_cm, 1),
            "height_cm": round(result.height_cm, 1) if result.height_cm is not None else None,
            "object_type": result.object_type,
            "material_class": result.material_class,
            "material_detail": result.material_detail,
            "image_path": image_path,
        }
        print(f"[CameraDimensioner] Captured: {result_dict}")

        if self.on_capture:
            self.on_capture(result_dict)

    def _loop(self):
        cap, first_frame = open_camera()
        if cap is None:
            print("[CameraDimensioner] ERROR: could not open any camera.")
            return

        os.makedirs(self.output_dir, exist_ok=True)
        prev_gray = cv2.GaussianBlur(cv2.resize(cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY), (160, 120)), (9, 9), 0)
        state = ScanState.EMPTY
        still_start_time = 0.0

        print("[CameraDimensioner] Started (headless).")

        try:
            while self._running:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue

                motion_score, prev_gray = compute_motion_score(prev_gray, frame)
                result, box_points_original = None, None
                warped_mat, mat_corners, M = None, None, None

                if state == ScanState.EMPTY:
                    if motion_score > MOTION_THRESHOLD:
                        state = ScanState.MOVING
                        print("[CameraDimensioner] Motion detected. Parcel entering mat...", flush=True)
                        if self.on_motion:
                            self.on_motion()
                        if self.conveyor:
                            self.conveyor.stop()  # stop belt immediately so it can settle

                elif state == ScanState.MOVING:
                    if motion_score < MOTION_THRESHOLD:
                        state, still_start_time = ScanState.STILL, time.time()

                elif state == ScanState.STILL:
                    if motion_score > MOTION_THRESHOLD:
                        state = ScanState.MOVING
                    elif time.time() - still_start_time >= STILL_DURATION_SEC:
                        # Heavy image warping and contour measurement runs ONCE here!
                        warped_mat, mat_corners, M = detect_and_warp_mat(frame)
                        if warped_mat is not None:
                            result = measure_parcel_in_mat(warped_mat)
                            if result is not None:
                                try:
                                    pts_original = cv2.perspectiveTransform(
                                        result.box_points_warped.reshape(-1, 1, 2), np.linalg.inv(M))
                                    box_points_original = np.intp(pts_original.reshape(-1, 2))
                                except np.linalg.LinAlgError:
                                    result = None

                        if result is not None:
                            self._save_capture(frame, mat_corners, result, box_points_original)
                        else:
                            print("[CameraDimensioner] Scan failed: parcel moved or lost.")
                            self._save_failed_capture(frame, mat_corners)
                            if self.on_reset:
                                self.on_reset()
                        state = ScanState.WAIT_FOR_EXIT
                        if self.conveyor:
                            self.conveyor.start()  # measuring done (or failed) - move it along

                elif state == ScanState.WAIT_FOR_EXIT:
                    if motion_score > MOTION_THRESHOLD:
                        time.sleep(1.0)
                        state = ScanState.EMPTY
                        print("[CameraDimensioner] Parcel removed. Ready for next scan.")
                        if self.on_reset:
                            self.on_reset()

                time.sleep(0.01)  # small yield, no waitKey() anymore
        finally:
            cap.release()
            print("[CameraDimensioner] Camera released.")

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)


if __name__ == "__main__":
    print("=" * 60)
    print("       Standalone Camera Dimensioner Test Mode (Pi)")
    print("=" * 60)

    def test_callback(result):
        print("\n" + "=" * 40)
        print("    CAMERA CAPTURED PARCEL INDEPENDENTLY")
        print("=" * 40)
        for k, v in result.items():
            print(f"  {k}: {v}")
        print("=" * 40 + "\n")

    dim = CameraDimensioner(on_capture=test_callback)
    dim.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[+] Exiting Camera Standalone Test.")
    finally:
        dim.stop()
