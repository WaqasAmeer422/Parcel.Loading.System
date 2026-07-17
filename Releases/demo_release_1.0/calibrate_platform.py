"""
Run this ONCE (or again if the camera/belt physically moves) to mark
the 4 corners of your 10x10cm platform area - the zone inside the
4 L-brackets. Saves the pixel coordinates to platform_calibration.json,
which camera_module.py then reuses on every frame instead of trying
to re-detect the platform from scratch each time.

Usage:
    python3 calibrate_platform.py

Controls:
    Left-click each of the 4 platform corners, in any order
    (r)eset points and start over
    (s)ave once you have 4 points placed correctly
    (q)uit without saving
"""

import json
import os
import platform
import cv2

CAMERA_INDEX = 1
CAMERA_FALLBACK_INDICES = (0, 2)
OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "platform_calibration.json")

points = []


def get_backend():
    system = platform.system()
    if system == "Windows":
        return cv2.CAP_DSHOW
    elif system == "Linux":
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append([x, y])
        print(f"[Calibrate] Point {len(points)}: ({x}, {y})")


def open_camera():
    for idx in (CAMERA_INDEX, *CAMERA_FALLBACK_INDICES):
        cap = cv2.VideoCapture(idx, get_backend())
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            ok, frame = cap.read()
            if ok:
                print(f"[Calibrate] Using camera index {idx}")
                return cap
            cap.release()
    return None


def main():
    cap = open_camera()
    if cap is None:
        print("[Calibrate] ERROR: could not open any camera.")
        return

    cv2.namedWindow("Calibrate Platform")
    cv2.setMouseCallback("Calibrate Platform", on_mouse)

    print("Click the 4 corners of the platform (inside the L-brackets), in any order.")
    print("Press 's' to save, 'r' to reset, 'q' to quit without saving.\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        display = frame.copy()
        for i, pt in enumerate(points):
            cv2.circle(display, tuple(pt), 6, (0, 255, 0), -1)
            cv2.putText(display, str(i + 1), (pt[0] + 10, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if len(points) == 4:
            pts = points + [points[0]]
            for i in range(4):
                cv2.line(display, tuple(pts[i]), tuple(pts[i + 1]), (0, 255, 255), 2)

        cv2.putText(display, f"Points: {len(points)}/4  (s=save, r=reset, q=quit)",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Calibrate Platform", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            points.clear()
            print("[Calibrate] Points reset.")
        elif key == ord('s'):
            if len(points) != 4:
                print("[Calibrate] Need exactly 4 points before saving.")
                continue
            with open(OUTPUT_FILE, "w") as f:
                json.dump(points, f)
            print(f"[Calibrate] Saved to {OUTPUT_FILE}")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
