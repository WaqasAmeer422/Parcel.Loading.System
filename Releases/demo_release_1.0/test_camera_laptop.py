import time
import cv2
import numpy as np
import sys
import os

# Add local path to import camera_module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import camera_module as cam


def main():
    print("============================================================")
    camera_idx = 1
    print(f"[Laptop Test] Opening webcam at index {camera_idx}...")
    cap = cv2.VideoCapture(camera_idx, cam.get_backend())
    if not cap.isOpened():
        print(f"[ERROR] Could not open webcam at index {camera_idx}. Trying fallback index 0...")
        cap = cv2.VideoCapture(0, cam.get_backend())
        if not cap.isOpened():
            print("[ERROR] No webcam detected. Please plug in your USB camera.")
            return

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    time.sleep(1.0)
    ok, frame = cap.read()
    if not ok or frame is None:
        print("[ERROR] Camera connected but failed to read frames.")
        cap.release()
        return

    prev_gray = cv2.GaussianBlur(cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (160, 120)), (9, 9), 0)
    state = cam.ScanState.EMPTY
    still_start_time = 0.0

    print("\n============================================================")
    print("      CONVEYOR CAMERA LAPTOP TESTING MODE STARTED           ")
    print("============================================================\n")
    print("Controls: Press 'q' in the video window to quit.\n")

    frame_count = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[ERROR] Frame acquisition failed.")
            break

        motion_score, prev_gray = cam.compute_motion_score(prev_gray, frame)
        
        # Print live motion debug score every 15 frames (approx 0.5s) to the console
        frame_count += 1
        if frame_count % 15 == 0:
            print(f"[Debug Console] State: {state.name} | Motion Score: {motion_score} (Threshold: {cam.MOTION_THRESHOLD})", flush=True)
            
        warped_mat, platform_corners, M = cam.detect_and_warp_mat(frame)
        result = None
        box_points_original = None

        if warped_mat is not None:
            result = cam.measure_parcel_in_mat(warped_mat)
            if result is not None:
                try:
                    pts_original = cv2.perspectiveTransform(
                        result.box_points_warped.reshape(-1, 1, 2), np.linalg.inv(M))
                    box_points_original = np.intp(pts_original.reshape(-1, 2))
                except np.linalg.LinAlgError:
                    result = None

        display_frame = frame.copy()

        if platform_corners is not None:
            cv2.polylines(display_frame, [platform_corners.astype(int)], True, (255, 0, 0), 2)
            cv2.putText(display_frame, "Platform Area", (int(platform_corners[0][0]), int(platform_corners[0][1] - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        if result is not None and box_points_original is not None:
            height_display = f"{result.height_cm:.1f} cm" if result.height_cm is not None else "N/A"
            cv2.drawContours(display_frame, [box_points_original], 0, (0, 255, 0), 2)
            cam.draw_label(display_frame, f"L: {result.length_cm:.1f} cm", int(box_points_original[0][0]), int(box_points_original[0][1] - 15), (0, 255, 0), 150)
            cam.draw_label(display_frame, f"W: {result.width_cm:.1f} cm", int(box_points_original[1][0]), int(box_points_original[1][1] - 15), (0, 255, 0), 150)
            cam.draw_label(display_frame, f"H: {height_display}", int(box_points_original[2][0]), int(box_points_original[2][1] - 15), (0, 255, 0), 150)

        status_color = (0, 255, 255)
        if state == cam.ScanState.EMPTY:
            status_text = "STATE: EMPTY (Waiting for parcel)"
            status_color = (0, 255, 0)
        elif state == cam.ScanState.MOVING:
            status_text = "STATE: MOVING (Parcel entering/aligning)"
            status_color = (0, 165, 255)
        elif state == cam.ScanState.STILL:
            status_text = f"STATE: STILL (Settling: {time.time() - still_start_time:.1f}s)"
            status_color = (0, 0, 255)
        else:
            status_text = "STATE: WAIT FOR REMOVAL"
            status_color = (255, 0, 255)

        cv2.putText(display_frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(display_frame, f"Motion Score: {motion_score} (Threshold: {cam.MOTION_THRESHOLD})",
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        if warped_mat is not None:
            if state == cam.ScanState.EMPTY:
                if motion_score > cam.MOTION_THRESHOLD:
                    state = cam.ScanState.MOVING
                    print("[State Machine] Motion detected. Scan sequence started...")

            elif state == cam.ScanState.MOVING:
                if motion_score < cam.MOTION_THRESHOLD:
                    state, still_start_time = cam.ScanState.STILL, time.time()

            elif state == cam.ScanState.STILL:
                if motion_score > cam.MOTION_THRESHOLD:
                    state = cam.ScanState.MOVING
                elif time.time() - still_start_time >= cam.STILL_DURATION_SEC:
                    if result is not None:
                        height_str = f"{result.height_cm:.1f} cm" if result.height_cm is not None else "N/A (low confidence - see note)"
                        print("\n==========================================")
                        print("          PARCEL CAPTURE SUCCESS          ")
                        print("==========================================")
                        print(f"  Length: {result.length_cm:.1f} cm")
                        print(f"  Width:  {result.width_cm:.1f} cm")
                        print(f"  Height: {height_str}")
                        print(f"  Type:   {result.object_type}")
                        print(f"  Detail: {result.material_class} ({result.material_detail})")
                        print("==========================================\n")

                        try:
                            output_dir = r"D:\img"
                            os.makedirs(output_dir, exist_ok=True)
                            timestamp = int(time.time())
                            out_path = os.path.join(output_dir, f"measured_result_{timestamp}.png")
                            cv2.imwrite(out_path, display_frame)
                            print(f"[Laptop Test] Saved annotated image to: {out_path}\n")
                        except Exception as e:
                            print(f"[Laptop Test] WARNING: Failed to save image: {e}\n")
                    else:
                        print("[State Machine] Capture failed: No valid parcel found in the center.")
                        try:
                            output_dir = r"D:\img"
                            os.makedirs(output_dir, exist_ok=True)
                            timestamp = int(time.time())
                            out_path = os.path.join(output_dir, f"failed_capture_{timestamp}.png")
                            cv2.imwrite(out_path, display_frame)
                            print(f"[Laptop Test] Saved failed capture image to: {out_path}\n")
                        except Exception as e:
                            print(f"[Laptop Test] WARNING: Failed to save image: {e}\n")
                    state = cam.ScanState.WAIT_FOR_EXIT

            elif state == cam.ScanState.WAIT_FOR_EXIT:
                if motion_score > cam.MOTION_THRESHOLD:
                    time.sleep(1.0)
                    state = cam.ScanState.EMPTY
                    print("[State Machine] Mat cleared. Ready for next parcel.")

        cv2.imshow("Laptop Conveyor Scan Test (Press Q to quit)", display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[Laptop Test] Closed. Exiting.")


if __name__ == "__main__":
    main()