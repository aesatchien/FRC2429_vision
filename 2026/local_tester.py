#!/usr/bin/env python3
"""
local_tester.py - Standalone Test Harness for Vision Pipelines

Purpose:
    This script provides a lightweight, interactive environment for testing and tuning
    the vision detection algorithms (AprilTag and HSV) on a local machine using a
    standard webcam, without needing to deploy to a robot.

How to Use:
    1. Run from the project's root directory:
       python local_tester.py

    2. To use a different webcam, specify its index:
       python local_tester.py --camera 1

    3. For accurate distance/pose results, it is CRITICAL to update the
       'intrinsics' and 'distortions' variables in the "Component Setup"
       section below. Copy these values from your target camera's profile
       in `config/vision.json`.

Interactive Controls (in the OpenCV window):
    't' - Toggle Training Mode
    'd' - Toggle Debug Mode
    'c' - Cycle Target Color
    'w'/'a'/'s'/'e' - Move Training Box
    'space' - Pause/Resume Video
    'n' - Next Frame (when paused)
    'q' - Quit
"""
import cv2
import time
import argparse

from vision.camera_model import CameraModel
from vision.detectors import TagDetector, HSVDetector
from vision.visual_overlays import draw_overlays

# Mock context object for passing data to the overlay drawer
class MockContext:
    def __init__(self, name, colors, intrinsics, distortions, camera_type):
        self.name = name
        self.colors = colors
        self.intrinsics = intrinsics
        self.distortions = distortions
        self.camera_type = camera_type

def main():
    parser = argparse.ArgumentParser(description="Local vision pipeline tester.")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (e.g., 0 for /dev/video0)")
    args = parser.parse_args()

    # --- Initialization ---
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        return

    # Get camera resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {width}x{height}")

    # --- Component Setup ---
    # IMPORTANT: For accurate results, copy these values from your config/vision.json
    camera_type = 'c920'
    intrinsics = {'fx': 484, 'fy': 484, 'cx': width / 2, 'cy': height / 2}
    distortions = [0.0, 0.0, 0.0, 0.0, 0.0]

    cam_model = CameraModel(width, height, camera_type, intrinsics, distortions)
    tag_detector = TagDetector(cam_model)
    hsv_detector = HSVDetector(cam_model)

    # --- State for Interactive Loop ---
    colors_to_cycle = ["orange", "yellow", "purple", "green"]
    color_idx = 0
    training_mode = False
    debug_mode = False
    find_tags = True
    find_colors = True
    train_box = [0.5, 0.5] # Normalized [x, y] center
    paused = False

    # Mock context for the overlay drawer
    mock_ctx = MockContext("LocalTest", colors_to_cycle, intrinsics, distortions, camera_type)

    print("\n--- Interactive Controls ---")
    print("  't' - Toggle Training Mode")
    print("  'd' - Toggle Debug Mode")
    print("  'w/a/s/e' - Move Training Box")
    print("  'c' - Cycle Target Color")
    print("  'space' - Pause/Resume")
    print("  'n' - Next Frame (when paused)")
    print("  'q' - Quit")
    print("--------------------------\n")

    # --- Main Loop ---
    frame = None
    while True:
        # --- Keystroke Handling ---
        # We handle keystrokes first to decide if we need to grab a new frame
        should_grab_frame = not paused
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('t'):
            training_mode = not training_mode
            print(f"Training Mode: {'ON' if training_mode else 'OFF'}")
        elif key == ord('d'):
            debug_mode = not debug_mode
            print(f"Debug Mode: {'ON' if debug_mode else 'OFF'}")
        elif key == ord('c'):
            color_idx = (color_idx + 1) % len(colors_to_cycle)
            print(f"Detecting Color: {colors_to_cycle[color_idx]}")
        elif key == ord('w'): train_box[1] = max(0.0, train_box[1] - 0.02)
        elif key == ord('s'): train_box[1] = min(1.0, train_box[1] + 0.02)
        elif key == ord('a'): train_box[0] = max(0.0, train_box[0] - 0.02)
        elif key == ord('e'): train_box[0] = min(1.0, train_box[0] + 0.02)
        elif key == ord(' '): # spacebar
            paused = not paused
            print(f"Paused: {paused}")
        elif key == ord('n') and paused:
            should_grab_frame = True

        # --- Frame Acquisition ---
        if should_grab_frame or frame is None:
            ret, new_frame = cap.read()
            if not ret:
                print("Error: Failed to grab frame")
                break
            frame = new_frame

        if frame is None: continue

        # --- Detection ---
        tag_results = tag_detector.detect(frame) if find_tags else {}
        
        current_color = colors_to_cycle[color_idx]
        hsv_results = {
            current_color: hsv_detector.process(frame, current_color, training=training_mode, train_box=train_box)
        } if find_colors else {}

        # Hoist training stats for the overlay drawer
        if training_mode and hsv_results.get(current_color, {}).get('training_stats'):
            hsv_results['training_stats'] = hsv_results[current_color]['training_stats']

        # --- Visualization ---
        # We draw on a copy so the original 'paused' frame is not modified
        display_frame = frame.copy()
        draw_overlays(display_frame, tag_results, hsv_results, mock_ctx, training=training_mode, debug=debug_mode, train_box=train_box)
        cv2.imshow("Local Tester", display_frame)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()