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
print('starting imports...')
import cv2

# Suppress "drawFrameAxes" warnings
try:
    cv2.setLogLevel(2)
except AttributeError:
    pass

import time
import argparse
from ntcore import NetworkTableInstance
print('finished initial imports after ms ...')
start = time.time()
from vision.camera_model import CameraModel
from vision.detect_tags import TagDetector
from vision.detect_hsv import HSVDetector
from vision.visual_overlays import draw_overlays
from vision.tagmanager import TagManager
from vision.network import init_cam_entries, update_cam_entries
from vision.wpi_config import load_vision_cfg
from tests.debug_pose import MultiTagResidualLogger

print(f'finished custom imports after {(time.time()-start)*1000:.0f} ms... opening captures')


# Mock context object for passing data to the overlay drawer and NT
class MockContext:
    def __init__(self, name, colors, intrinsics, distortions, camera_type):
        self.name = name
        self.colors = colors
        self.intrinsics = intrinsics
        self.distortions = distortions
        self.camera_type = camera_type
        self.table_name = f"Cameras/{name}"
        self.nt = {}
        
        # Stats
        self.success_counter = 0
        self.fps = 0.0
        self.previous_time = time.time()
        self.previous_counts = 0

def main():
    parser = argparse.ArgumentParser(description="Local vision pipeline tester.")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (e.g., 0 for /dev/video0)")
    parser.add_argument("--ip", default="127.0.0.1", help="NetworkTables server IP (default: localhost)")
    parser.add_argument("--config", default="config/vision.json", help="Path to vision.json")
    parser.add_argument("--profile", default="local", help="Host profile to load from vision.json")
    args = parser.parse_args()

    # --- Initialization ---
    cap = cv2.VideoCapture(args.camera)
    
    # Set default resolution to 1280x720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        return

    # Get camera resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {width}x{height}")

    # --- Component Setup ---
    # Load configuration from vision.json
    vcfg = load_vision_cfg(args.config)
    profile = vcfg.get("hosts", {}).get(args.profile)
    if not profile:
        print(f"Warning: Profile '{args.profile}' not found in {args.config}. Using defaults.")
        cam_cfg = {}
    else:
        # Use the first camera in the profile
        cam_cfg = profile.get("cameras", [{}])[0]
        print(f"Loaded config for camera: {cam_cfg.get('name', 'unknown')}")

    # Resolve Hardware Definition
    cam_defs = vcfg.get("camera_definitions", {})
    cam_def = {}
    cid = cam_cfg.get("camera_id")
    if cid:
        cam_def = cam_defs.get(cid, {})

    labeling = cam_cfg.get("labeling", {})
    activities = cam_cfg.get("activities", {})
    cam_props = cam_cfg.get("camera_properties", {})
    tag_config_in = cam_cfg.get("tag_config", {})

    camera_type = cam_cfg.get("camera_type") or cam_def.get("camera_type", 'c920')
    intrinsics = cam_props.get("intrinsics") or cam_def.get("intrinsics", {'fx': 484, 'fy': 484, 'cx': width / 2, 'cy': height / 2})
    distortions = cam_props.get("distortions") or cam_def.get("distortions", [])
    colors_to_cycle = activities.get("colors", ["orange", "yellow", "purple", "green"])
    orientation = cam_props.get("orientation", {"tx": 0, "ty": 0, "tz": 0, "rx": 0, "ry": 0, "rz": 0})
    max_dist = tag_config_in.get("max_tag_distance", 3.0)

    cam_model = CameraModel(width, height, camera_type, intrinsics, distortions)
    tag_config = cam_cfg.get("tag_config", {}).copy()
    
    tag_detector = TagDetector(cam_model, config=tag_config)
    hsv_detector = HSVDetector(cam_model)
    tag_manager = TagManager(max_dt=0.5, max_averages=10, max_std=0.05)
    mt_log = MultiTagResidualLogger(min_period_s=5.0)

    # --- State for Interactive Loop ---
    color_idx = 0
    training_mode = False
    debug_mode = False
    averaging_mode = False
    train_box = [0.5, 0.5] # Normalized [x, y] center
    paused = False

    # Mock context for the overlay drawer
    mock_ctx = MockContext("LocalTest", colors_to_cycle, intrinsics, distortions, camera_type)

    # --- NetworkTables Setup ---
    ntinst = NetworkTableInstance.getDefault()
    ntinst.startClient4("LocalTester")
    ntinst.setServer(args.ip)
    init_cam_entries(ntinst, mock_ctx)

    print("\n--- Interactive Controls ---")
    print("  't' - Toggle Training Mode")
    print("  'd' - Toggle Debug Mode")
    print("  'v' - Toggle Tag Averaging")
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
        elif key == ord('v'):
            averaging_mode = not averaging_mode
            print(f"Tag Averaging: {'ON' if averaging_mode else 'OFF'}")
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
            
            # Update Stats
            mock_ctx.success_counter += 1
            if mock_ctx.success_counter % 10 == 0:
                now = time.time()
                dt = max(now - mock_ctx.previous_time, 1e-3)
                mock_ctx.fps = (mock_ctx.success_counter - mock_ctx.previous_counts) / dt
                mock_ctx.previous_counts, mock_ctx.previous_time = mock_ctx.success_counter, now
                
                if "frames" in mock_ctx.nt: mock_ctx.nt["frames"].set(mock_ctx.success_counter)
                if "fps" in mock_ctx.nt: mock_ctx.nt["fps"].set(mock_ctx.fps)

        if frame is None: continue

        # --- Detection ---
        tag_results = tag_detector.detect(frame, cam_orientation=orientation, max_distance=max_dist)
        tag_results = tag_manager.process(tag_results, averaging_enabled=averaging_mode)
        mt_log.dump(tag_results, label="Local")
        
        current_color = colors_to_cycle[color_idx]
        hsv_results = {
            current_color: hsv_detector.process(frame, current_color, training=training_mode, train_box=train_box)
        }

        # Hoist training stats for the overlay drawer
        if training_mode and hsv_results.get(current_color, {}).get('training_stats'):
            hsv_results['training_stats'] = hsv_results[current_color]['training_stats']

        # --- Visualization ---
        # We draw on a copy so the original 'paused' frame is not modified
        display_frame = frame.copy()
        draw_overlays(display_frame, tag_results, hsv_results, mock_ctx, training=training_mode, debug=debug_mode, train_box=train_box)
        cv2.imshow("Local Tester", display_frame)
        
        # --- NetworkTables Update ---
        update_cam_entries(mock_ctx, tag_results, hsv_results, ntinst)
        if "averaging_enabled" in mock_ctx.nt:
            mock_ctx.nt["averaging_enabled"].set(averaging_mode)


    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()