
2026 0102 CJH

This file contains the configuration of ALL of the cameras we use.
They are organized by IP address, so as long as the hostname / IP is set properly on the pi, it will get the right camera settings.

WHO USES THIS FILE:
-------------------
1. main_multi_processor.py (Multi-process supervisor): Reads the "hosts" section to decide which camera processes to spawn based on the machine's IP.
2. main_single_processor.py: Reads the profile to configure the pipeline in single-process mode.
3. camera_node.py (Worker process): Reads specific camera settings (resolution, orientation, intrinsics) to configure the vision pipeline.
4. local_tester.py: Can load profiles for local development.

FIELD DEFINITIONS:
------------------
Top Level:
  "hosts": Dictionary of IP addresses (e.g., "10.24.29.12") mapping to a specific robot profile.
  "default": Fallback profile if the current IP is not found in "hosts".

Profile Fields:
  "role": Human-readable description of this robot/computer (e.g., "practicebot_pi4").
  "reserve_cores": (Optional) List of CPU cores [0, 1...] to reserve/exclude from automatic assignment in multi-process mode.
  "brightness_overrides": Dictionary mapping camera names or types to brightness values (e.g., {"c920": 40}).
  "cameras": List of camera configuration objects.

Camera Object Fields:
  "name": MUST match the camera name defined in /boot/frc.json.
  "camera_type": Hardware type ("c920", "arducam", "genius"). Triggers specific driver workarounds (e.g., C920 exposure fix).
  "enabled": Set to false to disable this camera without removing it from config.
  
  "labeling": {
    "raw_port": Port for the raw camera stream (optional, usually 1181+).
    "processed_port": Port for the vision-processed stream with overlays (usually 1186+).
    "stream_label": Name displayed in the dashboard stream widget.
    "table_name": NetworkTables path for publishing data (e.g., "Cameras/LogitechFront").
    "stream_fps": Target FPS for the MJPEG output stream (saves bandwidth).
    "stream_max_width": Max width for the MJPEG output stream (saves bandwidth).
  },

  "activities": {
    "greyscale": Convert frame to greyscale before processing (faster).
    "find_tags": Enable AprilTag detection.
    "find_colors": Enable HSV color blob detection.
    "colors": List of color names to detect (e.g., ["orange", "yellow"]).
  },

  "camera_properties": {
    "max_tag_distance": Ignore tags further than this distance (meters).
    "orientation": Camera position on robot. {tx, ty, tz} in meters, {rx, ry, rz} in degrees.
    "intrinsics": Camera lens properties {fx, fy, cx, cy} for accurate PnP pose estimation.
    "distortions": Lens distortion coefficients [k1, k2, p1, p2, k3].
    "use_distortions": Boolean to apply undistortion (computationally expensive).
  }
