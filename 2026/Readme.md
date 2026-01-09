# FRC 2429 Vision — 2026 Refactor

Small, testable modules; two run modes: **single‑process** (classic) and **multi‑process** (one process per camera).  
Now featuring a threaded pipeline (Acquisition, Tags, HSV, NT, Stream) for higher performance.

---
## Folder Structure

2026/
├─ README.md                     # You are here
├─ main_multi_processor.py       # Multi-process supervisor (formerly launcher.py)
├─ main_single_processor.py      # Single-process runtime (formerly multiCameraServer.py)
├─ local_tester.py               # Local development test harness with GUI
├─ config/
│  ├─ vision.json                # Host profiles + per-camera runtime settings
│  └─ readme_vision.txt          # Documentation for vision.json fields
├─ vision/                       # Library modules
│  ├─ camera_context.py          # Data class for camera state
│  ├─ camera_controls.py         # Hardware controls (exposure/brightness workarounds)
│  ├─ camera_model.py            # Pinhole camera model logic
│  ├─ camera_node.py             # Worker process entry point (runs one camera)
│  ├─ detectors.py               # AprilTag and HSV detection logic
│  ├─ hsv_config.py              # Color threshold configurations
│  ├─ network.py                 # NetworkTables publishers/subscribers
│  ├─ pipeline_setup.py          # Factory for creating camera contexts (DRY)
│  ├─ tagmanager.py              # Temporal smoothing for tags
│  ├─ threaded_pipeline.py       # 5-stage threaded pipeline engine
│  ├─ visual_overlays.py         # Drawing logic for streams
│  ├─ wpi_config.py              # Config loading and profile selection
│  ├─ wpi_rio.py                 # WPILib/cscore hardware interaction
│  └─ wpi_stream.py              # MJPEG streaming logic
├─ setup_files/                  # Scripts and configs for Pi setup
│  ├─ runCamera                  # Startup script
│  ├─ runCamera.service          # Systemd service file
│  └─ ...
├─ tests/                        # Diagnostic scripts
│  ├─ debug_apriltag.py
│  ├─ minimal_streamer.py
│  └─ ...
└─ logs/                         # Runtime logs

---
## What Each Piece Does (short)
- **main_multi_processor.py** — Reads `config/vision.json`, validates names vs `/boot/frc.json`, spawns one `vision.camera_node` process per camera, prints one live line with `FPS S F` per cam, writes logs to `./logs/`.
- **main_single_processor.py** — Classic single‑process runtime; starts all cameras and runs threaded pipelines in one process.
- **local_tester.py** — Runs a pipeline on your local webcam with a GUI window. Allows tuning HSV thresholds and testing tag detection without a robot.
- **vision/threaded_pipeline.py** — The core engine. Runs 5 threads per camera: Acquisition, Tag Detect, HSV Detect, NT Update, and Stream/Overlay.
- **config/vision.json** — Host‑specific runtime (which cameras to run, processed ports, stream FPS/size, orientations, etc.).
- **/boot/frc.json** — WPILib camera bring‑up (device path, format, width/height, exposure, etc.). Names must match `vision.json`.

---
## Run Modes
### Single‑process
```
python3 main_single_processor.py [optional /path/to/frc.json]
```

### Multi‑process (recommended on Pi)
```
python3 main_multi_processor.py --autorestart --vision config/vision.json [--frc /boot/frc.json]
```
- Shows one in‑place console line: `camA:40.0fps S:1203 F:7  camB:39.9fps S:1188 F:6`.
- Logs: `./logs/camera-<name>.log` (workers’ stdout/stderr).
- Optional CPU pinning via `vision.json`: set `reserve_cores` at host level and per‑camera `cpu`.

---
## Notes
- Launcher prefers NT `_fps` if available; otherwise falls back to parsing each worker’s stdout line.
- If streams are up but FPS shows `0.0`, tail `./logs/camera-<name>.log` to confirm the worker prints the stats line.
- Keep AprilTag detection full‑rate; adjust stream bandwidth with `stream_fps` / `stream_max_width` in `vision.json` if needed.
