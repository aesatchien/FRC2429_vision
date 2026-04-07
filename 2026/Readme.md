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
│  ├─ detect_hsv.py              # HSV detection logic
│  ├─ detect_tags.py             # AprilTag detection logic
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
## Automatic Restart and Reboot on Camera Failure

If a camera stops delivering frames, the acquisition thread detects it after `MAX_CONSECUTIVE_ACQUISITION_FAILURES` consecutive failures (~5 seconds) and signals a fatal error. The process then exits with code `2`.

- **Multi-process mode** (`main_multi_processor.py --autorestart`): the supervisor sees `rc=2`, increments a per-camera restart counter, and relaunches the worker. After **2 consecutive fatal restarts** it calls `sudo reboot`.
- **Single-process mode** (`main_single_processor.py`): the restart count is stored in `/tmp/vision_fatal_count` so it survives process restarts. After **2 consecutive fatal restarts** it calls `sudo reboot`. The counter resets automatically after 60 seconds of healthy operation.

### Why the consecutive-failure counter alone is insufficient

The `MAX_CONSECUTIVE_ACQUISITION_FAILURES` counter works well the first time a camera dies mid-run: `grabFrame` returns `ts=0` rapidly, the counter reaches 200, and the process exits cleanly with `rc=2`.

After the **first restart**, however, WPILib's cscore camera manager enters a persistent reconnection loop that defeats this mechanism in two ways:

1. **`grabFrame` blocks indefinitely.** While cscore is actively attempting to re-open the USB device it spams connection-attempt messages and may never return from `grabFrame`, so the acquisition thread never reaches the failure-counting code at all.
2. **Reconnection flaps reset the counter.** Even if `grabFrame` does return, cscore occasionally delivers a frame (`ts > 0`) during a brief reconnection window before the camera drops again. Each such frame resets `_consecutive_failures` to zero, preventing the counter from ever reaching the threshold.

In both cases the process stays alive indefinitely — streaming nothing — and the second restart (which would trigger the reboot) never happens.

### Two-layer watchdog (current fix)

**Layer 1 — internal watchdog thread** (`threaded_pipeline.py`):  
A dedicated `Watch` daemon thread stamps `_last_good_frame_time` on every successful `grabFrame`. It then polls independently every 5 seconds. If no good frame has arrived within `DEAD_CAMERA_WATCHDOG_S` (20 s) it calls `self.fatal.set()` directly — bypassing `grabFrame` entirely so it cannot be blocked or fooled by reconnect flaps.

**Layer 2 — supervisor FPS watchdog** (`main_multi_processor.py`, multi-process mode only):  
The supervisor already reads each worker's output FPS every 2 seconds. If a worker process is still running but has reported zero FPS for `SUPERVISOR_FPS_TIMEOUT_S` (30 s), the supervisor force-kills it (`supervisor_killed = True`). That flag is treated the same as `rc=2` when computing the restart counter, so the reboot path fires after two such events.

Layer 2 is a belt-and-suspenders backstop for any scenario where the worker process is alive but the watchdog thread itself cannot fire. Single-process mode does not need it because the `Watch` thread and the main health loop are in the same process.

### Pi setup — passwordless sudo for reboot

The vision process does not need to run as root. Grant only the reboot permission by adding one line via `sudo visudo`:

```
pi ALL=(ALL) NOPASSWD: /sbin/reboot
```

Replace `pi` with the user that runs the vision process. If vision runs as a **systemd service**, you can use `systemctl reboot` instead (no sudoers change needed in most configurations) — change the `os.system(...)` call in the relevant `main_*.py` accordingly.

---
## Notes
- Launcher prefers NT `_fps` if available; otherwise falls back to parsing each worker’s stdout line.
- If streams are up but FPS shows `0.0`, tail `./logs/camera-<name>.log` to confirm the worker prints the stats line.
- Keep AprilTag detection full‑rate; adjust stream bandwidth with `stream_fps` / `stream_max_width` in `vision.json` if needed.
