# FRC 2429 Vision — 2026 Refactor

Small, testable modules; two run modes: **single‑process** (classic) and **multi‑process** (one process per camera).  
Now featuring a threaded pipeline (Acquisition, Tags, HSV, NT, Stream) for higher performance.

---
## Folder Structure
```
python_2026_refactor_2429/
├─ README.md                     # You are here
├─ bin/                          # Entrypoints you run
│  ├─ launcher.py                # Multi‑process supervisor (spawns camera_worker per cam)
│  ├─ multiCameraServer.py       # Single‑process (all cams in one process)
│  └─ camera_worker.py           # Per‑camera process (capture → overlay → NT → stream)
├─ vision/                    # Library modules (rarely change at events)
│  ├─ __init__.py
│  ├─ camctx.py                  # CamCtx dataclass: per‑camera state & options
│  ├─ config_io.py               # Load vision.json + select host profile by IP/hostname
│  ├─ rio.py                  # WPILib frc.json loader + startCamera()
│  ├─ ntio.py                    # NetworkTables topics (publish/subscribe wiring)
│  ├─ streaming.py               # CvSource/MjpegServer helpers + push_frame()
│  ├─ vision_worker.py           # Per‑frame loop: grab → process → NT updates → stream
│  ├─ spartan_overlay_2025.py    # AprilTag + color overlay processing
│  ├─ tagmanager.py              # Tag pose bookkeeping/utilities (used by overlay)
│  ├─ grip.py                    # (Legacy/optional) GRIP pipeline hooks
│  └─ util.py                    # Threads, SIGINT stop, small camera helpers
├─ config/
│  └─ vision.json                # Host profiles + per‑camera runtime settings
├─ files/                        # Static assets (calibration, samples, etc.)
└─ logs/                         # Runtime logs (created by launcher)
```

---
## What Each Piece Does (short)
- **bin/launcher.py** — Reads `config/vision.json`, validates names vs `/boot/frc.json`, spawns one `camera_worker.py` per camera, prints one live line with `FPS S F` per cam, writes logs to `./logs/`.
- **bin/multiCameraServer.py** — Classic single‑process runtime; starts all cameras and runs worker threads in one process.
- **bin/camera_worker.py** — One camera end‑to‑end (capture → overlay → NT → MJPEG). Prints `name: XX.Xfps  S:#  F:#` once per second.
- **vision/** — Implementation modules: context, IO, NT topics, streaming, frame tick, overlay, tag utilities, legacy GRIP, and helpers.
- **config/vision.json** — Host‑specific runtime (which cameras to run, processed ports, stream FPS/size, orientations, etc.).
- **/boot/frc.json** — WPILib camera bring‑up (device path, format, width/height, exposure, etc.). Names must match `vision.json`.

---
## Run Modes
### Single‑process
```
python3 bin/multiCameraServer.py [optional /path/to/frc.json]
```

### Multi‑process (recommended on Pi)
```
python3 bin/launcher.py --autorestart --vision config/vision.json [--frc /boot/frc.json]
```
- Shows one in‑place console line: `camA:40.0fps S:1203 F:7  camB:39.9fps S:1188 F:6`.
- Logs: `./logs/camera-<name>.log` (workers’ stdout/stderr).
- Optional CPU pinning via `vision.json`: set `reserve_cores` at host level and per‑camera `cpu`.

---
## Notes
- Launcher prefers NT `_fps` if available; otherwise falls back to parsing each worker’s stdout line.
- If streams are up but FPS shows `0.0`, tail `./logs/camera-<name>.log` to confirm the worker prints the stats line.
- Keep AprilTag detection full‑rate; adjust stream bandwidth with `stream_fps` / `stream_max_width` in `vision.json` if needed.
