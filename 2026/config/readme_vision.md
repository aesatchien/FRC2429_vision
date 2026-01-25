# Vision Configuration (`vision.json`)

This file controls the runtime behavior of the vision system. It allows you to configure cameras, pipelines, and network settings without modifying Python code.

The system selects a profile based on the **IP Address** or **Hostname** of the device running the code.

---

## Top Level Structure

```json
{
  "hosts": {
    "10.24.29.12": { ... profile ... },
    "raspberrypi": { ... profile ... }
  },
  "default": { ... fallback profile ... }
}
```

---

## Host Profile

A profile defines the set of cameras active on a specific coprocessor.

| Field | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `role` | String | "unknown" | Human-readable name for this setup (e.g., "practicebot_pi4"). |
| `reserve_cores` | List[int] | `[0]` | CPU cores to exclude from automatic assignment in multi-process mode. |
| `brightness_overrides` | Dict | `{}` | Map of camera names (or types) to brightness values (e.g., `{"c920": 40}`). |
| `cameras` | List[Obj] | `[]` | List of camera configuration objects (see below). |

---

## Camera Configuration

Each object in the `cameras` list corresponds to a physical camera defined in `/boot/frc.json`.

### 1. Identity
| Field | Default | Description |
| :--- | :--- | :--- |
| `name` | **Required** | Must match the `name` in `/boot/frc.json`. |
| `camera_type` | "c920" | Hardware type (`c920`, `arducam`, `genius`). Triggers specific driver workarounds. |
| `serial` | "" | Documentation only; useful for tracking physical devices. |

### 2. Labeling (Network & Stream)
Controls how the camera is exposed to the network.

| Field | Default | Description |
| :--- | :--- | :--- |
| `raw_port` | `None` | Port for the raw (unprocessed) camera stream (e.g., 1181). |
| `processed_port` | `1186` | Port for the vision-processed stream with overlays. |
| `stream_label` | Name | Name displayed in the dashboard stream widget. |
| `table_name` | `Cameras/<Name>` | NetworkTables path for publishing data. |
| `stream_fps` | `16` | Target FPS for the MJPEG output stream (saves bandwidth). |
| `stream_max_width` | `640` | Max width for the MJPEG output stream (saves bandwidth). |

### 3. Activities (Pipeline Features)
Controls which detection algorithms run.

| Field | Default | Description |
| :--- | :--- | :--- |
| `greyscale` | `False` | Convert frame to greyscale before processing (faster, but disables color detection). |
| `find_tags` | `True` | Enable AprilTag detection. |
| `find_colors` | `False` | Enable HSV color blob detection. |
| `colors` | `["orange"]` | List of color profiles to detect (defined in `vision/hsv_config.py`). |

### 4. Camera Properties (Physical & Calibration)
Critical for accurate 3D pose estimation.

| Field | Default | Description |
| :--- | :--- | :--- |
| `orientation` | `{0,0,0...}` | Camera position on robot. `{tx, ty, tz}` (meters), `{rx, ry, rz}` (degrees). |
| `intrinsics` | `None` | Lens properties `{fx, fy, cx, cy}`. If missing, estimated from resolution. |
| `distortions` | `None` | Lens distortion coefficients `[k1, k2, p1, p2, k3]`. |

### 5. Tag Config (Detector Tuning)
Fine-tune the AprilTag algorithm performance vs. accuracy.

| Field | Default | Description |
| :--- | :--- | :--- |
| `max_tag_distance` | `3.5` | Ignore tags detected further than this distance (meters). |
| `decimate` | `1.0` | Downsample factor. `1.0` = Full Res (Best Range), `2.0` = Half Res (Faster). |
| `sigma` | `0.6` | Gaussian blur sigma. `0.0` = Sharp/Noisy, `0.6-0.8` = Smooths sensor noise. |
| `threads` | `1` | Number of threads for the detector core. |
| `min_cluster_pixels` | `25` | Reject blobs smaller than this (removes speckle noise). |
| `decision_margin` | `35` | Minimum confidence margin to accept a tag. |
| `hamming` | `1` | Maximum corrected bit errors allowed (0 or 1). |
| `refine_edges` | `True` | Enable sub-pixel edge refinement (slower, more accurate). |
| `decode_sharpening` | `0.25` | Sharpening applied to decoded quads to help with soft focus. |
| `allow_multi_tag` | `True` | If `True`, computes a global Multi-Tag PnP pose when 2+ tags are visible. |
| `undistort_image` | `False` | If `True`, undistorts image before tag detection (CPU intensive). |

---

## Example Snippet

```json
{
  "name": "arducam",
  "camera_type": "arducam",
  "labeling": {
    "stream_max_width": 320,
    "stream_fps": 15
  },
  "tag_config": {
    "decimate": 2.0,
    "threads": 4
  }
}
```