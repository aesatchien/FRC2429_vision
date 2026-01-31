import time
import logging
import numpy as np
from cscore import CameraServer

from vision.camera_context import CameraContext
from vision.camera_model import CameraModel
from vision.detect_tags import TagDetector
from vision.detect_hsv import HSVDetector
from vision.camera_controls import set_camera_robust_defaults
from vision.network import init_cam_entries
from vision.wpi_stream import build_stream, build_raw_stream
from vision import wpi_rio

log = logging.getLogger("setup")

def attach_sink(ctx):
    """Attaches a CvSink to the camera to allow frame grabbing."""
    ctx.sink = CameraServer.getVideo(camera=ctx.camera)
    ctx.img_buf = np.zeros((ctx.y_resolution or 480, ctx.x_resolution or 640, 3), dtype=np.uint8)

def get_scaled_intrinsics(cam_def, target_w, target_h):
    """
    Scales the canonical intrinsics from camera_definitions to match the
    actual runtime resolution of the camera.
    """
    intrinsics = cam_def.get("intrinsics")
    if not intrinsics: return None

    def_res = cam_def.get("resolution", [1280, 720]) # Default to 720p master
    dw, dh = def_res[0], def_res[1]

    # Case 1: Exact Match
    if dw == target_w and dh == target_h:
        return intrinsics

    # Case 2: Arducam 720p (Center Crop) -> 800p (Full Height)
    # We assume the 720p calibration is the center of the 800p frame.
    if dw == 1280 and dh == 720 and target_w == 1280 and target_h == 800:
        new_k = intrinsics.copy()
        new_k['cy'] += 40.0
        return new_k

    # Case 3: Scaling / Binning (e.g. 1280x720 -> 640x360)
    sx = target_w / dw
    sy = target_h / dh
    
    new_k = intrinsics.copy()
    new_k['fx'] *= sx
    new_k['fy'] *= sy
    new_k['cx'] *= sx
    new_k['cy'] *= sy
    return new_k

def deploy_camera_pipeline(cam_obj, cam_profile, rio_config, ntinst, camera_definitions=None):
    """
    Factory function to initialize a complete camera pipeline context.
    Shared by main_single_processor.py and camera_node.py.
    """
    if camera_definitions is None: camera_definitions = {}
    name = cam_profile["name"]
    
    # 1. Wait for connection to get actual resolution
    while not cam_obj.isConnected(): 
        time.sleep(0.1)
    
    vm = cam_obj.getVideoMode()
    width = vm.width if vm.width > 0 else 640
    height = vm.height if vm.height > 0 else 480

    # Extract sub-objects
    labeling = cam_profile.get("labeling", {})
    activities = cam_profile.get("activities", {})
    cam_props = cam_profile.get("camera_properties", {})
    tag_config_in = cam_profile.get("tag_config", {})

    # Resolve Hardware Definition via camera_id
    cam_def = {}
    cid = cam_profile.get("camera_id")
    if cid:
        cam_def = camera_definitions.get(cid, {})
        if not cam_def:
            log.warning(f"Camera ID '{cid}' not found in definitions!")

    # Calculate intrinsics based on actual resolution vs definition resolution
    base_intrinsics = get_scaled_intrinsics(cam_def, width, height)
    final_intrinsics = cam_props.get("intrinsics") or base_intrinsics

    # 2. Build Context
    ctx = CameraContext(
        name=name,
        camera=cam_obj,
        x_resolution=width,
        y_resolution=height,
        camera_type= cam_profile.get("camera_type") or cam_def.get("camera_type", 'c920'),
        raw_port=labeling.get("raw_port"),
        processed_port=labeling.get("processed_port", 1186),
        table_name=labeling.get("table_name", f"Cameras/{name}"),
        stream_fps=labeling.get("stream_fps", 16),
        stream_max_width=labeling.get("stream_max_width", 640),
        greyscale=bool(activities.get("greyscale", False)),
        find_tags=activities.get("find_tags", True),
        find_colors=activities.get("find_colors", False),
        colors=activities.get("colors", ["orange"]),
        orientation=cam_props.get("orientation", {"tx": 0, "ty": 0, "tz": 0, "rx": 0, "ry": 0, "rz": 0}),
        intrinsics=final_intrinsics,
        distortions=cam_props.get("distortions") or cam_def.get("distortions"),
        use_distortions=tag_config_in.get("undistort_image", False),
        max_tag_distance=tag_config_in.get("max_tag_distance", 3.5),
    )
    
    # Ensure we resize frames before sending to cscore to prevent stalls
    ctx.reduce_bandwidth = True

    # 3. Setup Streams
    if ctx.raw_port:
        # Use the stream config from the RIO object if available
        sc = rio_config.streamConfig if rio_config else None
        build_raw_stream(name, cam_obj, ctx.raw_port, sc)
    
    build_stream(ctx)
    attach_sink(ctx)
    
    # 4. Initialize NetworkTables
    init_cam_entries(ntinst, ctx)

    # 5. Initialize Detectors
    cam_model = CameraModel(
        ctx.x_resolution, ctx.y_resolution, ctx.camera_type,
        ctx.intrinsics, ctx.distortions
    )
    tag_config = cam_profile.get("tag_config", {}).copy()
    ctx.tag_detector = TagDetector(cam_model, config=tag_config)
    ctx.hsv_detector = HSVDetector(cam_model)

    # 6. Apply Hardware Fixes
    set_camera_robust_defaults(ctx.camera, rio_config, ctx.camera_type, delay=2.5)

    log.info(f"Deployed {ctx.name}: Raw={ctx.raw_port} Proc={ctx.processed_port} Table={ctx.table_name}")
    return ctx