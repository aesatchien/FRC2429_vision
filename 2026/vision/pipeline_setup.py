import time
import logging
import numpy as np
from cscore import CameraServer

from vision.camera_context import CameraContext
from vision.camera_model import CameraModel
from vision.detectors import TagDetector, HSVDetector
from vision.camera_controls import set_camera_robust_defaults
from vision.network import init_cam_entries
from vision.wpi_stream import build_stream, build_raw_stream
from vision import wpi_rio

log = logging.getLogger("setup")

def attach_sink(ctx):
    """Attaches a CvSink to the camera to allow frame grabbing."""
    ctx.sink = CameraServer.getVideo(camera=ctx.camera)
    ctx.img_buf = np.zeros((ctx.y_resolution or 480, ctx.x_resolution or 640, 3), dtype=np.uint8)

def deploy_camera_pipeline(cam_obj, cam_profile, rio_config, ntinst):
    """
    Factory function to initialize a complete camera pipeline context.
    Shared by main_single_processor.py and camera_node.py.
    """
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

    # 2. Build Context
    ctx = CameraContext(
        name=name,
        camera=cam_obj,
        x_resolution=width,
        y_resolution=height,
        camera_type= cam_profile.get("camera_type", 'c920'),
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
        intrinsics=cam_props.get("intrinsics"),
        distortions=cam_props.get("distortions"),
        use_distortions=cam_props.get("use_distortions", False),
        max_tag_distance=cam_props.get("max_tag_distance", 3.5),
    )

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
    # Inject distortion flag from camera properties
    tag_config["use_distortions"] = ctx.use_distortions
    ctx.tag_detector = TagDetector(cam_model, config=tag_config)
    ctx.hsv_detector = HSVDetector(cam_model)

    # 6. Apply Hardware Fixes
    set_camera_robust_defaults(ctx.camera, rio_config, ctx.camera_type, delay=2.5)

    log.info(f"Deployed {ctx.name}: Raw={ctx.raw_port} Proc={ctx.processed_port} Table={ctx.table_name}")
    return ctx