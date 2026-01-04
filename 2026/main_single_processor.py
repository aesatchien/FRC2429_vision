#!/usr/bin/env python3
import sys, time, logging, argparse
from ntcore import NetworkTableInstance

from vision.wpi_config import load_vision_cfg, select_profile
from vision.camera_context import CameraContext
from vision.wpi_stream import build_stream, push_frame, build_raw_stream
from vision.wpi_attach_sink import attach_sink
from vision.network import init_global_flags, init_cam_entries
from vision.camera_controls import set_camera_robust_defaults

from vision import wpi_rio
from vision.threaded_pipeline import ThreadedVisionPipeline
from vision.camera_model import CameraModel
from vision.detectors import TagDetector, HSVDetector

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("vision")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="10.24.29.2", help="NetworkTables server IP")
    parser.add_argument("--frc", default="/boot/frc.json", help="Path to frc.json")
    parser.add_argument("--vision", default="/boot/vision.json", help="Path to vision.json")
    parser.add_argument("--autorestart", action="store_true", help="Ignored; for compatibility with launcher")
    args = parser.parse_args()

    wpi_rio.configFile = args.frc
    if not wpi_rio.readConfig():
        log.error(f"could not read {wpi_rio.configFile}")
        sys.exit(1)

    # Start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    log.info(f"Starting NT client connecting to {args.ip}")
    ntinst.startClient4("CJHpi5")
    ntinst.setServer(args.ip)
    ntinst.startDSClient()

    # Start cameras from frc.json (local list) and map by name
    cams = [wpi_rio.startCamera(cfg) for cfg in wpi_rio.cameraConfigs]
    name_to_cam = {cc.name: cam for cc, cam in zip(wpi_rio.cameraConfigs, cams)}
    log.info(f"FRC cameras: {list(name_to_cam.keys())}")

    # Select per-host vision profile
    vcfg = load_vision_cfg(args.vision)
    prof = select_profile(vcfg)
    
    cam_count = len(prof.get("cameras", []))
    log.info(f"Selected profile: '{prof.get('role', 'unknown')}' with {cam_count} cameras")
    if cam_count == 0:
        log.warning("No cameras found in profile! (Check IP address or vision.json)")

    # Global NT flags
    nt_global = init_global_flags(ntinst)

    # Build CamCtx list from profile
    contexts = []
    for idx, c in enumerate(prof.get("cameras", [])):
        cam_obj = name_to_cam.get(c["name"])
        if cam_obj is None:
            log.warning(f"camera '{c['name']}' not found; skipping")
            continue

        ctx = CameraContext(
            name=c["name"],
            camera=cam_obj,
            camera_type= c.get("camera_type", 'c920'),
            processed_port=c.get("processed_port", 1186 + idx),
            table_name=c.get("table_name", f"Cameras/{c['name']}"),
            stream_fps=c.get("stream_fps", 16),
            stream_max_width=c.get("stream_max_width", 640),
            greyscale=bool(c.get("greyscale", False)),
            find_tags=c.get("find_tags", True),
            find_colors=c.get("find_colors", False),
            colors=c.get("colors", ["orange"]),
            orientation=c.get("orientation", {"tx": 0, "ty": 0, "tz": 0, "rx": 0, "ry": 0, "rz": 0}),
            intrinsics=c.get("intrinsics"),
            distortions=c.get("distortions"),
            use_distortions=c.get("use_distortions", False),
            max_tag_distance=c.get("max_tag_distance", 3.5),
        )

        # Stream + sink + NT + pipeline
        if c.get("raw_port"):
            # camera object is `cam_obj`; profile entry is `c`; cfg is the matching rio camera config
            cfg = next(cc for cc in wpi_rio.cameraConfigs if cc.name == c["name"])
            build_raw_stream(c["name"], cam_obj, c["raw_port"], cfg.streamConfig)
        build_stream(ctx)
        attach_sink(ctx)
        init_cam_entries(ntinst, ctx)
        
        cam_model = CameraModel(
            ctx.x_resolution, ctx.y_resolution, ctx.camera_type,
            ctx.intrinsics, ctx.distortions
        )
        ctx.tag_detector = TagDetector(cam_model)
        ctx.hsv_detector = HSVDetector(cam_model)

        # Apply robust defaults (brightness nudge + v4l2 exposure)
        cfg = next((cc for cc in wpi_rio.cameraConfigs if cc.name == ctx.name), None)
        set_camera_robust_defaults(ctx.camera, cfg, ctx.camera_type)

        contexts.append(ctx)
        print(f"Added {ctx.name} stream on {ctx.processed_port} and serving table {ctx.table_name}")

    # Run workers
    for ctx in contexts:
        log.info(f"Starting threaded pipeline for {ctx.name}")
        pipeline = ThreadedVisionPipeline(ctx, ntinst, nt_global, push_frame)
        pipeline.start()

    # Tiny stats loop
    while True:
        time.sleep(1.0)
        msg = " ".join(
            [f"{ctx.name}:{ctx.fps:0.1f}fps S:{ctx.success_counter} F:{ctx.failure_counter}" for ctx in contexts]
        )
        print(msg, end="\r", flush=True)
