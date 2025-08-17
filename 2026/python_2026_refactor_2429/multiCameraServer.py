#!/usr/bin/env python3
import sys, time, logging
from ntcore import NetworkTableInstance
from spartan_overlay_2025 import SpartanOverlay

from visionlib.config_io import load_vision_cfg, select_profile
from visionlib.camctx import CamCtx
from visionlib.streaming import build_stream, push_frame, build_raw_stream
from visionlib.vision_worker import attach_sink, tick
from visionlib.ntio import init_global_flags, init_cam_entries
from visionlib.util import run_in_thread, nudge_brightness

from visionlib import frc_io

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("vision")

if __name__ == "__main__":
    # --- Read /boot/frc.json (or argv override) ---
    if len(sys.argv) >= 2:
        frc_io.configFile = sys.argv[1]
    if not frc_io.readConfig():
        log.error(f"could not read {frc_io.configFile}")
        sys.exit(1)

    # Start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    log.info("Starting NT client for team 2429")
    ntinst.startClient4("CJHpi5")
    ntinst.setServerTeam(2429)
    ntinst.startDSClient()

    # Start cameras from frc.json (local list) and map by name
    cams = [frc_io.startCamera(cfg) for cfg in frc_io.cameraConfigs]
    name_to_cam = {cc.name: cam for cc, cam in zip(frc_io.cameraConfigs, cams)}
    log.info(f"FRC cameras: {list(name_to_cam.keys())}")

    # Select per-host vision profile
    vcfg = load_vision_cfg()
    prof = select_profile(vcfg)

    # Global NT flags
    nt_global = init_global_flags(ntinst)

    # Build CamCtx list from profile
    contexts = []
    for idx, c in enumerate(prof.get("cameras", [])):
        cam_obj = name_to_cam.get(c["name"])
        if cam_obj is None:
            log.warning(f"camera '{c['name']}' not found; skipping")
            continue

        ctx = CamCtx(
            name=c["name"],
            camera=cam_obj,
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
            max_tag_distance=c.get("max_tag_distance", 3),
        )

        # Stream + sink + NT + pipeline
        if c.get("raw_port"):
            # camera object is `cam_obj`; profile entry is `c`; cfg is the matching frc_io camera config
            cfg = next(cc for cc in frc_io.cameraConfigs if cc.name == c["name"])
            build_raw_stream(c["name"], cam_obj, c["raw_port"], cfg.streamConfig)
        build_stream(ctx)
        attach_sink(ctx)
        init_cam_entries(ntinst, ctx)
        ctx.pipeline = SpartanOverlay(
            colors=[k for k in ctx.colors if k != "tags"],
            camera=ctx.name,
            greyscale=ctx.greyscale,
            x_resolution=ctx.x_resolution,
            y_resolution=ctx.y_resolution,
            intrinsics=ctx.intrinsics,
            distortions=ctx.distortions,
            max_tag_distance=ctx.max_tag_distance,
        )

        # Optional brightness override
        b = prof.get("brightness_overrides", {}).get(ctx.name)
        if b is not None:
            nudge_brightness(ctx.camera, b)

        contexts.append(ctx)
        print(f"Added {ctx.name} stream on {ctx.processed_port} and serving table {ctx.table_name}")

    # Run workers
    @run_in_thread
    def worker(ctx, stop_flag):
        while not stop_flag.is_set():
            training = nt_global["training"].get()
            debug = nt_global["debug"].get()
            tick(nt_global, ntinst, ctx, training, debug, push_frame)

    for ctx in contexts:
        worker(ctx)

    # Tiny stats loop
    while True:
        time.sleep(1.0)
        msg = " ".join(
            [f"{ctx.name}:{ctx.fps:0.1f}fps S:{ctx.success_counter} F:{ctx.failure_counter}" for ctx in contexts]
        )
        print(msg, end="\r", flush=True)
