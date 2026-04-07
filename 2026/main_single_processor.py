#!/usr/bin/env python3
import sys, os, time, logging, argparse
from pathlib import Path
from ntcore import NetworkTableInstance

from vision.wpi_config import load_vision_cfg, select_profile
from vision.wpi_stream import push_frame
from vision.network import init_global_flags
from vision import wpi_rio
from vision.threaded_pipeline import ThreadedVisionPipeline
from vision.pipeline_setup import deploy_camera_pipeline

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("vision")

# Persistent restart counter — survives process restarts, reset after healthy uptime
FATAL_RESTART_COUNT_FILE = Path("/tmp/vision_fatal_count")
MAX_FATAL_RESTARTS = 2
HEALTHY_UPTIME_S = 60.0  # seconds of clean running before the restart streak is forgiven

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="10.24.29.2", help="NetworkTables server IP")
    parser.add_argument("--frc", default="/boot/frc.json", help="Path to frc.json")
    parser.add_argument("--vision", default="/boot/vision.json", help="Path to vision.json")
    parser.add_argument("--autorestart", action="store_true", help="Ignored; for compatibility with launcher")
    args = parser.parse_args()

    if not wpi_rio.readConfig(args.frc):
        log.error(f"could not read {args.frc}")
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
    
    cam_defs = vcfg.get("camera_definitions", {})

    # Global NT flags
    nt_global = init_global_flags(ntinst)

    # Build CamCtx list from profile
    contexts = []
    for idx, c in enumerate(prof.get("cameras", [])):
        cam_obj = name_to_cam.get(c["name"])
        if cam_obj is None:
            log.warning(f"camera '{c['name']}' not found; skipping")
            continue

        # Use the factory to build the pipeline context
        rio_cfg = next((cc for cc in wpi_rio.cameraConfigs if cc.name == c["name"]), None)
        ctx = deploy_camera_pipeline(cam_obj, c, rio_cfg, ntinst, camera_definitions=cam_defs)
        contexts.append(ctx)

    # Run workers
    pipelines = []
    for ctx in contexts:
        log.info(f"Starting threaded pipeline for {ctx.name}")
        pipeline = ThreadedVisionPipeline(ctx, ntinst, nt_global, push_frame)
        pipeline.start()
        pipelines.append(pipeline)

    start_time = time.time()

    # Stats + health loop
    while True:
        time.sleep(2.0)

        # After running cleanly long enough, forgive any prior restart streak
        if time.time() - start_time > HEALTHY_UPTIME_S:
            FATAL_RESTART_COUNT_FILE.unlink(missing_ok=True)

        # Check for fatal acquisition failures
        failed = [p for p in pipelines if p.fatal.is_set()]
        if failed:
            names = [p.ctx.name for p in failed]
            count = int(FATAL_RESTART_COUNT_FILE.read_text()) if FATAL_RESTART_COUNT_FILE.exists() else 0
            count += 1
            FATAL_RESTART_COUNT_FILE.write_text(str(count))
            if count >= MAX_FATAL_RESTARTS:
                log.error(f"Camera(s) {names} fatal — restart count {count} >= {MAX_FATAL_RESTARTS}, rebooting")
                os.system("sudo reboot")
            else:
                log.error(f"Camera(s) {names} fatal — restart count {count}, exiting for restart")
                sys.exit(2)

        parts = []
        for ctx in contexts:
            tf = getattr(ctx, "thread_fps", {})
            tt = getattr(ctx, "thread_time_ms", {})
            stats = " ".join([f"{k}:{v:0.0f}/{tt.get(k,0):0.1f}ms" for k,v in tf.items()])
            parts.append(f"{ctx.name}:{ctx.fps:0.1f}fps [{stats}]")
        print("  ".join(parts), end="\r", flush=True)
