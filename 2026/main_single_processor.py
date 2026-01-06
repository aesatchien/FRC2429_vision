#!/usr/bin/env python3
import sys, time, logging, argparse
from ntcore import NetworkTableInstance

from vision.wpi_config import load_vision_cfg, select_profile
from vision.wpi_stream import push_frame
from vision.network import init_global_flags
from vision import wpi_rio
from vision.threaded_pipeline import ThreadedVisionPipeline
from vision.pipeline_setup import deploy_camera_pipeline

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("vision")

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
        ctx = deploy_camera_pipeline(cam_obj, c, rio_cfg, ntinst)
        contexts.append(ctx)

    # Run workers
    for ctx in contexts:
        log.info(f"Starting threaded pipeline for {ctx.name}")
        pipeline = ThreadedVisionPipeline(ctx, ntinst, nt_global, push_frame)
        pipeline.start()

    # Tiny stats loop
    while True:
        time.sleep(1.0)
        parts = []
        for ctx in contexts:
            tf = getattr(ctx, "thread_fps", {})
            stats = " ".join([f"{k}:{v:0.0f}" for k,v in tf.items()])
            parts.append(f"{ctx.name}:{ctx.fps:0.1f}fps [{stats}]")
        print("  ".join(parts), end="\r", flush=True)
        # print(msg, end="\r", flush=True)
