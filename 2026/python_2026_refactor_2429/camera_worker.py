#!/usr/bin/env python3
import os, sys, time, logging, argparse
import numpy as np, cv2
from ntcore import NetworkTableInstance
from cscore import CameraServer
from config_io import load_vision_cfg, select_profile
import frc_io  # <-- module import so we can set frc_io.configFile
from camctx import CamCtx
from streaming import build_stream, push_frame
from vision_worker import tick, attach_sink
from ntio import init_cam_entries, init_global_flags
from spartan_overlay_2025 import SpartanOverlay

log = logging.getLogger("camproc")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

def pin_to_cpu(cpu: int | None):
    if cpu is None: return
    try: os.sched_setaffinity(0, {int(cpu)})
    except Exception as e: log.warning(f"CPU pin failed: {e}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", required=True, help="camera name (must match frc.json)")
    ap.add_argument("--cpu", type=int, default=None, help="CPU core to pin")
    ap.add_argument("--vision", default="/boot/vision.json")
    ap.add_argument("--frc", default="/boot/frc.json")
    args = ap.parse_args()

    pin_to_cpu(args.cpu)

    # read frc.json and start ONLY this camera
    frc_io.configFile = args.frc
    if not frc_io.readConfig():
        log.error(f"could not read {frc_io.configFile}")
        sys.exit(1)

    cc = next((c for c in frc_io.cameraConfigs if c.name == args.cam), None)
    if cc is None:
        log.error(f"camera '{args.cam}' not found in {args.frc}; available: {[c.name for c in frc_io.cameraConfigs]}")
        sys.exit(1)

    cam = frc_io.startCamera(cc)

    # per-host profile
    vcfg = load_vision_cfg(args.vision)
    prof = select_profile(vcfg)
    cam_prof = next((c for c in prof.get("cameras", []) if c["name"] == args.cam), None)
    if cam_prof is None:
        log.error(f"'{args.cam}' missing from profile; add it to {args.vision}")
        sys.exit(1)

    # NT
    ntinst = NetworkTableInstance.getDefault()
    ntinst.startClient4(f"proc-{args.cam}")
    ntinst.setServerTeam(2429)  # change to setServer("x.y.z.w") if off-robot
    ntinst.startDSClient()
    nt_global = init_global_flags(ntinst)

    # Build context
    ctx = CamCtx(
        name=args.cam, camera=cam,
        processed_port=cam_prof.get("processed_port", 1186),
        table_name=cam_prof.get("table_name", f"Cameras/{args.cam}"),
        stream_fps=cam_prof.get("stream_fps", 30),
        stream_max_width=cam_prof.get("stream_max_width", 640),
        greyscale=bool(cam_prof.get("greyscale", False)),
        find_tags=cam_prof.get("find_tags", True),
        find_colors=cam_prof.get("find_colors", False),
        colors=cam_prof.get("colors", ["orange"]),
        orientation=cam_prof.get("orientation", {"tx":0,"ty":0,"tz":0,"rx":0,"ry":0,"rz":0}),
        intrinsics=cam_prof.get("intrinsics"),
        distortions=cam_prof.get("distortions"),
        use_distortions=cam_prof.get("use_distortions", False),
        max_tag_distance=cam_prof.get("max_tag_distance", 3),
        tag_every_n=cam_prof.get("tag_every_n", 2)
    )

    # stream + sink + NT + pipeline
    build_stream(ctx)
    attach_sink(ctx)
    init_cam_entries(ntinst, ctx)
    ctx.pipeline = SpartanOverlay(
        colors=[k for k in ctx.colors if k != "tags"],
        camera=ctx.name, greyscale=ctx.greyscale,
        x_resolution=ctx.x_resolution, y_resolution=ctx.y_resolution,
        intrinsics=ctx.intrinsics, distortions=ctx.distortions,
        max_tag_distance=ctx.max_tag_distance
    )

    # optional brightness override
    b = prof.get("brightness_overrides", {}).get(ctx.name)
    if b is not None:
        try:
            cam.setBrightness(int(b)+1); time.sleep(0.2); cam.setBrightness(int(b))
        except Exception as e:
            log.debug(f"brightness nudge failed: {e}")

    log.info(f"{ctx.name}: streaming on {ctx.processed_port}")

    # main loop
    while True:
        training = False if nt_global.get("training") is None else nt_global["training"].get()
        debug    = False if nt_global.get("debug")    is None else nt_global["debug"].get()
        tick(nt_global, ntinst, ctx, training, debug, push_frame)

if __name__ == "__main__":
    main()
