#!/usr/bin/env python3
import os, sys, time, logging, argparse, subprocess, traceback
from ntcore import NetworkTableInstance
from cscore import CameraServer
from vision.wpi_config import load_vision_cfg, select_profile
from vision import wpi_rio
from vision.camera_context import CameraContext
from vision.wpi_stream import build_stream, push_frame, build_raw_stream
from vision.wpi_attach_sink import attach_sink
from vision.threaded_pipeline import ThreadedVisionPipeline
from vision.network import init_cam_entries, init_global_flags
from vision.camera_controls import set_camera_robust_defaults
from vision.camera_model import CameraModel
from vision.detectors import TagDetector, HSVDetector

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
    wpi_rio.configFile = args.frc
    if not wpi_rio.readConfig():
        log.error(f"could not read {wpi_rio.configFile}")
        sys.exit(1)

    cc = next((c for c in wpi_rio.cameraConfigs if c.name == args.cam), None)
    if cc is None:
        log.error(f"camera '{args.cam}' not found in {args.frc}; available: {[c.name for c in wpi_rio.cameraConfigs]}")
        sys.exit(1)

    cam = wpi_rio.startCamera(cc)

    # Re-enabled: Get actual camera resolution to ensure buffer sizes match
    # We wait for connection to ensure we don't get a default 0x0
    while not cam.isConnected(): time.sleep(0.1)
    vm = cam.getVideoMode()
    width = vm.width if vm.width > 0 else 640
    height = vm.height if vm.height > 0 else 480

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
    if os.environ.get("NT_SERVER"):
        ntinst.setServer(os.environ["NT_SERVER"])  # e.g., "127.0.0.1"
    else:
        ntinst.setServerTeam(2429)
        ntinst.startDSClient()

    nt_global = init_global_flags(ntinst)

    # Build context
    ctx = CameraContext(
        name=args.cam, camera=cam,
        x_resolution=width,
        y_resolution=height,
        camera_type= cam_prof.get("camera_type", 'c920'),
        raw_port=cam_prof.get("raw_port"),
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
        max_tag_distance=cam_prof.get("max_tag_distance", 3)
    )

    raw_port = cam_prof.get("raw_port")
    print(f'{ctx.name} serving raw on {ctx.raw_port} and processed on {ctx.processed_port}', flush=True)
    if raw_port:
        # pass cc.streamConfig if you want width/fps/compression applied to raw
        try:
            sc = next(c for c in wpi_rio.cameraConfigs if c.name == args.cam).streamConfig
        except StopIteration:
            sc = None
        build_raw_stream(args.cam, cam, raw_port, sc)

    # stream + sink + NT + pipeline
    build_stream(ctx)
    attach_sink(ctx)
    init_cam_entries(ntinst, ctx)
    
    # Initialize Detectors
    cam_model = CameraModel(ctx.x_resolution, ctx.y_resolution, ctx.camera_type, ctx.intrinsics, ctx.distortions)
    ctx.tag_detector = TagDetector(cam_model)
    ctx.hsv_detector = HSVDetector(cam_model)

    # Apply robust defaults (brightness nudge + v4l2 exposure)
    # cc is the CameraConfig object from rio
    set_camera_robust_defaults(cam, cc, ctx.camera_type, delay=3.0)

    log.info(f"{ctx.name}: streaming on {ctx.processed_port}")

    # Start Threaded Pipeline
    pipeline = ThreadedVisionPipeline(ctx, ntinst, nt_global, push_frame)
    pipeline.start()

    # Main thread just monitors or sleeps
    last_print = 0.0
    while True:
        time.sleep(1.0)
        print(f"{ctx.name}: {ctx.fps:0.1f}fps  S:{ctx.success_counter}  F:{ctx.failure_counter}", flush=True)

if __name__ == "__main__":
    main()
