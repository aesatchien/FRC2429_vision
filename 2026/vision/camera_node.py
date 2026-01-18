#!/usr/bin/env python3
import os, sys, time, logging, argparse, subprocess, traceback
from ntcore import NetworkTableInstance
from cscore import CameraServer
from vision.wpi_config import load_vision_cfg, select_profile
from vision import wpi_rio
from vision.wpi_stream import push_frame
from vision.threaded_pipeline import ThreadedVisionPipeline
from vision.network import init_global_flags
from vision.pipeline_setup import deploy_camera_pipeline

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
    if not wpi_rio.readConfig(args.frc):
        log.error(f"could not read {args.frc}")
        sys.exit(1)

    cc = next((c for c in wpi_rio.cameraConfigs if c.name == args.cam), None)
    if cc is None:
        log.error(f"camera '{args.cam}' not found in {args.frc}; available: {[c.name for c in wpi_rio.cameraConfigs]}")
        sys.exit(1)

    cam = wpi_rio.startCamera(cc)

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

    cam_defs = vcfg.get("camera_definitions", {})
    # Use factory to build context
    ctx = deploy_camera_pipeline(cam, cam_prof, cc, ntinst, camera_definitions=cam_defs)

    # Start Threaded Pipeline
    pipeline = ThreadedVisionPipeline(ctx, ntinst, nt_global, push_frame)
    pipeline.start()

    # Main thread just monitors or sleeps
    last_print = 0.0
    while True:
        time.sleep(2.0)
        # Format thread stats
        tf = getattr(ctx, "thread_fps", {})
        tt = getattr(ctx, "thread_time_ms", {})
        stats = " ".join([f"{k}:{v:0.0f}/{tt.get(k,0):0.1f}ms" for k,v in tf.items()])
        print(f"{ctx.name}: {ctx.fps:0.1f}fps ({stats}) S:{ctx.success_counter} F:{ctx.failure_counter}", flush=True)

if __name__ == "__main__":
    main()
