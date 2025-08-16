#!/usr/bin/env python3
import sys, time, logging
import json
from cscore import CameraServer, UsbCamera, VideoSource
from ntcore import NetworkTableInstance
from spartan_overlay_2025 import SpartanOverlay

from config_io import load_vision_cfg, select_profile
from camctx import CamCtx
from streaming import build_stream, push_frame
from vision_worker import attach_sink, tick
from ntio import init_global_flags, init_cam_entries
from util import run_in_thread, nudge_brightness

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("vision")

configFile = "/boot/frc.json"

# --- existing WPILib JSON loader (your readConfig/readCameraConfig) stays as-is ---
# (Reuse  existing WPILIB functions here)
class CameraConfig: pass

team = None
server = False
cameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False
    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))
        print(f'ATTEMPTING TO SET streamConfig ... on {camera.getName()}')
    else:
        print(f'UNABLE TO SET streamConfig ... on {camera.getName()}')

    return camera
# --- end of existing WPILib stuff ---

if __name__ == "__main__":
    if len(sys.argv) >= 2: configFile = sys.argv[1]
    if not readConfig(): log.error(f"could not read {configFile}"); sys.exit(1)

    ntinst = NetworkTableInstance.getDefault()
    log.info("Starting NT client for team 2429")
    ntinst.startClient4("CJHpi5"); ntinst.setServerTeam(2429); ntinst.startDSClient()

    # start cameras from frc.json
    for cfg in cameraConfigs: cameras.append(startCamera(cfg))

    # per-host profile
    vcfg = load_vision_cfg()
    prof = select_profile(vcfg)

    # global NT flags
    nt_global = init_global_flags(ntinst)

    # build CamCtx list from profile
    contexts = []
    for idx, c in enumerate(prof.get("cameras", [])):
        cam_obj = next((cam for cam, cc in zip(cameras, cameraConfigs) if cc.name == c["name"]), None)
        if cam_obj is None:
            log.warning(f"camera '{c['name']}' not found; skipping"); continue
        ctx = CamCtx(
            name=c["name"], camera=cam_obj,
            processed_port=c.get("processed_port", 1186+idx),
            table_name=c.get("table_name", f"Cameras/{c['name']}"),
            stream_fps=c.get("stream_fps", 16),
            stream_max_width=c.get("stream_max_width", 640),
            greyscale=bool(c.get("greyscale", False)),
            find_tags=c.get("find_tags", True),
            find_colors=c.get("find_colors", False),
            colors=c.get("colors", ["orange"]),
            orientation=c.get("orientation", {"tx":0,"ty":0,"tz":0,"rx":0,"ry":0,"rz":0}),
            intrinsics=c.get("intrinsics"),
            distortions=c.get("distortions"),
            use_distortions=c.get("use_distortions", False),
            max_tag_distance=c.get("max_tag_distance", 3),
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
        if b is not None: nudge_brightness(ctx.camera, b)
        contexts.append(ctx)

    # run workers
    @run_in_thread
    def worker(ctx, stop_flag):
        while not stop_flag.is_set():
            training = nt_global["training"].get()
            debug    = nt_global["debug"].get()
            tick(nt_global, ntinst, ctx, training, debug, push_frame)

    for ctx in contexts: worker(ctx)

    # tiny stats loop
    while True:
        time.sleep(1.0)
        msg = " ".join([f"{ctx.name}:{ctx.fps:0.1f}fps S:{ctx.success_counter} F:{ctx.failure_counter}" for ctx in contexts])
        print(msg, end="\r", flush=True)
