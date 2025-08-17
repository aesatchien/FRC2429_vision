#!/usr/bin/env python3
# frc_io.py — functions taken from the WPILIB cameraserver script
import sys, json
from cscore import CameraServer, VideoSource, UsbCamera

# Same globals/names your code expects
configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
cameras = []
servers = {}  # name -> MjpegServer

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
        s = j["ntmode"]
        if s.lower() == "client":
            server = False
        elif s.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(s))

    # cameras
    try:
        cams = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for c in cams:
        if not readCameraConfig(c):
            return False
    return True

def startCamera(config, *_, **__):
    # UsbCamera only; no auto MJPEG server here
    print(f"Starting camera '{config.name}' on {config.path}")
    camera = UsbCamera(config.name, config.path)
    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    return camera

def set_stream_port(name: str, port: int) -> bool:
    srv = servers.get(name)
    if not srv:
        print(f"[frc_io] no server for '{name}'", file=sys.stderr)
        return False
    try:
        srv.setPort(int(port))
        return True
    except Exception as e:
        print(f"[frc_io] set_stream_port({name},{port}) failed: {e}", file=sys.stderr)
        return False

