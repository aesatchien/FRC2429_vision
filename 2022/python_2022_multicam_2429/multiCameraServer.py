#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys
import numpy as np

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance

from cscore import HttpCamera, CvSource, VideoMode  # CJH
from spartan_overlay import SpartanOverlay

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
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

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
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

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        NetworkTablesInstance.NotifyFlags.IMMEDIATE |
        NetworkTablesInstance.NotifyFlags.NEW |
        NetworkTablesInstance.NotifyFlags.UPDATE)

    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
        ntinst.startDSClient()

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)


    #  -------   CJH  stuff - add this to get networktables and processed stream  ------------
    ballTable = ntinst.getTable("BallCam")
    targets_entry = ballTable.getEntry("targets")
    distance_entry = ballTable.getEntry("distance")
    rotation_entry = ballTable.getEntry("rotation")
    strafe_entry = ballTable.getEntry("strafe")

    vm = cameras[0].getVideoMode()
    x_resolution = vm.width
    y_resolution = vm.height
    #fps = cameras
    processed_port = 1182
    # add an image source, should probably read camera[0] to get the resolution.  I think it ignores FPS, or is
    # fixed by how often you put an image (below)
    image_source = CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, x_resolution, y_resolution, 30)
    # start a stream
    cvStream = MjpegServer("CV Image Stream", processed_port)
    # compress the stream - 1 cuts data by 16x but is unrecognizable, 100 is no compression and 320x240x30fps  is 11Mbps
    # 50 seems to cut it ~ 5x t- 2.3Mbps, still looks pretty good, 25 is pretty marginal at 1.6Mbps.  Default is -1?
    #cvStream.getProperty("compression").set(75)
    cvStream.setSource(image_source)  # now the stream is updated by image_source
    cs = CameraServer.getInstance()
    cs.addCamera(image_source)  # is this really necessary?  we add another one later
    print(f"*** Starting 2429 BallCam Processed stream on {processed_port}")
    # camera = HttpCamera("BallCam Processed", "http://10.24.29.12:1182/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
    # actually, this should be more bulletproof, esp when testing at home
    camera = HttpCamera("BallCam Processed", "http://127.0.0.1:1182/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
    cs.addCamera(camera)  # how does this know we want cvStream assigned to the web?
    # -------------------------------------------'''

    #  ----------------  start a vision thread (CJH)  --------------------
    # TODO - turn this into a thread running in the background
    pipeline = SpartanOverlay()

    cs = CameraServer.getInstance()  # already taken care of above, but if i comment it out this is necessary
    # this next part will be trickier if we have more than one camera, but we can have separate pipelines
    # also, since i added a camera above, this doesn't work if i just use cs.getVideo() - i think adding that one changes the primary
    sink = cs.getVideo(camera=cameras[0])
    #sink = cs.getVideo() # can only do this if a camera is started, i.e. camera = cs.startAutomaticCapture()
    vm = cameras[0].getVideoMode()  # getting the resolution, etc from the camera
    img = np.zeros((vm.height, vm.width, 3))  # make our first dummy image for cv

    success_counter = 0
    previous_counts = 0
    failure_counter= 0
    print(f'Entering image process loop with a {vm.width}x{vm.height} stream...', flush=True)
    previous_time = time.time()
    while True and failure_counter < 100:
        if len(cameras) >= 1:
            image_time, captured_img = sink.grabFrame(img)
            if image_time > 0: # actually got an image
                targets, distance_to_target, strafe_to_target, height, rotation_to_target = pipeline.process(captured_img)
                targets_entry.setNumber(targets)
                distance_entry.setNumber(distance_to_target)
                rotation_entry.setNumber(strafe_to_target)
                strafe_entry.setNumber(rotation_to_target)
                ntinst.flush()
                image_source.putFrame(pipeline.image)
                success_counter += 1
            else:  # keep track of failures to read the image from the sink
                failure_counter += 1
            if success_counter % 30 == 0:
                fps = (success_counter - previous_counts) / (time.time() - previous_time)
                print(f'Cameras: {len(cameras)}  Avg FPS: {fps:0.1f}  Reported:{cameras[0].getActualFPS()}  Success: {success_counter:10d}  Failure:{failure_counter:10d}', end='\r', flush=True)
                previous_counts = success_counter
                previous_time = time.time()
        # ----------------------------------------------------

    # loop forever
    #while True:
    #    time.sleep(9)
