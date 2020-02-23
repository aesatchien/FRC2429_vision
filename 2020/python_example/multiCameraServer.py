#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
import ntcore

from cscore import HttpCamera, CvSource, VideoMode  # CJH
# from networktables import NetworkTables  # CJH
# from networktables import NetworkTableEntry  # CJH
from spartan_overlay import SpartanOverlay  # CJH
from threading import Thread  # CJH
import cv2  # CJH
import numpy as np


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
streams = []

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

    return inst, camera

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
        ntcore.constants.NT_NOTIFY_IMMEDIATE |
        ntcore.constants.NT_NOTIFY_NEW |
        ntcore.constants.NT_NOTIFY_UPDATE)

    return server


# make a pipeline thread
def spartan_process(method='size', nt_inst=None, nt_entries=[], camera=None, image_source=None, cv_stream=None):
    pipeline = SpartanOverlay()
    image = np.zeros(shape=(256, 320, 3), dtype=np.uint8)
    cs = CameraServer.getInstance()
    cv_sink = cs.getVideo()
    if True:  # can be a while true
        # targets, distance_to_target, strafe_to_target, height, rotation_to_target = pipeline.process(cameras.get(0))
        # image = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        # pipeline_values = pipeline.process(cameras.get(0), method=method)
        cv_time, image = cv_sink.grabFrame(image)
        if cv_time == 0:
            # Send the output the error.
            print("failed to get cv_sink")
            # cv_stream.notifyError(cv_sink.getError());
            # skip the rest of the current iteration
            #continue
        pipeline_values = pipeline.process(image, method=method)
        for entry, val in zip(nt_entries, pipeline_values):
            entry.setNumber(val)
            # targets_entry.setNumber(targets)
            # distance_entry.setNumber(distance_to_target)
            # rotation_entry.setNumber(strafe_to_target)
            # strafe_entry.setNumber(rotation_to_target)
        if nt_inst is not None:
            nt_inst.flush()
        if image_source is not None:
            image_source.putFrame(pipeline.image)

def test_loop(image_source=None):
    image = np.zeros(shape=(256, 320, 3), dtype=np.uint8)
    while True:
        image_source.putFrame(image)

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
        #ntinst.startClientTeam(team)
        ntinst.startClient("192.168.1.28")  # CJH for testing at home

    #  CJH  stuff - add this to get networktables and processed stream
    ballTable = ntinst.getTable("BallCam")
    targets_entry = ballTable.getEntry("targets")
    distance_entry = ballTable.getEntry("distance")
    rotation_entry = ballTable.getEntry("rotation")
    strafe_entry = ballTable.getEntry("strafe")
    nt_entries = [targets_entry, distance_entry, rotation_entry, strafe_entry]
    # set up the camera resolution and ports - TODO: read this from the config json
    x_resolution, y_resolution = 320, 256
    processed_port = 1182
    # we need an image source to put frames to
    image_source = CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, x_resolution, y_resolution, 20)
    cv_stream = MjpegServer("CV Image Stream", processed_port)
    cv_stream.setSource(image_source)
    # cvStream.getProperty("compression").set(3)

    #camera_server.addCamera(image_source)

    # this just adds the processed camera entry to the networktables
    print(f"*** Starting 2429 BallCam Processed stream on {processed_port}")
    camera = HttpCamera("BallCam Processed", "http://192.168.1.31:1182/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
    camera_server = CameraServer.getInstance()
    camera_server.addCamera(camera)

    # start cameras
    for config in cameraConfigs:
        #cameras.append(startCamera(config))
        cs, cameraCapture = startCamera(config)
        streams.append(cs)
        cameras.append(cameraCapture)

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    #  start a vision thread (CJH)
    # TODO - pull in method from the co-pilot
    print('Starting vision thread - ball processing...')
    #Thread(target=spartan_process, args=(), kwargs={'method':'size', 'nt_inst':ntinst, 'nt_entries':nt_entries,
    #                                                'camera':cameras[0],'image_source':image_source, 'cv_stream':cv_stream}).start()
    #Thread(target=test_loop(),args=(), kwargs={'image_source':image_source})
    print('Thread started.')
    #spartan_process(**{'method':'size', 'nt_inst':ntinst, 'nt_entries':nt_entries, 'camera':cameras[0],'image_source':image_source, 'cv_stream':cv_stream})

    # loop forever - this just keeps the camera server alive
    image = np.zeros(shape=(256, 320, 3), dtype=np.uint8)
    stream = streams[0].getVideo()

    while True:
        time.sleep(0.5)
        (timestamp, image) = stream.grabFrame(image)
        print(f'Timestamp: {timestamp}')
        image_source.putFrame(image)
