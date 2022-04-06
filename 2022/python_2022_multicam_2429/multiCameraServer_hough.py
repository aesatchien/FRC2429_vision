#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys, os, glob
import numpy as np

#from cv2 import imwrite, resize
import cv2

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance

from cscore import HttpCamera, CvSource, VideoMode  # 2429 CJH
from spartan_overlay import SpartanOverlay # 2429 CJH

from ctypes import Structure, c_uint


class Blocks(Structure):
    _fields_ = [("m_signature", c_uint),
                ("m_x", c_uint),
                ("m_y", c_uint),
                ("m_width", c_uint),
                ("m_height", c_uint),
                ("m_angle", c_uint),
                ("m_index", c_uint),
                ("m_age", c_uint)]

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


    #  -------   2429 CJH  stuff - add this to get networktables and processed stream  ------------

    ballTable = ntinst.getTable("BallCam")  # the network table served by the pi

    # add an image source, should probably read camera[0] to get the resolution.  I think it ignores FPS
    # except for how it reports, and the actual speed is determined by the rate the code puts images to the sink

    processed_ports = [1186, 1187]   # allow for multiple cameras on 1181-1185
    stream_labels = ['Ballcam', 'Shootercam']
    image_source = [None, None]
    cvStream = [None, None]
    cams = [None, None]
    for ix, cam in enumerate(cameras[::-1]):  # should allow me to have as many cameras as i want
        vm = cameras[ix].getVideoMode()
        x_resolution = vm.width
        y_resolution = vm.height
        stream_fps = 20
        image_source[ix] = CvSource(f"{stream_labels[ix]} CV Image Source", VideoMode.PixelFormat.kMJPEG, x_resolution, y_resolution, stream_fps)
        # start a stream
        cvStream[ix] = MjpegServer(f"{stream_labels[ix]} CV Image Stream", processed_ports[ix])
        # compress the stream - 1 cuts data by 16x but is unrecognizable, 100 is no compression and 320x240x30fps is 11Mbps
        # 50 seems to cut it ~ 5x t- 2.3Mbps, still looks pretty good, 25 is pretty marginal at 1.6Mbps.  Default is -1?
        if x_resolution > 300:  # just take the default for smaller formats
            cvStream[ix].getProperty("compression").set(25)
        cvStream[ix].setSource(image_source[ix])  # now the stream is updated by image_source
        cs = CameraServer.getInstance()
        cs.addCamera(image_source[ix])  # is this really necessary?  we add another one later
        print(f"*** Starting 2429 {stream_labels[ix]} Processed stream on {processed_ports[ix]}")
        # camera = HttpCamera("BallCam Processed", "http://10.24.29.12:1182/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
        # actually, this should be more bulletproof, esp when testing at home
        cams[ix] = HttpCamera(f"{stream_labels[ix]} Processed", f"http://127.0.0.1:{processed_ports[ix]}/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
        cs.addCamera(cams[ix])  # how does this know we want cvStream assigned to the web?
    # -------------------------------------------'''

    #  ----------------  start a vision thread (CJH)  --------------------
    # TODO - turn this into a thread running in the background


    ballframes = ballTable.getEntry('frames')
    camera_dict = {'red': {}, 'blue': {}, 'green':{}}  # the colors we need to check for
    # set up network tables and pipelines, one for each color
    for key in camera_dict.keys():
        camera_dict[key].update({'targets_entry': ballTable.getEntry(f"/{key}/targets")})
        camera_dict[key].update({'distance_entry': ballTable.getEntry(f"/{key}/distance")})
        camera_dict[key].update({'rotation_entry': ballTable.getEntry(f"/{key}/rotation")})
        camera_dict[key].update({'strafe_entry': ballTable.getEntry(f"/{key}/strafe")})
        if key == 'green':
            camera_dict[key].update({'pipeline': SpartanOverlay(color=key, camera='lifecam')})
        else:
            camera_dict[key].update({'pipeline': SpartanOverlay(color=key, camera='geniuscam')})

    cs = CameraServer.getInstance()  # already taken care of above, but if i comment it out above this is necessary
    # this next part will be trickier if we have more than one camera, but we can have separate pipelines
    # also, since i added a camera above, this doesn't work if i just use cs.getVideo() - i think adding that one changes the primary
    sink = cs.getVideo(camera=cameras[0])
    # sink = cs.getVideo() # can only do this if a camera is started, i.e. camera = cs.startAutomaticCapture()
    vm = cameras[0].getVideoMode()  # getting the resolution, etc from the camera again, repeating in case above changes
    img = np.zeros((vm.height, vm.width, 3))  # make our first dummy image for cv to write over

    if len(cameras) > 1:  # add a second camera
        sink_2 = cs.getVideo(camera=cameras[1])
        vm2 = cameras[1].getVideoMode()
        img2 = np.zeros((vm2.height, vm2.width, 3))


    ballcam_success_counter = 0  # keep track of FPS so we can tell in the console when things are going wrong
    shootercam_success_counter = 0
    previous_ball_counts = 0
    previous_shooter_counts = 0
    failure_counter = 0  # very first image often fails, and after that we are solid
    print(f'Entering image process loop with a {vm.width}x{vm.height} stream...', flush=True)
    previous_time = time.time()

    team_key = 'blue'  # default color for camera
    key_order = ['red', 'blue']  # process in this order

    server_dict = {'ballcam' : True, 'shootercam' : True}

    # make a folder to keep track of images
    save_images = False
    if save_images:
        image_counter = 0
        dir_count = len(glob.glob('image*')) + 2
        folder = f'images_{dir_count}'
        try:
            os.mkdir(folder)
        except Exception as e:
            pass


    while True and failure_counter < 100:

        if len(cameras) >= 1:
            # get the ball images
            image_time, captured_img = sink.grabFrame(img)  # default time out is about 4 FPS
            if image_time > 0:  # actually got an image
                for key in key_order:
                    targets, distance_to_target, strafe_to_target, height, rotation_to_target = camera_dict[key]['pipeline'].process(captured_img.copy())
                    camera_dict[key]['targets_entry'].setNumber(targets)  # should see if we can make this one dict to push, may be a one-liner
                    camera_dict[key]['distance_entry'].setNumber(distance_to_target)
                    camera_dict[key]['strafe_entry'].setNumber(strafe_to_target)
                    camera_dict[key]['rotation_entry'].setNumber(rotation_to_target)
                ntinst.flush()

                if ballcam_success_counter % 107 == 0 and save_images:  # save an image every few seconds

                    image_counter += 1
                    print(f'Writing image {image_counter%200:03d}...')
                    cv2.imwrite(f'{folder}/test_{image_counter%200:03d}.png', captured_img)

                # if we are connected to a robot, get its team color.  default to blue
                if ballcam_success_counter % 50 == 0:  # check every 5s for a team color update
                    ballframes.setNumber(ballcam_success_counter)
                    if ntinst.getTable('FMSInfo').getEntry('IsRedAlliance').getBoolean(True):
                        team_key = 'red'
                        key_order = ['blue', 'red']  # probably not necessary
                    else:
                        team_key = 'blue'
                        key_order = ['red', 'blue']  # probably not necessary
                    # print(f'At frame {success_counter} team key is {team_key}')
                if server_dict['ballcam']:
                    image_source[0].putFrame(camera_dict[team_key]['pipeline'].image)  # feeds the Http camera with a new image, either blue or red
                ballcam_success_counter += 1

            else:  # keep track of failures to read the image from the sink
                failure_counter += 1
            if ballcam_success_counter % 30 == 0:
                now = time.time()
                # update the console - most FPS issues are with auto-exposure or if exposure time is too long for FPS setting
                # the cameras seem to cut FPS down automatically by integer divisors - e.g. max 30 --> max 15 --> max 7.5
                fps1 = (ballcam_success_counter - previous_ball_counts) / (now - previous_time)
                fps2 = (shootercam_success_counter - previous_shooter_counts) / (now - previous_time)
                print(f'Cameras: {len(cameras)}  Avg Ball FPS: {fps1:0.1f} Avg Shoot FPS: {fps2:0.1f}  Reported:{cameras[0].getActualFPS()}  Success: {ballcam_success_counter:8d}  Failure:{failure_counter:3d}', end='\r', flush=True)
                previous_ball_counts = ballcam_success_counter
                previous_shooter_counts = shootercam_success_counter
                previous_time = now

        if len(cameras) >= 2:
            # get the shooter target images - green stuff
            image_time, captured_img2 = sink_2.grabFrame(img2)  # default time out is about 4 FPS
            # check if the foscam is making large images
            if captured_img2.shape[1] > 400:  # this is the width
                scale_percent = 50  # percent of original size
                width = int(captured_img2.shape[1] * scale_percent / 100)
                height = int(captured_img2.shape[0] * scale_percent / 100)
                dim = (width, height)
                image_to_process = cv2.resize(captured_img2.copy(), dim, interpolation=cv2.INTER_AREA)
            else:
                image_to_process = captured_img2.copy()

            if image_time > 0:  # actually got an image
                for key in ['green']:
                    targets, distance_to_target, strafe_to_target, height, rotation_to_target = camera_dict[key]['pipeline'].process(image_to_process)
                    camera_dict[key]['targets_entry'].setNumber(targets)  # should see if we can make this one dict to push, may be a one-liner
                    camera_dict[key]['distance_entry'].setNumber(distance_to_target)
                    camera_dict[key]['strafe_entry'].setNumber(strafe_to_target)
                    camera_dict[key]['rotation_entry'].setNumber(rotation_to_target)
                ntinst.flush()
                if server_dict['shootercam']:
                    image_source[1].putFrame(camera_dict[key]['pipeline'].image)
                shootercam_success_counter +=1
        # ----------------------------------------------------

