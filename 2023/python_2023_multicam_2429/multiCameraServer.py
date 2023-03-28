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
from ntcore import NetworkTableInstance, EventFlags

from cscore import HttpCamera, CvSource, VideoMode  # 2429 CJH
from spartan_overlay_2023 import SpartanOverlay  # 2429 CJH

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
#configFile = "./files/frc.json"

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
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))


    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.addSwitchedCamera(config.name)

    def listener(event):
        data = event.data
        if data is not None:
            value = data.value.value()
            if isinstance(value, int):
                if value >= 0 and value < len(cameras):
                    server.setSource(cameras[value])
            elif isinstance(value, float):
                i = int(value)
                if i >= 0 and i < len(cameras):
                    server.setSource(cameras[i])
            elif isinstance(value, str):
                for i in range(len(cameraConfigs)):
                    if value == cameraConfigs[i].name:
                        server.setSource(cameras[i])
                        break

    NetworkTableInstance.getDefault().addListener(
        NetworkTableInstance.getDefault().getEntry(config.key),
        EventFlags.kImmediate | EventFlags.kValueAll,
        listener)

    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        pass

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    # start cameras
    # work around wpilibsuite/allwpilib#5055
    CameraServer.setSize(CameraServer.kSize160x120)
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)


    #  -------   2429 CJH  stuff - add this to get networktables and processed stream  ------------

    top_table = ntinst.getTable("ArmCam")  # the network table served by the pi
    ground_table = ntinst.getTable("BottomCam")  # the network table served by the pi

    # add an image source, should probably read camera[0] to get the resolution.  I think it ignores FPS
    # except for how it reports, and the actual speed is determined by the rate the code puts images to the sink

    processed_ports = [1186, 1187]   # allow for multiple cameras on 1181-1185
    stream_labels = ['ArmCam', 'BottomCam']
    image_source = [None, None]
    cvStream = [None, None]
    cams = [None, None]
    for ix, cam in enumerate(cameras):  # should allow me to have as many cameras as i want
        vm = cameras[ix].getVideoMode()
        x_resolution = vm.width
        y_resolution = vm.height
        stream_fps = 30
        image_source[ix] = CvSource(f"{stream_labels[ix]} CV Image Source", VideoMode.PixelFormat.kMJPEG, x_resolution, y_resolution, stream_fps)
        # start a stream
        cvStream[ix] = MjpegServer(f"{stream_labels[ix]} CV Image Stream", processed_ports[ix])
        # compress the stream - 1 cuts data by 16x but is unrecognizable, 100 is no compression and 320x240x30fps is 11Mbps
        # 50 seems to cut it ~ 5x t- 2.3Mbps, still looks pretty good, 25 is pretty marginal at 1.6Mbps.  Default is -1?
        if x_resolution > 300:  # just take the default for smaller formats
            cvStream[ix].getProperty("compression").set(35)
        cvStream[ix].setSource(image_source[ix])  # now the stream is updated by image_source
        cs = CameraServer  #  .getInstance()
        cs.addCamera(image_source[ix])  # is this really necessary?  we add another one later
        print(f"*** Starting 2429 {stream_labels[ix]} Processed stream on {processed_ports[ix]}")
        # camera = HttpCamera("BallCam Processed", "http://10.24.29.12:1182/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
        # actually, this should be more bulletproof, esp when testing at home
        cams[ix] = HttpCamera(f"{stream_labels[ix]} Processed", f"http://127.0.0.1:{processed_ports[ix]}/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
        cs.addCamera(cams[ix])  # how does this know we want cvStream assigned to the web?
    # -------------------------------------------'''

    #  ----------------  start a vision thread (CJH)  --------------------
    # TODO - turn this into a thread running in the background
    training_topic_publisher = top_table.getBooleanTopic('training').publish()  # is it really this annoying?
    training_topic_publisher.set(False)
    training_topic_subscriber = top_table.getBooleanTopic('training').subscribe(False)

    debug_topic_publisher = top_table.getBooleanTopic('debug').publish()  # is it really this annoying?
    debug_topic_publisher.set(False)
    debug_topic_subscriber = top_table.getBooleanTopic('debug').subscribe(False)

    training_color = top_table.getStringTopic('training_color').publish()
    top_frames = top_table.getDoubleTopic('frames').publish()
    top_colors = top_table.getStringArrayTopic('colors').publish()
    # camera_dict = {'red': {}, 'blue': {}, 'green':{}}  # the colors we need to check for
    top_camera_dict = {'yellow': {}, 'purple': {}, 'green': {}}  # the colors we need to check for
    bottom_camera_dict = {'yellow': {}, 'purple': {}}
    # set up network tables and pipelines, one for each color
    for key in top_camera_dict.keys():
        top_camera_dict[key].update({'targets_entry': top_table.getDoubleTopic(f"/{key}/targets").publish()})
        top_camera_dict[key].update({'distance_entry': top_table.getDoubleTopic(f"/{key}/distance").publish()})
        top_camera_dict[key].update({'rotation_entry': top_table.getDoubleTopic(f"/{key}/rotation").publish()})
        top_camera_dict[key].update({'strafe_entry': top_table.getDoubleTopic(f"/{key}/strafe").publish()})
    for key in bottom_camera_dict.keys():
        bottom_camera_dict[key].update({'targets_entry': ground_table.getDoubleTopic(f"/{key}/targets").publish()})
        bottom_camera_dict[key].update({'distance_entry': ground_table.getDoubleTopic(f"/{key}/distance").publish()})
        bottom_camera_dict[key].update({'rotation_entry': ground_table.getDoubleTopic(f"/{key}/rotation").publish()})
        bottom_camera_dict[key].update({'strafe_entry': ground_table.getDoubleTopic(f"/{key}/strafe").publish()})
    ntinst.flush()
    # set up a pipeline for each camera
    top_pipeline = SpartanOverlay(colors=list(top_camera_dict.keys()), camera='lifecam')
    bottom_pipeline = SpartanOverlay(colors=list(bottom_camera_dict.keys()), camera='lifecam')
    top_colors.set(top_pipeline.colors)  # announce to NT that we have the right colors

    cs = CameraServer #  .getInstance()  # already taken care of above, but if i comment it out above this is necessary
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

    topcam_success_counter = 0  # keep track of FPS so we can tell in the console when things are going wrong
    groundcam_success_counter = 0
    previous_topcam_counts = 0
    previous_groundcam_counts = 0
    failure_counter = 0  # very first image often fails, and after that we are solid
    print(f'Entering image process loop with a {vm.width}x{vm.height} stream...', flush=True)
    previous_time = time.time()

    server_dict = {'armcam' : True, 'bottomcam' : True}

    # make a folder to keep track of images  TODO - just do this on the driverstation in a notebook? Turn on from dash?
    save_images = False
    if save_images:
        image_counter = 0
        dir_count = len(glob.glob('image*')) + 2
        folder = f'images_{dir_count}'
        try:
            os.mkdir(folder)
        except Exception as e:
            pass

    bright_list = [item.config["brightness"] for item in cameraConfigs]
    cameras[0].setBrightness(bright_list[0]+1) # seems to be a bug in 2023 code - setting brightness fixes exposure issues on boot
    time.sleep(0.25)
    print(f'Resetting brightness on cam 0 to {bright_list[0]}')
    cameras[0].setBrightness(bright_list[0])


    training = False  # for pipeline.process - are we in training mode
    debug = False # for pipeline process - do we want to see contours
    while True and failure_counter < 100:

        if len(cameras) >= 1:
            # get the armcam images
            image_time, captured_img = sink.grabFrame(img)  # default time out is about 4 FPS
            if image_time > 0:  # actually got an image
                results = top_pipeline.process(captured_img.copy(), method='size',  training=training, debug=debug)
                for key in top_pipeline.colors:  # doing all colors !
                    #targets, distance_to_target, strafe_to_target, height, rotation_to_target = camera_dict[key]['pipeline'].process(captured_img.copy())
                    targets = results[key]['targets']
                    top_camera_dict[key]['targets_entry'].set(targets)  # should see if we can make this one dict to push, may be a one-liner
                    if targets > 0:
                        top_camera_dict[key]['distance_entry'].set(results[key]['distances'][0])
                        top_camera_dict[key]['strafe_entry'].set(results[key]['strafes'][0])
                        top_camera_dict[key]['rotation_entry'].set(results[key]['rotations'][0])
                    else:
                        top_camera_dict[key]['distance_entry'].set(0)
                        top_camera_dict[key]['strafe_entry'].set(0)
                        top_camera_dict[key]['rotation_entry'].set(0)
                ntinst.flush()  # is this necessary?

                # if we are connected to a robot, get its team color.  default to blue
                if topcam_success_counter % 30 == 0:  # check every few seconds for a camera selection update
                    training = training_topic_subscriber.get()  # update if the dash has selected training mode
                    debug = debug_topic_subscriber.get()
                    top_frames.set(topcam_success_counter)
                    if ntinst.getTable('SmartDashboard').getEntry('StationNumber').getInteger(0) == 1:
                        pass
                    # print(f'At frame {success_counter} team key is {team_key}')
                if server_dict['armcam']:
                    image_source[0].putFrame(top_pipeline.image)  # feeds the Http camera with a new image
                topcam_success_counter += 1

            else:  # keep track of failures to read the image from the sink
                failure_counter += 1
            if topcam_success_counter % 30 == 0:
                now = time.time()
                # update the console - most FPS issues are with auto-exposure or if exposure time is too long for FPS setting
                # the cameras seem to cut FPS down automatically by integer divisors - e.g. max 30 --> max 15 --> max 7.5
                fps1 = (topcam_success_counter - previous_topcam_counts) / (now - previous_time)
                fps2 = (groundcam_success_counter - previous_groundcam_counts) / (now - previous_time)
                print(f'Cameras: {len(cameras)}  Avg Ball FPS: {fps1:0.1f} Avg Shoot FPS: {fps2:0.1f}  Reported:{cameras[0].getActualFPS()}  Success: {topcam_success_counter:8d}  Failure:{failure_counter:3d}', end='\r', flush=True)
                previous_topcam_counts = topcam_success_counter
                previous_groundcam_counts = groundcam_success_counter
                previous_time = now

        process_bottom_cam = False  # do we want to recognize targets on the bottom camera?
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
                if process_bottom_cam:
                    results = bottom_pipeline.process(captured_img2.copy())
                    for key in bottom_pipeline.colors:
                        targets = results[key]['targets']
                        bottom_camera_dict[key]['targets_entry'].set(targets)
                        if targets > 0:
                            bottom_camera_dict[key]['distance_entry'].set(results[key]['distances'][0])
                            bottom_camera_dict[key]['strafe_entry'].set(results[key]['strafes'][0])
                            bottom_camera_dict[key]['rotation_entry'].set(results[key]['rotations'][0])
                        else:
                            bottom_camera_dict[key]['distance_entry'].set(0)
                            bottom_camera_dict[key]['strafe_entry'].set(0)
                            bottom_camera_dict[key]['rotation_entry'].set(0)
                    ntinst.flush()
                    if server_dict['bottomcam']:
                        image_source[1].putFrame(bottom_pipeline.image)
                else: # skip the processing, stream the raw image
                    if server_dict['bottomcam']:
                        image_source[1].putFrame(captured_img2)
                groundcam_success_counter += 1

        if topcam_success_counter % 51 == 0 and save_images:  # save an image every few seconds
            image_counter += 1
            print(f'Writing image {image_counter % 200:03d}...')
            cv2.imwrite(f'{folder}/test_{image_counter%200:03d}.png', captured_img)
            #cv2.imwrite(f'{folder}/test_{image_counter % 200:03d}.png', captured_img2)

        # ----------------------------------------------------
