#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys, os, glob, signal
import subprocess
import threading
import socket
import numpy as np

#from cv2 import imwrite, resize
import cv2

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags, PubSubOptions

from cscore import HttpCamera, CvSource, VideoMode  # 2429 CJH
from spartan_overlay_2025 import SpartanOverlay  # 2429 CJH

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
        print(f'ATTEMPTING TO SET streamConfig ...')
    else:
        print(f'UNABLE TO SET streamConfig ...')


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


# allow camera acquisition and analysis to run in a thread - just give a function a stop flag argument
# then set stop_flag.set() to let it end
stop_flags = []

def run_in_thread(func):
    # decorator to wrap a function to run in a thread -
    def wrapper(*args, **kwargs):
        stop_flag = threading.Event()

        def target_func():
            func(*args, **kwargs, stop_flag=stop_flag)

        thread = threading.Thread(target=target_func, name=func.__name__)
        thread.start()
        stop_flags.append(stop_flag)  # back door to kill any thread i started with the set() event
        return stop_flag
    return wrapper

# decorator to simplify catching CTRL-C in ipython - not implemented yet
def catch_ctrl_c(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except KeyboardInterrupt:
            print("\nCtrl+C trapped! Terminating threads.")
            for stop_flag in stop_flags:
                stop_flag.set()
            time.sleep(0.1)
            sys.exit(0)
    return wrapper

def handle_sigint(signum, frame):
  """Function to execute when the SIGINT signal is received."""
  print("\nSIGINT received. Exiting gracefully...\n")
  for stop_flag in stop_flags:
      stop_flag.set()
      time.sleep(0.2)
  sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, handle_sigint)

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        print(f'COULD NOT READ CONFIG FILE {configFile}')

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi2024")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    # start cameras
    # work around wpilibsuite/allwpilib#5055
    # CameraServer.setSize(CameraServer.kSize160x120)  # seems to be disallowed in 2024
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)


    #  -------   2429 CJH  stuff - add this to get networktables and processed stream  ------------

    # add an image source, should probably read camera[0] to get the resolution.  I think it ignores FPS
    # except for how it reports, and the actual speed is determined by the rate the code puts images to the sink

    # make a folder to keep track of images  TODO - just do this on the driverstation in a notebook? Turn on from dash?
    save_images = False
    if save_images:
        saved_image_counter = 0
        dir_count = len(glob.glob('image*')) + 2
        folder = f'images_{dir_count}'
        try:
            os.mkdir(folder)
        except Exception as e:
            pass

    # determine which host we are on.  this should make this server file work on any of our pis
    host_name = socket.gethostname();
    ip_address = socket.gethostbyname(host_name);
    ip_address = socket.gethostbyname(host_name + ".local")  # trick for the pi?

    # socket hostname never seems to work right - use subprocess instead to get ifconfig output
    ifconfig_output = subprocess.run(['ifconfig'], capture_output=True, text=True).stdout
    if '10.24.29.12' in ifconfig_output:
        ip_address = '10.24.29.12'  # ringcam / back tagcam
    elif '10.24.29.13' in ifconfig_output:
        ip_address = '10.24.29.13'  # front tagcam
    else:
        ip_address = None
    print(f'Starting camera servers on {host_name} at {ip_address}')

    if ip_address == "10.24.29.12":  # this pi sees front ringcam and back tagcam
        cd = {0: {'name': 'c920', 'processed_port': 1186, 'stream_label': 'Tagcam', 'table': None, 'table_name': "Cameras/Tagcam", 'enabled': True,
                    'camera': cameras[0], 'image_source': None, 'cvstream': None, 'x_resolution': 0, 'y_resolution': 0, 'sink': None,
                    'find_tags': True, 'find_colors': False, 'front_cam': False, 'colors': ['orange'], 'target_results': {'orange': {}, 'tags': {}},
                    'pipeline': None, 'stream_fps': 16, 'stream_max_width': 640},
              1: {'name':'lifecam', 'processed_port': 1187, 'stream_label': 'Ringcam', 'table': None, 'table_name': "Cameras/Ringcam", 'enabled': True,
                    'camera': cameras[1], 'image_source': None, 'cvstream': None, 'x_resolution': 0, 'y_resolution': 0, 'sink': None,
                    'find_tags': False, 'find_colors': True, 'front_cam': False, 'colors': ['orange'], 'target_results': {'orange': {}, 'tags': {}},
                    'pipeline': None, 'stream_fps': 11, 'stream_max_width': 640},  # we watch ringcam during the match - must not stream too much
              }
    elif ip_address == "10.24.29.13":  # this pi sees the front tagcam
        cd = {0: {'name': 'c920', 'processed_port': 1186, 'stream_label': 'Tagcam', 'table': None,
                  'table_name': "Cameras/TagcamFront", 'enabled': True,
                  'camera': cameras[0], 'image_source': None, 'cvstream': None, 'x_resolution': 0, 'y_resolution': 0,
                  'sink': None,
                  'find_tags': True, 'find_colors': False, 'front_cam': True, 'colors': ['orange'],
                  'target_results': {'orange': {}, 'tags': {}},
                  'pipeline': None, 'stream_fps': 16, 'stream_max_width': 640},
              }
    else:
        # should I do this or just go for the default?
        # can I make the pi play a warning instead, like a beep, or do soemting with the LED?
        raise ValueError(f'host {ip_address} not in allowed range of 10.24.29.[12, 13]')

    # set up streams using config dictionary above
    print(f"Attempting to start cvstreams using camera keys: {list(cd.keys())}")
    for cam in cd.keys():
        vm = cd[cam]['camera'].getVideoMode()
        cd[cam]['x_resolution'] = vm.width
        cd[cam]['y_resolution'] = vm.height
        stream_fps = 20
        cd[cam]['image_source'] = CvSource(f"{cd[cam]['stream_label']} CV Image Source", VideoMode.PixelFormat.kMJPEG, cd[cam]['x_resolution'], cd[cam]['y_resolution'], stream_fps)
        # start a stream
        cd[cam]['cvstream'] = MjpegServer(f"{cd[cam]['stream_label']} CV Image Stream", cd[cam]['processed_port'])
        if cd[cam]['x_resolution'] > 300:  # just take the default for smaller formats
            cd[cam]['cvstream'].getProperty("compression").set(30)  # I think this screws up the incoming and outgoing images?  Need to check.
        cd[cam]['cvstream'].setSource(cd[cam]['image_source'])  # now the stream is updated by image_source
        cs = CameraServer  #  .getInstance()
        cs.addCamera(cd[cam]['image_source'])  # is this really necessary?  we add another one later
        print(f"*** Starting 2429 {cd[cam]['stream_label']} Processed stream on {cd[cam]['processed_port']} at {cd[cam]['x_resolution']}x{cd[cam]['y_resolution']} ***")
        cd[cam]['http_camera'] = HttpCamera(f"{cd[cam]['stream_label']} Processed",
                              f"http://127.0.0.1:{cd[cam]['processed_port']}/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer)
        cs.addCamera(cd[cam]['http_camera'])  # how does this know we want cvStream assigned to the web?


    # WHY THIS IS NECESSARY - IF YOU SET ABSOLUTE EXPOSURE IT DOES NOT ACTUALLY UPDATE, SO TOUCH THE BRIGHTNESS AND IT DOES
    # geniuscam seems to work, but mainly because you can't tell it anything.  lifecam and maybe logitech needs brightness updated
    # re-set camera parameters - may jump start them a bit
    for cam_idx in cd.keys():  # will be [0, 1] if we have two cameras, etc
        bright_list = [item.config["brightness"] for item in cameraConfigs]  # gets all the brightnesses, could instead just get the one at this index
        cd[cam_idx]['camera'].setBrightness(bright_list[cam_idx]+1)  # seems to be a bug in 2023 code - setting brightness fixes exposure issues on boot
        time.sleep(0.25)
        print(f'Resetting brightness on cam {cam_idx} to {bright_list[cam_idx]}')
        cd[cam_idx]['camera'].setBrightness(bright_list[cam_idx])
    else:
        pass

    #  ----------------  start a vision thread (CJH)  --------------------
    # TODO - turn this into a thread running in the background

    # set up our network tables
    cam_table = ntinst.getTable("Cameras")
    training = False  # for pipeline.process - are we in training mode
    debug = False # for pipeline process - do we want to see details on contours

    # set up the debug and training calls in the network tables
    training_topic_publisher = cam_table.getBooleanTopic('_training').publish()  # is it really this annoying?
    training_topic_publisher.set(False)
    training_topic_subscriber = cam_table.getBooleanTopic('_training').subscribe(False)

    debug_topic_publisher = cam_table.getBooleanTopic('_debug').publish()  # is it really this annoying?
    debug_topic_publisher.set(False)
    debug_topic_subscriber = cam_table.getBooleanTopic('_debug').subscribe(False)
    timestamp_subscriber = ntinst.getDoubleTopic('/SmartDashboard/_timestamp').subscribe(0)  # comes from robot
    training_color = cam_table.getStringTopic('_training_color').publish()


    for cam in cd.keys():  # set up everything you need in a camera
        cd[cam]['table'] = ntinst.getTable(cd[cam]['table_name'])
        cd[cam]['timestamp_entry'] = cd[cam]['table'].getDoubleTopic("_timestamp").publish()
        cd[cam]['timestamp_entry'].set(0)

       # keep track of pi(?) disconnections by incrementing this counter every time we connect to the NT server
        cd[cam]['connections_publisher'] = cd[cam]['table'].getDoubleTopic("_connections").publish(PubSubOptions(keepDuplicates=True))
        cd[cam]['connections_subscriber'] = cd[cam]['table'].getDoubleTopic("_connections").subscribe(0)
        time.sleep(0.25)  # without this pause the connections will not update - 100ms apparently not enough time for them to guarantee an update
        current_connections = cd[cam]['connections_subscriber'].get()
        cd[cam]['connections_publisher'].set(current_connections + 1)  # TODO: note, if the camera is unplugged this will not increment
        # TODO - maybe I should set the timestamp back to zero if there is no image received from the camera for x seconds?

        for key in cd[cam]['target_results'].keys():
            cd[cam]['target_results'][key].update({'id': cd[cam]['table'].getDoubleTopic(f"{key}/id").publish()})
            cd[cam]['target_results'][key].update({'targets_entry': cd[cam]['table'].getDoubleTopic(f"{key}/targets").publish()})
            cd[cam]['target_results'][key].update({'distance_entry': cd[cam]['table'].getDoubleTopic(f"{key}/distance").publish()})
            cd[cam]['target_results'][key].update({'rotation_entry': cd[cam]['table'].getDoubleTopic(f"{key}/rotation").publish()})
            cd[cam]['target_results'][key].update({'strafe_entry': cd[cam]['table'].getDoubleTopic(f"{key}/strafe").publish()})
        cd[cam]['tag_entries'] = [cd[cam]['table'].getDoubleArrayTopic(f"poses/tag1").publish(), cd[cam]['table'].getDoubleArrayTopic(f"poses/tag2").publish()]
        cd[cam]['frame_entry'] = cd[cam]['table'].getDoubleTopic("_frames").publish()
        cd[cam]['colors_entry'] = cd[cam]['table'].getStringArrayTopic("colors").publish()
        cd[cam]['colors_entry'].set(cd[cam]['colors'])
        actual_colors = [key for key in cd[cam]['target_results'].keys() if key != 'tags']
        cd[cam]['pipeline'] = SpartanOverlay(colors=actual_colors, camera=cd[cam]['name'])

        cs = CameraServer
        cd[cam]['sink'] = cs.getVideo(camera=cd[cam]['camera'])
        cd[cam]['img'] = np.zeros((cd[cam]['y_resolution'], cd[cam]['x_resolution'], 3))
        cd[cam]['success_counter'] = 0
        cd[cam]['previous_counts'] = 0
        cd[cam]['frame_counts'] = 0
        cd[cam]['fps'] = 0
        cd[cam]['previous_time'] = 0
        cd[cam]['previous_image_send'] = 0
        cd[cam]['failure_counter'] = 0
        cd[cam]['reduce_bandwidth'] = True

    # set up a pipeline for each camera

    failure_counter = 0  # very first image often fails, and after that we are solid
    camera_names = [cd[cam]['name'] for cam in cd.keys()]
    print(f'Entering image process loop with {camera_names}...', flush=True)
    loop_count = 0
    previous_time = 0

    # set up threads here - so cameras can run in parallel instead for waiting for each other to finish

    def update_stream(cam):
        global training, debug, failure_counter, saved_image_counter
        if cd[cam]['enabled']:
            # get the basecam images
            image_time, captured_img = cd[cam]['sink'].grabFrame(cd[cam]['img'])  # default time out is about 4 FPS
            if image_time > 0:  # actually got an image
                # print(f'image acquired on {cd[cam]["name"]}', flush='True')
                results, tags = cd[cam]['pipeline'].process(captured_img.copy(), method='size', training=training,
                                                            debug=debug, find_tags=cd[cam]['find_tags'],
                                                            find_colors=cd[cam]['find_colors'], front_cam=cd[cam]['front_cam'])
                for key in cd[cam]['target_results'].keys():  # doing all colors and tags !
                    # targets, distance_to_target, strafe_to_target, height, rotation_to_target = camera_dict[key]['pipeline'].process(captured_img.copy())
                    targets = results[key]['targets']
                    cd[cam]['target_results'][key]['targets_entry'].set(
                        targets)  # should see if we can make this one dict to push, may be a one-liner
                    if targets > 0:
                        cd[cam]['target_results'][key]['id'].set(results[key]['ids'][0])
                        cd[cam]['target_results'][key]['distance_entry'].set(results[key]['distances'][0])
                        cd[cam]['target_results'][key]['strafe_entry'].set(results[key]['strafes'][0])
                        cd[cam]['target_results'][key]['rotation_entry'].set(results[key]['rotations'][0])
                    else:
                        cd[cam]['target_results'][key]['id'].set(0)
                        cd[cam]['target_results'][key]['distance_entry'].set(0)
                        cd[cam]['target_results'][key]['strafe_entry'].set(0)
                        cd[cam]['target_results'][key]['rotation_entry'].set(0)

                # put the tag poses an info to NT
                ts = timestamp_subscriber.get()  # find out what time it is
                cd[cam]['timestamp_entry'].set(ts)  # show camera is alive even if no data returned

                if len(tags) > 0:
                    #  {'id': tag.getId(), 'rotation': pose.rotation().x,
                    #   'distance': pose.z, 'tx': tx, 'ty': ty, 'tz': tz, 'rx': rx, 'ry': ry, 'rz': rz}})
                    for idx, key in enumerate(tags.keys()):
                        data = [ts, tags[key]['id'], tags[key]['tx'], tags[key]['ty'], tags[key]['tz'],
                                tags[key]['rx'], tags[key]['ry'], tags[key]['rz']]
                        try:
                            cd[cam]['tag_entries'][idx].set(data)
                        except IndexError:
                            pass
                    if idx < 1:
                        cd[cam]['tag_entries'][1].set([ts] + [0] * 7)
                else:
                    for tag_entry in cd[cam]['tag_entries']:
                        tag_entry.set([ts] + [0] * 7)
                ntinst.flush()  # is this necessary?

                # cut down on streaming bandwidth - hard to figure out how to just set the stream to do it for you
                now = time.time()
                max_width = cd[cam]['stream_max_width']
                if now - cd[cam]['previous_image_send'] > 1 / cd[cam]['stream_fps']:  # ok to send an image
                    if cd[cam]['reduce_bandwidth'] and cd[cam]['pipeline'].image.shape[1] > max_width:
                        height, width = cd[cam]['pipeline'].image.shape[:2]
                        image = cv2.resize(cd[cam]['pipeline'].image, (int(max_width), int(height * max_width / width)))
                        cd[cam]['image_source'].putFrame(image)
                    else:
                        cd[cam]['image_source'].putFrame(
                            cd[cam]['pipeline'].image)  # feeds the Http camera with a new image
                    cd[cam]['previous_image_send'] = now  # resets the clock for sending frames

                if cd[cam]['success_counter'] % 30 == 0:  # check every few seconds for a camera selection update
                    training = training_topic_subscriber.get()  # update if the dash has selected training mode
                    debug = debug_topic_subscriber.get()
                    cd[cam]['frame_entry'].set(cd[cam]['success_counter'])
                    # update the console - most FPS issues are with auto-exposure or if exposure time is too long for FPS setting
                    # the cameras seem to cut FPS down automatically by integer divisors - e.g. max 30 --> max 15 --> max 7.5
                    cd[cam]['fps'] = (cd[cam]['success_counter'] - cd[cam]['previous_counts']) / (now - cd[cam]['previous_time'])
                    cd[cam]['previous_counts'] = cd[cam]['success_counter']
                    cd[cam]['previous_time'] = now

                    if save_images:
                        saved_image_counter += 1
                        print(f'Writing image {saved_image_counter % 200:03d}...')
                        cv2.imwrite(f'{folder}/test_{cam}{saved_image_counter % 200:03d}.png', captured_img)

                cd[cam]['success_counter'] += 1

            else:  # failed to acquire an image
                cd[cam]['failure_counter'] += 1
                failure_counter += 1  # any camera failure adds to the one on the while loop
                if failure_counter % 10 == 0:
                    print(
                        f"failures: {failure_counter} {cd[0]['failure_counter']}/{cd[1]['failure_counter']}{' ' * 40}",
                        flush=True)
                # cd[cam]['image_source'].putFrame(cd[cam]['img'])
        else:  # camera not enabled
            pass

    @run_in_thread
    def stream_loop(cam, stop_flag):
        while not stop_flag.is_set():
            update_stream(cam)

    # start the camera threads
    for cam in cd.keys():
        stream_loop(cam)

    while True and failure_counter < 100:

        try:
            # process each camera separately
            loop_count += 1
            time.sleep(1/30)
            # print(f'Attempting a capture cycle {loop_count}... ', end='\r', flush=True)

            if loop_count % 30 == 0:
                # report stats once per second
                ts = time.time()
                cam_fps = [cd[cam]['fps'] for cam in cd.keys()]
                fails = [cd[cam]['failure_counter'] for cam in cd.keys()]
                successes = [cd[cam]['success_counter'] for cam in cd.keys()]
                names = [cd[cam]['name'] for cam in cd.keys()]
                # TODO: make this work for arbitrary number of cameras
                if len(cd.keys()) == 2:
                    msg = f"Cameras: {len(cameras)}  Avg {names[0]} FPS: {cam_fps[0]:0.1f} Avg {names[1]} FPS: {cam_fps[1]:0.1f}"
                    msg += f" Success:{successes[0]}/{successes[1]}  Failure:{fails[0]}/{fails[1]}"
                else:
                    msg = f"Cameras: {len(cameras)}  Avg {names[0]} FPS: {cam_fps[0]:0.1f}"
                    msg += f" Success:{successes[0]}  Failure:{fails[0]}"
                print(msg, end='\r', flush=True)
                previous_time = ts
        except KeyboardInterrupt:  # not necessary since I'm trapping SIGNINT above
            print("\nCtrl+C trapped! Terminating threads and ending ...")
            for stop_flag in stop_flags:
                stop_flag.set()
                time.sleep(0.2)
            sys.exit(0)

        # ----------------------------------------------------
