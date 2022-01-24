01/20/2022 - Notes on getting the default multicamera py to work with our streaming camera
Basically added the following at the top:

from cscore import HttpCamera, CvSource, VideoMode  # CJH
from spartan_overlay import SpartanOverlay # CJH


And then set up our pipeline down below
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
    if x_resolution > 300:
        cvStream.getProperty("compression").set(60)
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