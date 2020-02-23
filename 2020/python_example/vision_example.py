
# vision example from https://robotpy.readthedocs.io/en/stable/vision/code.html

from cscore import CameraServer
from cscore import CvSink, CvSource

# Import OpenCV and NumPy
import cv2
import numpy as np

CameraServer.getInstance().startAutomaticCapture()
cvSink = CameraServer.getInstance().getVideo()
outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

# Capture from the first USB Camera on the system
camera = cs.startAutomaticCapture()
camera.setResolution(320, 240)

# Get a CvSink. This will capture images from the camera
cvSink = cs.getVideo()

# (optional) Setup a CvSource. This will send images back to the Dashboard
outputStream = cs.putVideo("Sample Stream", 320, 240)

# Allocating new images is very expensive, always try to preallocate
img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
target_text_location = (int(0.7 * 320), 13)
target_text_color = (255, 255, 0)

while True:
    # Tell the CvSink to grab a frame from the camera and put it
    # in the source image.  If there is an error notify the output.
    time, img = cvSink.grabFrame(img)
    if time == 0:
        # Send the output the error.
        outputStream.notifyError(cvSink.getError());
        # skip the rest of the current iteration
        continue

    #
    # Insert your image processing logic here!
    #

    # (optional) send some image back to the dashboard
    cv2.rectangle(img, (0, 0), (320, int(0.12 * 240)), (0, 0, 0), -1)
    cv2.putText(img, "Processed!", target_text_location, 1, 0.9, target_text_color, 1);
    outputStream.putFrame(img)