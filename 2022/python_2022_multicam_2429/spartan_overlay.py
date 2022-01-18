# this is a standalone that should be able to inherit any grip pipeline and decorate it
# intended to be used with the wpi multicamera server and a grip pipeline
# note that GRIP is old so sometimes you have to correct the cv2 return values on some of the auto-generated code
# 2/8/2020 CJH FRC team 2429
from grip import GripPipeline  #put your GRIP generated grip.py in the same folder
import cv2
import time
import math

from networktables import NetworkTablesInstance
from networktables import NetworkTable

class SpartanOverlay(GripPipeline):
    """Extend the GRIP pipeline for analysis and overlay w/o breaking the pure GRIP output pipeline"""
    def __init__(self):
        super().__init__()
        # can override the GRIP parameters here if we need to


        self.image = None
        self.start_time = 0
        self.end_time = 0
        self.camera_shift = 0

        # define what we send back at the end of the pipeline
        self.targets = 0
        self.distance_to_target = 0
        self.strafe_to_target = 0
        self.height = 0
        self.rotation_to_target = 0


    def bounding_box_sort_contours(self, method='size'):
        """Get sorted contours and bounding boxes from our list of filtered contours
            This can be very simple if all we wanted was size

            :param method: one of [size, left-to-right, right-to-left, top-down or bottom-up]
            :return: None but sets self.filter_contours_output and self.bounding_boxes
            """
        # method for the sort at https://www.pyimagesearch.com/2015/04/20/sorting-contours-using-python-and-opencv/
        for ix, contour in enumerate(self.filter_contours_output):
            pass
           # x, y, w, h = cv2.boundingRect(contour)
           # print(f"Contour {ix} has area {cv2.contourArea(contour)} with width {w} and height {h} at location ({x},{y})")

        # *** Give ourselves some options on sorting, defaulting to size (closest) ***
        # if sorting by location, figure out if sorting by x or y coordinate
        if method == "top-down" or method == "bottom-up":
            axis = 1
        else:
            axis = 0
        # below in zip sort we will sort on the axis (x or y) of second list in the zip (the bounding box)
        key = lambda b: b[1][axis]
        # handle if we need to sort in reverse, change sorting key for size
        if method == 'left-to-right' or method == 'top-down':
            reverse = False
        elif method == 'right-to-left' or method == 'bottom-up':
            reverse = True
        else:  # sort by size
            key = lambda b: cv2.contourArea(b[0])
            reverse = True

        # could easily append a max_contours parameter here and limit to [:n] choices
        # we want the bounding boxes later anyway for
        bounding_boxes = [cv2.boundingRect(c) for c in self.filter_contours_output]
        # neat use of zip(*foo) to to give you back the lists you zipped to sort
        (self.filter_contours_output, self.bounding_boxes) = \
            zip(*sorted(zip(self.filter_contours_output, bounding_boxes), key=key, reverse=reverse))

        # if you only wanted size it would be really simple - just do key = cv.contourArea and reverse = True w/o a zip
        #self.filter_contours_output = sorted(self.filter_contours_output, key=key, reverse=reverse)[:5]

    def get_target_attributes(self, camera='lifecam'):
        self.distance_to_target = 0
        self.rotation_to_target = 0
        self.strafe_to_target = 0
        self.aspect_ratio = 0
        self.height = 0

        # object parameters
        object_height = 7  # 2020 squishy yellow ball
        object_width = 7  # 2020 squishy yellow ball

        # camera specific parameters
        self.camera_shift = 0  # when the cameras are flawed (center of camera bore axis not center of image)
        # need to do extra math if camera bore at an angle to the object
        camera_height = 33  # camera vertical distance above ground, use when cam looking at objects on ground
        if camera =='lifecam':
            camera_fov = 55  # Lifecam 320x240
        elif camera =='geniuscam':
            camera_fov = 118  # Genius 120 352x288
            self.camera_shift = -8  # had one at 14 pixels, another at -8 - apparently genius cams have poor QC
        elif camera == 'c270':
            camera_fov = 59  # Logitech C290 432x240

        # let's just do the calculations on the closest one for now; we could loop through them all easily enough
        x, y, w, h = self.bounding_boxes[0]
        self.aspect_ratio = h/w
        self.height = h
        target_width_fraction_fov = w / self.x_resolution

        # X and Y are scaled from -1 to 1 in each direction to help with rotation
        # so distances in these coordinates need to be divided by two to get percentage of fov
        moments = cv2.moments(self.filter_contours_output[0])
        centroid_x = int(-self.camera_shift + (moments["m10"] / moments["m00"]))
        centroid_y = int(moments["m01"] / moments["m00"])
        target_x = (-1.0 + 2.0*centroid_x/self.x_resolution)  # could also be (x+w/2) / self.x_resolution
        target_y = (-1.0 + 2.0*centroid_y/self.y_resolution)
        self.rotation_to_target = target_x * camera_fov / 2.0
        self.distance_to_target = object_width / (2.0 * math.tan(target_width_fraction_fov * math.radians(camera_fov) / 2.0))
        self.strafe_to_target = math.sin(math.radians(self.rotation_to_target)) * self.distance_to_target

    def overlay_bounding_boxes(self):
        """Draw a box around all of our contours with the main one emphasized"""
        for ix, contour in enumerate(self.filter_contours_output):
            if ix == 0:
                color = (255, 0, 0)  # blue for our primary target
                thickness = 2
            else:
                color = (0, 255, 0) # green for the other bogeys
                thickness = 1
            rect = cv2.boundingRect(contour)
            #print(rect)
            self.image = cv2.rectangle(self.image, (int(rect[0]), int(rect[1])), (int(rect[0] + rect[2]), int(rect[1] + rect[3])), color, thickness)
        pass

    def overlay_text(self):
        """Write our object information to the image"""
        self.end_time = time.time()
        info_text_location = (int(0.035*self.x_resolution), 12)
        info_text_color = (0, 255, 255)
        target_text_color = (255, 255, 0)
        target_warning_color = (20, 20, 255)
        target_text_location = (int(0.7 * self.x_resolution), 13)
        target_area_text_location = (int(0.02 * self.x_resolution), 27)
        target_dist_text_location = (0.03 * self.x_resolution, self.y_resolution - 20)

        # black bar at top of image
        cv2.rectangle(self.image, (0, 0), (self.x_resolution, int(0.12 * self.y_resolution)), (0, 0, 0), -1)
        if len(self.filter_contours_output) > 0:  #  contours found
            cv2.putText(self.image, f"Dist: {self.distance_to_target:3.0f} Str: {self.strafe_to_target:2.1f} H: {self.height:2.0f} AR: {self.aspect_ratio:1.1f} Rot: {self.rotation_to_target:+2.0f} deg", target_area_text_location, 1, 0.9, target_text_color, 1)
            if (self.distance_to_target > 10):
                cv2.putText(self.image, "Targeted", target_text_location, 1, 0.9, target_text_color, 1);
            else:
                cv2.putText(self.image, "Eaten!", target_text_location, 1, 1.0, target_warning_color, 1);
            # decorations - target lines, boxes, bullseyes, etc
            # TODO - add decorations for when we have a target - needs the camera_shift to do it right
            cv2.line(self.image, (int(0.3*self.x_resolution + self.camera_shift), int(0.77*self.y_resolution)), (int(0.3*self.x_resolution + self.camera_shift), int(0.14*self.y_resolution)), (0,255,0), 2)
            cv2.line(self.image, (int(0.7*self.x_resolution + self.camera_shift), int(0.77*self.y_resolution)), (int(0.7*self.x_resolution + self.camera_shift), int(0.14*self.y_resolution)), (0,255,0), 2)
        else:  # no contours
            # decorations - target lines, boxes, bullseyes, etc for when there is no target recognized
            # TODO - add decorations for when we do not have a target
            cv2.line(self.image, (int(0.3*self.x_resolution + self.camera_shift), int(0.77*self.y_resolution)), (int(0.3*self.x_resolution + self.camera_shift), int(0.14*self.y_resolution)), (127,127,127), 1)
            cv2.line(self.image, (int(0.7*self.x_resolution + self.camera_shift), int(0.77*self.y_resolution)), (int(0.7*self.x_resolution + self.camera_shift), int(0.14*self.y_resolution)), (127,127,127), 1)
            pass

        cv2.putText(self.image, f"MS: {1000*(self.end_time - self.start_time):.1f} Bogeys: {len(self.filter_contours_output)}",
                        info_text_location, 1, 0.9, info_text_color, 1)


    def post_to_networktables(self):
        """Send object information to networktables"""
        pass

    def get_network_table(self):
        """Grab the appropriate network table for our team"""
        pass

    def process(self, image, method='size', post_to_nt=True):
        """Run the parent pipeline and then continue to do custom overlays and reporting
           Run this the same way you would the wpilib examples on pipelines
           e.g. call it in the capture section of the camera server
           :param method: sort method [size, left-to-right, right-to-left, top-down or bottom-up]
           :param post_to_nt: whether to post to a networktable named self.table_name
           """

        self.start_time = time.time()
        super(self.__class__, self).process(image)
        # we just processed the incoming image with the parent GRIP pipeline and have our filtered contours.  now sort
        self.image = image
        self.y_resolution, self.x_resolution, self.channels = self.image.shape
        if len(self.filter_contours_output) > 0:
            self.bounding_box_sort_contours(method=method)
            self.overlay_bounding_boxes()
            self.get_target_attributes()
        self.overlay_text()
        if post_to_nt:
            self.post_to_networktables()
        return self.targets, self.distance_to_target, self.strafe_to_target, self.height, self.rotation_to_target


if __name__ == "__main__":
    """Test things out with just a few images"""
    start_time = time.time()
    count = 0
    run_time = 1
    methods = ['top-down', 'bottom-up', 'right-to-left', 'left-to-right', 'size']
    methods = ['size', 'left-to-right']
    for method in methods:
        count += 1
        cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        s, im = cam.read()  # captures image
        pipeline = SpartanOverlay()
        pipeline.process(image=im, method=method)
        cv2.imshow(f"Test Picture: sorting by {method}", pipeline.image)  # displays captured image
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cam.release()

#        if cv2.waitKey(1) & 0xFF == ord('q'):
#            break
    #cv2.waitKey(0)

    print(f"Processed {count} images in {round(time.time()-start_time, 2)} seconds")
