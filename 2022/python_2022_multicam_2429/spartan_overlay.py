# this is a standalone that should be able to inherit any grip pipeline and decorate it
# intended to be used with the wpi multicamera server and a grip pipeline
# note that GRIP is old so sometimes you have to correct the cv2 return values on some of the auto-generated code
# 2/8/2020 CJH FRC team 2429
# updated 1/20/2022
import time
import math
import cv2
import numpy as np
from grip import GripPipeline  # put your GRIP generated grip.py in the same folder - this is our parent class

from networktables import NetworkTablesInstance
from networktables import NetworkTable


class SpartanOverlay(GripPipeline):
    """Extend the GRIP pipeline for analysis and overlay w/o breaking the pure GRIP output pipeline"""

    def __init__(self, color='yellow', camera='lifecam'):
        super().__init__()
        self.debug = False
        # print(self.__hsv_threshold_hue)  # does not exist?
        # updated the GRIP pipeline to take multiple colors - note, have to change it to unmangle __ variables
        # can override the GRIP parameters here if we need to
        # ToDo: pass the HSV in here so we can set it (config file) instead of hard-coding it? Need color-specific stuff though
        # ToDo: override the filter contours stuff here as well for balls vs vision targets
        self.color = color
        self.camera = camera

        oc = False

        if self.color == 'yellow':  # yellow balls
            self._hsv_threshold_hue = [20, 30]
            self._hsv_threshold_saturation = [128, 255]
            self._hsv_threshold_value = [100, 255]

        elif self.color == 'blue':  # blue balls
            self._hsv_threshold_hue = [114, 136]
            self._hsv_threshold_saturation = [80, 255]
            self._hsv_threshold_value = [80, 255]
            self._filter_contours_solidity = [50.0, 100.0]
            self._filter_contours_box_fill = [50.0, 95.0]
            self._filter_contours_max_ratio = 2.0
            self._filter_contours_min_area = 30.0
            self._filter_contours_max_height = 60

            if oc:
                # OC SPECIFIC WITH THE FILTER AND HARSH LIGHTING
                self._hsv_threshold_hue = [118, 132]
                self._hsv_threshold_saturation = [120, 255]
                self._hsv_threshold_value = [150, 255]
                self.ignore_y = [80, 180]  # above or below this we ignore detections

        elif self.color == 'red':  # red balls
            # can invert to cyan or just add a second range
            # currently grip pipleline is reflecting red around 180, so just use the 0-10 (ish values)
            self._hsv_threshold_hue = [0, 10]  # see comment above
            self._hsv_threshold_saturation = [150, 255]
            self._hsv_threshold_value = [50, 255]
            self._filter_contours_solidity = [50.0, 100.0]
            self._filter_contours_box_fill = [50.0, 95.0]
            self._filter_contours_max_ratio = 2.0
            self._filter_contours_max_height = 60

            if oc:
                # OC SPECIFIC WITH THE FILTER AND HARSH LIGHTING
                self._hsv_threshold_hue = [165, 180]
                self._hsv_threshold_saturation = [120, 255]
                self._hsv_threshold_value = [160, 255]
                self.ignore_y = [80, 180]  # above or below this we ignore detections

        elif self.color == 'green':  # vision targets
            self._hsv_threshold_hue = [80, 87]  # verified with lifecam 20220305 on training images
            self._hsv_threshold_saturation = [130, 255]  # retroreflectors tough to get low sat so this removes lights
            self._hsv_threshold_value = [150, 255]
            # in 2022 they are long and flat, so w/h >> 1.  small too.
            self._filter_contours_min_ratio = 0.5
            self._filter_contours_max_ratio = 6
            self._filter_contours_min_area = 10.0
            self._filter_contours_min_width = 3
            self._filter_contours_max_width = 40
            self._filter_contours_min_height = 3
            self._filter_contours_max_height = 25
            self.ignore_y = [0, 200]  # above or below this we ignore detections
        else:
            pass

        self.image = None
        self.original_image = None
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
        # self.filter_contours_output = sorted(self.filter_contours_output, key=key, reverse=reverse)[:5]

    ## AVR obnoxious green light
    def avr_green_correction(self):
        label = ''
        if self.color == 'green' and len(self.filter_contours_output) > 1:
            max_distance_x = 40
            max_distance_y = 20
            # remove x outlier on the left
            self.bounding_box_sort_contours(method='left-to-right')
            spacing = self.bounding_boxes[1][0] - self.bounding_boxes[0][0]
            if spacing > max_distance_x:
                label += f'X left limit violated: {spacing}'
                # remove the first contour
                self.filter_contours_output = self.filter_contours_output[1:]

            # remove y outlier on the bottom
            if len(self.filter_contours_output) > 1:
                self.bounding_box_sort_contours(method='bottom-up')
                spacing = self.bounding_boxes[0][1] - self.bounding_boxes[1][1]
                if spacing > max_distance_y:
                    label += f'Y lower limit violated: {spacing}'
                    self.filter_contours_output = self.filter_contours_output[1:]

            # possibly remove outlier on right
            if len(self.filter_contours_output) > 1:
                self.bounding_box_sort_contours(method='right-to-left')
                spacing = abs(self.bounding_boxes[0][0] - self.bounding_boxes[1][0])  # abs for safety
                if spacing > max_distance_x:
                    label += f'X right limit violated: {spacing}'
                    # remove the first contour
                    self.filter_contours_output = self.filter_contours_output[1:]

            # possibly remove outlier on top
            if len(self.filter_contours_output) > 1:
                self.bounding_box_sort_contours(method='top-down')
                spacing = self.bounding_boxes[1][1] - self.bounding_boxes[0][1]
                if spacing > max_distance_y:
                    label += f'Y upper limit violated: {spacing}'
                    self.filter_contours_output = self.filter_contours_output[1:]
        return label

    def get_target_attributes(self):
        """
        Figure out haw far away targets are
        Use the camera FoV and rescale to distance knowing the object's actual size
        Use the camera resolution to rescale to a xy coordinate system with origin at image center
        """
        self.distance_to_target = 0
        self.rotation_to_target = 0
        self.strafe_to_target = 0
        self.aspect_ratio = 0
        self.height = 0

        # object parameters
        if self.color == 'yellow':
            object_width = 7 * 0.0254  # 2020 squishy yellow ball, 7 inches to meters
        elif self.color == 'blue' or self.color == 'red':
            object_width = 9.5 * 0.0254  # 2022 big tennis ball, 9.5 inches to meters
        elif self.color == 'green':
            # ToDo: have to update this to do the distance differently - see shooter from 2019 maybe
            object_width = 9.5 * 0.0254  # 2022 big tennis ball, 9.5 inches to meters
        else:
            object_width = 9.5 * 0.0254  # default for now

        # camera specific parameters
        self.camera_shift = 0  # when the cameras are flawed (center of camera bore axis not center of image)
        # need to do extra math if camera bore at an angle to the object
        camera_height = 33  # camera vertical distance above ground, use when cam looking at objects on ground
        if self.camera == 'lifecam':
            camera_fov = 55  # Lifecam 320x240
        elif self.camera == 'geniuscam':
            camera_fov = 118  # Genius 120 352x288
            self.camera_shift = 14  # had one at 14 pixels, another at -8 - apparently genius cams have poor QC
        elif self.camera == 'c270':
            camera_fov = 59  # Logitech C290 432x240
        elif self.camera == 'elp100':
            camera_fov = 100  # little elp

        if self.color == 'green':
            hub_x, hub_y, left_boundary, right_boundary = self.find_centers(self.filter_contours_output)
            target_x = (-1.0 + 2.0 * hub_x / self.x_resolution)  # could also be (x+w/2) / self.x_resolution
            self.rotation_to_target = target_x * camera_fov / 2.0
            # self.distance_to_target = -0.219 + 4.5E-3 * hub_y + 1.89E-5 * hub_y ** 2 # needs a bit more complexity
            self.distance_to_target = 0.833 + 0.0183 * hub_y + -1.26E-4 * hub_y ** 2 + 7.36E-7 * hub_y ** 3  # needs a bit more complexityb_y

            self.strafe_to_target = 0

        else:
            # let's just do the calculations on the closest one for now; we could loop through them all easily enough
            x, y, w, h = self.bounding_boxes[0]  # returned rectangle parameters
            self.aspect_ratio = w / h
            self.height = h
            target_width_fraction_fov = w / self.x_resolution  #

            # X and Y are scaled from -1 to 1 in each direction to help with rotation
            # so distances in these coordinates need to be divided by two to get percentage of fov
            moments = cv2.moments(self.filter_contours_output[0])
            centroid_x = int(-self.camera_shift + (moments["m10"] / moments["m00"]))  # also easier as x + w/2
            centroid_y = int(moments["m01"] / moments["m00"])  # also easier as y +h/2
            target_x = (-1.0 + 2.0 * centroid_x / self.x_resolution)  # could also be (x+w/2) / self.x_resolution
            target_y = (-1.0 + 2.0 * centroid_y / self.y_resolution)
            self.rotation_to_target = target_x * camera_fov / 2.0
            self.distance_to_target = object_width / (
                        2.0 * math.tan(target_width_fraction_fov * math.radians(camera_fov) / 2.0))
            self.strafe_to_target = math.sin(math.radians(self.rotation_to_target)) * self.distance_to_target

    def overlay_bounding_boxes(self):
        """Draw a box around all of our contours with the main one emphasized"""
        if self.color != 'green':
            for ix, contour in enumerate(self.filter_contours_output):
                if ix == 0:
                    color = (0, 255, 0)  # green for our primary target
                    thickness = 2
                else:
                    color = (255, 0, 255)  # magenta for the other bogeys
                    thickness = 1
                rect = cv2.boundingRect(contour)
                x, y, w, h = rect
                # print(rect)
                self.image = cv2.rectangle(self.image, (int(rect[0]), int(rect[1])),
                                           (int(rect[0] + rect[2]), int(rect[1] + rect[3])), color, thickness)
                # test fill percent and solidity
                if self.debug:
                    box_fill, solidity = self.get_solidity(contour)
                    # self.image = cv2.putText(self.image, f'{int(box_fill)}, {int(solidity)}', (int(x), int(y)), 1, 1.5, (0, 255, 255), 1, 1)
                    self.image = cv2.putText(self.image, f'{int(x)}, {int(y)}', (int(x), int(y)), 1, 1.5,
                                             (0, 255, 255), 1, 1)

        elif self.color == 'green':
            for ix, contour in enumerate(self.filter_contours_output):
                color = (0, 0, 255)  # red outline for the green targets
                thickness = 2
                rect = cv2.boundingRect(contour)
                x, y, w, h = rect
                # print(rect)
                self.image = cv2.rectangle(self.image, (int(rect[0]), int(rect[1])),
                                           (int(rect[0] + rect[2]), int(rect[1] + rect[3])), color, thickness)

    def overlay_text(self, show_lines=False):
        """Write our object information to the image"""
        self.end_time = time.time()
        location = 'bottom'
        if location == 'top':
            info_text_location = (int(0.035 * self.x_resolution), 12)
            target_text_location = (int(0.7 * self.x_resolution), 13)
            target_area_text_location = (int(0.02 * self.x_resolution), 27)
            # black bar at top of image
            cv2.rectangle(self.image, (0, int(0.88 * self.y_resolution)), (self.x_resolution, 0), (0, 0, 0), -1)
        else:
            info_text_location = (int(0.035 * self.x_resolution), -15 + self.y_resolution)
            target_text_location = (int(0.7 * self.x_resolution), -13 + self.y_resolution)
            target_area_text_location = (int(0.02 * self.x_resolution), -2 + self.y_resolution)
            # black bar at bottom of image
            cv2.rectangle(self.image, (0, int(0.90 * self.y_resolution)), (self.x_resolution, self.y_resolution),
                          (0, 0, 0), -1)

        info_text_color = (0, 255, 255)
        target_text_color = (255, 255, 0)
        target_warning_color = (20, 20, 255)

        if len(self.filter_contours_output) > 0:  # contours found
            cv2.putText(self.image,
                        f"Dist: {self.distance_to_target:3.2f} Str: {self.strafe_to_target:2.1f} H: {self.height:2.0f} AR: {self.aspect_ratio:1.1f} Rot: {self.rotation_to_target:+2.0f} deg",
                        target_area_text_location, 1, 0.9, target_text_color, 1)
            if show_lines:
                if (self.distance_to_target > 0.5):
                    cv2.putText(self.image, "Targeted", target_text_location, 1, 0.9, target_text_color, 1);
                else:
                    cv2.putText(self.image, "Eaten!", target_text_location, 1, 1.0, target_warning_color, 1);
            # decorations - target lines, boxes, bullseyes, etc
            # TODO - add decorations for when we have a target - needs the camera_shift to do it right
            if show_lines:
                cv2.line(self.image, (int(0.3 * self.x_resolution + self.camera_shift), int(0.77 * self.y_resolution)),
                         (int(0.3 * self.x_resolution + self.camera_shift), int(0.14 * self.y_resolution)), (0, 255, 0),
                         2)
                cv2.line(self.image, (int(0.7 * self.x_resolution + self.camera_shift), int(0.77 * self.y_resolution)),
                         (int(0.7 * self.x_resolution + self.camera_shift), int(0.14 * self.y_resolution)), (0, 255, 0),
                         2)
            if self.debug:
                # display the stats on the main target we found
                x_center, y_center = self.x_resolution // 2, self.y_resolution // 2
                mask = np.ones_like(
                    self.image)  # True everywhere, so this would mask all data (mask=True means ignore the data)
                mask = cv2.drawContours(mask, self.filter_contours_output, 0, (0, 0, 0),
                                        -1)  # False (zero) inside the contour
                mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                                  iterations=2)  # have to cut off the edges a bit (grow the True region)
                t = np.ma.masked_where(mask == 1, cv2.cvtColor(self.original_image,
                                                               cv2.COLOR_BGR2HSV))  # data is valid if mask is False
                hue = t[:, :, 0].compressed();
                sat = t[:, :, 1].compressed();
                val = t[:, :, 2].compressed()  # don't want messages about masked format strings

                if len(hue) > 0:
                    self.image = cv2.putText(self.image, f"hue min max mean: {hue.min()} {hue.max()} {hue.mean():.1f}",
                                             (x_center // 2, 2 * y_center - 30), 1, 0.9, (0, 255, 200), 1)
                    self.image = cv2.putText(self.image, f"sat min max mean: {sat.min()} {sat.max()} {sat.mean():.1f}",
                                             (x_center // 2, 2 * y_center - 20), 1, 0.9, (0, 255, 200), 1)
                    self.image = cv2.putText(self.image, f"val min max mean: {val.min()} {val.max()} {val.mean():.1f}",
                                             (x_center // 2, 2 * y_center - 10), 1, 0.9, (0, 255, 200), 1)
        else:  # no contours
            # decorations - target lines, boxes, bullseyes, etc for when there is no target recognized
            # TODO - add decorations for when we do not have a target
            if show_lines:
                cv2.line(self.image, (int(0.3 * self.x_resolution + self.camera_shift), int(0.77 * self.y_resolution)),
                         (int(0.3 * self.x_resolution + self.camera_shift), int(0.14 * self.y_resolution)),
                         (127, 127, 127), 1)
                cv2.line(self.image, (int(0.7 * self.x_resolution + self.camera_shift), int(0.77 * self.y_resolution)),
                         (int(0.7 * self.x_resolution + self.camera_shift), int(0.14 * self.y_resolution)),
                         (127, 127, 127), 1)

            if self.debug:
                x_center, y_center = self.x_resolution // 2, self.y_resolution // 2
                width, height = 10, 20
                t = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)[y_center - height:y_center + height,
                    x_center - width:x_center + width, :]
                hue = t[:, :, 0];
                sat = t[:, :, 1];
                val = t[:, :, 2]
                cv2.putText(self.image, f"Center HSV: {int(hue.mean()):3d} {int(sat.mean()):3d} {int(val.mean()):3d}",
                            target_area_text_location, 1, 0.9, info_text_color, 1)

        cv2.putText(self.image,
                    f"MS: {1000 * (self.end_time - self.start_time):.1f} {self.color} bogeys: {len(self.filter_contours_output)}",
                    info_text_location, 1, 0.9, info_text_color, 1)

    def overlay_vision_text(self):
        """Write our object information to the image about the shooter"""
        self.end_time = time.time()
        info_text_location = (int(0.035 * self.x_resolution), 12)
        info_text_color = (0, 255, 255)
        target_text_color = (255, 255, 0)
        target_warning_color = (20, 20, 255)
        target_text_location = (int(0.7 * self.x_resolution), 13)
        target_area_text_location = (int(0.02 * self.x_resolution), 27)
        target_dist_text_location = (0.03 * self.x_resolution, self.y_resolution - 20)

        # black bar at top of image
        cv2.rectangle(self.image, (0, 0), (self.x_resolution, int(0.12 * self.y_resolution)), (0, 0, 0), -1)
        if len(self.filter_contours_output) > 0:  # contours found
            cv2.putText(self.image,
                        f"Dist: {self.distance_to_target:3.1f} Str: {self.strafe_to_target:2.1f} H: {self.height:2.0f} Rot: {self.rotation_to_target:+2.0f} deg",
                        target_area_text_location, 1, 0.9, target_text_color, 1)
            if (self.distance_to_target > 0.5):
                cv2.putText(self.image, "MSG 1", target_text_location, 1, 0.9, target_text_color, 1);
            else:
                cv2.putText(self.image, "MSG 2!", target_text_location, 1, 1.0, target_warning_color, 1);
            # decorations - target lines, boxes, bullseyes, etc
            # TODO - add decorations for when we have a target - needs the camera_shift to do it right
            hub_x, hub_y, left_boundary, right_boundary = self.find_centers(self.filter_contours_output)
            im_height, im_width, _ = self.image.shape
            cv2.line(self.image, (hub_x, 20), (hub_x, im_height - 20), (255, 127, 0),
                     2)  # Draw the line: image, start, stop, color, thickness
            cv2.line(self.image, (left_boundary, hub_y), (right_boundary, hub_y), (255, 127, 0),
                     2)  # Draw the line: image, start, stop,

        if self.debug:
            # display the stats on the main target we found
            x_center, y_center = self.x_resolution // 2, self.y_resolution // 2
            mask = np.ones_like(
                self.image)  # True everywhere, so this would mask all data (mask=True means ignore the data)
            mask = cv2.drawContours(mask, self.filter_contours_output, 0, (0, 0, 0),
                                    -1)  # False (zero) inside the contour
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                              iterations=2)  # have to cut off the edges a bit (grow the True region)
            t = np.ma.masked_where(mask == 1, cv2.cvtColor(self.original_image,
                                                           cv2.COLOR_BGR2HSV))  # data is valid if mask is False
            hue = t[:, :, 0].compressed();
            sat = t[:, :, 1].compressed();
            val = t[:, :, 2].compressed()  # don't want messages about masked format strings

            if len(hue) > 0:
                self.image = cv2.putText(self.image, f"hue min max mean: {hue.min()} {hue.max()} {hue.mean():.1f}",
                                         (x_center // 2, 2 * y_center - 30), 1, 0.9, (0, 255, 200), 1)
                self.image = cv2.putText(self.image, f"sat min max mean: {sat.min()} {sat.max()} {sat.mean():.1f}",
                                         (x_center // 2, 2 * y_center - 20), 1, 0.9, (0, 255, 200), 1)
                self.image = cv2.putText(self.image, f"val min max mean: {val.min()} {val.max()} {val.mean():.1f}",
                                         (x_center // 2, 2 * y_center - 10), 1, 0.9, (0, 255, 200), 1)
        else:  # no contours
            # decorations - target lines, boxes, bullseyes, etc for when there is no target recognized
            # TODO - add decorations for when we do not have a target
            cv2.line(self.image, (int(0.3 * self.x_resolution + self.camera_shift), int(0.77 * self.y_resolution)),
                     (int(0.3 * self.x_resolution + self.camera_shift), int(0.14 * self.y_resolution)), (127, 127, 127),
                     1)
            cv2.line(self.image, (int(0.7 * self.x_resolution + self.camera_shift), int(0.77 * self.y_resolution)),
                     (int(0.7 * self.x_resolution + self.camera_shift), int(0.14 * self.y_resolution)), (127, 127, 127),
                     1)

            if self.debug:
                x_center, y_center = self.x_resolution // 2, self.y_resolution // 2
                width, height = 10, 20
                t = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)[y_center - height:y_center + height,
                    x_center - width:x_center + width, :]
                hue = t[:, :, 0];
                sat = t[:, :, 1];
                val = t[:, :, 2]
                cv2.putText(self.image, f"Center HSV: {int(hue.mean()):3d} {int(sat.mean()):3d} {int(val.mean()):3d}",
                            target_area_text_location, 1, 0.9, info_text_color, 1)

        cv2.putText(self.image,
                    f"MS: {1000 * (self.end_time - self.start_time):.1f} {self.color} bogeys: {len(self.filter_contours_output)}",
                    info_text_location, 1, 0.9, info_text_color, 1)

    def get_solidity(self, contour):
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        box_area = w * h  # area of the bounding box
        area = cv2.contourArea(contour)  # area of the contour
        hull = cv2.convexHull(contour)  # area of the hull of the contour
        solidity = 100 * area / cv2.contourArea(hull)  # this measurement came with GRIP, i don't like it
        box_fill = 100 * area / box_area
        return box_fill, solidity

    def find_centers(self, contours):  # group the items
        x_centers = []
        y_centers = []
        left_boundary = 1000
        right_boundary = 0
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            x_centers.append(x + w // 2)
            y_centers.append(y + h // 2)
            left_boundary = x if x < left_boundary else left_boundary
            right_boundary = x + w if x + w > right_boundary else right_boundary
        return int(np.mean(x_centers)), int(np.mean(y_centers)), left_boundary, right_boundary

    def post_to_networktables(self):
        """Send object information to networktables"""
        # let the cameraserver do this, and skip it here
        pass

    def get_network_table(self):
        """Grab the appropriate network table for our team"""
        # let the cameraserver do this, and skip it here
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
        self.original_image = image
        self.y_resolution, self.x_resolution, self.channels = self.image.shape
        self.targets = len(self.filter_contours_output)
        if self.targets > 0:
            self.bounding_box_sort_contours(method=method)
            if self.color == 'green':
                self.avr_green_correction()  # fix the green light on the ceiling and elsewhere
            self.overlay_bounding_boxes()
            self.get_target_attributes()
        if self.color == 'green':
            contrast_method = 'clahe'
            if contrast_method == 'clahe':  # takes about 20ms on pi3b 320x240
                # CLAHE (Contrast Limited Adaptive Histogram Equalization)
                clahe = cv2.createCLAHE(clipLimit=5., tileGridSize=(8, 8))
                lab = cv2.cvtColor(self.image, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
                l, a, b = cv2.split(lab)  # split on 3 different channels
                l2 = clahe.apply(l)  # apply CLAHE to the L-channel
                lab = cv2.merge((l2, a, b))  # merge channels
                self.image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR
            elif contrast_method == 'auto_brightness':  # takes about 25ms on pi3b 320x240
                self.image, a, b = automatic_brightness_and_contrast(self.image, clip_hist_percent=10)
            elif contrast_method == 'add_weighted':  # takes about 5ms on pi3b 320x240 but is just a brightness enhancer
                self.image = cv2.addWeighted(self.image, 2, self.image, 0, 2)
            else:
                pass  # no contrast enhancement

            self.overlay_vision_text()

        else:
            self.overlay_text()
        if post_to_nt:
            self.post_to_networktables()
        # ToDo: return a dictionary, this is getting a bit long
        return self.targets, self.distance_to_target, self.strafe_to_target, self.height, self.rotation_to_target


def convertScale(img, alpha, beta):
    """Add bias and gain to an image with saturation arithmetics. Unlike
    cv2.convertScaleAbs, it does not take an absolute value, which would lead to
    nonsensical results (e.g., a pixel at 44 with alpha = 3 and beta = -210
    becomes 78 with OpenCV, when in fact it should become 0).
    """

    new_img = img * alpha + beta
    new_img[new_img < 0] = 0
    new_img[new_img > 255] = 255
    return new_img.astype(np.uint8)


# Automatic brightness and contrast optimization with optional histogram clipping
def automatic_brightness_and_contrast(image, clip_hist_percent=25):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Calculate grayscale histogram
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
    hist_size = len(hist)

    # Calculate cumulative distribution from the histogram
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index - 1] + float(hist[index]))

    # Locate points to clip
    maximum = accumulator[-1]
    clip_hist_percent *= (maximum / 100.0)
    clip_hist_percent /= 2.0

    # Locate left cut
    minimum_gray = 0
    while accumulator[minimum_gray] < clip_hist_percent:
        minimum_gray += 1

    # Locate right cut
    maximum_gray = hist_size - 1
    while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
        maximum_gray -= 1

    # Calculate alpha and beta values
    alpha = 255 / (maximum_gray - minimum_gray)
    beta = -minimum_gray * alpha

    auto_result = convertScale(image, alpha=alpha, beta=beta)
    return (auto_result, alpha, beta)


if __name__ == "__main__":

    """
    Test things out with just a few images if not using the pipeline  - updated 2022 0124
    Best to use on a computer (not pi) to check the HSV values of objects in front of the computer
    But to really get the targets right, save an image from the web
    Set the color below to yellow, blue, green or red (red is tougher) to test
    """
    import sys

    args = sys.argv[1:]
    colors = ['yellow', 'blue', 'green', 'red']
    if len(args) > 0 and args[0] in colors:  # allow color to be passed from command line
        color = args[0]
    else:  # default color
        color = "green"

    start_time = time.time()
    count = 0
    run_time = 1
    methods = ['top-down', 'bottom-up', 'right-to-left', 'left-to-right', 'size']
    methods = ['size', 'left-to-right']

    sort_test = False  # test the sorting methods of bounding boxes
    if sort_test:
        for method in methods:
            count += 1
            cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            s, im = cam.read()  # captures image - note for processing that it is BGR, not RGB!
            pipeline = SpartanOverlay(color=color)
            pipeline.debug = True
            pipeline.process(image=im, method=method)
            cv2.imshow(f"Test Picture: sorting by {method}", pipeline.image)  # displays captured image
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            cam.release()

    video = True
    if video:  # run continuous video and report HSV on objects found
        start_time = time.time()
        end_time = start_time
        pipeline = SpartanOverlay(color=color)
        pipeline.debug = True
        cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        np.set_printoptions(precision=1)
        while end_time - start_time < 2:  # take video for x seconds
            count += 1
            s, im = cam.read()  # captures image - note for processing that it is BGR, not RGB!
            if s > 0:  # Found an image
                pipeline.process(image=im, method='size')
                x_center, y_center = pipeline.x_resolution // 2, pipeline.y_resolution // 2
                if pipeline.targets < 1:  # get some diagnostics on a box if nothing is recognized
                    width, height = 10, 20
                    t = cv2.cvtColor(pipeline.image, cv2.COLOR_BGR2HSV)[y_center - height:y_center + height,
                        x_center - width:x_center + width, :]
                    pipeline.image[:, :,
                    0] = pipeline.hsv_threshold_output  # make a color cast to show what is thresholded (maybe should draw the contour instead)
                    pipeline.image = cv2.rectangle(pipeline.image, (x_center - width, y_center - height),
                                                   (x_center + width, y_center + height), (0, 255, 255))
                    hue = t[:, :, 0];
                    sat = t[:, :, 1];
                    val = t[:, :, 2]
                else:  # display the stats on the main target we found
                    mask = np.ones_like(
                        pipeline.image)  # True everywhere, so this would mask all data (mask=True means ignore the data)
                    mask = cv2.drawContours(mask, pipeline.filter_contours_output, 0, (0, 0, 0),
                                            -1)  # False (zero) inside the contour
                    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                                      iterations=5)  # have to cut off the edges a bit (grow the True region)
                    t = np.ma.masked_where(mask == 1, cv2.cvtColor(pipeline.original_image,
                                                                   cv2.COLOR_BGR2HSV))  # data is valid if mask is False
                    hue = t[:, :, 0].compressed();
                    sat = t[:, :, 1].compressed();
                    val = t[:, :, 2].compressed()  # don't want messages about masked format strings

                if len(hue) > 0:
                    pipeline.image = cv2.putText(pipeline.image,
                                                 f"hue min max mean: {hue.min()} {hue.max()} {hue.mean():.1f}",
                                                 (x_center, 2 * y_center - 30), 1, 0.9, (0, 255, 200), 1)
                    pipeline.image = cv2.putText(pipeline.image,
                                                 f"sat min max mean: {sat.min()} {sat.max()} {sat.mean():.1f}",
                                                 (x_center, 2 * y_center - 20), 1, 0.9, (0, 255, 200), 1)
                    pipeline.image = cv2.putText(pipeline.image,
                                                 f"val min max mean: {val.min()} {val.max()} {val.mean():.1f}",
                                                 (x_center, 2 * y_center - 10), 1, 0.9, (0, 255, 200), 1)
                else:
                    pipeline.image = cv2.putText(pipeline.image, f"Detected region too small?",
                                                 (x_center, 2 * y_center - 10), 1, 0.9, (0, 255, 200), 1)

                print(f'mean: {t[:, :, 0].mean()}', end='\r', flush='True')
                cv2.imshow(f"Test Picture: sorting by size on {pipeline.color}",
                           pipeline.image)  # displays captured image
                end_time = time.time()
                cv2.waitKey(delay=100)

        cam.release()
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print(f'mean: {t[:, :, 0].mean():.2f}', end='\n', flush='True')
        # print(f'hist: {np.histogram(t[:, :, 0], bins=90, range=(0,179))}', end='\n', flush='True')
        print(
            f'Captured {count} frames in {end_time - start_time:.1f}s - average fps is {count / (end_time - start_time):.1f}')

        if False:
            import matplotlib.pyplot as plt

            plt.hist([t[:, :, 0].flatten(), t[:, :, 1].flatten(), t[:, :, 2].flatten()], label=['h', 's', 'v'],
                     bins=255)
            plt.legend()
            plt.show()