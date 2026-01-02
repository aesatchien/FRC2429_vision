# this started out as a simple HSV segmentation routing but has grown over the years.  needs to be refactored and simplified
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
import robotpy_apriltag as ra
import wpimath.geometry as geo
from wpimath import objectToRobotPose  # 2024 version
from tagmanager import TagManager

from ntcore import NetworkTableInstance
from ntcore import NetworkTable

class SpartanOverlay(GripPipeline):
    """Extend the GRIP pipeline for analysis and overlay w/o breaking the pure GRIP output pipeline"""

    # set up all the tag positions
    field = ra.AprilTagField.k2025ReefscapeWelded
    layout = ra.AprilTagFieldLayout.loadField(field)

    def __init__(self, colors=['orange'], camera='c920', x_resolution=640, y_resolution=360, greyscale=False, intrinsics=None, distortions=False, max_tag_distance=10):
        super().__init__()
        self.debug = False  #  show HSV info as a written overlay
        self.hardcore = True  # if debugging, show even more info on HSV of detected contours
        self.training = False  # mode to reveal object colors for quick training,  need to pass this to process
        self.replace_zeros = True  # seems like purple in particular is susceptible to getting zeros in the image
        # updated the GRIP pipeline to take multiple colors - note, have to change it to unmangle __ variables
        # can override the GRIP parameters here if we need to
        # ToDo: pass the HSV in here so we can set it (config file) instead of hard-coding it? Need color-specific stuff though
        # ToDo: override the filter contours stuff here as well for balls vs vision targets
        self.colors = colors
        self.color = None
        self.camera = camera
        self.image = None
        self.original_image = None
        self.start_time = 0
        self.end_time = 0
        self.camera_shift = 0
        self.x_resolution = x_resolution  # not really, this is just a placeholder until we get an image; I should pass it in
        self.y_resolution = y_resolution
        self.greyscale = greyscale
        self.max_tag_distance = max_tag_distance  # cutoff after which we no longer accept an april tag detection  20250316 CJH
        self.intrinsics = intrinsics
        self.distortions = distortions
        self.tagmanager = TagManager()

        # set up an apriltag detector, and try to get the poses - in 2024 the tag is 6.5" (0.1651m)
        self.detector = ra.AprilTagDetector()
        self.detector.addFamily('tag36h11')
        qt = self.detector.getQuadThresholdParameters()  # 2025 they made this way too big - default is 300 !
        qt.minClusterPixels = 10
        self.detector.setQuadThresholdParameters(qt)
        cfg = self.detector.getConfig()
        cfg.numThreads = 2  # attempt to thread this, hopefully it does not mess with the threaded execution of the images
        self.detector.setConfig(cfg)
        # need to calculate this based on camera and resolution
        print(f'estimating image parameters using {self.camera} at resolution x:{self.x_resolution} y: {self.y_resolution}')
        fx = 1; fy = 1; cx = 1; cy = 1  # IDE complains
        if self.intrinsics is not None:
            print(f'using passed in intrinsics {self.intrinsics} on cam {self.camera}')
            fx = self.intrinsics['fx']
            fy = self.intrinsics['fy']
            cx = self.intrinsics['cx']
            cy = self.intrinsics['cy']
        else:
            if self.camera == 'lifecam':
                fx = 342.3; fy = 335.1; cx = 320 / 2; cy = 240 / 2
            elif self.camera == 'arducam':
                # should be able to calculate this based on my x and y resolution  - TODO - measure on field with laser
                if self.x_resolution == 1280:
                    fx, fy = 905, 905
                elif self.x_resolution == 800:
                    fx, fy = 691, 691
                elif self.x_resolution == 640:
                    #  going lower makes things closer  was 525 for 2.26, 500 for 2.11, 2.50 for 600
                    fx, fy = 590, 590
                else:
                    print(f'Unable to set arducam intrinsics using resolution {self.x_resolution}, {self.y_resolution} ')
                    fx, fy = 1,1
                cx, cy = self.x_resolution // 2, self.y_resolution // 2
            elif self.camera == 'c920':
                if self.x_resolution < 1000:
                    # 3D Zephry did this one
                    # config = ra.AprilTagPoseEstimator.Config(tagSize=0.1651, fx=459.5, fy=459.5, cx=640/2, cy=360/2)  # logitech at 640x360
                    # frontcam AND backcam both work best at 3m with these settings - CJH optimizing in shop on robot 20240401
                    fx = 478; fy = 478; cx = 640 / 2; cy = 360 / 2
                else:
                    fx = 924.4; fy = 924.4; cx = 640; cy = 360
            else:
                pass

        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        config = ra.AprilTagPoseEstimator.Config(tagSize=0.1651, fx=fx, fy=fy, cx=cx, cy=cy)  # catch all
        print(f'double check  {self.camera} - {config.fx} {config.fy} {config.cx} {config.cy}')
        self.estimator = ra.AprilTagPoseEstimator(config)

        # define what we send back at the end of the pipeline
        self.results = {}  # new in 2023
        self.tags = {}  # new in 2024
        self.contours = {}
        for color in self.colors:
            self.results.update({color:{'targets': 0, 'previous_targets': 0, 'ids':[], 'distances': [],
                                        'strafes': [], 'heights': [], 'rotations':[], 'contours':[], 
                                        'bounding_rects': [], 'cx': [], 'cy': []}})
            self.contours.update({color: {'contours': []}})

        self.targets = 0
        self.distance_to_target = 0
        self.strafe_to_target = 0
        self.height = 0
        self.rotation_to_target = 0

    def process(self, image, method='size', draw_overlay=True, reset_hsv=True, training=False,
                skip_overlay=False, debug=False, find_tags=True, find_colors=True, cam_orientation=None, use_distortions=None):
        """Run the parent pipeline and then continue to do custom overlays and reporting
           Run this the same way you would the wpilib examples on pipelines
           e.g. call it in the capture section of the camera server
           e.g. call it in the capture section of the camera serversort
           :param method: sort method [size, left-to-right, right-to-left, top-down or bottom-up]
           :param post_to_nt: whether to post to a networktable named self.table_name
           """

        self.training = training  # boolean to see of we switch to training mode
        self.debug = debug
        self.start_time = time.time()
        self.image = image
        self.original_image = image.copy()  # supposedly expensive, but if we need it later - doesn't seem to cost time
        self.y_resolution, self.x_resolution, self.channels = self.image.shape
        if cam_orientation is not None:
            self.cam_orientation = cam_orientation
        else:  # provide a default with no offsets from center of robot
            self.cam_orientation = {'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz':0}

        # color section
        for idx, color in enumerate(self.colors):
            self.color = color
            self.results.update(
                {color: {'targets': 0, 'previous_targets': self.results[color]['targets'], 'ids': [], 'distances': [],
                         'strafes': [], 'heights': [], 'rotations': [], 'contours': [], 
                         'bounding_rects': [], 'cx': [], 'cy': []}})
            self.contours.update({color: {'contours': []}})

            if self.training and idx == 0:
                self.get_center_hsv()
            elif self.training and idx > 0:
                continue
            elif reset_hsv:
                self.set_hsv()  # otherwise this does not allow us to pass in or set the hsv externally

            if find_colors:
                # maybe be I should rename this as not overloading the parent, it's not pythonic
                super(self.__class__, self).process(self.original_image)
                # update targets for a given color
                targets = len(self.filter_contours_output)
                self.results[self.color]['previous_targets'] = self.results[self.color]['targets']
                self.results[self.color]['targets'] = targets

                if targets > 0:
                    self.bounding_box_sort_contours(method='center')
                    self.contours[self.color]['contours'] = self.filter_contours_output  # TODO - sort these
                    # Pass geometry to results for the overlay drawer
                    self.results[self.color]['contours'] = self.filter_contours_output
                    self.results[self.color]['bounding_rects'] = self.bounding_boxes
                    self.get_target_attributes()  # updates self.results
            else:
                self.filter_contours_output = []

        # tag section
        self.results.update({'tags': {'ids': [], 'targets': 0, 'distances': [], 'strafes': [], 'heights': [], 'rotations': []}})
        if find_tags:
            self.find_apriltags(draw_tags=draw_overlay, use_distortions=use_distortions)

        # Pass training stats if active
        if self.training:
            self.results['training_stats'] = {
                'hue': self.temp_hue, 'sat': self.temp_sat, 'val': self.temp_val
            }

        return self.results, self.tags

    def find_apriltags(self, draw_tags=True, decision_margin=30, use_distortions=False):
        self.tags = {}
        if self.greyscale:  # arducams already in grey, but somehow coming in as not?  TBD if this will help
            pass
        else:
            pass
        grey_image = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(grey_image)
        # can the following be just fed to detector?
        tags = [tag for tag in tags if tag.getDecisionMargin() > decision_margin and tag.getHamming() < 2]
        original_tags = tags.copy()  # keep this for drawing all tags later - even the ones we reject
        # we have a 3D translation and a 3D rotation coming from each detection
        if use_distortions:  # correct for distorted lenses
            temp_corners = [np.array(tag.getCorners([0] * 8)).reshape((-1, 1, 2)).astype(dtype=np.float32) for tag in tags]
            new_corners = [cv2.undistortPoints(corners, self.camera_matrix, self.distortions, None, self.camera_matrix) for corners in temp_corners]
            at_poses = [self.estimator.estimateOrthogonalIteration(tag.getHomography(), new_corner.flatten().tolist(), 50) for tag, new_corner in zip(tags, new_corners)]
        else:
            at_poses = [self.estimator.estimateOrthogonalIteration(tag, 50) for tag in tags]
        ambiguities = [ at_pose.getAmbiguity() for at_pose in at_poses]

        # TODO - move these tags to a "bad tag" list so I can draw them anyway - easy to tell why rejected then
        # cull all tags where ambiguity > 0.2
        indices_to_remove = [i for i, amb in enumerate(ambiguities) if amb > 0.2]
        for index in reversed(indices_to_remove):
            del tags[index]
            del at_poses[index]
            del ambiguities[index]

        # cull all tags where distance > limit imposed for this camera
        indices_to_remove = [i for i, pose in enumerate(at_poses) if pose.pose1.z > self.max_tag_distance]
        for index in reversed(indices_to_remove):
            del tags[index]
            del at_poses[index]
            del ambiguities[index]

        poses = [at_pose.pose1 for at_pose in at_poses]  # only work on the culled ones

        # sort the tags based on their distance from the camera - this comes from the pose
        if len(tags) > 1:
            sorted_lists = sorted(zip(poses, tags), key=lambda x: x[0].translation().z)
            poses, tags = zip(*sorted_lists)

        tag_count = len(tags)
        ids = [tag.getId() for tag in tags]
        distances = [pose.z for pose in poses]
        strafes = [pose.x for pose in poses]
        rotations = [pose.rotation().y_degrees for pose in poses]
        self.results.update({'tags': {'ids':ids, 'targets': tag_count, 'distances': distances, 'strafes': strafes, 'rotations': rotations}})

        # translating poses into field coordinates
        if len(tags) > 0:
            for tag, pose in zip(tags, poses):
                # pose = self.estimator.estimate(tag)  #  already did this above
                pose_camera = geo.Transform3d(geo.Translation3d(pose.x, pose.y, pose.z),
                    geo.Rotation3d(-pose.rotation().x - np.pi, -pose.rotation().y, pose.rotation().z - np.pi))
                pose_nwu = geo.CoordinateSystem.convert(pose_camera, geo.CoordinateSystem.EDN(), geo.CoordinateSystem.NWU())
                # where is camera on robot - origin of frame is center of robot
                # use tx for moving robot fwd or back, ty left and right, tx up off the ground (positive only)
                # use rx for rotations about x, ry for rotations about y (negative looks up), rz for rotations about z (0 is forward)
                # 2024 frontcam was {'tx': 0.3, 'ty': 0.05, 'tz': 0.2, 'rx': 0, 'ry': -30, 'rz':0}
                # 2024 backcam was {'tx': -0.3, 'ty': -0.1, 'tz': 0.2, 'rx': 0, 'ry': -30, 'rz':180}

                so = self.cam_orientation  # shortcut to save typing
                camera_in_robot_frame = geo.Transform3d(geo.Translation3d(so['tx'], so['ty'], so['tz']),
                                        geo.Rotation3d(math.radians(so['rx']), math.radians(so['ry']), math.radians(so['rz'])))  # back of robot, rotate up in y?

                tag_in_field_frame = self.layout.getTagPose(tag.getId())
                try:
                    robot_in_field_frame = objectToRobotPose(objectInField=tag_in_field_frame, cameraToObject=pose_nwu, robotToCamera=camera_in_robot_frame)
                    rft = robot_in_field_frame.translation()
                    rfr = robot_in_field_frame.rotation()
                    tx, ty, tz = rft.x, rft.y, rft.z
                    rx, ry, rz = rfr.x, rfr.y, rfr.z

                    # attempt to denoise the data - should probably see if this is actually wanted
                    #tx, ty, tz, rx, ry, rz = self.tagmanager.update(tag.getId(), tx, ty, tz, rx, ry, rz)
                    center = tag.getCenter()
                    self.tags.update({f'tag{tag.getId():02d}': {'id': tag.getId(), 'rotation': pose.rotation().x, 'distance': pose.z,
                                                  'tx': tx, 'ty': ty, 'tz': tz, 'rx': rx, 'ry': ry, 'rz': rz, 
                                                  'cx': center.x, 'cy': center.y,
                                                  'corners': tag.getCorners([0]*8) # Pass corners for drawing
                                                  }})
                except Exception as e:
                    print(f'Attempted to get field frame but got error {e} on tag id {tag.getId()}')

    def set_hsv(self):
        # each color has a different set of rules for segmenting it
        # do not change a value in one color that is not changed in another - so should make this a dictionary to clean
        home = False

        if self.color == 'orange':  # 2024 orange rings - started 20240305
            self._hsv_threshold_hue = [0, 10]  # unknown - probably need to wrap around this year - too close to red
            self._hsv_threshold_saturation = [50, 254]  # unknown
            self._hsv_threshold_value = [10, 254]  # unknown
            self._blur_radius = 3  # do i need to blur more or less?
            self._filter_contours_max_ratio = 10  # ring lying flat on ground
            self._filter_contours_min_ratio = 2  # facing the circle
            self._filter_contours_min_area = 100.0
            self._filter_contours_min_height = 15
            self._filter_contours_min_width = 15
            self._filter_contours_max_width = self.x_resolution // 2
            self._filter_contours_max_height = self.y_resolution // 2
            self._filter_contours_solidity = [10.0, 100.0]
            self._filter_contours_box_fill = [10.0, 100]
            # above (y is 0 at top) or below this we ignore detections  - (.9*yres, yres) would ignore the top 90%
            self.ignore_y = [0.3 * self.y_resolution, self.y_resolution]  # ignore top 30% - lights, etc

        elif self.color == 'purple':  # 2023 purple cubes
            self._hsv_threshold_hue = [117, 126]  # this is too close to blue...
            self._hsv_threshold_saturation = [110, 255]
            self._hsv_threshold_value = [130, 255]  # tends to be a bit dark
            self._blur_radius = 3  # do i need to blur more or less?  i hate purple
            #self._filter_contours_solidity = [50.0, 100.0]
            #self._filter_contours_box_fill = [50.0, 95.0]
            self._filter_contours_max_ratio = 2  # 1.5 for cube - need to cut false positives
            self._filter_contours_min_ratio = 0.5  # .67 for cube
            self._filter_contours_min_area = 100.0
            self._filter_contours_min_height = 12
            self._filter_contours_min_width = 12
            self._filter_contours_max_width = 200
            self._filter_contours_max_height = 200
            # self._filter_contours_max_height = 60  # resolution dependent
            if home:  # values at home
                self._hsv_threshold_hue = [118, 123]  # this is too close to blue... and shadowy carpet, i guess
                self._hsv_threshold_saturation = [125, 210]
                self._hsv_threshold_value = [130, 255]  # tends to be a bit dark

        elif self.color == 'yellow':  # 2023 yellow cones
            self._hsv_threshold_hue = [17, 23]
            self._hsv_threshold_saturation = [150, 254]
            self._hsv_threshold_value = [120, 254]
            if home:
                self._hsv_threshold_hue = [11, 28]
                self._hsv_threshold_saturation = [90, 255]
                self._hsv_threshold_value = [90, 255]
            self._filter_contours_max_ratio = 3  # 2 h/w so still gets a tall cone
            self._filter_contours_min_ratio = .1  # 0.5 cone lying down
            self._filter_contours_min_area = 2000.0
            self._filter_contours_min_height = 30
            self._filter_contours_min_width = 30
            self._filter_contours_max_width = 200
            self._filter_contours_max_height = 200
            #self._filter_contours_solidity = [50.0, 100.0]

        elif self.color == 'yellow_balls':  # 2020 yellow balls
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

        elif self.color == 'green':  # vision targets for 2023 are small squares
            self._hsv_threshold_hue = [80, 88]
            # self._hsv_threshold_hue = [80, 87]  # verified with lifecam 20220305 on training images
            self._hsv_threshold_saturation = [110, 254]  # retroreflectors tough to get low sat so this removes lights
            self._hsv_threshold_value = [150, 254]
            self._filter_contours_min_width = 5
            self._filter_contours_min_height = 5
            self._filter_contours_min_ratio = 0.2
            self._filter_contours_max_ratio = 3

            if home:  # should not make green special here - will cause problems
                self._hsv_threshold_hue = [68, 99]
                self._hsv_threshold_saturation = [80, 250]
                self._hsv_threshold_value = [80, 250]
                self._filter_contours_min_width = 20
                self._filter_contours_min_height = 20
                self._filter_contours_max_width = 80
                self._filter_contours_max_height = 80
            # in 2022 they are long and flat, so w/h >> 1.  small too.  min ratio is .5, max is 6
            # in 2023 they are tall and thin (4in tall) (2in wide?) ratio is w/h
                #self._filter_contours_min_area = 10.0

        else:
            print(f'{self.camera}: no valid color provided')

    def get_center_hsv(self, width=5, height=10):  # attempt to train hsv detector with center objects
        x_center, y_center = self.x_resolution // 2, self.y_resolution // 2
        t = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)[y_center - height:y_center + height,
                x_center - width:x_center + width, :]
        self.image = cv2.rectangle(self.image, (x_center - width, y_center - height), (x_center + width, y_center + height), (255, 255, 255))
        hue, sat, val = t[:, :, 0],  t[:, :, 1], t[:, :, 2]
        # h,s and v all seem to have zeros show up when saturated?  Replace zeros?

        if self.replace_zeros:  # i think this kills saturated pixels well
            for channel in [hue, sat, val]:
                channel[channel == 0] = np.median(channel[channel > 0])
        hue_mean, hue_std = np.median(hue), np.std(hue)  # median works better
        sat_mean, sat_std = np.median(sat), np.std(sat)
        val_mean, val_std = np.median(val), np.std(val)
        stats_width = 2 # 3 gives 99% of the bell curve area
        self.temp_hue = [max(np.floor(hue_mean-3), np.floor(hue_mean - stats_width * hue_std)), min(np.ceil(hue_mean+3), np.ceil(hue_mean + stats_width * hue_std))]
        # why did i do these mins and maxes?
        min_allowed, max_allowed = 20, 50
        self.temp_sat = [max(min_allowed, min(max_allowed, np.floor(sat_mean - stats_width * sat_std))), min(np.ceil(sat_mean + stats_width * sat_std), 255)]  # must have some color, but can't be too dark?
        self.temp_val = [max(min_allowed, min(max_allowed, np.floor(val_mean - stats_width * val_std))), min(np.ceil(val_mean + stats_width * val_std), 255)]

        self._hsv_threshold_hue = self.temp_hue
        self._hsv_threshold_saturation = self.temp_sat
        self._hsv_threshold_value = self.temp_val
        self.training_data = t
        # self.color = f'{hue_mean:0f}_{sat_mean:0f}_{val_mean:0f}'


    def bounding_box_sort_contours(self, method='size'):
        """Get sorted contours and bounding boxes from our list of filtered contours
            This can be very simple if all we wanted was size

            :param method: one of [size, center, left-to-right, right-to-left, top-down or bottom-up]
            :return: None but sets self.filter_contours_output and self.bounding_boxes
            """
        # method for the sort at https://www.pyimagesearch.com/2015/04/20/sorting-contours-using-python-and-opencv/
        for ix, contour in enumerate(self.filter_contours_output):
            pass
            x, y, w, h = cv2.boundingRect(contour)
            #print(f"Contour {ix} has area {cv2.contourArea(contour)} with width {w} and height {h} at location ({x},{y})")

        # *** Give ourselves some options on sorting, defaulting to size (closest) ***
        # if sorting by location, figure out if sorting by x or y coordinate
        if method == "top-down" or method == "bottom-up":
            axis = 1
        else:  # left right, right left, center uses x axis
            axis = 0
        # below in zip sort we will sort on the axis (x or y) of second list in the zip (the bounding box)
        # in the way we did the zip below, b[1] is the bounding box
        key = lambda b: b[1][axis]
        # handle if we need to sort in reverse, change sorting key for size
        if method == 'left-to-right' or method == 'top-down':
            reverse = False
        elif method == 'right-to-left' or method == 'bottom-up':
            reverse = True
        elif method == 'center':
            # distance from center is x+w - x_resolution/2
            key = lambda b: abs( b[1][0]+b[1][2] - self.x_resolution/2 )
            reverse = False  # want closest to the center first
        else:  # sort by size
            key = lambda b: cv2.contourArea(b[0])  # in the zip, b[0] is the contour
            reverse = True

        # could easily append a max_contours parameter here and limit to [:n] choices
        # we want the bounding boxes later anyway for
        bounding_boxes = [cv2.boundingRect(c) for c in self.filter_contours_output]
        # neat use of zip(*foo) to to give you back the lists you zipped to sort
        # in this, the lambda variable [0] is the first thing in the list, [1] is the second
        # so if sorting on bounding boxes, you use [1], but on countours you use [0]
        (self.filter_contours_output, self.bounding_boxes) = \
            zip(*sorted(zip(self.filter_contours_output, bounding_boxes), key=key, reverse=reverse))

        # if you only wanted size it would be really simple - just do key = cv.contourArea and reverse = True w/o a zip
        # self.filter_contours_output = sorted(self.filter_contours_output, key=key, reverse=reverse)[:5]

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
            object_width = 8 * 0.0254  # 2023 cone - 8" wide at the base
        elif self.color == 'blue' or self.color == 'red':
            object_width = 9.5 * 0.0254  # 2022 big tennis ball, 9.5 inches to meters
        elif self.color == 'green':
            # ToDo: have to update this to do the 2023 distance differently - use height, not width
            object_width = 2 * 0.0254  # 2022 big tennis ball, 9.5 inches to meters
        elif self.color == 'orange':
            object_width = 14 * 0.0254  # 2024 ring is 14" wide, and should present that way (wide)
        else:
            object_width = 9.5 * 0.0254  # default for now

        # camera specific parameters
        self.camera_shift = 0  # when the cameras are flawed (center of camera bore axis not center of image)
        # need to do extra math if camera bore at an angle to the object
        camera_height = 33  # camera vertical distance above ground, use when cam looking at objects on ground
        if self.camera == 'lifecam':
            # camera_fov = 55  # Lifecam 320x240
            camera_fov = 55  # Lifecam 640x360
        elif self.camera == 'c920':
            camera_fov = 77  # logitech c920 in wide mode
            camera_fov = 70  # logitech c920 in 640x360
        elif self.camera == 'geniuscam':
            camera_fov = 118  # Genius 120 352x288
            self.camera_shift = 0 # 14  # had one at 14 pixels, another at -8 - apparently genius cams have poor QC
        elif self.camera == 'c270':
            camera_fov = 59  # Logitech C290 432x240
        elif self.camera == 'elp100':
            camera_fov = 100  # little elp
        else:
            raise ValueError(f'Invalid camera specification {self.camera}')

        # let's just do the calculations on the closest one for now; we could loop through them all easily enough
        for idx, bbox in enumerate(self.bounding_boxes):
            x, y, w, h = bbox  # returned rectangle parameters
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
            self.rotation_to_target = - target_x * camera_fov / 2.0
            self.distance_to_target = object_width / (
                        2.0 * math.tan(target_width_fraction_fov * math.radians(camera_fov) / 2.0))
            self.strafe_to_target = math.sin(math.radians(self.rotation_to_target)) * self.distance_to_target

            # append results to our dictionaries
            self.results[self.color]['ids'].append(idx+1)
            self.results[self.color]['rotations'].append(self.rotation_to_target)
            self.results[self.color]['distances'].append(self.distance_to_target)
            self.results[self.color]['strafes'].append(self.strafe_to_target)
            self.results[self.color]['heights'].append(self.height)
            self.results[self.color]['cx'].append(centroid_x)
            self.results[self.color]['cy'].append(centroid_y)

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


if __name__ == "__main__":

    """
    Test things out with just a few images if not using the pipeline  - updated 2022 0124
    Best to use on a computer (not pi) to check the HSV values of objects in front of the computer
    But to really get the targets right, save an image from the web
    Set the color below to yellow, blue, green or red (red is tougher) to test
    """
    import sys
    video_seconds = 9

    args = sys.argv[1:]
    allowed_colors = ['purple', 'yellow', 'blue', 'green', 'red', 'yellow-ball']
    if len(args) > 0 and args[0] in allowed_colors:  # allow color to be passed from command line
        colors = [args[0]]
    else:  # default color
        colors = ['yellow', 'purple', 'green']
        #colors = ['green']

    start_time = time.time()
    count = 0
    run_time = 1
    methods = ['top-down', 'bottom-up', 'right-to-left', 'left-to-right', 'size']
    methods = ['size', 'left-to-right']
    usb_port = 1
    w, h = 640, 360

    sort_test = False  # test the sorting methods of bounding boxes
    if sort_test:
        for method in methods:
            count += 1
            cam = cv2.VideoCapture(usb_port, cv2.CAP_DSHOW)
            s, im = cam.read()  # captures image - note for processing that it is BGR, not RGB!
            pipeline = SpartanOverlay(colors=colors)
            pipeline.debug = True
            pipeline.process(image=im, method=method)
            cv2.imshow(f"Test Picture: sorting by {method}", pipeline.image)  # displays captured image
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            cam.release()

    video = True
    training = True
    if video:  # run continuous video and report HSV on objects found
        start_time = time.time()
        end_time = start_time
        pipeline = SpartanOverlay(colors=colors)
        pipeline.debug = False
        cam = cv2.VideoCapture(usb_port, cv2.CAP_DSHOW)
        np.set_printoptions(precision=1)
        while end_time - start_time < video_seconds:  # take video for x seconds
            count += 1
            s, im = cam.read()  # captures image - note for processing that it is BGR, not RGB!
            if s > 0:  # Found an image
                im = cv2.resize(im.copy(), (w, h), interpolation=cv2.INTER_AREA)
                pipeline.process(image=im, method='size', training=training)

                if pipeline.training:
                    pass
                else:
                    x_center, y_center = pipeline.x_resolution // 2, pipeline.y_resolution // 2
                    targets = sum([pipeline.results[color]['targets'] for color in colors])
                    if targets < 1:  # get some diagnostics on a box if nothing is recognized
                        width, height = 10, 20
                        t = cv2.cvtColor(pipeline.image, cv2.COLOR_BGR2HSV)[y_center - height:y_center + height,
                            x_center - width:x_center + width, :]
                        pipeline.image[:, :, 0] = pipeline.hsv_threshold_output  # make a color cast to show what is thresholded (maybe should draw the contour instead)
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

                # end if on the train color
                cv2.imshow(f"Test Picture: sorting by size on {pipeline.color}",
                           pipeline.image)  # displays captured image
                end_time = time.time()
                cv2.waitKey(delay=50)

        # print(f'hist: {np.histogram(t[:, :, 0], bins=90, range=(0,179))}', end='\n', flush='True')
        print(f'Captured {count} frames in {end_time - start_time:.1f}s - average fps is {count / (end_time - start_time):.1f}')
        if pipeline.training:
            import matplotlib.pyplot as plt
            t = pipeline.training_data
            print(f' hsv mean: {t[:, :, 0].mean():.1f} {t[:, :, 1].mean():.1f} {t[:, :, 2].mean():.1f}', end='\n', flush='True')
            plt.hist([t[:, :, 0].flatten(), t[:, :, 1].flatten(), t[:, :, 2].flatten()], label=['h', 's', 'v'], bins=48, density=True, rwidth=1, range=(0,255))
            plt.legend()
            plt.show()
        else:
            print(f'mean: {t[:, :, 0].mean():.2f}', end='\n', flush='True')

        cam.release()
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # print(pipeline.results)