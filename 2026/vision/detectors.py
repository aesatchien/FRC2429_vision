import cv2
import numpy as np
import math
import robotpy_apriltag as ra
import wpimath.geometry as geo
from wpimath import objectToRobotPose
from vision.hsv_config import get_hsv_config, get_target_dimensions

class TagDetector:
    def __init__(self, camera_model, field_layout=None, config=None):
        self.cam = camera_model
        if config is None: config = {}
        
        # Setup Detector
        self.detector = ra.AprilTagDetector()
        self.detector.addFamily('tag36h11')
        
        # Config (2025 defaults are aggressive, tuning down)
        qt = self.detector.getQuadThresholdParameters()
        qt.minClusterPixels = config.get("min_cluster_pixels", 25)
        self.detector.setQuadThresholdParameters(qt)
        
        cfg = self.detector.getConfig()
        cfg.numThreads = config.get("threads", 1)
        cfg.quadDecimate = config.get("decimate", 1.0) # 1.0 = Best Quality (Full Res)
        cfg.quadSigma = config.get("sigma", 0.6)       # 0.6 = Smooths sensor noise
        cfg.refineEdges = config.get("refine_edges", True) # Turn FALSE to speed up
        cfg.decodeSharpening = config.get("decode_sharpening", 0.25)
        self.detector.setConfig(cfg)
        
        # Filter Params
        self.min_margin = config.get("decision_margin", 35) # Reject weak tags
        self.max_hamming = config.get("hamming", 1)         # Reject bit errors
        self.allow_multi_tag = config.get("allow_multi_tag", True)

        # Distortion Correction
        self.use_distortions = config.get("use_distortions", False)
        self.map1, self.map2 = None, None
        if self.use_distortions and self.cam.dist_coeffs is not None:
            K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]])
            D = self.cam.dist_coeffs
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, K, (self.cam.width, self.cam.height), cv2.CV_16SC2)

        # Pose Estimator
        est_config = ra.AprilTagPoseEstimator.Config(
            tagSize=0.1651, 
            fx=self.cam.fx, fy=self.cam.fy, 
            cx=self.cam.cx, cy=self.cam.cy
        )
        self.estimator = ra.AprilTagPoseEstimator(est_config)
        
        # Field Layout (Optional)
        if field_layout is None:
            try:
                if hasattr(ra.AprilTagField, "k2025ReefscapeWelded"):
                    field = ra.AprilTagField.k2025ReefscapeWelded
                    field = ra.AprilTagField.k2024Crescendo
                else:
                    field = ra.AprilTagField.k2025Reefscape 
                self.layout = ra.AprilTagFieldLayout.loadField(field)
            except:
                self.layout = None
        else:
            self.layout = field_layout

    def detect(self, image, cam_orientation=None, max_distance=3.0):
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if self.use_distortions and self.map1 is not None:
            grey = cv2.remap(grey, self.map1, self.map2, cv2.INTER_LINEAR)
            
        detections = self.detector.detect(grey)
        
        results = {}
        
        # Filter
        valid_tags = [t for t in detections if t.getDecisionMargin() > self.min_margin and t.getHamming() <= self.max_hamming]
        
        # 1. Prioritize Multi-Tag Solution (More Robust)
        # We need to process them individually first to get their field poses for averaging
        # But we can optimize by collecting valid results first
        processed_tags = []
        for tag in valid_tags:
            res = self._process_single_tag(tag, cam_orientation, max_distance)
            if res:
                results[f'tag{tag.getId():02d}'] = res
                if res.get('in_layout'):
                    processed_tags.append(res)

        # 2. Compute Multi-Tag Pose if possible
        if self.allow_multi_tag and len(processed_tags) > 1:
            multi_res = self._process_multi_tag(processed_tags, cam_orientation)
            if multi_res:
                # We use ID -1 to indicate a "virtual" combined tag
                results['multi_tag'] = multi_res
            
        return results

    def _process_single_tag(self, tag, cam_orientation, max_distance):
        """
        Calculates pose for a single detected tag.
        Returns a dictionary of results or None if filtered out.
        """
        # Pose Estimation
        # Orthogonal Iteration is generally more robust
        pose_est = self.estimator.estimateOrthogonalIteration(tag, 50)
        pose = pose_est.pose1
        
        # Distance Filter
        if pose.z > max_distance:
            return None
            
        # Field Pose Calculation
        tx, ty, tz, rx, ry, rz = 0,0,0,0,0,0
        in_layout = False
        
        if self.layout and cam_orientation:
            try:
                # Camera to Robot
                so = cam_orientation
                robot_to_cam = geo.Transform3d(
                    geo.Translation3d(so['tx'], so['ty'], so['tz']),
                    geo.Rotation3d(math.radians(so['rx']), math.radians(so['ry']), math.radians(so['rz']))
                )
                
                # Tag to Camera (NWU conversion)
                # OpenCV: Z forward, X right, Y down
                # WPILib: X forward, Y left, Z up
                pose_camera = geo.Transform3d(
                    geo.Translation3d(pose.x, pose.y, pose.z),
                    geo.Rotation3d(-pose.rotation().x - np.pi, -pose.rotation().y, pose.rotation().z - np.pi)
                )
                pose_nwu = geo.CoordinateSystem.convert(pose_camera, geo.CoordinateSystem.EDN(), geo.CoordinateSystem.NWU())
                
                tag_field_pose = self.layout.getTagPose(tag.getId())  # returns None if tag not in field layout
                if tag_field_pose:
                    robot_pose = objectToRobotPose(tag_field_pose, pose_nwu, robot_to_cam)
                    t = robot_pose.translation()
                    r = robot_pose.rotation()
                    tx, ty, tz = t.x, t.y, t.z
                    rx, ry, rz = r.x, r.y, r.z
                    in_layout = True
            except Exception:
                pass # Layout lookup failed

        center = tag.getCenter()

        # Calculate Targeting Info (Rotation/Strafe)
        # Map cx from [0, width] to [-1, 1]
        fov_h = self.cam.get_fov()
        norm_x = (2.0 * center.x / self.cam.width) - 1.0
        rotation = -norm_x * (fov_h / 2.0)
        strafe = math.sin(math.radians(rotation)) * pose.z

        # Flatten corners to [x1, y1, x2, y2, ...]
        corners = tag.getCorners([0.0]*8)

        return {
            'id': tag.getId(),
            'in_layout': in_layout,
            'tx': tx, 'ty': ty, 'tz': tz,
            'rx': rx, 'ry': ry, 'rz': rz,
            'dist': pose.z,
            'cx': center.x, 'cy': center.y,
            'rotation': rotation,
            'strafe': strafe,
            'corners': corners,
            # Raw pose for drawing axes
            'rvec': self._quat_to_rvec(pose.rotation().getQuaternion()),
            'tvec': np.array([pose.x, pose.y, pose.z])
        }

    def _process_multi_tag(self, tag_results, cam_orientation):
        """
        Calculates a combined pose from multiple detected tags.
        Uses cv2.solvePnP with SQPNP for high-accuracy global pose estimation.
        """
        # Only use tags that have a valid field pose
        valid_tags = [t for t in tag_results if t.get('in_layout')]
        
        if len(valid_tags) < 2:
            return None
        
        # 1. Collect Correspondences
        obj_points = [] # 3D Field Coordinates
        img_points = [] # 2D Pixel Coordinates
        
        tag_size = 0.1651 # Meters
        s = tag_size / 2.0
        
        # Local corners in Standard AprilTag Frame (X right, Y down, Z forward/out)
        # Order: Bottom-Left, Bottom-Right, Top-Right, Top-Left (Counter-Clockwise)
        # Note: This order must match detector output.
        local_corners = [
            geo.Translation3d(0, s, -s),  # BL (Y is left, so +s is left/bottom?)
            geo.Translation3d(0, -s, -s), # BR
            geo.Translation3d(0, -s, s),  # TR
            geo.Translation3d(0, s, s)    # TL
        ]

        for tag_res in valid_tags:
            tid = tag_res['id']
            field_pose = self.layout.getTagPose(tid) # Pose3d
            if not field_pose: continue
            
            # Transform local corners to Field Coordinates
            for i, corner_offset in enumerate(local_corners):
                field_corner = field_pose.transformBy(geo.Transform3d(corner_offset, geo.Rotation3d()))
                obj_points.append([field_corner.X(), field_corner.Y(), field_corner.Z()])
                
                # Append corresponding pixel corner
                # tag_res['corners'] is [p0, p1, p2, p3]
                # We assume detector returns BL, BR, TR, TL order (Standard AprilTag)
                c = tag_res['corners']
                if len(c) == 8:
                    p = c[i*2 : i*2+2]
                else:
                    # Handle list of points (len 4)
                    pt = c[i]
                    p = [pt.x, pt.y] if hasattr(pt, 'x') else pt
                img_points.append(p)

        if not obj_points: return None

        # 2. Solve PnP
        # K = Camera Matrix, D = Distortions
        K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]])
        D = self.cam.dist_coeffs if self.cam.dist_coeffs is not None else np.zeros(5)
        
        # SQPNP is robust for planar targets (like tags)
        success, rvec, tvec = cv2.solvePnP(np.array(obj_points, dtype=np.float32), 
                                           np.array(img_points, dtype=np.float32), 
                                           K, D, flags=cv2.SOLVEPNP_SQPNP)
        
        if not success: return None

        # 3. Convert to WPILib Geometry
        # solvePnP returns the Transform from Field(Object) -> Camera(CV)
        # T_camera_field (in CV frame)
        
        # Convert rvec to Rotation3d
        R_f_c, _ = cv2.Rodrigues(rvec)
        # WPILib Rotation3d can take a 3x3 matrix (requires flattened tuple or numpy array depending on version)
        # Safest is to convert to Quaternion or use RPY, but let's try the matrix constructor if available,
        # or manual conversion. To be safe across versions, let's use a helper.
        rot_f_c = self._matrix_to_rotation3d(R_f_c)
        
        # Construct Pose of Field Origin in Camera Frame (CV Coordinates)
        # OpenCV: X Right, Y Down, Z Forward (matches WPILib EDN: East-Down-North)
        pose_field_in_cam_cv = geo.Pose3d(
            geo.Translation3d(tvec[0][0], tvec[1][0], tvec[2][0]), 
            rot_f_c
        )
        
        # Convert to NWU (North-West-Up) which is the standard WPILib Robot/Field Frame
        # EDN -> NWU maps: X(East)->-Y(West), Y(Down)->-Z(Up), Z(North)->X(North)
        pose_field_in_cam_nwu = geo.CoordinateSystem.convert(
            pose_field_in_cam_cv, 
            geo.CoordinateSystem.EDN(), 
            geo.CoordinateSystem.NWU()
        )
        
        # We now have the Pose of the Field Origin relative to the Camera (in NWU).
        # We want the Pose of the Camera relative to the Field.
        # This is simply the inverse.
        # Pose3d does not have inverse(), so we use Transform3d
        tf_field_in_cam = geo.Transform3d(pose_field_in_cam_nwu.translation(), pose_field_in_cam_nwu.rotation())
        tf_cam_in_field = tf_field_in_cam.inverse()
        camera_pose = geo.Pose3d(tf_cam_in_field.translation(), tf_cam_in_field.rotation())

        # 4. Apply Camera Offset to get Robot Pose
        if cam_orientation:
            so = cam_orientation
            robot_to_cam = geo.Transform3d(
                geo.Translation3d(so['tx'], so['ty'], so['tz']),
                geo.Rotation3d(math.radians(so['rx']), math.radians(so['ry']), math.radians(so['rz']))
            )
            robot_pose = camera_pose.transformBy(robot_to_cam.inverse())
            
            tx, ty, tz = robot_pose.X(), robot_pose.Y(), robot_pose.Z()
            rx, ry, rz = robot_pose.rotation().X(), robot_pose.rotation().Y(), robot_pose.rotation().Z()
        else:
            # Fallback if no orientation provided (return camera pose)
            tx, ty, tz = camera_pose.X(), camera_pose.Y(), camera_pose.Z()
            rx, ry, rz = camera_pose.rotation().X(), camera_pose.rotation().Y(), camera_pose.rotation().Z()

        best_tag = min(valid_tags, key=lambda x: x['dist'])
        
        return {
            'id': -1, # Virtual ID for multi-tag solution
            'in_layout': True, # Set True so TagManager smooths it and NetworkTables sends it
            'tx': tx, 'ty': ty, 'tz': tz,
            'rx': rx, 'ry': ry, 'rz': rz, # Use PnP rotation
            'dist': best_tag['dist'], # Use closest distance
            'rotation': best_tag['rotation'],
            'strafe': best_tag['strafe'],
            'cx': 0, 'cy': 0, # Virtual center
            'corners': [],
            'rvec': best_tag['rvec'],
            'tvec': best_tag['tvec']
        }

    def _quat_to_rvec(self, q):
        w, x, y, z = q.W(), q.X(), q.Y(), q.Z()
        # Rotation Matrix from Quaternion
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
        ])
        rvec, _ = cv2.Rodrigues(R)
        return rvec

    def _matrix_to_rotation3d(self, R):
        # Converts 3x3 Rotation Matrix to WPILib Rotation3d
        # Using Tait-Bryan angles (Z-Y-X) for robustness
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return geo.Rotation3d(x, y, z)

class HSVDetector:
    def __init__(self, camera_model):
        self.cam = camera_model
        self.fov_h = self.cam.get_fov()

    def process(self, image, color, training=False, train_box=None):
        # 1. Get Config
        cfg = get_hsv_config(color, self.cam.width, self.cam.height)
        if not cfg: return {}

        # 2. Pre-process
        if cfg.get('_blur_radius', 0) > 0:
            image = cv2.blur(image, (cfg['_blur_radius'], cfg['_blur_radius']))
            
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 3. Threshold
        lower = np.array([cfg['_hsv_threshold_hue'][0], cfg['_hsv_threshold_saturation'][0], cfg['_hsv_threshold_value'][0]])
        upper = np.array([cfg['_hsv_threshold_hue'][1], cfg['_hsv_threshold_saturation'][1], cfg['_hsv_threshold_value'][1]])
        
        stats = {}
        if training:
            stats = self._calc_training_stats(hsv, self.cam.width, self.cam.height, train_box)
            if stats:
                lower = np.array([stats['hue'][0], stats['sat'][0], stats['val'][0]])
                upper = np.array([stats['hue'][1], stats['sat'][1], stats['val'][1]])

        mask = cv2.inRange(hsv, lower, upper)
        
        # 4. Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 5. Filter
        filtered = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            if area < cfg.get('_filter_contours_min_area', 0): continue
            if w < cfg.get('_filter_contours_min_width', 0): continue
            ratio = w / h
            if ratio < cfg.get('_filter_contours_min_ratio', 0): continue
            if ratio > cfg.get('_filter_contours_max_ratio', 100): continue
            
            # Y-limit check
            ignore_y = cfg.get('ignore_y', [9999, 9999])
            if y > ignore_y[0]: continue
            
            filtered.append(c)
            
        # 6. Sort (Size)
        filtered.sort(key=cv2.contourArea, reverse=True)
        
        # 7. Calculate Attributes for Top Target
        results = {
            'targets': len(filtered),
            'contours': filtered,
            'ids': [], 'distances': [], 'strafes': [], 'rotations': [], 'heights': [],
            'cx': [], 'cy': []
        }
        
        target_width_m = get_target_dimensions(color)
        
        for i, c in enumerate(filtered):
            x, y, w, h = cv2.boundingRect(c)
            
            # Distance Calc
            # distance = object_width / (2 * tan(width_in_fov * fov / 2))
            width_fraction = w / self.cam.width
            distance = target_width_m / (2 * math.tan(width_fraction * math.radians(self.fov_h) / 2.0))
            
            # Rotation/Strafe
            cx = x + w/2
            cy = y + h/2
            
            # Map cx from [0, width] to [-1, 1]
            norm_x = (2.0 * cx / self.cam.width) - 1.0
            rotation = -norm_x * (self.fov_h / 2.0)
            strafe = math.sin(math.radians(rotation)) * distance
            
            results['ids'].append(0) # Color targets don't have specific IDs
            results['distances'].append(distance)
            results['rotations'].append(rotation)
            results['strafes'].append(strafe)
            results['heights'].append(h)
            results['cx'].append(cx)
            results['cy'].append(cy)
            
        # 8. Training Stats (if requested)
        if training and stats:
            results['training_stats'] = stats
            
        return results

    def _calc_training_stats(self, hsv_img, w, h, train_box=None):
        # Sample center box
        cw, ch = 10, 20
        if train_box and len(train_box) >= 2:
            cx = int(w * train_box[0])
            cy = int(h * train_box[1])
        else:
            cx, cy = w // 2, h // 2
        sample = hsv_img[cy-ch:cy+ch, cx-cw:cx+cw, :]
        
        if sample.size == 0: return {}
        
        hue = sample[:,:,0]
        sat = sample[:,:,1]
        val = sample[:,:,2]
        
        # Simple stats
        return {
            'hue': [float(np.min(hue)), float(np.max(hue))],
            'sat': [float(np.min(sat)), float(np.max(sat))],
            'val': [float(np.min(val)), float(np.max(val))]
        }