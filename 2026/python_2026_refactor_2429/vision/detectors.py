import cv2
import numpy as np
import math
import robotpy_apriltag as ra
import wpimath.geometry as geo
from wpimath import objectToRobotPose
from vision.hsv_config import get_hsv_config, get_target_dimensions

class TagDetector:
    def __init__(self, camera_model, field_layout=None):
        self.cam = camera_model
        
        # Setup Detector
        self.detector = ra.AprilTagDetector()
        self.detector.addFamily('tag36h11')
        
        # Config (2025 defaults are aggressive, tuning down)
        qt = self.detector.getQuadThresholdParameters()
        qt.minClusterPixels = 10
        self.detector.setQuadThresholdParameters(qt)
        
        cfg = self.detector.getConfig()
        cfg.numThreads = 2
        self.detector.setConfig(cfg)

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
                field = ra.AprilTagField.k2025ReefscapeWelded
                self.layout = ra.AprilTagFieldLayout.loadField(field)
            except:
                self.layout = None
        else:
            self.layout = field_layout

    def detect(self, image, cam_orientation=None, max_distance=3.0):
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(grey)
        
        results = {}
        
        # Filter
        valid_tags = [t for t in detections if t.getDecisionMargin() > 30 and t.getHamming() < 2]
        
        for tag in valid_tags:
            # Pose Estimation
            # Orthogonal Iteration is generally more robust
            pose_est = self.estimator.estimateOrthogonalIteration(tag, 50)
            pose = pose_est.pose1
            
            # Distance Filter
            if pose.z > max_distance:
                continue
                
            # Field Pose Calculation
            tx, ty, tz, rx, ry, rz = 0,0,0,0,0,0
            
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
                    
                    tag_field_pose = self.layout.getTagPose(tag.getId())
                    if tag_field_pose:
                        robot_pose = objectToRobotPose(tag_field_pose, pose_nwu, robot_to_cam)
                        t = robot_pose.translation()
                        r = robot_pose.rotation()
                        tx, ty, tz = t.x, t.y, t.z
                        rx, ry, rz = r.x, r.y, r.z
                except Exception:
                    pass # Layout lookup failed

            center = tag.getCenter()

            # Calculate Targeting Info (Rotation/Strafe)
            # Map cx from [0, width] to [-1, 1]
            # We need camera width/fov. Assuming cam object has them.
            # TagDetector init didn't store fov_h, let's grab it or calc it.
            fov_h = self.cam.get_fov()
            norm_x = (2.0 * center.x / self.cam.width) - 1.0
            rotation = -norm_x * (fov_h / 2.0)
            strafe = math.sin(math.radians(rotation)) * pose.z

            results[f'tag{tag.getId():02d}'] = {
                'id': tag.getId(),
                'tx': tx, 'ty': ty, 'tz': tz,
                'rx': rx, 'ry': ry, 'rz': rz,
                'dist': pose.z,
                'cx': center.x, 'cy': center.y,
                'rotation': rotation,
                'strafe': strafe,
                'corners': tag.getCorners([0]*8),
                # Raw pose for drawing axes
                'rvec': np.array([pose.rotation().x, pose.rotation().y, pose.rotation().z]), # Approx
                'tvec': np.array([pose.x, pose.y, pose.z])
            }
            
        return results


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