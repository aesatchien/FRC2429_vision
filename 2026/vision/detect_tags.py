import cv2
import numpy as np
import math
import robotpy_apriltag as ra
import wpimath.geometry as geo
from wpimath import objectToRobotPose


class TagDetector:
    def __init__(self, camera_model, field_layout=None, config=None):
        self.cam = camera_model
        if config is None: config = {}

        # Setup Detector
        self.detector = ra.AprilTagDetector()
        self.detector.addFamily('tag36h11')

        # Config
        qt = self.detector.getQuadThresholdParameters()
        qt.minClusterPixels = config.get("min_cluster_pixels", 25)
        self.detector.setQuadThresholdParameters(qt)

        cfg = self.detector.getConfig()
        cfg.numThreads = config.get("threads", 1)
        cfg.quadDecimate = config.get("decimate", 1.0)
        cfg.quadSigma = config.get("sigma", 0.6)
        cfg.refineEdges = config.get("refine_edges", True)
        cfg.decodeSharpening = config.get("decode_sharpening", 0.25)
        self.detector.setConfig(cfg)

        self.min_margin = config.get("decision_margin", 35)
        self.max_hamming = config.get("hamming", 1)
        self.allow_multi_tag = config.get("allow_multi_tag", True)
        self.default_max_dist = config.get("max_tag_distance", 4.0)

        # Distortion
        self.use_distortions = config.get("use_distortions", False)
        self.map1, self.map2 = None, None
        if self.use_distortions and self.cam.dist_coeffs is not None:
            K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]])
            D = self.cam.dist_coeffs
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, K, (self.cam.width, self.cam.height),
                                                               cv2.CV_16SC2)

        # Legacy Estimator (Required for CJH fallback)
        est_config = ra.AprilTagPoseEstimator.Config(
            tagSize=0.1651, fx=self.cam.fx, fy=self.cam.fy, cx=self.cam.cx, cy=self.cam.cy
        )
        self.estimator = ra.AprilTagPoseEstimator(est_config)

        # Field Layout
        if field_layout is None:
            try:
                # Force 2024 for your test board
                field = ra.AprilTagField.k2024Crescendo
                self.layout = ra.AprilTagFieldLayout.loadField(field)
            except:
                self.layout = None
        else:
            self.layout = field_layout

        # Memory for Ambiguity Resolution
        # Key: TagID, Value: {'rvec': ..., 'tvec': ...}
        self._pnp_hist = {}

        # Pre-allocate Matrices for New Solver
        self.K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]], dtype=np.float64)
        self.D = np.array(self.cam.dist_coeffs) if self.cam.dist_coeffs is not None else np.zeros((5, 1))

        # Standard Object Points (WPILib Frame: X-out, Y-Left, Z-Up)
        # Order: BL, BR, TR, TL (Counter-Clockwise)
        s = 0.1651 / 2.0
        self.std_obj_points = np.array([
            [0.0, s, -s],  # 0: BL
            [0.0, -s, -s],  # 1: BR
            [0.0, -s, s],  # 2: TR
            [0.0, s, s]  # 3: TL
        ], dtype=np.float64)

    def detect(self, image, cam_orientation=None, max_distance=None):
        if max_distance is None: max_distance = self.default_max_dist
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if not hasattr(self, "_kcheck_printed"):
            h, w = grey.shape[:2]
            self._kcheck_printed = True
            print(f"[Kcheck] grey={w}x{h} cam={self.cam.width}x{self.cam.height} fx={self.cam.fx:.2f}")

        if self.use_distortions and self.map1 is not None:
            grey = cv2.remap(grey, self.map1, self.map2, cv2.INTER_LINEAR)

        detections = self.detector.detect(grey)
        results = {}

        valid_tags = [t for t in detections if
                      t.getDecisionMargin() > self.min_margin and t.getHamming() <= self.max_hamming]

        processed_tags = []
        for tag in valid_tags:
            # NEW: Try the New Solver with Memory + Field Constraint
            res = self._process_single_tag_new(tag, cam_orientation, max_distance)

            # FALLBACK: If new solver fails, use CJH (Original Backup)
            if not res:
                res = self._process_single_tag_cjh(tag, cam_orientation, max_distance)

            if res:
                results[f'tag{tag.getId():02d}'] = res
                if res.get('in_layout'): processed_tags.append(res)

        # Attempt Multi-Tag with "Rev S1" Mapping (Clockwise)
        if self.allow_multi_tag and len(processed_tags) > 1:
            multi_res = self._process_multi_tag(processed_tags, cam_orientation)
            if multi_res:
                results['multi_tag'] = multi_res

        return results

    def _process_single_tag_new(self, tag, cam_orientation, max_distance):
        """
        New Single Tag Solver with History & Relaxed Field Constraints.
        """
        tid = int(tag.getId())
        c = tag.getCorners([0.0] * 8)

        # MAPPING: "Rev S1" (Winner) -> Standard Object Points (BL, BR, TR, TL)
        # Det[0]=BR, Det[1]=BL, Det[2]=TL, Det[3]=TR
        # Obj 0 (BL) <-> Det 1
        # Obj 1 (BR) <-> Det 0
        # Obj 2 (TR) <-> Det 3
        # Obj 3 (TL) <-> Det 2
        img_points = np.array([
            [c[2], c[3]],  # Det 1 (BL)
            [c[0], c[1]],  # Det 0 (BR)
            [c[6], c[7]],  # Det 3 (TR)
            [c[4], c[5]],  # Det 2 (TL)
        ], dtype=np.float64)

        try:
            ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(self.std_obj_points, img_points, self.K, self.D,
                                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except:
            return None

        if not ok or len(rvecs) == 0: return None

        # --- AMBIGUITY RESOLUTION ---
        best_idx = -1
        best_score = -1e9  # Higher is better

        history = self._pnp_hist.get(tid)

        for i in range(len(rvecs)):
            r, t = rvecs[i], tvecs[i]

            score = 0.0

            # 1. Physical Validity (Z forward)
            if t[2] < 0: continue

            # 2. Field Constraint (Is the robot vaguely on the field?)
            pose_field = self._compute_robot_pose(tid, r, t, cam_orientation)
            if pose_field:
                on_field = self._is_on_field(pose_field)
                if on_field:
                    score += 1000.0  # Huge bonus for being on the field
                else:
                    score -= 500.0  # Penalty for being in the wall

            # 3. History (Consistency)
            if history:
                dist = np.linalg.norm(t - history['tvec'])
                if dist < 0.2:
                    score += 500.0
                elif dist > 1.0:
                    score -= 200.0

            # 4. Reprojection Error
            proj, _ = cv2.projectPoints(self.std_obj_points, r, t, self.K, self.D)
            e = proj.reshape(-1, 2) - img_points
            rms = np.sqrt(np.mean(np.sum(e * e, axis=1)))
            score -= (rms * 10.0)

            if score > best_score:
                best_score = score
                best_idx = i

        if best_idx == -1: return None

        rvec, tvec = rvecs[best_idx], tvecs[best_idx]

        proj, _ = cv2.projectPoints(self.std_obj_points, rvec, tvec, self.K, self.D)
        e = proj.reshape(-1, 2) - img_points
        rms = float(np.sqrt(np.mean(np.sum(e * e, axis=1))))

        if rms < 3.0:
            self._pnp_hist[tid] = {'rvec': rvec, 'tvec': tvec}

        if rms > 5.0 or tvec[2] > max_distance: return None

        robot_pose = self._compute_robot_pose(tid, rvec, tvec, cam_orientation)
        if not robot_pose: return None

        center = tag.getCenter()
        fov_h = self.cam.get_fov()
        norm_x = (2.0 * center.x / self.cam.width) - 1.0
        rotation = -norm_x * (fov_h / 2.0)
        strafe = math.sin(math.radians(rotation)) * tvec[2]

        return {
            'id': tid, 'in_layout': True,
            'tx': robot_pose.X(), 'ty': robot_pose.Y(), 'tz': robot_pose.Z(),
            'rx': robot_pose.rotation().X(), 'ry': robot_pose.rotation().Y(), 'rz': robot_pose.rotation().Z(),
            'dist': float(tvec[2]), 'cx': center.x, 'cy': center.y,
            'rotation': rotation, 'strafe': strafe,
            'corners': c,
            'rvec': rvec, 'tvec': tvec, 'pnp_rms_px': rms
        }

    def _compute_robot_pose(self, tid, rvec, tvec, cam_orientation):
        if not self.layout: return None
        tag_pose = self.layout.getTagPose(tid)
        if not tag_pose: return None

        R_cv, _ = cv2.Rodrigues(rvec)
        C_in_tag = -R_cv.T @ tvec

        P = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
        R_cam_in_tag_wpi = R_cv.T @ P.T

        cam_pose_in_tag = geo.Pose3d(
            geo.Translation3d(C_in_tag[0][0], C_in_tag[1][0], C_in_tag[2][0]),
            self._matrix_to_rotation3d(R_cam_in_tag_wpi)
        )

        cam_pose_field = tag_pose.transformBy(geo.Transform3d(geo.Pose3d(), cam_pose_in_tag))

        if cam_orientation:
            so = cam_orientation
            robot_to_cam = geo.Transform3d(
                geo.Translation3d(so["tx"], so["ty"], so["tz"]),
                geo.Rotation3d(math.radians(so["rx"]), math.radians(so["ry"]), math.radians(so["rz"]))
            )
            return cam_pose_field.transformBy(robot_to_cam.inverse())
        return cam_pose_field

    def _is_on_field(self, pose):
        # Relaxed Constraints to prevent rejection due to Z noise or bumper overhang
        x, y, z = pose.X(), pose.Y(), pose.Z()
        # Field is approx 16.5m long, 8.2m wide
        return (-0.25 < x < 18.0) and (-0.25 < y < 10.0) and (-2.5 < z < 4.5)

    def _process_single_tag_cjh(self, tag, cam_orientation, max_distance):
        """Original Fallback Logic"""
        pose_est = self.estimator.estimateOrthogonalIteration(tag, 50)
        pose = pose_est.pose1
        if pose.z > max_distance: return None

        tx, ty, tz, rx, ry, rz = 0, 0, 0, 0, 0, 0
        in_layout = False

        if self.layout and cam_orientation:
            try:
                so = cam_orientation
                robot_to_cam = geo.Transform3d(
                    geo.Translation3d(so['tx'], so['ty'], so['tz']),
                    geo.Rotation3d(math.radians(so['rx']), math.radians(so['ry']), math.radians(so['rz']))
                )
                pose_camera = geo.Transform3d(
                    geo.Translation3d(pose.x, pose.y, pose.z),
                    geo.Rotation3d(-pose.rotation().x - np.pi, -pose.rotation().y, pose.rotation().z - np.pi)
                )
                pose_nwu = geo.CoordinateSystem.convert(pose_camera, geo.CoordinateSystem.EDN(),
                                                        geo.CoordinateSystem.NWU())

                tag_field_pose = self.layout.getTagPose(tag.getId())
                if tag_field_pose:
                    robot_pose = objectToRobotPose(tag_field_pose, pose_nwu, robot_to_cam)
                    t = robot_pose.translation()
                    r = robot_pose.rotation()
                    tx, ty, tz = t.x, t.y, t.z
                    rx, ry, rz = r.x, r.y, r.z
                    in_layout = True
            except Exception:
                pass

        center = tag.getCenter()
        fov_h = self.cam.get_fov()
        norm_x = (2.0 * center.x / self.cam.width) - 1.0
        rotation = -norm_x * (fov_h / 2.0)
        strafe = math.sin(math.radians(rotation)) * pose.z
        corners = tag.getCorners([0.0] * 8)
        s = 0.1651 / 2.0
        obj_points = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float64)
        K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]])
        D = self.cam.dist_coeffs if self.cam.dist_coeffs is not None else np.zeros(5)
        rvec_calc = self._quat_to_rvec(pose.rotation().getQuaternion())
        tvec_calc = np.array([pose.x, pose.y, pose.z])
        proj, _ = cv2.projectPoints(obj_points, rvec_calc, tvec_calc, K, D)
        e = proj.reshape(-1, 2) - np.array(corners).reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum(e * e, axis=1))))

        return {
            'id': tag.getId(), 'in_layout': in_layout,
            'tx': tx, 'ty': ty, 'tz': tz, 'rx': rx, 'ry': ry, 'rz': rz,
            'dist': pose.z, 'cx': center.x, 'cy': center.y,
            'rotation': rotation, 'strafe': strafe, 'corners': corners,
            'rvec': rvec_calc, 'tvec': tvec_calc, 'pnp_rms_px': rms
        }

    def _process_multi_tag(self, tag_results, cam_orientation):
        """Multi-Tag with "Rev S1" Mapping."""
        valid_tags = [t for t in tag_results if t.get("in_layout")]
        if len(valid_tags) < 2 or self.layout is None: return None
        s = 0.1651 / 2.0
        local_corners = [
            geo.Translation3d(0.0, -s, -s),  # 0: BR
            geo.Translation3d(0.0, s, -s),  # 1: BL
            geo.Translation3d(0.0, s, s),  # 2: TL
            geo.Translation3d(0.0, -s, s),  # 3: TR
        ]
        obj_points, img_points = [], []
        for tag_res in valid_tags:
            tag_pose = self.layout.getTagPose(int(tag_res["id"]))
            if not tag_pose: continue
            c = tag_res["corners"]
            pts = [[c[0], c[1]], [c[2], c[3]], [c[4], c[5]], [c[6], c[7]]]
            for i in range(4):
                p_field = tag_pose.transformBy(geo.Transform3d(local_corners[i], geo.Rotation3d()))
                obj_points.append([p_field.X(), p_field.Y(), p_field.Z()])
                img_points.append(pts[i])

        if len(obj_points) < 8: return None
        obj_points = np.asarray(obj_points, dtype=np.float64)
        img_points = np.asarray(img_points, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.K, self.D, flags=cv2.SOLVEPNP_SQPNP)
        if not ok: return None
        proj, _ = cv2.projectPoints(obj_points, rvec, tvec, self.K, self.D)
        rms = float(np.sqrt(np.mean(np.linalg.norm(img_points - proj.reshape(-1, 2), axis=1) ** 2)))
        if rms > 10.0: return None
        robot_pose = self._compute_robot_pose(0, rvec, tvec, cam_orientation)  # ID 0 for multi
        best_tag = min(valid_tags, key=lambda x: float(x.get("dist", 1e9)))
        return {
            "id": -1, "in_layout": True,
            "tx": robot_pose.X(), "ty": robot_pose.Y(), "tz": robot_pose.Z(),
            "rx": robot_pose.rotation().X(), "ry": robot_pose.rotation().Y(), "rz": robot_pose.rotation().Z(),
            "dist": best_tag["dist"], "rotation": best_tag["rotation"], "strafe": best_tag["strafe"],
            "cx": 0, "cy": 0, "corners": [], "rvec": rvec, "tvec": tvec, "pnp_rms_px": rms
        }

    def _compute_robot_pose(self, tid, rvec, tvec, cam_orientation):
        # Universal pose compute for both Single (via ID) and Multi (via rvec/tvec)
        if tid == 0:
            # Multi-Tag: rvec/tvec are Field->Cam
            R_cv, _ = cv2.Rodrigues(rvec)
            C_field_cv = -R_cv.T @ tvec
            P = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
            R_wpi = (R_cv.T @ P.T)
            cam_pose_field = geo.Pose3d(geo.Translation3d(C_field_cv[0][0], C_field_cv[1][0], C_field_cv[2][0]),
                                        self._matrix_to_rotation3d(R_wpi))
        else:
            # Single Tag: rvec/tvec are Obj->Cam
            # Re-implementing single tag logic here to ensure _process_single_tag_new uses it
            tag_pose = self.layout.getTagPose(tid)
            if not tag_pose: return None
            R_cv, _ = cv2.Rodrigues(rvec)
            C_in_tag = -R_cv.T @ tvec
            P = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
            R_wpi = R_cv.T @ P.T
            cam_pose_in_tag = geo.Pose3d(geo.Translation3d(C_in_tag[0][0], C_in_tag[1][0], C_in_tag[2][0]),
                                         self._matrix_to_rotation3d(R_wpi))
            cam_pose_field = tag_pose.transformBy(geo.Transform3d(geo.Pose3d(), cam_pose_in_tag))

        if cam_orientation:
            so = cam_orientation
            robot_to_cam = geo.Transform3d(
                geo.Translation3d(so["tx"], so["ty"], so["tz"]),
                geo.Rotation3d(math.radians(so["rx"]), math.radians(so["ry"]), math.radians(so["rz"]))
            )
            return cam_pose_field.transformBy(robot_to_cam.inverse())
        return cam_pose_field

    def _quat_to_rvec(self, q):
        w, x, y, z = q.W(), q.X(), q.Y(), q.Z()
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        rvec, _ = cv2.Rodrigues(R)
        return rvec

    def _matrix_to_rotation3d(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        if sy < 1e-6:
            x, y, z = math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0
        else:
            x, y, z = math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
        return geo.Rotation3d(x, y, z)