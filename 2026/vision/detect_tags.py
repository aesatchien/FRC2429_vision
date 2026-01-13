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
        self.default_max_dist = config.get("max_tag_distance", 3.0)

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

        # history for continuity
        self._pnp_hist = {}  # key: tagId -> dict(rvec,tvec,ts, robot_pose)

    def detect(self, image, cam_orientation=None, max_distance=None):
        if max_distance is None:
            max_distance = self.default_max_dist

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        h, w = grey.shape[:2]
        if not hasattr(self, "_kcheck_printed"):
            self._kcheck_printed = True
            print(f"[Kcheck] grey={w}x{h} cam={self.cam.width}x{self.cam.height} "
                  f"fx={self.cam.fx:.2f} fy={self.cam.fy:.2f} cx={self.cam.cx:.2f} cy={self.cam.cy:.2f}")

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

    def _process_single_tag_cjh(self, tag, cam_orientation, max_distance):
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

        # Calculate RMS for filtering
        tag_size = 0.1651
        s = tag_size / 2.0
        obj_points = np.array([
            [-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]
        ], dtype=np.float64)
        
        K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]])
        D = self.cam.dist_coeffs if self.cam.dist_coeffs is not None else np.zeros(5)
        
        rvec_calc = self._quat_to_rvec(pose.rotation().getQuaternion())
        tvec_calc = np.array([pose.x, pose.y, pose.z])
        
        proj, _ = cv2.projectPoints(obj_points, rvec_calc, tvec_calc, K, D)
        e = proj.reshape(-1, 2) - np.array(corners).reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum(e * e, axis=1))))

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
            'rvec': rvec_calc,
            'tvec': tvec_calc,
            'pnp_rms_px': rms
        }

    def _process_single_tag(self, tag, cam_orientation, max_distance):
        # PhotonVision-like single-tag PnP: try IPPE_SQUARE then ITERATIVE, test 8 windings,
        # seed with history from self._pnp_hist, choose best by reprojection RMS.
        # Returns same schema as older functions plus diagnostic fields to help debug axis/flip issues.

        # --- intrinsics + distortion ---
        K = np.array([[self.cam.fx, 0.0, self.cam.cx],
                      [0.0, self.cam.fy, self.cam.cy],
                      [0.0, 0.0, 1.0]], dtype=np.float64)

        if getattr(self.cam, 'dist_coeffs', None) is not None:
            D = np.asarray(self.cam.dist_coeffs, dtype=np.float64).reshape(-1, 1)
        else:
            D = np.zeros((5, 1), dtype=np.float64)

        tid = int(tag.getId())

        # --- detector corners (4x2) ---
        c = tag.getCorners([0.0] * 8)
        if len(c) != 8:
            return None

        pts_det = np.array([
            [c[0], c[1]],
            [c[2], c[3]],
            [c[4], c[5]],
            [c[6], c[7]],
        ], dtype=np.float64)

        # --- tag-local model (XY plane, Z=0) ---
        tag_size = 0.1651
        s = tag_size / 2.0
        obj = np.array([
            [-s, -s, 0.0],
            [s, -s, 0.0],
            [s, s, 0.0],
            [-s, s, 0.0],
        ], dtype=np.float64)

        # --- reprojection RMS helper ---
        def reproj_rms(rvec, tvec, img_pts):
            try:
                proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
                proj = proj.reshape(-1, 2)
                e = proj - img_pts
                return float(np.sqrt(np.mean(np.sum(e * e, axis=1))))
            except Exception:
                return float('inf')

        # --- history / continuity ---
        hist = getattr(self, '_pnp_hist', None)
        if hist is None:
            self._pnp_hist = {}
            hist = self._pnp_hist

        prev = hist.get(tid)
        now = float(cv2.getTickCount() / cv2.getTickFrequency())

        # --- test 8 windings (4 cyclic shifts x 2 directions) ---
        tried_perms = []
        rms_list = []
        candidates = []  # tuples (rms, rvec, tvec, perm_idx, img_pts)
        rejected_physical = []

        # Precompute layout/tag pose and robot_to_cam if available for physical plausibility checks
        have_layout_pose = False
        robot_to_cam = None
        tag_field_pose = None
        if self.layout and cam_orientation:
            try:
                tag_field_pose = self.layout.getTagPose(tid)
                if tag_field_pose is not None:
                    have_layout_pose = True
                    so = cam_orientation
                    robot_to_cam = geo.Transform3d(
                        geo.Translation3d(so["tx"], so["ty"], so["tz"]),
                        geo.Rotation3d(math.radians(so["rx"]), math.radians(so["ry"]), math.radians(so["rz"]))
                    )
            except Exception:
                have_layout_pose = False
                robot_to_cam = None

        # Reasonable field bounds (meters) to reject wildly-unphysical candidates.
        # These are conservative defaults for FRC fields; change if you have custom field extents.
        FIELD_X_MIN, FIELD_X_MAX = 0.0, 17.0
        FIELD_Y_MIN, FIELD_Y_MAX = 0.0, 9.0
        FIELD_Z_MIN, FIELD_Z_MAX = -1.0, 10.0
        MAX_DIST_FROM_TAG = 50.0  # meters

        for k in range(4):
            p = np.roll(pts_det, -k, axis=0)
            for rev in (False, True):
                img = p[::-1].copy() if rev else p.copy()
                perm_idx = (k, rev)
                tried_perms.append(perm_idx)

                # Try IPPE_SQUARE (if available) as a fast, planar-optimized solver
                try:
                    ok = False
                    try:
                        ok, rvec_ippe, tvec_ippe = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        if ok:
                            rms_ippe = reproj_rms(rvec_ippe, tvec_ippe, img)
                            # Physical plausibility check for this candidate
                            plausible = True
                            if have_layout_pose:
                                try:
                                    R_cv_cand, _ = cv2.Rodrigues(rvec_ippe)
                                    C_tag_cand = -R_cv_cand.T @ tvec_ippe.reshape(3, 1)
                                    P = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
                                    R_cam2tag_cv_cand = R_cv_cand.T
                                    R_cam2tag_wpi_cand = R_cam2tag_cv_cand @ P.T
                                    cam_pose_tag_cand = geo.Pose3d(
                                        geo.Translation3d(float(C_tag_cand[0]), float(C_tag_cand[1]), float(C_tag_cand[2])),
                                        self._matrix_to_rotation3d(R_cam2tag_wpi_cand)
                                    )
                                    cam_pose_field_cand = tag_field_pose.transformBy(geo.Transform3d(geo.Pose3d(), cam_pose_tag_cand))
                                    cx, cy, cz = cam_pose_field_cand.X(), cam_pose_field_cand.Y(), cam_pose_field_cand.Z()
                                    # distance from tag
                                    tpos = tag_field_pose.translation()
                                    dx = cx - tpos.X(); dy = cy - tpos.Y(); dz = cz - tpos.Z()
                                    dtag = math.sqrt(dx*dx + dy*dy + dz*dz)
                                    if not (FIELD_X_MIN <= cx <= FIELD_X_MAX and FIELD_Y_MIN <= cy <= FIELD_Y_MAX and FIELD_Z_MIN <= cz <= FIELD_Z_MAX):
                                        plausible = False
                                    if dtag > MAX_DIST_FROM_TAG:
                                        plausible = False
                                except Exception:
                                    plausible = True

                            if plausible:
                                candidates.append((rms_ippe, rvec_ippe, tvec_ippe, perm_idx, img))
                                rms_list.append(rms_ippe)
                            else:
                                rejected_physical.append((perm_idx, 'IPPE', float(rms_ippe)))
                    except Exception:
                        # IPPE may not be available on all OpenCV builds; ignore failures
                        pass

                    # Try seeded ITERATIVE if we have a recent previous solution
                    seeded = False
                    if prev is not None and (now - prev.get('ts', 0.0)) < 0.5:
                        try:
                            rvec_seed = np.asarray(prev['rvec']).copy()
                            tvec_seed = np.asarray(prev['tvec']).copy()
                            ok, rvec_g, tvec_g = cv2.solvePnP(obj, img, K, D,
                                                              rvec=rvec_seed, tvec=tvec_seed,
                                                              useExtrinsicGuess=True,
                                                              flags=cv2.SOLVEPNP_ITERATIVE)
                            if ok:
                                rms_g = reproj_rms(rvec_g, tvec_g, img)
                                # physical plausibility
                                plausible = True
                                if have_layout_pose:
                                    try:
                                        R_cv_cand, _ = cv2.Rodrigues(rvec_g)
                                        C_tag_cand = -R_cv_cand.T @ tvec_g.reshape(3, 1)
                                        P = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
                                        R_cam2tag_cv_cand = R_cv_cand.T
                                        R_cam2tag_wpi_cand = R_cam2tag_cv_cand @ P.T
                                        cam_pose_tag_cand = geo.Pose3d(
                                            geo.Translation3d(float(C_tag_cand[0]), float(C_tag_cand[1]), float(C_tag_cand[2])),
                                            self._matrix_to_rotation3d(R_cam2tag_wpi_cand)
                                        )
                                        cam_pose_field_cand = tag_field_pose.transformBy(geo.Transform3d(geo.Pose3d(), cam_pose_tag_cand))
                                        cx, cy, cz = cam_pose_field_cand.X(), cam_pose_field_cand.Y(), cam_pose_field_cand.Z()
                                        tpos = tag_field_pose.translation()
                                        dx = cx - tpos.X(); dy = cy - tpos.Y(); dz = cz - tpos.Z()
                                        dtag = math.sqrt(dx*dx + dy*dy + dz*dz)
                                        if not (FIELD_X_MIN <= cx <= FIELD_X_MAX and FIELD_Y_MIN <= cy <= FIELD_Y_MAX and FIELD_Z_MIN <= cz <= FIELD_Z_MAX):
                                            plausible = False
                                        if dtag > MAX_DIST_FROM_TAG:
                                            plausible = False
                                    except Exception:
                                        plausible = True

                                if plausible:
                                    candidates.append((rms_g, rvec_g, tvec_g, perm_idx, img))
                                    rms_list.append(rms_g)
                                    seeded = True
                                else:
                                    rejected_physical.append((perm_idx, 'SEED', float(rms_g)))
                        except Exception:
                            seeded = False

                    # Always try a fresh ITERATIVE fallback
                    try:
                        ok, rvec_it, tvec_it = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
                        if ok:
                            rms_it = reproj_rms(rvec_it, tvec_it, img)
                            plausible = True
                            if have_layout_pose:
                                try:
                                    R_cv_cand, _ = cv2.Rodrigues(rvec_it)
                                    C_tag_cand = -R_cv_cand.T @ tvec_it.reshape(3, 1)
                                    P = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
                                    R_cam2tag_cv_cand = R_cv_cand.T
                                    R_cam2tag_wpi_cand = R_cam2tag_cv_cand @ P.T
                                    cam_pose_tag_cand = geo.Pose3d(
                                        geo.Translation3d(float(C_tag_cand[0]), float(C_tag_cand[1]), float(C_tag_cand[2])),
                                        self._matrix_to_rotation3d(R_cam2tag_wpi_cand)
                                    )
                                    cam_pose_field_cand = tag_field_pose.transformBy(geo.Transform3d(geo.Pose3d(), cam_pose_tag_cand))
                                    cx, cy, cz = cam_pose_field_cand.X(), cam_pose_field_cand.Y(), cam_pose_field_cand.Z()
                                    tpos = tag_field_pose.translation()
                                    dx = cx - tpos.X(); dy = cy - tpos.Y(); dz = cz - tpos.Z()
                                    dtag = math.sqrt(dx*dx + dy*dy + dz*dz)
                                    if not (FIELD_X_MIN <= cx <= FIELD_X_MAX and FIELD_Y_MIN <= cy <= FIELD_Y_MAX and FIELD_Z_MIN <= cz <= FIELD_Z_MAX):
                                        plausible = False
                                    if dtag > MAX_DIST_FROM_TAG:
                                        plausible = False
                                except Exception:
                                    plausible = True

                            if plausible:
                                candidates.append((rms_it, rvec_it, tvec_it, perm_idx, img))
                                rms_list.append(rms_it)
                            else:
                                rejected_physical.append((perm_idx, 'ITER', float(rms_it)))
                    except Exception:
                        pass

                except Exception:
                    # Guard outermost in case of unexpected failures
                    continue

        if not candidates:
            return None

        # Choose best candidate by RMS
        best = min(candidates, key=lambda x: x[0])
        rms, rvec, tvec, best_perm, best_img = best

        dist = float(tvec[2])

        # Gates: distance + reprojection error
        if dist <= 0 or dist > max_distance:
            return None
        if rms > 10.0:
            return None

        # --- invert TAG->CAM to get CAM pose in TAG frame ---
        R_cv, _ = cv2.Rodrigues(rvec)
        C_tag = -R_cv.T @ tvec.reshape(3, 1)

        # --- compose field pose if layout available ---
        tx = ty = tz = rx = ry = rz = 0.0
        in_layout = False
        robot_pose = None

        if self.layout and cam_orientation:
            tag_field_pose = self.layout.getTagPose(tid)
            if tag_field_pose:
                # Map CV camera->WPILib camera axes using same P as multi-tag
                P = np.array([
                    [0.0, 0.0, 1.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0],
                ], dtype=np.float64)

                # Camera pose relative to tag (WPILib axes)
                R_cam2tag_cv = R_cv.T
                R_cam2tag_wpi = R_cam2tag_cv @ P.T

                cam_pose_tag = geo.Pose3d(
                    geo.Translation3d(float(C_tag[0]), float(C_tag[1]), float(C_tag[2])),
                    self._matrix_to_rotation3d(R_cam2tag_wpi)
                )

                so = cam_orientation
                robot_to_cam = geo.Transform3d(
                    geo.Translation3d(so["tx"], so["ty"], so["tz"]),
                    geo.Rotation3d(math.radians(so["rx"]), math.radians(so["ry"]), math.radians(so["rz"]))
                )

                cam_pose_field = tag_field_pose.transformBy(geo.Transform3d(geo.Pose3d(), cam_pose_tag))
                robot_pose = cam_pose_field.transformBy(robot_to_cam.inverse())

                t = robot_pose.translation()
                r = robot_pose.rotation()
                tx, ty, tz = t.X(), t.Y(), t.Z()
                rx, ry, rz = r.X(), r.Y(), r.Z()
                in_layout = True

                # Plausibility check vs previous robot pose to prevent jumps
                if prev is not None and prev.get('robot_pose') is not None:
                    rp0 = prev['robot_pose']
                    dx = robot_pose.X() - rp0.X()
                    dy = robot_pose.Y() - rp0.Y()
                    dz = robot_pose.Z() - rp0.Z()
                    dpos = math.sqrt(dx * dx + dy * dy + dz * dz)

                    yaw = robot_pose.rotation().Z()
                    yaw0 = rp0.rotation().Z()
                    dyaw = (yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi

                    if dpos > 0.50 or abs(dyaw) > math.radians(25):
                        # reject this robot_pose and keep previous
                        robot_pose = rp0
                        t = robot_pose.translation()
                        r = robot_pose.rotation()
                        tx, ty, tz = t.X(), t.Y(), t.Z()
                        rx, ry, rz = r.X(), r.Y(), r.Z()

        # --- update history (store copies) ---
        try:
            hist[tid] = {"rvec": np.asarray(rvec).copy(), "tvec": np.asarray(tvec).copy(), "ts": now, "robot_pose": robot_pose}
        except Exception:
            hist[tid] = {"rvec": rvec, "tvec": tvec, "ts": now, "robot_pose": robot_pose}

        # --- targeting info ---
        center = tag.getCenter()
        fov_h = self.cam.get_fov()
        norm_x = (2.0 * center.x / self.cam.width) - 1.0
        rotation = -norm_x * (fov_h / 2.0)
        strafe = math.sin(math.radians(rotation)) * dist

        # Diagnostics to help diagnose axis/flip issues
        diag_R_cv = R_cv.tolist() if isinstance(R_cv, np.ndarray) else None
        try:
            # camera position in field (if available)
            cam_pos_field = None
            if in_layout and 'cam_pose_field' in locals():
                cam_pos_field = [cam_pose_field.X(), cam_pose_field.Y(), cam_pose_field.Z()]
        except Exception:
            cam_pos_field = None

        # Summarize candidates and rejected_physical for diagnostics
        try:
            diag_candidates = [{'perm': c[3], 'rms': float(c[0])} for c in candidates]
        except Exception:
            diag_candidates = None

        diag = {
            'diag_tried_perms': tried_perms,
            'diag_rms_list': rms_list,
            'diag_best_perm': best_perm,
            'diag_candidates': diag_candidates,
            'diag_rejected_physical': rejected_physical,
            'diag_R_cv': diag_R_cv,
            'diag_R_wpi': (R_cam2tag_wpi.tolist() if 'R_cam2tag_wpi' in locals() else None),
            'diag_cam_pos_field': cam_pos_field,
        }

        # --- return result ---
        res = {
            'id': tid,
            'in_layout': in_layout,
            'tx': tx, 'ty': ty, 'tz': tz,
            'rx': rx, 'ry': ry, 'rz': rz,
            'dist': dist,
            'cx': center.x, 'cy': center.y,
            'rotation': rotation,
            'strafe': strafe,
            'corners': c,
            'rvec': rvec,
            'tvec': tvec,
            'pnp_rms_px': rms,
        }
        res.update(diag)
        return res

    def _process_multi_tag(self, tag_results, cam_orientation):
        """
        Multi-tag PnP using field-layout corner coordinates.

        - Builds 3D object points in FIELD coordinates (WPILib field layout coords).
        - Uses OpenCV solvePnP ITERATIVE (works with robotpy_apriltag corner points).
        - Applies a fixed, empirically-verified corner permutation per tag:
            perm = ('rot', 1, True)  => roll by -1 then reverse winding.

        Conventions:
          OpenCV solvePnP returns FIELD -> CAMERA(CV):
            X_cam = R * X_field + t

          Camera position in field:
            C_field = -R^T * t

          Camera orientation in field:
            R_c2f_cv = R^T (maps camera(CV) vectors -> field vectors)

          Convert camera axes CV -> WPILib camera axes via:
            v_wpi = P * v_cv
            P = [[ 0, 0, 1],
                 [-1, 0, 0],
                 [ 0,-1, 0]]

          So:
            v_field = R_c2f_cv * v_cv = R_c2f_cv * P^T * v_wpi
            => R_c2f_wpi = R_c2f_cv @ P.T
        """
        valid_tags = [t for t in tag_results if t.get("in_layout")]
        if len(valid_tags) < 2 or self.layout is None:
            return None

        # Intrinsics (no distortion)
        K = np.array([[self.cam.fx, 0.0, self.cam.cx],
                      [0.0, self.cam.fy, self.cam.cy],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
        D = np.zeros((5, 1), dtype=np.float64)

        tag_size = 0.1651
        s = tag_size / 2.0

        # Tag-local corners (same model used in your single-tag ITERATIVE test)
        local_corners = np.array([
            [-s, -s, 0.0],
            [s, -s, 0.0],
            [s, s, 0.0],
            [-s, s, 0.0],
        ], dtype=np.float64)

        # Empirically verified permutation for robotpy_apriltag corners:
        # perm=('rot',1,True) means: roll by -1, then reverse.
        def apply_perm(pts4: np.ndarray) -> np.ndarray:
            p = np.roll(pts4, -1, axis=0)
            p = p[::-1].copy()
            return p

        # Rotation3d -> 3x3 matrix (no assumptions about API surface)
        def rot3d_to_matrix(r: geo.Rotation3d) -> np.ndarray:
            q = r.getQuaternion()
            w, x, y, z = q.W(), q.X(), q.Y(), q.Z()
            return np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ], dtype=np.float64)

        obj_points = []
        img_points = []

        for tag_res in valid_tags:
            tid = int(tag_res["id"])
            tag_field_pose = self.layout.getTagPose(tid)
            if not tag_field_pose:
                continue

            c = tag_res.get("corners", [])
            if len(c) != 8:
                continue

            pts = np.array([
                [c[0], c[1]],
                [c[2], c[3]],
                [c[4], c[5]],
                [c[6], c[7]],
            ], dtype=np.float64)

            # Apply the proven permutation so 2D corners line up with our local_corners
            pts = apply_perm(pts)

            # Field <- Tag pose from layout
            t_f = tag_field_pose.translation()
            R_f = rot3d_to_matrix(tag_field_pose.rotation())
            t_f = np.array([t_f.X(), t_f.Y(), t_f.Z()], dtype=np.float64)

            # Add 4 correspondences
            # X_field = R_f * X_tag + t_f
            X_field = (R_f @ local_corners.T).T + t_f.reshape(1, 3)

            obj_points.extend(X_field.tolist())
            img_points.extend(pts.tolist())

        if len(obj_points) < 8:
            return None

        obj_points = np.asarray(obj_points, dtype=np.float64)
        img_points = np.asarray(img_points, dtype=np.float64)

        # Solve FIELD -> CAMERA(CV)
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            K,
            D,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return None

        # Compute RMS (optional gate; set None/large if you don't want gating yet)
        proj, _ = cv2.projectPoints(obj_points, rvec, tvec, K, D)
        proj = proj.reshape(-1, 2)
        e = proj - img_points
        rms = float(np.sqrt(np.mean(np.sum(e * e, axis=1))))

        # Invert to get camera pose in field
        R_f2c, _ = cv2.Rodrigues(rvec)
        R_c2f_cv = R_f2c.T
        t_f2c = tvec.reshape(3, 1)
        C_field = -R_c2f_cv @ t_f2c  # 3x1

        # Convert camera rotation CV axes -> WPILib camera axes
        P = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ], dtype=np.float64)
        R_c2f_wpi = R_c2f_cv @ P.T

        cam_pose_field = geo.Pose3d(
            geo.Translation3d(float(C_field[0]), float(C_field[1]), float(C_field[2])),
            self._matrix_to_rotation3d(R_c2f_wpi)
        )

        # Apply camera extrinsics to get ROBOT pose in field
        if cam_orientation:
            so = cam_orientation
            robot_to_cam = geo.Transform3d(
                geo.Translation3d(so["tx"], so["ty"], so["tz"]),
                geo.Rotation3d(math.radians(so["rx"]),
                               math.radians(so["ry"]),
                               math.radians(so["rz"]))
            )
            robot_pose = cam_pose_field.transformBy(robot_to_cam.inverse())
        else:
            robot_pose = cam_pose_field

        # Preserve your prior behavior: choose closest tag's targeting info
        best_tag = min(valid_tags, key=lambda x: float(x.get("dist", 1e9)))

        return {
            "id": -1,
            "in_layout": True,
            "tx": robot_pose.X(), "ty": robot_pose.Y(), "tz": robot_pose.Z(),
            "rx": robot_pose.rotation().X(),
            "ry": robot_pose.rotation().Y(),
            "rz": robot_pose.rotation().Z(),
            "dist": float(best_tag.get("dist", 0.0)),
            "rotation": best_tag.get("rotation", 0.0),
            "strafe": best_tag.get("strafe", 0.0),
            "cx": 0, "cy": 0,
            "corners": [],
            "rvec": rvec,
            "tvec": tvec,
            "pnp_rms_px": rms,
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

    def _order_corners_tl_tr_br_bl(self, pts4):
        """
        pts4: (4,2) array of pixel corners in arbitrary order.
        Returns corners in TL, TR, BR, BL order (image coords: +x right, +y down).
        """
        pts = np.asarray(pts4, dtype=np.float64).reshape(4, 2)

        # centroid + angle sort gives a consistent cyclic order
        c = np.mean(pts, axis=0)
        ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
        cyc = pts[np.argsort(ang)]  # CCW around centroid in image coords

        # rotate so index 0 is top-left (min x+y)
        i0 = np.argmin(cyc[:, 0] + cyc[:, 1])
        cyc = np.roll(cyc, -i0, axis=0)

        # Now cyc[0] is TL. Decide if cyc is TL,TR,BR,BL or TL,BL,BR,TR
        # Compare x of the next two points: TR should have larger x than BL.
        if cyc[1, 0] < cyc[3, 0]:
            # swap to enforce TL,TR,BR,BL
            cyc = np.array([cyc[0], cyc[3], cyc[2], cyc[1]], dtype=np.float64)

        return cyc
