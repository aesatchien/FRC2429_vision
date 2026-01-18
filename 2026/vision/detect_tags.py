"""
CRITICAL SYSTEM CONTEXT - READ BEFORE MODIFYING
===============================================
This vision pipeline solves for FRC AprilTag poses using a Dual Hypothesis strategy.

1. HARDWARE REALITY:
   - Cameras are mounted in LANDSCAPE orientation.
   - Do NOT assume or suggest the cameras are physically rotated/portrait.

2. COORDINATE MAPPING (THE "SHIFT 2"):
   - Problem: The raw detector corner order (0,1,2,3) does not align with the
     standard object points for solvePnP in this specific software stack.
   - Solution: We apply a LOGICAL index shift (Corner 0 maps to Top-Right).
     
     Detector Output (Standard AprilTag Library):
       Index 0: Bottom-Left
       Index 1: Bottom-Right
       Index 2: Top-Right
       Index 3: Top-Left
     
     Solver Expectation (Standard Object Points):
       We map these to align with a Z-up (WPILib) or Z-forward (OpenCV) model.
   - Cause: Mismatch between Apriltag library winding order and WPILib/OpenCV
     axis definitions. This is a SOFTWARE patch, not a physical description.

3. THE "GHOST" SOLUTION:
   - Problem: Planar tags produce two valid mathematical poses (Ambiguity).
   - Strategy: "Dual Hypothesis." We explicitly calculate BOTH solutions
     (Standard vs Flipped) and use the Single-Tag Consensus to vote for the winner.
   - Do NOT use iterative solvers without this consensus check.
   - TODO - figure out if the single tags are flipped too

4. CONSTANTS:
   - Tag Size is dynamic (from config). Never hardcode 0.1651m.
"""

import logging
import cv2
import numpy as np
import math
import robotpy_apriltag as ra
import wpimath.geometry as geo
from wpimath import objectToRobotPose

log = logging.getLogger("tags")

class TagDetector:
    def __init__(self, camera_model, field_layout=None, config=None):
        self.cam = camera_model
        if config is None: config = {}

        self.detector = ra.AprilTagDetector()
        self.detector.addFamily('tag36h11')

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
        self.tag_size = config.get("tag_size", 0.1651)

        self.use_distortions = config.get("use_distortions", False)
        self.map1, self.map2 = None, None
        if self.use_distortions and self.cam.dist_coeffs is not None:
            K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]])
            D = self.cam.dist_coeffs
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                K, D, None, K, (self.cam.width, self.cam.height), cv2.CV_16SC2)

        est_config = ra.AprilTagPoseEstimator.Config(
            tagSize=self.tag_size, fx=self.cam.fx, fy=self.cam.fy, cx=self.cam.cx, cy=self.cam.cy
        )
        self.estimator = ra.AprilTagPoseEstimator(est_config)

        if field_layout is None:
            try:
                field = ra.AprilTagField.k2024Crescendo
                self.layout = ra.AprilTagFieldLayout.loadField(field)
            except:
                self.layout = None
        else:
            self.layout = field_layout

        self._pnp_hist = {}

        self.K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]], dtype=np.float64)
        
        # If we pre-undistort the image, PnP should assume a perfect pinhole (D=0).
        # Otherwise, PnP needs the distortion coefficients to match points correctly.
        if self.use_distortions:
            self.D = np.zeros((5, 1), dtype=np.float64)
        else:
            self.D = np.array(self.cam.dist_coeffs) if self.cam.dist_coeffs is not None else np.zeros((5, 1))

        # Standard Object Points (BL, BR, TR, TL)
        s = self.tag_size / 2.0
        self.std_obj_points = np.array([
            [0.0, s, -s], [0.0, -s, -s], [0.0, -s, s], [0.0, s, s]
        ], dtype=np.float64)

        log.info(f"Initialized TagDetector for {self.cam.width}x{self.cam.height}")
        log.info(f"  Config: {config}")
        log.info(f"  Intrinsics: fx={self.cam.fx:.1f} fy={self.cam.fy:.1f} cx={self.cam.cx:.1f} cy={self.cam.cy:.1f}")
        log.info(f"  Distortions: {self.cam.dist_coeffs if any(self.cam.dist_coeffs) else 'None'} (Used in PnP: {not self.use_distortions})")

    def detect(self, image, cam_orientation=None, max_distance=None, robot_pose=None):
        """
        Wrapper that runs both detection and pose estimation sequentially.
        Used by local_tester.py.
        """
        tags = self.detect_tags(image)
        return self.solve_poses(tags, cam_orientation, max_distance, robot_pose)

    def detect_tags(self, image):
        """Phase 1: Image processing to find tag corners."""
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.use_distortions and self.map1 is not None:
            grey = cv2.remap(grey, self.map1, self.map2, cv2.INTER_LINEAR)

        detections = self.detector.detect(grey)
        return [t for t in detections if t.getDecisionMargin() > self.min_margin and t.getHamming() <= self.max_hamming]

    def solve_poses(self, valid_tags, cam_orientation=None, max_distance=None, robot_pose=None):
        """Phase 2: Math to convert tag corners to 3D poses."""
        if max_distance is None: max_distance = self.default_max_dist
        results = {}
        processed_tags = []
        for tag in valid_tags:
            # 1. Single Tag (New Solver)
            res = self._process_single_tag_new(tag, cam_orientation, max_distance)

            # 2. Fallback
            if not res:
                res = self._process_single_tag_cjh(tag, cam_orientation, max_distance)

            if res:
                results[f'tag{tag.getId():02d}'] = res
                if res.get('in_layout'): processed_tags.append(res)

        # 3. Multi-Tag (Dual Hypothesis)
        if self.allow_multi_tag and len(processed_tags) > 1:
            multi_res = self._process_multi_tag_dual(valid_tags, processed_tags, cam_orientation)
            if multi_res:
                results['multi_tag'] = multi_res

        return results

    def _process_single_tag_new(self, tag, cam_orientation, max_distance):
        tid = int(tag.getId())
        c = tag.getCorners([0.0] * 8)
        # Mapping: Shift 2 (0=TR)
        img_points = np.array([
            [c[4], c[5]], [c[6], c[7]], [c[0], c[1]], [c[2], c[3]]
        ], dtype=np.float64)

        try:
            ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(self.std_obj_points, img_points, self.K, self.D,
                                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except:
            return None
        if not ok or len(rvecs) == 0: return None

        best_idx = -1
        best_score = -1e9
        history = self._pnp_hist.get(tid)

        for i in range(len(rvecs)):
            r, t = rvecs[i], tvecs[i]
            if t[2] < 0: continue
            score = 0.0

            pose_field = self._compute_robot_pose(tid, r, t, cam_orientation)
            if pose_field and self._is_on_field(pose_field):
                score += 5000.0
            else:
                score -= 5000.0

            if history:
                dist = np.linalg.norm(t - history['tvec'])
                if dist < 0.2:
                    score += 500.0
                elif dist > 1.0:
                    score -= 200.0

            proj, _ = cv2.projectPoints(self.std_obj_points, r, t, self.K, self.D)
            rms = np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_points) ** 2, axis=1)))
            score -= (rms * 10.0)

            if score > best_score:
                best_score = score
                best_idx = i

        if best_idx == -1: return None
        rvec, tvec = rvecs[best_idx], tvecs[best_idx]

        proj, _ = cv2.projectPoints(self.std_obj_points, rvec, tvec, self.K, self.D)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_points) ** 2, axis=1))))

        if rms < 3.0: self._pnp_hist[tid] = {'rvec': rvec, 'tvec': tvec}
        if rms > 10.0 or tvec[2] > max_distance: return None

        robot_pose = self._compute_robot_pose(tid, rvec, tvec, cam_orientation)
        if not robot_pose: return None

        center = tag.getCenter()
        fov_h = self.cam.get_fov()
        norm_x = (2.0 * center.x / self.cam.width) - 1.0
        return {
            'id': tid, 'in_layout': True,
            'tx': robot_pose.X(), 'ty': robot_pose.Y(), 'tz': robot_pose.Z(),
            'rx': robot_pose.rotation().X(), 'ry': robot_pose.rotation().Y(), 'rz': robot_pose.rotation().Z(),
            'dist': float(tvec[2]), 'cx': center.x, 'cy': center.y,
            'rotation': -norm_x * (fov_h / 2.0),
            'strafe': math.sin(math.radians(-norm_x * (fov_h / 2.0))) * tvec[2],
            'corners': c, 'rvec': rvec, 'tvec': tvec, 'pnp_rms_px': rms
        }

    def _process_multi_tag_dual(self, raw_tags, valid_single_results, cam_orientation):
        valid_raw = [t for t in raw_tags if self.layout.getTagPose(int(t.getId()))]
        if len(valid_raw) < 2 or not valid_single_results: return None

        s = self.tag_size / 2.0

        # Candidate A: Shift 2 (Standard CCW)
        corners_A = [
            geo.Translation3d(0.0, -s, s), geo.Translation3d(0.0, s, s),
            geo.Translation3d(0.0, s, -s), geo.Translation3d(0.0, -s, -s)
        ]

        # Candidate B: Rev S1 (Mirrored CW - "The Ghost")
        corners_B = [
            geo.Translation3d(0.0, -s, -s), geo.Translation3d(0.0, s, -s),
            geo.Translation3d(0.0, s, s), geo.Translation3d(0.0, -s, s)
        ]

        obj_pts_A, obj_pts_B, img_points = [], [], []

        for tag_res in valid_raw:
            tag_pose = self.layout.getTagPose(int(tag_res.getId()))
            if not tag_pose: continue
            c = tag_res.getCorners([0] * 8)
            pts = [[c[0], c[1]], [c[2], c[3]], [c[4], c[5]], [c[6], c[7]]]
            for i in range(4):
                pA = tag_pose.transformBy(geo.Transform3d(corners_A[i], geo.Rotation3d()))
                pB = tag_pose.transformBy(geo.Transform3d(corners_B[i], geo.Rotation3d()))

                obj_pts_A.append([pA.X(), pA.Y(), pA.Z()])
                obj_pts_B.append([pB.X(), pB.Y(), pB.Z()])
                img_points.append(pts[i])

        np_img = np.asarray(img_points, dtype=np.float64)
        np_obj_A = np.asarray(obj_pts_A, dtype=np.float64)
        np_obj_B = np.asarray(obj_pts_B, dtype=np.float64)

        # Solve BOTH explicitly
        okA, rvecA, tvecA = cv2.solvePnP(np_obj_A, np_img, self.K, self.D, flags=cv2.SOLVEPNP_SQPNP)
        okB, rvecB, tvecB = cv2.solvePnP(np_obj_B, np_img, self.K, self.D, flags=cv2.SOLVEPNP_SQPNP)

        candidates = []
        if okA:
            projA, _ = cv2.projectPoints(np_obj_A, rvecA, tvecA, self.K, self.D)
            rmsA = float(np.sqrt(np.mean(np.linalg.norm(np_img - projA.reshape(-1, 2), axis=1) ** 2)))
            candidates.append({'rvec': rvecA, 'tvec': tvecA, 'rms': rmsA})

        if okB:
            projB, _ = cv2.projectPoints(np_obj_B, rvecB, tvecB, self.K, self.D)
            rmsB = float(np.sqrt(np.mean(np.linalg.norm(np_img - projB.reshape(-1, 2), axis=1) ** 2)))
            candidates.append({'rvec': rvecB, 'tvec': tvecB, 'rms': rmsB})

        if not candidates: return None

        # Calculate Consensus (Average Single Tag Position)
        txs = [r['tx'] for r in valid_single_results]
        tys = [r['ty'] for r in valid_single_results]
        avg_x = sum(txs) / len(txs)
        avg_y = sum(tys) / len(tys)

        # The Judge: Pick candidate closest to consensus
        best_cand = None
        best_diff = 1e9
        best_pose = None

        for cand in candidates:
            # Reconstruct Robot Pose
            pose = self._compute_robot_pose(0, cand['rvec'], cand['tvec'], cam_orientation)

            diff = math.sqrt((pose.X() - avg_x) ** 2 + (pose.Y() - avg_y) ** 2)

            if diff < best_diff and cand['rms'] < 15.0:
                best_diff = diff
                best_cand = cand
                best_pose = pose

        if best_cand and best_diff < 1.5:
            best_tag = min(valid_single_results, key=lambda x: x['dist'])
            return {
                "id": best_tag['id'], "in_layout": True, "is_multi_tag": True,
                "tx": best_pose.X(), "ty": best_pose.Y(), "tz": best_pose.Z(),
                "rx": best_pose.rotation().X(), "ry": best_pose.rotation().Y(), "rz": best_pose.rotation().Z(),
                "dist": best_tag["dist"], "rotation": best_tag["rotation"], "strafe": best_tag["strafe"],
                "cx": 0, "cy": 0, "corners": [],
                "rvec": best_cand['rvec'], "tvec": best_cand['tvec'],
                "pnp_rms_px": best_cand['rms']
            }

        return None

    def _compute_robot_pose(self, tid, rvec, tvec, cam_orientation):
        if tid == 0:
            R_cv, _ = cv2.Rodrigues(rvec)
            C_field_cv = (-R_cv.T @ tvec).flatten()
            P = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
            R_wpi = (R_cv.T @ P.T)
            cam_pose_field = geo.Pose3d(geo.Translation3d(C_field_cv[0], C_field_cv[1], C_field_cv[2]),
                                        self._matrix_to_rotation3d(R_wpi))
        else:
            tag_pose = self.layout.getTagPose(tid)
            if not tag_pose: return None
            R_cv, _ = cv2.Rodrigues(rvec)
            C_in_tag = (-R_cv.T @ tvec).flatten()
            P = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
            R_wpi = R_cv.T @ P.T
            cam_pose_in_tag = geo.Pose3d(geo.Translation3d(C_in_tag[0], C_in_tag[1], C_in_tag[2]),
                                         self._matrix_to_rotation3d(R_wpi))
            cam_pose_field = tag_pose.transformBy(geo.Transform3d(geo.Pose3d(), cam_pose_in_tag))

        if cam_orientation:
            so = cam_orientation
            robot_to_cam = geo.Transform3d(geo.Translation3d(so["tx"], so["ty"], so["tz"]),
                                           geo.Rotation3d(math.radians(so["rx"]), math.radians(so["ry"]),
                                                          math.radians(so["rz"])))
            return cam_pose_field.transformBy(robot_to_cam.inverse())
        return cam_pose_field

    def _is_on_field(self, pose):
        x, y, z = pose.X(), pose.Y(), pose.Z()
        # Strict constraints: Positive X (0-16.55m), Reasonable Y, Tight Z
        return (0.0 < x < 16.55) and (-1.0 < y < 10.0) and (-1.0 < z < 2.5)

    def _process_single_tag_cjh(self, tag, cam_orientation, max_distance):
        # Fallback
        pose_est = self.estimator.estimateOrthogonalIteration(tag, 50)
        pose = pose_est.pose1
        if pose.z > max_distance: return None
        tx, ty, tz, rx, ry, rz = 0, 0, 0, 0, 0, 0
        in_layout = False
        if self.layout and cam_orientation:
            try:
                so = cam_orientation
                robot_to_cam = geo.Transform3d(geo.Translation3d(so['tx'], so['ty'], so['tz']),
                                               geo.Rotation3d(math.radians(so['rx']), math.radians(so['ry']),
                                                              math.radians(so['rz'])))
                pose_camera = geo.Transform3d(geo.Translation3d(pose.x, pose.y, pose.z),
                                              geo.Rotation3d(-pose.rotation().x - np.pi, -pose.rotation().y,
                                                             pose.rotation().z - np.pi))
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
        s = self.tag_size / 2.0
        obj_points = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float64)
        K = np.array([[self.cam.fx, 0, self.cam.cx], [0, self.cam.fy, self.cam.cy], [0, 0, 1]])
        D = self.cam.dist_coeffs if self.cam.dist_coeffs is not None else np.zeros(5)
        rvec_calc = self._quat_to_rvec(pose.rotation().getQuaternion())
        tvec_calc = np.array([[pose.x], [pose.y], [pose.z]])
        proj, _ = cv2.projectPoints(obj_points, rvec_calc, tvec_calc, K, D)
        e = proj.reshape(-1, 2) - np.array(corners).reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum(e * e, axis=1))))
        return {
            'id': tag.getId(), 'in_layout': in_layout, 'tx': tx, 'ty': ty, 'tz': tz,
            'rx': rx, 'ry': ry, 'rz': rz, 'dist': pose.z, 'cx': center.x, 'cy': center.y,
            'rotation': rotation, 'strafe': strafe, 'corners': corners,
            'rvec': rvec_calc, 'tvec': tvec_calc, 'pnp_rms_px': rms
        }

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