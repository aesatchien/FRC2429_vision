import cv2
import numpy as np
import math
from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Transform3d, Quaternion, CoordinateSystem


class PhotonOps:
    """
    Stateless mathematical operations for robust AprilTag localization.
    """

    @staticmethod
    def solve_single_tag(corners, K, D, tag_size, tag_field_pose=None, prev_robot_pose=None, camera_to_robot=None):
        """
        Solves for single tag pose using IPPE_SQUARE with ambiguity resolution.

        Args:
            corners: (4, 2) numpy array. Expects detector order [TL, TR, BR, BL].
            K, D: Intrinsics.
            tag_size: Physical size (meters).
            tag_field_pose: (Optional) Pose3d of the tag on the field.
            prev_robot_pose: (Optional) Pose3d of the robot from the previous frame.
            camera_to_robot: (Optional) Transform3d from Camera to Robot (necessary if using history).

        Returns:
            dict: { 'rvec', 'tvec', 'error', 'ambiguity', 'robot_pose': Pose3d } or None
        """
        s = tag_size
        # IPPE_SQUARE strict order: Bottom-Left, Bottom-Right, Top-Right, Top-Left
        object_points = np.array([
            [-s / 2, s / 2, 0],  # BL
            [s / 2, s / 2, 0],  # BR
            [s / 2, -s / 2, 0],  # TR
            [-s / 2, -s / 2, 0]  # TL
        ], dtype=np.float64).reshape(4, 1, 3)

        # Map Detector [TL, TR, BR, BL] -> IPPE [BL, BR, TR, TL]
        # TL(0) -> IPPE(3)
        # TR(1) -> IPPE(2)
        # BR(2) -> IPPE(1)
        # BL(3) -> IPPE(0)
        img_ordered = corners[[3, 2, 1, 0], :].astype(np.float64).reshape(4, 1, 2)

        K = K.astype(np.float64)
        D = D.astype(np.float64).reshape(-1, 1)

        try:
            # Robust unpacking (handles varying OpenCV versions)
            res = cv2.solvePnPGeneric(
                object_points, img_ordered, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            # Find the vector lists in the return tuple
            rvecs, tvecs = None, None
            for item in res:
                if isinstance(item, (tuple, list)) and len(item) > 0:
                    if isinstance(item[0], np.ndarray) and item[0].size == 3:
                        if rvecs is None:
                            rvecs = item
                        elif tvecs is None:
                            tvecs = item

            if rvecs is None or len(rvecs) == 0: return None

        except Exception:
            return None

        # --- CANDIDATE EVALUATION ---
        candidates = []
        obj_pts_flat = object_points.reshape(4, 3)

        for i in range(len(rvecs)):
            r, t = rvecs[i], tvecs[i]

            # 1. Reprojection Error
            proj, _ = cv2.projectPoints(obj_pts_flat, r, t, K, D)
            err = cv2.norm(img_ordered.reshape(4, 2), proj.reshape(4, 2), cv2.NORM_L2)

            # 2. Physical Consistency (if history is available)
            dist_to_history = float('inf')

            if tag_field_pose and prev_robot_pose and camera_to_robot:
                # Calculate what the Robot Pose WOULD be for this solution
                try:
                    # CV Camera -> Field
                    pose_cam_field = PhotonOps.get_cam_pose_in_field(r, t, tag_field_pose)
                    # Field -> Robot
                    pose_robot = pose_cam_field.transformBy(camera_to_robot)

                    # Distance to previous robot pose
                    dist_to_history = pose_robot.translation().distance(prev_robot_pose.translation())
                except Exception:
                    pass

            candidates.append({'rvec': r, 'tvec': t, 'err': err, 'dist_hist': dist_to_history})

        # --- SELECTION LOGIC ---
        # Sort by reprojection error first (default)
        candidates.sort(key=lambda x: x['err'])

        best = candidates[0]
        alt = candidates[1] if len(candidates) > 1 else None

        ambiguity = 0.0
        if alt:
            ambiguity = best['err'] / (alt['err'] + 1e-9)

            # SPECIAL SAUCE: Disambiguate using History
            # If ambiguity is high (>0.15 is typical "danger zone") AND we have history data
            if ambiguity > 0.15 and best['dist_hist'] != float('inf'):
                # Check if the "worse" solution is actually physically closer to where we were
                # Threshold: If Alternate is < 1 meter away, and Best is > 2 meters away? Swap.
                if alt['dist_hist'] < 0.5 and best['dist_hist'] > 1.0:
                    # SWAP: History overrides Reprojection Error
                    best, alt = alt, best
                    # Recalculate ambiguity to reflect confidence in the SWAP?
                    # No, keep raw ambiguity metric, but we chose the "right" one.

        return {
            'rvec': best['rvec'],
            'tvec': best['tvec'],
            'error': best['err'],
            'ambiguity': ambiguity
        }

    @staticmethod
    def solve_multi_tag(detections, field_layout, K, D, tag_size=0.1651):
        """
        Performs a single rigid-body solve using all visible tags via SOLVEPNP_SQPNP.
        Returns: { 'rvec', 'tvec', 'rmse', 'field_pose': Pose3d (Camera in Field) }
        """
        obj_points = []
        img_points = []

        s = tag_size / 2.0
        # Local corners in WPILib Tag Frame (X-fwd, Y-left, Z-up)
        # Order: TL, TR, BR, BL (Detector Standard)
        local_corners = [
            Translation3d(0.0, s, s),  # TL
            Translation3d(0.0, -s, s),  # TR
            Translation3d(0.0, -s, -s),  # BR
            Translation3d(0.0, s, -s),  # BL
        ]

        valid_count = 0
        for det in detections:
            tag_id = int(det['id'])
            corners = det['corners']
            tag_pose = field_layout.getTagPose(tag_id)
            if tag_pose is None: continue

            valid_count += 1
            for i, pt_local in enumerate(local_corners):
                global_pt = tag_pose.transformBy(Transform3d(pt_local, Rotation3d()))
                obj_points.append([global_pt.X(), global_pt.Y(), global_pt.Z()])
                img_points.append(corners[i])

        if valid_count < 1 or len(obj_points) < 4: return None

        obj_points = np.array(obj_points, dtype=np.float64)
        img_points = np.array(img_points, dtype=np.float64)
        K = K.astype(np.float64)
        D = D.astype(np.float64).reshape(-1, 1)

        try:
            # SQPNP is the gold standard for multi-point
            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, K, D, flags=cv2.SOLVEPNP_SQPNP
            )
        except Exception:
            return None

        if not success: return None

        # RMSE
        proj, _ = cv2.projectPoints(obj_points, rvec, tvec, K, D)
        err = cv2.norm(img_points.reshape(-1, 2), proj.reshape(-1, 2), cv2.NORM_L2)
        rmse = err / np.sqrt(len(obj_points))

        # Convert to Camera Pose in Field
        # We solved: Field -> CameraCV
        # We want: CameraWPI in Field

        # 1. Get Camera Pose in Field (CV Frame)
        R_cv, _ = cv2.Rodrigues(rvec)
        C_field = -R_cv.T @ tvec

        # 2. Get Rotation of Camera in Field (WPI Frame)
        # Construct Rotation Matrix from Basis Vectors
        R_f2c_T = R_cv.T
        c_fwd = R_f2c_T[:, 2]  # Z_cv is Forward
        c_left = -R_f2c_T[:, 0]  # -X_cv is Left
        c_up = -R_f2c_T[:, 1]  # -Y_cv is Up

        R_wpi = np.column_stack((c_fwd, c_left, c_up))

        rot_wpi = PhotonOps.matrix_to_rotation3d(R_wpi)
        trans_wpi = Translation3d(C_field[0][0], C_field[1][0], C_field[2][0])

        return {
            'rvec': rvec,
            'tvec': tvec,
            'rmse': rmse,
            'field_pose': Pose3d(trans_wpi, rot_wpi)
        }

    @staticmethod
    def get_cam_pose_in_field(rvec, tvec, tag_field_pose):
        """
        Calculates Camera Pose in Field given a Single Tag solve (Camera->Tag).
        """
        # 1. Transform Camera(CV) -> Tag(CV Local)
        # T_cam_to_tag
        # Wait, solvePnP gives T_tag_to_cam (Object->Camera)
        # X_cam = R * X_obj + t

        # We want X_obj = R^T * (X_cam - t)
        # So Camera Position in Object Frame = -R^T * t
        R, _ = cv2.Rodrigues(rvec)
        cam_in_tag_cv = -R.T @ tvec

        # Camera Rotation in Object Frame
        # R maps Tag->Cam. R^T maps Cam->Tag.
        R_cam2tag = R.T

        # 2. Convert to WPILib Axes (Tag Frame)
        # Tag Frame: X-out, Y-left, Z-up
        # CV Frame:  Z-out, X-right, Y-down
        # We need to map the Camera's basis vectors into the Tag's frame

        # Cam Forward (Z_cv) in Tag Frame
        c_fwd = R_cam2tag[:, 2]
        # Cam Left    (-X_cv) in Tag Frame
        c_left = -R_cam2tag[:, 0]
        # Cam Up      (-Y_cv) in Tag Frame
        c_up = -R_cam2tag[:, 1]

        R_cam_in_tag_wpi = np.column_stack((c_fwd, c_left, c_up))

        # Construct Pose3d of Camera relative to Tag
        rot = PhotonOps.matrix_to_rotation3d(R_cam_in_tag_wpi)
        trans = Translation3d(cam_in_tag_cv[0][0], cam_in_tag_cv[1][0], cam_in_tag_cv[2][0])
        pose_cam_rel_tag = Pose3d(trans, rot)

        # 3. Combine with Tag's Field Pose
        # FieldPose = TagFieldPose * CamRelTag
        return tag_field_pose.transformBy(Transform3d(pose_cam_rel_tag.translation(), pose_cam_rel_tag.rotation()))

    @staticmethod
    def matrix_to_rotation3d(R):
        """Robust conversion of 3x3 Matrix to WPILib Rotation3d via Quaternion."""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        return Rotation3d(Quaternion(w, x, y, z))