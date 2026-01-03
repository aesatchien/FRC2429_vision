import numpy as np
import cv2

class CameraModel:
    def __init__(self, width, height, camera_type, intrinsics=None, distortions=None):
        self.width = width
        self.height = height
        self.type = camera_type
        
        # Intrinsics: [fx, fy, cx, cy]
        if intrinsics:
            self.fx = intrinsics.get('fx', 1.0)
            self.fy = intrinsics.get('fy', 1.0)
            self.cx = intrinsics.get('cx', width / 2.0)
            self.cy = intrinsics.get('cy', height / 2.0)
        else:
            self._estimate_intrinsics()

        self.matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)

        # Distortions
        self.dist_coeffs = np.array(distortions, dtype=np.float64) if distortions else None

    def _estimate_intrinsics(self):
        """Estimates intrinsics based on camera type and resolution if not provided."""
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        
        # Defaults
        self.fx = self.width  # Approx FOV ~50-60 deg
        self.fy = self.width

        if self.type == 'lifecam':
            # Based on 320x240 scaling
            scale = self.width / 320.0
            self.fx = 342.3 * scale
            self.fy = 335.1 * scale
            
        elif self.type == 'arducam':
            if self.width == 1280: self.fx, self.fy = 905, 905
            elif self.width == 800: self.fx, self.fy = 691, 691
            elif self.width == 640: self.fx, self.fy = 590, 590
            
        elif self.type == 'c920':
            if self.width < 1000: # e.g. 640x360
                self.fx, self.fy = 478, 478
            else:
                self.fx, self.fy = 924.4, 924.4
        
        elif self.type == 'genius':
            # Wide FOV
            self.fx = 308.67 * (self.width / 640.0)
            self.fy = 309.11 * (self.height / 480.0)

    def get_fov(self):
        """Returns horizontal FOV in degrees."""
        return 2 * np.arctan(self.width / (2 * self.fx)) * (180 / np.pi)

    def undistort_points(self, points):
        if self.dist_coeffs is None:
            return points
        return cv2.undistortPoints(points, self.matrix, self.dist_coeffs, None, self.matrix)

    def project_3d_to_2d(self, rvec, tvec, object_points):
        if self.dist_coeffs is None:
            dist = np.zeros(5)
        else:
            dist = self.dist_coeffs
        
        imgpts, _ = cv2.projectPoints(object_points, rvec, tvec, self.matrix, dist)
        return imgpts