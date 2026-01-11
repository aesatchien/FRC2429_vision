# AprilTag Detection Workflow

This document explains the logic inside `vision/detectors.py`, specifically how we derive the robot's position on the field from raw camera images.

## 1. Single Tag Processing

When only one tag is visible (or `allow_multi_tag` is false), we calculate the robot's pose relative to that specific tag.

### A. Detection
1.  **Grayscale Conversion:** The image is converted to grayscale (`cv2.cvtColor`).
2.  **Undistortion (Optional):** If `use_distortions` is true, `cv2.remap` is applied using pre-computed maps to remove lens distortion.
3.  **AprilTag Library:** The standard `robotpy_apriltag` detector runs on the image to find quad boundaries and decode IDs.
4.  **Filtering:** Detections are discarded if:
    *   `decision_margin` < Configured Threshold (default 35).
    *   `hamming` error > Configured Max (default 1).

### B. Pose Estimation (Tag-Relative)
We use the **Orthogonal Iteration** algorithm provided by `AprilTagPoseEstimator`.
*   **Input:** Tag corners (2D pixels), Tag Size (0.1651m), Camera Intrinsics (fx, fy, cx, cy).
*   **Output:** A `Transform3d` representing the tag's position relative to the camera in the **EDN** frame (East-Down-North / OpenCV standard: Z forward, X right, Y down).

### C. Coordinate Transformation (Robot-Relative)
To get the robot's field pose, we perform a chain of coordinate transformations.

1.  **Camera Frame Conversion (EDN -> NWU):**
    The raw estimator output is in OpenCV coordinates. We convert this to WPILib's standard **NWU** (North-West-Up: X forward, Y left, Z up) frame.
    ```python
    pose_nwu = geo.CoordinateSystem.convert(
        pose_camera, 
        geo.CoordinateSystem.EDN(), 
        geo.CoordinateSystem.NWU()
    )
    ```

2.  **Robot Pose Calculation:**
    We use the WPILib helper `objectToRobotPose`.
    $$ T_{field}^{robot} = T_{field}^{tag} \times (T_{camera}^{tag})^{-1} \times (T_{robot}^{camera})^{-1} $$
    
    *   `tag_field_pose`: Known from `.json` layout (Field -> Tag).
    *   `pose_nwu`: Measured by camera (Camera -> Tag).
    *   `robot_to_cam`: Static config from `vision.json` (Robot -> Camera).

    The result is the **Robot's Pose in the Field**.

---

## 2. Multi-Tag Processing (Global PnP)

When 2 or more tags are visible, we use a global **Perspective-n-Point (PnP)** solver. This is significantly more accurate than averaging single-tag results because it minimizes error across all visible corners simultaneously and handles camera distortion natively.

### A. Collecting Correspondences
We build two arrays of matching points:

1.  **Object Points (3D Field Coordinates):**
    For every visible tag, we calculate the 3D field coordinates of its 4 corners.
    *   We define the 4 corners in the **Tag's Local Frame**.
    *   We transform these local corners by the tag's known **Field Pose** (from layout).
    *   Result: A list of $(x, y, z)_{field}$ points.

2.  **Image Points (2D Pixel Coordinates):**
    The corresponding $(u, v)$ pixel coordinates for those corners, provided directly by the detector.

### B. Solving PnP
We use OpenCV's `cv2.solvePnP` with the **SQPNP** (Square Planar PnP) flag.
*   **Why SQPNP?** It is specifically optimized for planar markers (like AprilTags) and guarantees finding the global optimum, avoiding the ambiguity/flipping issues common with iterative solvers.
*   **Inputs:** Object Points (Field), Image Points (Pixels), Camera Matrix ($K$), Distortion Coefficients ($D$).
*   **Outputs:** Rotation Vector ($rvec$) and Translation Vector ($tvec$).

These vectors represent the transform from **Field Space** to **Camera Space** (OpenCV axes).
$$ P_{camera} = T_{field \to camera} \times P_{field} $$

### C. Coordinate Transformation
We must convert the OpenCV transform into a WPILib Field Pose.

1.  **Construct Matrix:** Convert $rvec/tvec$ to a $4 \times 4$ transformation matrix $T_{f \to c}^{CV}$.
2.  **Swap Axes:** Convert the camera frame from OpenCV (Z-fwd) to NWU (X-fwd).
    $$ T_{f \to c}^{NWU} = M_{swap} \times T_{f \to c}^{CV} $$
    Where $M_{swap}$ maps:
    *   $X_{nwu} = Z_{cv}$
    *   $Y_{nwu} = -X_{cv}$
    *   $Z_{nwu} = -Y_{cv}$
3.  **Invert:** We now have Field $\to$ Camera. We invert this matrix to get Camera $\to$ Field.
    $$ T_{c \to f} = (T_{f \to c}^{NWU})^{-1} $$
    The translation component of $T_{c \to f}$ is the **Camera's Position on the Field**.

*(Note: Currently, the code returns this high-accuracy translation combined with the rotation from the closest single tag to avoid Euler angle conversion complexity).*

---

## 3. Coordinate Systems Reference

Understanding the axes is critical for debugging.

### Field Frame (WPILib Standard)
*   **Origin:** Right side of the driver station wall (Blue Alliance origin).
*   **X:** Forward (towards opposing alliance).
*   **Y:** Left.
*   **Z:** Up.

### Camera Frame (OpenCV Standard)
*   **Origin:** Optical center of the camera.
*   **X:** Right (along sensor width).
*   **Y:** Down (along sensor height).
*   **Z:** Forward (optical axis).

### Camera Frame (WPILib/NWU)
*   **Origin:** Optical center of the camera.
*   **X:** Forward (optical axis).
*   **Y:** Left.
*   **Z:** Up.

### Tag Frame (Layout)
*   **Origin:** Center of the tag.
*   **X:** Normal to the tag surface (pointing out).
*   **Y:** Left (looking at the tag).
*   **Z:** Up.