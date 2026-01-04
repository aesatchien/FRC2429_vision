import time
import sys
import subprocess
import logging
import threading

log = logging.getLogger("cam_ctrl")

def set_camera_robust_defaults(camera, frc_config, camera_type="c920", delay=0):
    """
    Applies brightness and exposure settings robustly, mimicking the 2025/old behavior.
    1. Nudges brightness (set +1, wait, set back) to wake up the camera.
    2. Sets exposure manually via cscore and v4l2-ctl (Linux only) to ensure it sticks.

    CRITICAL HARDWARE NOTE:
    Newer Logitech C920 revisions (0x21) exhibit a firmware bug where exposure
    collapses to a coarse ladder (31, 63, 127...) when the camera is open for streaming.
    Verified: Both cscore and OpenCV trigger this quantization upon VIDIOC_STREAMON.
    There is NO workaround. We are stuck with coarse discrete steps on these units.
    """

    if not frc_config:
        return

    if delay > 0:
        def _run_delayed():
            time.sleep(delay)
            set_camera_robust_defaults(camera, frc_config, camera_type, delay=0)
        threading.Thread(target=_run_delayed, daemon=True).start()
        return

    # frc_config is the CameraConfig object from rio
    # It has a .config dictionary attribute containing the JSON data
    config_dict = getattr(frc_config, "config", {})
    
    # 1. Brightness Nudge
    # This helps "wake up" the camera driver or apply pending settings
    brightness = config_dict.get("brightness")
    if brightness is not None:
        try:
            b_val = int(brightness)
            # log.info(f"Nudging brightness for {camera.getName()} around {b_val}")
            camera.setBrightness(b_val + 1)
            time.sleep(0.25)
            camera.setBrightness(b_val)
        except Exception as e:
            log.warning(f"Brightness nudge failed: {e}")

    # 2. Exposure
    # Look for 'exposure_time_absolute' in properties
    props = config_dict.get("properties", [])
    exp_prop = next((p for p in props if p["name"] == "exposure_time_absolute"), None)

    if exp_prop:
        val = exp_prop["value"] + 1
        # The magic multiplier from 2025 code (likely for C920 specific behavior)
        exp_val_v4l2 = int(val * 20)  # want to get the crappy c920 to be close to what we wanted
        
        log.info(f"Configuring exposure for {camera.getName()}: base={val}, v4l2={exp_val_v4l2}")

        # A. Set cscore state (so it doesn't fight back)
        try:
            camera.setExposureManual(int(val))
        except Exception as e:
            log.warning(f"cscore setExposureManual failed: {e}")

        # B. Force v4l2-ctl on Linux for C920s
        if sys.platform.startswith("linux") and camera_type == "c920":
            path = camera.getPath()
            # Allow /dev/videoX AND /dev/v4l/by-id/... paths
            if path.startswith("/dev/"):
                try:
                    # Force Manual Mode (1)
                    # subprocess.run(["v4l2-ctl", "-d", path, "--set-ctrl=exposure_auto=1"], check=False, capture_output=True)
                    
                    time.sleep(2.5)

                    # Set Absolute Exposure
                    log.info(f"v4l2-ctl setting exposure to {exp_val_v4l2} on {path}")
                    subprocess.run(["v4l2-ctl", "-d", path, f"--set-ctrl=exposure_time_absolute={exp_val_v4l2}"], check=False, capture_output=True)
                except Exception as e:
                    log.warning(f"v4l2-ctl failed: {e}")