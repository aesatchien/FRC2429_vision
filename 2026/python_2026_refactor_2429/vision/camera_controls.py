import time
import sys
import subprocess
import logging

log = logging.getLogger("cam_ctrl")

def set_camera_robust_defaults(camera, frc_config, camera_type="c920"):
    """
    Applies brightness and exposure settings robustly, mimicking the 2025/old behavior.
    1. Nudges brightness (set +1, wait, set back) to wake up the camera.
    2. Sets exposure manually via cscore and v4l2-ctl (Linux only) to ensure it sticks.
    """
    if not frc_config:
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
        val = exp_prop["value"]
        # The magic multiplier from 2025 code (likely for C920 specific behavior)
        exp_val_v4l2 = int(val * 20)
        
        log.info(f"Configuring exposure for {camera.getName()}: base={val}, v4l2={exp_val_v4l2}")

        # A. Set cscore state (so it doesn't fight back)
        try:
            camera.setExposureManual(int(val))
        except Exception as e:
            log.warning(f"cscore setExposureManual failed: {e}")

        # B. Force v4l2-ctl on Linux for C920s
        if sys.platform.startswith("linux") and camera_type == "c920":
            path = camera.getPath()
            if path.startswith("/dev/video"):
                try:
                    # Force Manual Mode (1)
                    subprocess.run(
                        ["v4l2-ctl", "-d", path, "--set-ctrl=exposure_auto=1"],
                        check=False, capture_output=True
                    )
                    
                    # Set Absolute Exposure
                    subprocess.run(
                        ["v4l2-ctl", "-d", path, f"--set-ctrl=exposure_time_absolute={exp_val_v4l2}"],
                        check=False, capture_output=True
                    )
                    log.info(f"v4l2-ctl applied to {path}")
                except Exception as e:
                    log.warning(f"v4l2-ctl failed: {e}")