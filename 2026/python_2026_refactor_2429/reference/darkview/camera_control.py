"""
camera_control.py  - 20250412 CJH

Provides utility functions for controlling camera hardware using v4l2-ctl
and for automatically tuning exposure based on image saturation levels.

Functions:
- set_camera_param(device, param, value):
    Sends a v4l2-ctl command to set a camera control parameter.

- auto_exposure_tune(cam_device, cam_queue, target_pct=1.5, exposure_list=None):
    Iteratively selects the lowest exposure from the list that keeps saturation
    below the given target percentage, using frames from the provided queue.

    The queue should provide dicts with a 'mask' array indicating saturated pixels.
    In a dual-queue architecture, this should be the *view queue* (not the data queue),
    to avoid interfering with downstream processing.

    hints on things to do to cam1 / cam2 -
    v4l2-ctl -d /dev/video0 --list-ctrls
    v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
    v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=50
    v4l2-ctl -d /dev/video0 --set-ctrl=gain=1

    Useful settings for arducams: - set the auto_exposure to 1, then set exposure time
    v4l2-ctl -d /dev/video0 --list-ctrls
    User Controls
                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
                       contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32
                          gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
                      sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3

    Camera Controls
                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=1 (Manual Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=50
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0


"""


import subprocess
import numpy as np
import time
import queue


def set_camera_param(device, param, value):
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", device, f"--set-ctrl={param}={value}"],
            check=True
        )
        print(f"[CameraControl] Set {param} to {value} on {device}")
    except subprocess.CalledProcessError as e:
        print(f"[CameraControl] Failed to set {param} on {device}: {e}")


def auto_exposure_tune(cam_device, cam_queue, target_pct=1.5, exposure_list=None):
    exposure_param = 'exposure_time_absolute'  # "exposure_absolute"
    if exposure_list is None:
        # seems on the dual arducam I can use 1 through 100, so about two orders of magnitude of dynamic range
        exposure_list = [16000, 8000, 4000, 2000, 1000, 500]
        exposure_list = [200, 100, 50, 30, 20, 10, 5, 2]  # logitechs

    print(f"\n[AutoExposure] Starting sweep on {cam_device}")
    best_exposure = None
    for exposure in exposure_list:

        set_camera_param(cam_device, exposure_param, exposure)
        time.sleep(0.1)  # let setting take effect

        try:
            frame_data = cam_queue.get(timeout=1.0)
        except queue.Empty:
            print(f"[AutoExposure] Timeout at exposure {exposure}")
            continue

        mask = frame_data['mask']
        saturation_pct = 100.0 * np.count_nonzero(mask) / mask.size
        print(f"Exposure {exposure:5d} µs → Saturation: {saturation_pct:.2f}%")

        if saturation_pct <= target_pct:
            best_exposure = exposure
            break

    if best_exposure:
        print(f"[AutoExposure] Selected exposure: {best_exposure} \n")
        set_camera_param(cam_device, exposure_param, best_exposure)
    else:
        print("[AutoExposure] No exposure found below target saturation threshold")
