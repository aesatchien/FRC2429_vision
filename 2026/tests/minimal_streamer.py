#!/usr/bin/env python3
import time
import json
import numpy as np
import subprocess
from cscore import CameraServer, VideoSource, VideoMode

def main():
    default_delay = 2
    t = 0

    print("--- Minimal C920 Streamer Test (Step-by-Step) ---")
    print("Watch your v4l2-ctl monitor to see when exposure behavior changes.")
    
    # 1. Basic Open
    print(f"\n[T={t}s] Opening /dev/video2 via startAutomaticCapture...")
    cam2 = CameraServer.startAutomaticCapture(name="Cam2", path="/dev/video2")
    time.sleep(default_delay)
    t += default_delay

    # 2. Set Resolution/FPS
    print(f"[T={t}s] Setting Resolution 640x480 @ 30 FPS...")
    cam2.setResolution(640, 480)
    cam2.setFPS(30)
    time.sleep(default_delay)
    t += default_delay

    # 3. Connection Strategy (wpi_rio.py does this)
    # print(f"[T={t}s] Setting ConnectionStrategy.kConnectionKeepOpen...")
    # cam2.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    # time.sleep(default_delay)
    # t += default_delay

    # 4. Pixel Format (MJPEG)
    print(f"[T={t}s] Setting PixelFormat to MJPEG...")
    cam2.setPixelFormat(VideoMode.PixelFormat.kMJPEG)
    time.sleep(default_delay)
    t += default_delay

    # 5. Brightness (cscore API)
    print(f"[T={t}s] Setting Brightness to 50 (cscore)...")
    cam2.setBrightness(50)
    time.sleep(default_delay)
    t += default_delay

    # 6. White Balance (cscore API)
    print(f"[T={t}s] Skipping White Balance (prone to crash)...")
    # cam2.setWhiteBalanceManual(4000)
    # time.sleep(5)

    # 7. Exposure (cscore API)
    # NOTE: cscore value 10 * ~20 multiplier = ~200 absolute exposure
    print(f"[T={t}s] Setting Exposure Manual to 10 (result ~200) (cscore)...")
    cam2.setExposureManual(10)
    time.sleep(default_delay)
    t += default_delay

    # 8. JSON Config (The big hammer from wpi_rio.py)
    print(f"[T={t}s] Applying full JSON config (simulating frc.json)...")
    config_json = {
        "fps": 30, "height": 480, "width": 640, "pixel format": "mjpeg",
        "properties": [
            {"name": "exposure_auto", "value": 1}, # Manual
            {"name": "exposure_time_absolute", "value": 20}
        ]
    }
    cam2.setConfigJson(json.dumps(config_json))
    print("JSON Config applied.")
    
    time.sleep(default_delay)
    t += default_delay

    # 9. Attach CvSink (Simulates vision processing connection)
    print(f"[T={t}s] Attaching CvSink (SMOKING GUN: This starts the stream and triggers the bug)...")
    sink = CameraServer.getVideo(camera=cam2)
    time.sleep(default_delay)
    t += default_delay

    # 10. Grab a Frame (Forces stream start)
    print(f"[T={t}s] Grabbing a frame (starts actual streaming)...")
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    ts, img = sink.grabFrame(img)
    print(f"  -> Frame grabbed! ts={ts}")
    time.sleep(default_delay)
    t += default_delay

    # 11. Brightness Nudge (Simulating camera_controls.py)
    print(f"[T={t}s] Performing Brightness Nudge (50 -> 51 -> 50)...")
    cam2.setBrightness(51)
    time.sleep(0.5)
    cam2.setBrightness(50)
    time.sleep(default_delay)
    t += default_delay

    # 12. Verify Workaround (v4l2-ctl)
    # The sink (Step 9) broke the exposure. Now we prove we can fix it externally.
    print(f"[T={t}s] Applying v4l2-ctl workaround (Force Manual -> 150)...")
    subprocess.run(["v4l2-ctl", "-d", "/dev/video2", "--set-ctrl=exposure_auto=1"], check=False)
    subprocess.run(["v4l2-ctl", "-d", "/dev/video2", "--set-ctrl=exposure_time_absolute=150"], check=False)
    print("  -> Workaround applied. Check monitor for '150' (not 127/255).")
    time.sleep(default_delay)
    t += default_delay

    print("\nTest sequence complete. Streaming continues...")
    while True:
        time.sleep(1.0)

if __name__ == "__main__":
    main()