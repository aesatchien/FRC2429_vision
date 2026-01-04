#!/usr/bin/env python3
import cv2
import time
import sys

def main():
    print("--- Minimal OpenCV Streamer Test ---")
    print("Testing if pure OpenCV triggers the C920 'ladder bug' (exposure quantization).")
    print("Opening /dev/video2...")

    # Force V4L2 backend
    # Note: index 2 corresponds to /dev/video2
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("Error: Could not open /dev/video2")
        sys.exit(1)

    # Configure
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    print("Stream started. Go check 'v4l2-ctl -d /dev/video2 -C exposure_time_absolute'")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Frame read failed.")
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    cap.release()
    print("\nCamera released.")

if __name__ == "__main__":
    main()