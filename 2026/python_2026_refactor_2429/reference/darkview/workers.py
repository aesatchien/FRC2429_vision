"""
workers.py

Defines the generic, single-purpose worker classes for the camera processing pipeline.
This file now includes a FinalProcessor for post-fusion rendering.

Classes:
- Worker: A base class for all threaded workers.
- FrameGrabber: Connects to a camera or test source and grabs raw frames.
- ContourProcessor: Performs saturation masking and contour detection on a frame.
- FinalProcessor: Applies CLAHE and draws contours on the final fused image.
"""

import cv2
import threading
import time
import queue
import numpy as np

from shared_state import shutdown_requested

class Worker(threading.Thread):
    """Base class for all pipeline workers."""
    def __init__(self, name, camera_config=None, fusion_config=None, input_queue=None, output_queue=None):
        super().__init__(name=name, daemon=True)
        self.name = name
        self.camera_config = camera_config if camera_config is not None else {}
        self.fusion_config = fusion_config if fusion_config is not None else {}
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.running = True
        self.pause = threading.Event()
        self.frame_counter = 0

    def stop(self):
        self.running = False

    def run(self):
        while self.running and not shutdown_requested.is_set():
            if self.pause.is_set():
                time.sleep(0.01)
                continue
            self.process_item()

    def process_item(self):
        raise NotImplementedError


class FrameGrabber(Worker):
    """Grabs frames from a camera or test source and puts them on a queue."""
    def __init__(self, name, camera_config, output_queue):
        super().__init__(name=name, camera_config=camera_config, output_queue=output_queue)
        self.source = self.camera_config['source']
        self.resolution = self.camera_config['resolution']
        self.is_test_source = callable(self.source)

        if not self.is_test_source:
            self.cap = cv2.VideoCapture(self.source)
            self.cap.set(cv2.CAP_PROP_FOURCC, 1196444237) # 'MJPG'
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FPS, 60)
            print(f"[{self.name}] Opened camera {self.source}")

    def stop(self):
        super().stop()
        if not self.is_test_source and hasattr(self, 'cap') and self.cap:
            self.cap.release()

    def process_item(self):
        if self.is_test_source:
            raw_frame = self.source()
        else:
            ret, raw_frame = self.cap.read()
            if not ret:
                print(f"[{self.name}] Frame grab failed")
                time.sleep(0.05)
                return

        frame_data = {
            'timestamp': time.time(),
            'source_id': self.name,
            'raw_frame': raw_frame,
            'overlay_color': self.camera_config.get('overlay_color', (0, 255, 0))
        }

        if self.output_queue.full():
            try: self.output_queue.get_nowait()
            except queue.Empty: pass
        self.output_queue.put(frame_data)
        self.frame_counter += 1
        time.sleep(0.001)


class ContourProcessor(Worker):
    """Performs saturation masking and contour detection on a raw frame."""
    def __init__(self, name, camera_config, input_queue, output_queue):
        super().__init__(name=name, camera_config=camera_config, input_queue=input_queue, output_queue=output_queue)
        self.saturation_threshold = self.camera_config.get('saturation_threshold', 240)

    def process_item(self):
        try:
            data = self.input_queue.get(timeout=0.1)
        except queue.Empty:
            return

        raw_frame = data['raw_frame']
        gray_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY) if len(raw_frame.shape) > 2 else raw_frame

        mask = cv2.inRange(gray_frame, self.saturation_threshold, 255)
        outlined_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(outlined_frame, contours, -1, data['overlay_color'], 2)

        # Explicitly carry forward all original data (like raw_frame, timestamp)
        # and add the new processed data to the packet.
        output_data = {
            **data,
            'gray_frame': gray_frame,
            'mask': mask,
            'outlined': outlined_frame,
            'contours': contours
        }

        if self.output_queue.full():
            try: self.output_queue.get_nowait()
            except queue.Empty: pass
        self.output_queue.put(output_data)
        self.frame_counter += 1


class FinalProcessor(Worker):
    """Applies CLAHE and draws contours on the final fused image."""
    def __init__(self, name, fusion_config, input_queue, output_queue):
        super().__init__(name=name, fusion_config=fusion_config, input_queue=input_queue, output_queue=output_queue)
        self.clahe = cv2.createCLAHE(
            clipLimit=self.fusion_config.get('clahe_clip_limit', 2.0),
            tileGridSize=self.fusion_config.get('clahe_tile_grid_size', (8, 8))
        )

    def process_item(self):
        try:
            data = self.input_queue.get(timeout=0.1)
        except queue.Empty:
            return

        fused_gray = data['fused_gray']

        # 1. Apply CLAHE to the entire fused image
        enhanced_gray = self.clahe.apply(fused_gray)

        # 2. Convert to color for drawing
        final_image = cv2.cvtColor(enhanced_gray, cv2.COLOR_GRAY2BGR)

        # 3. Draw contours from both sources
        cv2.drawContours(final_image, data['contours1'], -1, data['color1'], 1)
        cv2.drawContours(final_image, data['contours2'], -1, data['color2'], 1)

        # 4. Create the final output packet for the web view
        output_data = {
            'timestamp': time.time(),
            'source_id': 'fusion',
            'outlined': final_image # Use 'outlined' key for web compatibility
        }

        if self.output_queue.full():
            try: self.output_queue.get_nowait()
            except queue.Empty: pass
        self.output_queue.put(output_data)
        self.frame_counter += 1
