"""
fusion_worker.py

Defines the FusionWorker thread. Its role has been simplified to only
synchronize and fuse the grayscale images from two sources.

Responsibilities:
- Continuously polls its source queues for the latest available frames.
- Fuses the two grayscale images.
- Packages the fused image, contours, and colors into an intermediate packet
  for the FinalProcessor.
"""

import threading
import time
import numpy as np
import queue
import cv2

from shared_state import shutdown_requested

class FusionWorker(threading.Thread):
    def __init__(self, fusion_config, input_queues, output_queue):
        super().__init__(name="FusionWorker")
        self.config = fusion_config
        self.input_queues = input_queues
        self.output_queue = output_queue

        self.source_ids = self.config.get('sources', [])
        self.overlap_trim_x = self.config.get('overlap_trim_x', 0)
        self.overlap_trim_y = self.config.get('overlap_trim_y', 0)
        
        self.running = True
        self.frame_counter = 0

    def stop(self):
        self.running = False

    def run(self):
        if len(self.source_ids) != 2:
            print("[FusionWorker] Error: Fusion requires exactly two sources.")
            return

        cam1_id, cam2_id = self.source_ids
        cam1_q = self.input_queues[cam1_id]
        cam2_q = self.input_queues[cam2_id]

        latest_data1 = None
        latest_data2 = None

        while self.running and not shutdown_requested.is_set():
            try:
                latest_data1 = cam1_q.get_nowait()
            except queue.Empty:
                pass

            try:
                latest_data2 = cam2_q.get_nowait()
            except queue.Empty:
                pass

            if latest_data1 and latest_data2:
                data1 = latest_data1
                data2 = latest_data2

                # Fuse the raw grayscale images
                img1, img2 = self.crop_and_shift(data1['gray_frame'], data2['gray_frame'])
                mask1, _ = self.crop_and_shift(data1['mask'], data2['mask'])
                fused_gray = self.fuse_images(img1, img2, mask1)
                padded_fused_gray = self.pad_to_full_width(fused_gray)

                # Prepare the intermediate data packet for the FinalProcessor
                output_data = {
                    'timestamp': time.time(),
                    'fused_gray': padded_fused_gray,
                    'contours1': self.shift_contours(data1['contours'], dx=0, dy=-self.overlap_trim_y if self.overlap_trim_y > 0 else 0),
                    'contours2': self.shift_contours(data2['contours'], dx=self.overlap_trim_x, dy=self.overlap_trim_y if self.overlap_trim_y < 0 else 0),
                    'color1': data1.get('overlay_color', (255, 0, 0)),
                    'color2': data2.get('overlay_color', (0, 0, 255)),
                }

                if self.output_queue.full():
                    try: self.output_queue.get_nowait()
                    except queue.Empty: pass
                self.output_queue.put(output_data)
                self.frame_counter += 1

                latest_data1 = None
                latest_data2 = None
            else:
                time.sleep(0.005)

    def crop_and_shift(self, img1, img2):
        x = self.overlap_trim_x
        y = self.overlap_trim_y

        # Crop horizontally based on the x overlap
        img1_x = img1[:, x:]
        img2_x = img2[:, :-x if x > 0 else None]

        # Crop vertically based on the y offset
        y_top_crop = max(0, y)    # Crop from top of img1 if y is positive
        y_bottom_crop = max(0, -y) # Crop from bottom of img1 if y is negative

        img1_cropped = img1_x[y_top_crop : img1_x.shape[0] - y_bottom_crop, :]
        img2_cropped = img2_x[y_bottom_crop : img2_x.shape[0] - y_top_crop, :]

        return img1_cropped, img2_cropped

    def fuse_images(self, img1, img2, mask1):
        fused = img1.copy()
        fused[mask1 > 0] = img2[mask1 > 0]
        return fused

    def pad_to_full_width(self, cropped_img):
        h, w = cropped_img.shape[:2]
        x = self.overlap_trim_x
        full_w = w + 2 * x
        padded = np.full((h, full_w), 128, dtype=np.uint8)
        padded[:, x:x + w] = cropped_img
        return padded

    def shift_contours(self, contours, dx=0, dy=0):
        shifted = []
        for cnt in contours:
            cnt_shifted = cnt + np.array([[[dx, dy]]], dtype=cnt.dtype)
            shifted.append(cnt_shifted)
        return shifted
