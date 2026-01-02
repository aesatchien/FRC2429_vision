import threading
import time
import cv2
import numpy as np
import traceback
import logging

from visionlib.overlay_drawer import draw_overlays
log = logging.getLogger("pipeline")

class ThreadedVisionPipeline:
    def __init__(self, ctx, ntinst, nt_global, push_frame_fn):
        self.ctx = ctx
        self.ntinst = ntinst
        self.nt_global = nt_global
        self.push_frame_fn = push_frame_fn
        self.running = False

        # --- Shared Data & Synchronization ---
        
        # Frame Storage
        self.frame_lock = threading.Lock()
        self.frame_cv = threading.Condition(self.frame_lock)
        self.latest_frame = None     # The raw image (np.array)
        self.latest_ts = 0.0         # Timestamp of the frame
        self.frame_id = 0            # Counter to detect new frames

        # Results Storage
        self.result_lock = threading.Lock()
        self.latest_tag_results = {}
        self.latest_color_results = {}
        self.latest_result_ts = 0.0

        # Threads
        self.threads = [
            threading.Thread(target=self._thread_acquisition, name="Acq", daemon=True),
            threading.Thread(target=self._thread_detect_tags, name="Tags", daemon=True),
            threading.Thread(target=self._thread_detect_hsv, name="HSV", daemon=True),
            threading.Thread(target=self._thread_nt_update, name="NT", daemon=True),
            threading.Thread(target=self._thread_stream, name="Stream", daemon=True),
        ]

    def start(self):
        self.running = True
        for t in self.threads:
            t.start()
        log.info(f"Pipeline started with {len(self.threads)} threads")

    def join(self):
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)

    # ---------------------------------------------------------
    # 1. Image Acquisition Thread
    # ---------------------------------------------------------
    def _thread_acquisition(self):
        # Pre-allocate buffer
        h, w = self.ctx.y_resolution or 480, self.ctx.x_resolution or 640
        img_buf = np.zeros((h, w, 3), dtype=np.uint8)
        
        while self.running:
            try:
                ts, img = self.ctx.sink.grabFrame(img_buf)
                if ts > 0:
                    with self.frame_lock:
                        # We copy here so processing threads don't read while we write next frame
                        # On Pi 4, 640x480 copy is fast (~1ms)
                        self.latest_frame = img.copy()
                        self.latest_ts = ts
                        self.frame_id += 1
                        self.frame_cv.notify_all() # Wake up detectors
                else:
                    self.ctx.failure_counter += 1
                    time.sleep(0.005)
            except Exception as e:
                log.error(f"Acquisition error: {traceback.format_exc()}")
                time.sleep(0.1)

    # ---------------------------------------------------------
    # 2. Tag Detection Thread
    # ---------------------------------------------------------
    def _thread_detect_tags(self):
        last_processed_id = -1
        local_img = None
        
        while self.running:
            try:
                # Wait for new frame
                with self.frame_cv:
                    self.frame_cv.wait_for(lambda: self.frame_id > last_processed_id or not self.running)
                    if not self.running: break
                    last_processed_id = self.frame_id
                    if self.latest_frame is not None:
                        local_img = self.latest_frame # Read-only reference is fine if Acq copies
                        local_ts = self.latest_ts

                if local_img is None or not self.ctx.find_tags:
                    time.sleep(0.01)
                    continue

                # Run pipeline ONLY for tags, NO overlay drawing
                # We assume SpartanOverlay.process is thread-safe if we don't draw
                res, tags = self.ctx.pipeline.process(
                    local_img, method="size", 
                    training=False, debug=False, 
                    find_tags=True, find_colors=False, 
                    draw_overlay=False, # Crucial: Don't modify image
                    cam_orientation=self.ctx.orientation,
                    use_distortions=self.ctx.use_distortions
                )
                
                with self.result_lock:
                    self.latest_tag_results = tags
                    self.latest_result_ts = local_ts
            except Exception as e:
                log.error(f"Tag thread error: {traceback.format_exc()}")

    # ---------------------------------------------------------
    # 3. HSV Detection Thread
    # ---------------------------------------------------------
    def _thread_detect_hsv(self):
        last_processed_id = -1
        local_img = None

        while self.running:
            try:
                with self.frame_cv:
                    self.frame_cv.wait_for(lambda: self.frame_id > last_processed_id or not self.running)
                    if not self.running: break
                    last_processed_id = self.frame_id
                    if self.latest_frame is not None:
                        local_img = self.latest_frame

                if local_img is None or not self.ctx.find_colors:
                    time.sleep(0.01)
                    continue

                training = False if self.nt_global.get("training") is None else self.nt_global["training"].get()
                debug = False if self.nt_global.get("debug") is None else self.nt_global["debug"].get()

                res, _ = self.ctx.pipeline.process(
                    local_img, method="size",
                    training=training, debug=debug,
                    find_tags=False, find_colors=True,
                    draw_overlay=False,
                    cam_orientation=self.ctx.orientation,
                    use_distortions=self.ctx.use_distortions
                )

                with self.result_lock:
                    self.latest_color_results = res
            except Exception as e:
                log.error(f"HSV thread error: {traceback.format_exc()}")

    # ---------------------------------------------------------
    # 4. NetworkTables Updater Thread
    # ---------------------------------------------------------
    def _thread_nt_update(self):
        while self.running:
            time.sleep(0.02) # 50Hz update rate
            try:
                with self.result_lock:
                    tags = self.latest_tag_results
                    colors = self.latest_color_results
                    ts = self.latest_result_ts

                # Update Timestamp
                self.ctx.nt["timestamp"].set(ts)

                # Update Colors
                keys = list(self.ctx.colors)
                for key in keys:
                    tgt = colors.get(key, {})
                    self.ctx.nt["targets"][key]["targets"].set(tgt.get("targets", 0))
                    if tgt.get("targets", 0) > 0:
                        self.ctx.nt["targets"][key]["id"].set(tgt.get("ids", [0])[0])
                        self.ctx.nt["targets"][key]["distance"].set(tgt.get("distances", [0])[0])
                        self.ctx.nt["targets"][key]["rotation"].set(tgt.get("rotations", [0])[0])
                        self.ctx.nt["targets"][key]["strafe"].set(tgt.get("strafes", [0])[0])
                    else:
                        # Zero out if lost
                        self.ctx.nt["targets"][key]["distance"].set(0)
                        self.ctx.nt["targets"][key]["rotation"].set(0)

                # Update Tags
                if len(tags) > 0:
                    keys = list(tags.keys())
                    for i in range(2):
                        if i < len(keys):
                            k = keys[i]; d = tags[k]
                            self.ctx.nt["tag_poses"][i].set([ts, d["id"], d["tx"], d["ty"], d["tz"], d["rx"], d["ry"], d["rz"]])
                        else:
                            self.ctx.nt["tag_poses"][i].set([ts] + [0]*7)
                else:
                    for i in range(2): self.ctx.nt["tag_poses"][i].set([ts] + [0]*7)

                self.ntinst.flush()
            except Exception as e:
                log.error(f"NT update error: {traceback.format_exc()}")

    # ---------------------------------------------------------
    # 5. Stream & Overlay Thread
    # ---------------------------------------------------------
    def _thread_stream(self):
        last_processed_id = -1
        
        while self.running:
            draw_img = None
            try:
                with self.frame_cv:
                    self.frame_cv.wait_for(lambda: self.frame_id > last_processed_id or not self.running)
                    if not self.running: break
                    last_processed_id = self.frame_id
                    if self.latest_frame is not None:
                        # Copy for drawing so we don't block detectors reading the raw frame
                        draw_img = self.latest_frame.copy()
                
                if draw_img is None: continue

                # Get latest results to draw
                with self.result_lock:
                    tags = self.latest_tag_results
                    colors = self.latest_color_results

                # Get flags for drawing
                training = False if self.nt_global.get("training") is None else self.nt_global["training"].get()
                debug = False if self.nt_global.get("debug") is None else self.nt_global["debug"].get()

                # Draw Overlay using the dedicated drawer
                # This keeps the stream thread clean and the visualization logic centralized
                draw_overlays(draw_img, tags, colors, self.ctx, training=training, debug=debug)

                # Push to stream
                self.push_frame_fn(self.ctx, draw_img)
                
                # Update Stats
                self.ctx.success_counter += 1
                if self.ctx.success_counter % 30 == 0:
                    now = time.time()
                    dt = max(now - self.ctx.previous_time, 1e-3)
                    self.ctx.fps = (self.ctx.success_counter - self.ctx.previous_counts) / dt
                    self.ctx.previous_counts, self.ctx.previous_time = self.ctx.success_counter, now
                    self.ctx.nt["frames"].set(self.ctx.success_counter)
                    if "fps" in self.ctx.nt:
                        self.ctx.nt["fps"].set(self.ctx.fps)
            except Exception as e:
                log.error(f"Stream thread error: {traceback.format_exc()}")