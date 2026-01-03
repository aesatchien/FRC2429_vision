import threading
import time
import cv2
import numpy as np
import traceback
import logging
import copy
import ntcore

from vision.visual_overlays import draw_overlays
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
                    # If we fail too many times, maybe we should try to restart the camera?
                    # For now, just log occasionally or update status
                    if self.ctx.failure_counter % 100 == 0:
                         log.warning(f"Camera acquisition failing... count={self.ctx.failure_counter}")
                         # Optionally update NT status here if desired
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
                    with self.result_lock:
                        self.latest_tag_results = {}
                    time.sleep(0.01)
                    continue

                # Run Tag Detector
                tags = self.ctx.tag_detector.detect(
                    local_img, 
                    cam_orientation=self.ctx.orientation,
                    max_distance=self.ctx.max_tag_distance
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
                    with self.result_lock:
                        self.latest_color_results = {}
                    time.sleep(0.01)
                    continue

                # Check training/debug flags from NT global
                training = False if self.nt_global.get("training") is None else self.nt_global["training"].get()
                debug = False if self.nt_global.get("debug") is None else self.nt_global["debug"].get()

                # Run HSV Detector for each color
                res = {}
                for color in self.ctx.colors:
                    if color == 'tags': continue
                    color_res = self.ctx.hsv_detector.process(local_img, color, training=training)
                    res[color] = color_res
                    
                    # Hoist training stats to top level so overlay_drawer can find it
                    if training and 'training_stats' in color_res:
                        res['training_stats'] = color_res['training_stats']

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
                # Heartbeat
                # self.ctx.nt["timestamp"].set(ntcore._now())

                with self.result_lock:
                    tags = self.latest_tag_results
                    colors = self.latest_color_results

                # Update Colors
                keys = list(self.ctx.colors)
                for key in keys:
                    tgt = colors.get(key, {})
                    self.ctx.nt["targets"][key]["targets"].set(tgt.get("targets", 0))
                    if tgt.get("targets", 0) > 0:
                        # Safely get first element of lists, defaulting to 0 if list is empty
                        ids = tgt.get("ids", [])
                        dists = tgt.get("distances", [])
                        rots = tgt.get("rotations", [])
                        strafes = tgt.get("strafes", [])
                        self.ctx.nt["targets"][key]["id"].set(ids[0] if ids else 0)
                        self.ctx.nt["targets"][key]["distance"].set(dists[0] if dists else 0)
                        self.ctx.nt["targets"][key]["rotation"].set(rots[0] if rots else 0)
                        self.ctx.nt["targets"][key]["strafe"].set(strafes[0] if strafes else 0)
                    else:
                        # Zero out if lost
                        self.ctx.nt["targets"][key]["id"].set(0)
                        self.ctx.nt["targets"][key]["distance"].set(0)
                        self.ctx.nt["targets"][key]["rotation"].set(0)
                        self.ctx.nt["targets"][key]["strafe"].set(0)

                # Update Tags Summary (targets/tags)
                # We pick the closest tag to report as the primary target
                tag_count = len(tags)
                self.ctx.nt["targets"]["tags"]["targets"].set(tag_count)
                
                if tag_count > 0:
                    # Sort tags by distance to find the "best" one
                    best_tag = min(tags.values(), key=lambda x: x['dist'])
                    
                    self.ctx.nt["targets"]["tags"]["id"].set(best_tag['id'])
                    self.ctx.nt["targets"]["tags"]["distance"].set(best_tag['dist'])
                    self.ctx.nt["targets"]["tags"]["rotation"].set(best_tag.get('rotation', 0))
                    self.ctx.nt["targets"]["tags"]["strafe"].set(best_tag.get('strafe', 0))
                else:
                    self.ctx.nt["targets"]["tags"]["id"].set(0)
                    self.ctx.nt["targets"]["tags"]["distance"].set(0)
                    self.ctx.nt["targets"]["tags"]["rotation"].set(0)
                    self.ctx.nt["targets"]["tags"]["strafe"].set(0)

                # Update Tags
                if len(tags) > 0:
                    keys = list(tags.keys())
                    for i in range(2):
                        if i < len(keys):
                            k = keys[i]; d = tags[k]
                            self.ctx.nt["tag_poses"][i].set([d["id"], d["tx"], d["ty"], d["tz"], d["rx"], d["ry"], d["rz"]])
                        else:
                            self.ctx.nt["tag_poses"][i].set([0]*7)
                else:
                    for i in range(2): self.ctx.nt["tag_poses"][i].set([0]*7)

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
                # We need to ensure these are updated regularly from NT
                training = False if self.nt_global.get("training") is None else self.nt_global["training"].get()
                debug = False if self.nt_global.get("debug") is None else self.nt_global["debug"].get()

                # Draw Overlay using the dedicated drawer
                # This keeps the stream thread clean and the visualization logic centralized
                try:
                    draw_overlays(draw_img, tags, colors, self.ctx, training=training, debug=debug)
                except Exception as e:
                    log.error(f"Overlay drawing error: {traceback.format_exc()}")
                    # Draw a red X or error message on the image if overlay fails
                    cv2.putText(draw_img, "OVERLAY ERROR", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

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