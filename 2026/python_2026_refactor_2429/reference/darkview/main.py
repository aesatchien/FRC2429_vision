"""
main.py

Main entry point for the multi-camera processing framework.
This file now orchestrates the final, more efficient fusion pipeline.

Responsibilities:
- Instantiates and runs the ApplicationManager.
- Registers a SIGINT handler to trigger a graceful shutdown.
"""

import signal
import time
import sys
import threading
import queue

# Import application modules
import config
import shared_state
from pipeline import CameraPipeline
from fusion_worker import FusionWorker
from workers import FinalProcessor
from flask_server import app, start_stream_feeder
from utils import monitor_fps, shutdown_handler_factory

class ApplicationManager:
    """Orchestrates the creation and lifecycle of all processing pipelines."""
    def __init__(self, app_config):
        self.config = app_config
        self.pipelines = {}
        self.fusion_worker = None
        self.final_processor = None
        self.web_server_thread = None
        shared_state.pipelines = self.pipelines
        shared_state.fusion_worker = None
        shared_state.final_processor = None # For FPS monitor discovery

        print("[Manager] Building camera pipelines from configuration...")
        for cam_config in self.config.CAMERAS:
            if cam_config.get('enabled', False):
                cam_id = cam_config['id']
                self.pipelines[cam_id] = CameraPipeline(cam_config)

        # --- Fusion Pipeline Instantiation (FusionWorker -> FinalProcessor) ---
        if self.config.FUSION_CONFIG.get('enabled', False):
            print("[Manager] Building fusion pipeline...")
            source_ids = self.config.FUSION_CONFIG.get('sources', [])
            input_queues = {}
            for cam_id in source_ids:
                if cam_id in self.pipelines:
                    # Input for fusion is the output of the contour processor
                    queue_name = f"{cam_id}_process_contours_out"
                    if queue_name in self.pipelines[cam_id].queues:
                        input_queues[cam_id] = self.pipelines[cam_id].queues[queue_name]
                    else:
                        print(f"[Manager] Error: Could not find input queue '{queue_name}' for fusion.")
                        input_queues = {}
                        break
                else:
                    print(f"[Manager] Error: Source camera '{cam_id}' for fusion not found.")
                    input_queues = {}
                    break
            
            if len(input_queues) == len(source_ids):
                # Create the intermediate queue between FusionWorker and FinalProcessor
                fusion_to_final_q = queue.Queue(maxsize=2)

                self.fusion_worker = FusionWorker(
                    fusion_config=self.config.FUSION_CONFIG,
                    input_queues=input_queues,
                    output_queue=fusion_to_final_q
                )
                self.final_processor = FinalProcessor(
                    name='FinalProcessor',
                    fusion_config=self.config.FUSION_CONFIG,
                    input_queue=fusion_to_final_q,
                    output_queue=shared_state.fusion_view_queue
                )
                # Make workers globally accessible for the FPS monitor
                shared_state.fusion_worker = self.fusion_worker
                shared_state.final_processor = self.final_processor

    def start(self):
        """Starts all enabled pipelines and utility threads."""
        print("[Manager] Starting all pipelines...")
        for pipeline in self.pipelines.values():
            pipeline.start()

        if self.fusion_worker and self.final_processor:
            print("[Manager] Starting fusion pipeline...")
            self.fusion_worker.start()
            self.final_processor.start()

        print("[Manager] Starting web interface feeder...")
        start_stream_feeder()

        print("[Manager] Starting FPS monitor...")
        threading.Thread(target=monitor_fps, daemon=True).start()

        print("[Manager] Starting Flask web server...")
        self.web_server_thread = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=self.config.WEB_SERVER_CONFIG.get('port', 5000)),
            daemon=True
        )
        self.web_server_thread.start()

        print("[Manager] Application started. Running until SIGINT.")

    def stop(self):
        """Stops all running pipelines and threads gracefully."""
        print("\n[Manager] Shutdown requested. Stopping all pipelines...")
        shared_state.shutdown_requested.set()

        if self.fusion_worker:
            self.fusion_worker.stop()
        if self.final_processor:
            self.final_processor.stop()

        for pipeline in self.pipelines.values():
            pipeline.stop()

        if self.fusion_worker:
            self.fusion_worker.join()
        if self.final_processor:
            self.final_processor.join()

        print("[Manager] All pipelines stopped. Shutdown complete.")


# --- Main Execution ---
if __name__ == '__main__':
    app_manager = ApplicationManager(config)

    sigint_handler = shutdown_handler_factory(app_manager)
    signal.signal(signal.SIGINT, sigint_handler)

    app_manager.start()

    while not shared_state.shutdown_requested.is_set():
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            pass
