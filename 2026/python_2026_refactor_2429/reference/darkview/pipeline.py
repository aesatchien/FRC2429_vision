"""
threaded_pipeline.py

Defines the CameraPipeline class, which constructs and manages the set of
workers and queues for a single camera's processing chain.
"""

import queue
from workers import FrameGrabber, ContourProcessor

# A mapping from pipeline step names in the config to worker classes
WORKER_MAP = {
    'process_contours': ContourProcessor,
}

class CameraPipeline:
    """Manages the workers and queues for a single camera stream."""
    def __init__(self, camera_config):
        self.id = camera_config['id']
        self.config = camera_config
        self.workers = []
        self.queues = {}

        self._build_pipeline()

    def _build_pipeline(self):
        """Dynamically builds the worker pipeline based on the camera's config."""
        pipeline_steps = self.config.get('pipeline', [])
        if not pipeline_steps:
            return

        # The first worker is always a FrameGrabber
        # Its output queue is the first link in the chain.
        grabber_out_q_name = f"{self.id}_grabber_out"
        last_output_q = queue.Queue(maxsize=2)
        self.queues[grabber_out_q_name] = last_output_q
        
        frame_grabber = FrameGrabber(name=self.id, camera_config=self.config, output_queue=last_output_q)
        self.workers.append(frame_grabber)

        # Build the rest of the processing chain from the config
        for i, step_name in enumerate(pipeline_steps):
            worker_class = WORKER_MAP.get(step_name)
            if not worker_class:
                print(f"[Pipeline] Warning: Unknown pipeline step '{step_name}' for camera '{self.id}'")
                continue

            output_q_name = f"{self.id}_{step_name}_out"
            output_q = queue.Queue(maxsize=2)
            self.queues[output_q_name] = output_q

            worker = worker_class(
                name=f"{self.id}_{step_name}",
                camera_config=self.config,
                input_queue=last_output_q,
                output_queue=output_q
            )
            self.workers.append(worker)
            last_output_q = output_q

        print(f"[Pipeline] Built pipeline for '{self.id}' with {len(self.workers)} workers.")

    def start(self):
        """Starts all workers in the pipeline.""" 
        for worker in self.workers:
            worker.start()

    def stop(self):
        """Stops all workers in the pipeline."""
        for worker in self.workers:
            worker.stop()
        for worker in self.workers:
            worker.join()
