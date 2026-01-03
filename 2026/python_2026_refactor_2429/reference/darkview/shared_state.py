"""
shared_state.py

Defines all global shared state for the multi-camera framework.
This has been refactored to support a generic, scalable pipeline architecture.

Responsibilities:
- Defines the global shutdown event.
- Defines an event to control the web stream feeder.
- Defines the global data structures for pipeline management and web streaming.
"""

import threading
import queue

# --- Global Shutdown Event ---
shutdown_requested = threading.Event()

# --- Feeder Control Event ---
# An event to signal the stream_feeder thread to pause its operation.
# This allows other parts of the application to safely access frame queues
# without contention from the web view feeder.
feeder_paused = threading.Event()


# --- Global Data Structures ---

# A dictionary to hold all camera pipeline objects, keyed by camera ID.
# This is populated by the ApplicationManager at startup.
pipelines = {}

# A dedicated queue for the FusionWorker's output, to be consumed by the web feeder.
fusion_view_queue = queue.Queue(maxsize=2)

# A dictionary to hold the latest processed frame data for web streaming,
# keyed by stream ID (e.g., 'cam1', 'cam2', 'fusion').
stream_data = {}
stream_data_lock = threading.Lock()
