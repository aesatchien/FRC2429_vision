"""
recorder.py

Provides a generic, stream-agnostic recording service.
This module can now record different frame types (e.g., raw, outlined)
and gracefully handles requests for unavailable frame types.

Responsibilities:
- Manages the state of multiple, simultaneous recordings.
- Provides a public API to start recordings for any given stream ID and frame type.
- Provides a public API to get the status of all active recordings.
"""

import os
import time
from datetime import datetime
import cv2
import threading

import shared_state

# --- Module State ---
active_recordings = {}
active_recordings_lock = threading.Lock()


# --- Private Recording Worker ---

def _record_stream(stream_id, frame_type='outlined', duration_s=10.0, target_fps=30):
    """The actual recording logic that runs in a background thread."""
    os.makedirs("clips", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Make the filename more descriptive
    out_path = os.path.join("clips", f"{stream_id}_{frame_type}_{ts}.mp4")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = None
    frames_written = 0
    dt = 1.0 / float(target_fps)
    t0 = time.time()

    with active_recordings_lock:
        active_recordings[stream_id] = {
            'active': True,
            't_start': t0,
            'duration': float(duration_s),
            'filename': out_path
        }

    try:
        while time.time() - t0 < duration_s:
            if shared_state.shutdown_requested.is_set():
                break

            frame_to_write = None
            with shared_state.stream_data_lock:
                packet = shared_state.stream_data.get(stream_id)
                if packet:
                    # Dynamically get the requested frame type
                    frame_to_write = packet.get(frame_type)
                    # Failsafe: if requested type doesn't exist, fall back to 'outlined'
                    if frame_to_write is None:
                        frame_to_write = packet.get('outlined')

            if frame_to_write is not None:
                if writer is None:
                    h, w = frame_to_write.shape[:2]
                    writer = cv2.VideoWriter(out_path, fourcc, target_fps, (w, h))
                    if not writer.isOpened():
                        print(f"[Recorder] Failed to open writer for {out_path}")
                        break
                
                if len(frame_to_write.shape) == 2:
                    frame_bgr = cv2.cvtColor(frame_to_write, cv2.COLOR_GRAY2BGR)
                else:
                    frame_bgr = frame_to_write
                writer.write(frame_bgr)
                frames_written += 1

            time.sleep(dt)

        print(f"[Recorder] Saved {frames_written} frames to {out_path}")

    except Exception as e:
        print(f"[Recorder] Error during recording for stream '{stream_id}': {e}")
    finally:
        if writer is not None:
            writer.release()
        with active_recordings_lock:
            if stream_id in active_recordings:
                del active_recordings[stream_id]


# --- Public API for the Flask Server ---

def start_recording(stream_id, frame_type='outlined', duration_s=10.0, target_fps=30):
    """Starts a new recording for a given stream_id in a background thread."""
    with active_recordings_lock:
        if stream_id in active_recordings:
            print(f"[Recorder] A recording for '{stream_id}' is already in progress.")
            return False
        
        print(f"[Recorder] Starting {duration_s}s recording of '{frame_type}' for stream '{stream_id}'...")
        thread = threading.Thread(
            target=_record_stream,
            kwargs={
                'stream_id': stream_id,
                'frame_type': frame_type,
                'duration_s': duration_s,
                'target_fps': target_fps
            },
            daemon=True
        )
        thread.start()
        return True

def get_status():
    """Returns the status of all active recordings."""
    status_report = {}
    with active_recordings_lock:
        for stream_id, state in active_recordings.items():
            status_report[stream_id] = {
                'active': state['active'],
                'duration': state['duration'],
                'remaining': max(0.0, state['duration'] - (time.time() - state['t_start'])),
                'filename': state['filename']
            }
    return status_report


__all__ = ['start_recording', 'get_status']
