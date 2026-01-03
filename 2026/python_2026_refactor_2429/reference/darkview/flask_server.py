"""
flask_server.py

Provides a generic web interface for viewing multiple camera streams.
This has been refactored to be data-driven, based on the global state,
and now includes a pausing mechanism for the stream feeder.

Responsibilities:
- Defines Flask routes for the web UI (/), streaming, and controls.
- Defines a feeder thread to populate the web view data from the pipelines.
- Provides controls to pause and resume the feeder for special tasks.
"""

from flask import Flask, Response, render_template, jsonify, request
import threading
import time
import queue
import cv2
import contextlib

import shared_state
from recorder import start_recording, get_status
from utils import estimate_shift

app = Flask(__name__)


# --- Stream Feeder Control ---

def pause_feeder():
    """Signals the stream_feeder to pause."""
    print("[Feeder] Pausing...")
    shared_state.feeder_paused.set()

def resume_feeder():
    """Signals the stream_feeder to resume."""
    print("[Feeder] Resuming...")
    shared_state.feeder_paused.clear()

@contextlib.contextmanager
def feeder_paused_context():
    """A context manager to safely pause and resume the stream feeder."""
    pause_feeder()
    try:
        yield
    finally:
        resume_feeder()


# --- Generic Stream Feeder ---

def stream_feeder():
    """Pulls the latest processed frame from each pipeline for web display."""
    while not shared_state.shutdown_requested.is_set():
        # If the feeder is paused, wait until it's resumed.
        if shared_state.feeder_paused.is_set():
            time.sleep(0.1)
            continue

        # 1. Pull from the final output queue of each camera pipeline
        for cam_id, pipeline in shared_state.pipelines.items():
            # The final output queue contains the fully processed packet
            final_queue_name = f"{cam_id}_{pipeline.config['pipeline'][-1]}_out"
            if final_queue_name in pipeline.queues:
                view_queue = pipeline.queues[final_queue_name]
                if not view_queue.empty():
                    try:
                        data = view_queue.get_nowait()
                        with shared_state.stream_data_lock:
                            shared_state.stream_data[cam_id] = data
                    except queue.Empty:
                        pass

        # 2. Pull from the fusion worker's final output queue
        if not shared_state.fusion_view_queue.empty():
            try:
                data = shared_state.fusion_view_queue.get_nowait()
                with shared_state.stream_data_lock:
                    shared_state.stream_data['fusion'] = data
            except queue.Empty:
                pass

        time.sleep(0.01)

def start_stream_feeder():
    """Starts the stream_feeder in a background thread."""
    threading.Thread(target=stream_feeder, daemon=True).start()


# --- Web Routes ---

@app.route('/')
def index():
    """Renders the main page, passing the list of available stream IDs."""
    stream_ids = sorted(list(shared_state.pipelines.keys()))
    if hasattr(shared_state, 'fusion_worker') and shared_state.fusion_worker:
        stream_ids.append('fusion')
    
    return render_template('index.html', stream_ids=stream_ids)


@app.route('/stream/<stream_id>')
def stream(stream_id):
    """A dynamic route to serve the video stream for any given camera ID."""
    def gen():
        while not shared_state.shutdown_requested.is_set():
            frame_to_stream = None
            with shared_state.stream_data_lock:
                stream_packet = shared_state.stream_data.get(stream_id)
                if stream_packet:
                    frame_to_stream = stream_packet.get('outlined')
            
            if frame_to_stream is not None:
                ret, jpeg = cv2.imencode('.jpg', frame_to_stream)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.01)

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


# --- Control Routes ---

@app.route('/align', methods=['POST'])
def align_cameras():
    """Pauses the feeder and gets fresh frames to estimate camera alignment."""
    with feeder_paused_context():
        try:
            # Get the final output queues for the source cameras
            cam1_q_name = f"cam1_{shared_state.pipelines['cam1'].config['pipeline'][-1]}_out"
            cam2_q_name = f"cam2_{shared_state.pipelines['cam2'].config['pipeline'][-1]}_out"
            cam1_q = shared_state.pipelines['cam1'].queues[cam1_q_name]
            cam2_q = shared_state.pipelines['cam2'].queues[cam2_q_name]

            # Clear any stale frames
            while not cam1_q.empty(): cam1_q.get_nowait()
            while not cam2_q.empty(): cam2_q.get_nowait()

            # Get the latest raw frames directly from the queues
            mov_img = cam1_q.get(timeout=1.0)['raw_frame']
            ref_img = cam2_q.get(timeout=1.0)['raw_frame']

        except (KeyError, queue.Empty) as e:
            return jsonify({'status': 'error', 'message': f'Could not get fresh frames: {e}'}), 503

    dx, dy = estimate_shift(ref_img, mov_img)
    print(f'[Align] Estimated offsets: dx={dx}, dy={dy}')
    
    return jsonify({'status': 'success', 'dx': dx, 'dy': dy})


# --- Recording Routes ---

@app.route('/record/start', methods=['POST'])
def start_multiple_recordings():
    """
    Starts recordings for a list of stream IDs provided in a JSON payload.
    """
    data = request.get_json()
    if not data or 'streams' not in data:
        return jsonify({'status': 'error', 'message': 'Invalid payload'}), 400

    stream_ids = data.get('streams', [])
    frame_type = data.get('frame_type', 'outlined')
    
    started_count = 0
    for stream_id in stream_ids:
        success = start_recording(stream_id, frame_type=frame_type, duration_s=10.0)
        if success:
            started_count += 1
            
    return jsonify({'status': 'success', 'message': f'Started {started_count} new recordings.'})


@app.route('/record_status')
def record_status():
    """Returns the status of all active recordings."""
    return jsonify(get_status())


__all__ = ['app', 'start_stream_feeder']
