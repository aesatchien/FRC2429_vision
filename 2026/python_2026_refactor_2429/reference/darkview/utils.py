import time
import sys
import numpy as np
import cv2
import shared_state

# --- FPS Monitor ---
def monitor_fps():
    """Dynamically monitors and prints the FPS of all active workers."""
    last_counts = {}
    prev_time = time.time()

    while not shared_state.shutdown_requested.is_set():
        now = time.time()
        if now - prev_time < 1.0:
            time.sleep(0.1)
            continue

        dt = now - prev_time
        prev_time = now
        
        fps_strings = []
        all_workers = []

        # 1. Gather workers from all camera pipelines
        for pipeline in shared_state.pipelines.values():
            all_workers.extend(pipeline.workers)

        # 2. Add the fusion worker if it exists
        if hasattr(shared_state, 'fusion_worker') and shared_state.fusion_worker:
            all_workers.append(shared_state.fusion_worker)

        # 3. Calculate and format FPS for each worker
        for worker in all_workers:
            current_count = worker.frame_counter
            last_count = last_counts.get(worker.name, 0)
            fps = (current_count - last_count) / dt
            fps_strings.append(f"{worker.name}: {fps:02.1f} FPS")
            last_counts[worker.name] = current_count

        # Print the combined FPS string, clearing the line
        print(" | ".join(fps_strings) + "        ", end="\r")
        
        time.sleep(0.1)


# --- Graceful Shutdown ---
def shutdown_handler_factory(app_manager):
    """Factory that creates a SIGINT handler with access to the app_manager."""
    def sigint_handler(signum, frame):
        print("\n[Main] Caught SIGINT, shutting down application...")
        app_manager.stop()
        sys.exit(0)
    return sigint_handler


# --- Test Image Generators ---

def static_test_image():
    img = np.zeros((720, 1280), dtype=np.uint8)
    cv2.rectangle(img, (300, 300), (1000, 500), 255, -1)
    return img

def static_test_grid():
    height, width = 720, 1280
    tile_size = 64
    gap = 10
    white_size = tile_size - gap

    rows = height // tile_size
    cols = width // tile_size

    img = np.full((height, width), 64, dtype=np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                y0 = r * tile_size + gap // 2
                x0 = c * tile_size + gap // 2
                y1 = y0 + white_size
                x1 = x0 + white_size
                y1 = min(y1, height)
                x1 = min(x1, width)
                img[y0:y1, x0:x1] = 255
    return img

def dynamic_test_image():
    t = int(time.time() * 90) % 1280
    img = np.zeros((720, 1280), dtype=np.uint8)
    cv2.rectangle(img, (t, 300), (t + 100, 500), 128, -1)
    cv2.rectangle(img, (t+100, 300), (t + 200, 500), 255, -1)
    return img

# --- Alignment ---
def estimate_shift(ref, mov):
    # returns (dx, dy) so that translating mov by (-dx,-dy) aligns to ref
    A = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY) if ref.ndim==3 else ref
    B = cv2.cvtColor(mov, cv2.COLOR_BGR2GRAY) if mov.ndim==3 else mov
    A, B = A.astype(np.float32), B.astype(np.float32)
    win = cv2.createHanningWindow((A.shape[1], A.shape[0]), cv2.CV_32F)
    (dx, dy), _ = cv2.phaseCorrelate(A*win, B*win)
    return dx, dy
