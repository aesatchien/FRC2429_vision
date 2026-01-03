# Darkview - Multi-Camera Processing Framework

Darkview is a scalable, configuration-driven Python framework for capturing and processing multiple simultaneous camera streams. It provides a dynamic web interface for live viewing and recording of raw or processed video feeds.

---

## Features

- **Multi-Camera Support**: Easily configure and run multiple camera pipelines in parallel.
- **Dynamic Web Interface**: A web UI that automatically updates to display all active camera and processed streams.
- **Live Stream Fusion**: A dedicated worker that can synchronize and fuse the video from two source cameras into a single composite view.
- **Flexible Processing Pipelines**: Define a unique chain of processing steps for each camera through a simple configuration file.
- **Multi-Stream Recording**: Select any combination of raw or processed streams from the UI and record them simultaneously to MP4 files.
- **Test Mode**: Run the framework without physical cameras by using built-in synthetic image generators.

---

## Architecture Overview

The framework is designed with a modular, multi-threaded architecture that separates concerns into distinct components. This makes the system easy to extend and maintain.

- **`main.py`**: The main entry point. Contains the `ApplicationManager` class, which reads the configuration and orchestrates the entire application lifecycle (startup and shutdown).

- **`config.py`**: The central configuration file. This is where you define all cameras, their processing pipelines, and fusion settings. The entire application is driven by the data in this file.

- **`workers.py`**: Contains the individual, single-purpose processing units of the framework. Each worker is a thread that performs a specific task (e.g., grabbing frames, finding contours, applying filters).
    - `FrameGrabber`: Connects to a camera and captures raw frames.
    - `ContourProcessor`: Finds and draws saturation-based contours.
    - `FinalProcessor`: Applies post-processing (like CLAHE) and draws final overlays on the fused image.

- **`pipeline.py`**: Defines the `CameraPipeline` class. This class reads a camera's configuration and dynamically constructs a chain of workers and queues to form that camera's processing pipeline.

- **`fusion_worker.py`**: Contains the `FusionWorker`, which is responsible for synchronizing and compositing the grayscale images from two source pipelines.

- **`flask_server.py`**: Manages the web interface. It serves the main HTML page and provides dynamic routes for video streaming (`/stream/<stream_id>`) and recording control.

- **`recorder.py`**: A generic, stream-agnostic recording service. It can handle multiple simultaneous recording requests for any available stream and frame type.

- **`shared_state.py`**: Defines the global, thread-safe data structures (like the shutdown event and data dictionaries) that allow the different modules to communicate.

- **`utils.py`**: A collection of general-purpose helper functions, such as the FPS monitor and synthetic test image generators.

- **`templates/index.html`**: The Jinja2 template for the web interface. It dynamically generates controls based on the streams provided by the Flask server.

---

## Configuration

All application behavior is controlled by `config.py`.

### Adding a Camera

To add a new camera, simply add a new dictionary to the `CAMERAS` list:

```python
CAMERAS = [
    {
        'id': 'cam1',
        'enabled': True,
        'source': '/dev/video0',
        'resolution': (1280, 720),
        'pipeline': ['process_contours'],
        'overlay_color': (255, 0, 0),
    },
    # ... add more cameras here ...
]
```

### Enabling Test Mode

To run a camera in test mode without hardware, import a test generator function from `utils.py` and set it as the `source`:

```python
from utils import static_test_grid

CAMERAS = [
    {
        'id': 'cam1',
        'enabled': True,
        'source': static_test_grid, # Use function instead of path string
        # ...
    },
]
```

### Configuring Fusion

The `FUSION_CONFIG` dictionary controls the fusion process. You can specify which two cameras to use as sources and configure post-processing settings like CLAHE.

---

## How to Run

1.  Install the required Python packages (e.g., `Flask`, `opencv-python`, `numpy`).
2.  Configure your cameras and pipelines in `config.py`.
3.  Run the main application from your terminal:
    ```bash
    python main.py
    ```
4.  Open a web browser and navigate to `http://0.0.0.0:5000` to view the live interface.
