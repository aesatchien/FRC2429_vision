"""
wpi_config.py

Stores all static configuration for the dual-camera fusion vision system.
The camera pipelines have been simplified to offload CLAHE processing until after fusion.
"""

# Import the test image generators from their new location in utils.py
from utils import static_test_grid, dynamic_test_image

# --- New, Scalable Configuration Structure ---
resolution = (640, 480)  #  (1280, 720)
CAMERAS = [
    {
        'id': 'cam1',
        'enabled': True,
        # usually source for cam1 is "/dev/video0"
        # 'source': static_test_grid,  # Use function static_test_grid instead of path string
        'source': "/dev/video0",
        'resolution': resolution,
        # The pipeline is now simpler: just grab and find contours.
        'pipeline': ['process_contours'],
        'overlay_color': (255, 0, 0),
    },
    {
        'id': 'cam2',
        'enabled': True,
        # usually this is "/dev/video2"
        # 'source': dynamic_test_image,  # Use function dynamic_test_image instead of path string
        'source': "/dev/video2",
        'resolution': resolution,
        'pipeline': ['process_contours'],
        'overlay_color': (0, 0, 255),
    },
]

# The Fusion worker is also defined in a structured way.
FUSION_CONFIG = {
    'enabled': True,
    'sources': ['cam1', 'cam2'],
    'overlap_trim_x': 5 ,  # 48 when objects are close (1m), 9 when far (4m)
    'overlap_trim_y': 51,
    # CLAHE settings are now part of the fusion process, applied by FinalProcessor.
    'clahe_clip_limit': 4.0,
    'clahe_tile_grid_size': (8, 8),
}

# Global processing settings
SATURATION_THRESHOLD = 240

# Web server settings
WEB_SERVER_CONFIG = {
    'port': 5000,
}
