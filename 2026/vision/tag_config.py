import logging
import robotpy_apriltag as ra

log = logging.getLogger("tag_cfg")

# Configuration Constants
FIELD_LAYOUT_PATH = './config/2026-rebuilt-welded_json'
TAG_FAMILY = 'tag36h11'
DEFAULT_TAG_SIZE = 0.1651

def load_field_layout(path=None):
    """
    Loads the AprilTagFieldLayout from the specified JSON file.
    """
    if path is None:
        path = FIELD_LAYOUT_PATH

    try:
        layout = ra.AprilTagFieldLayout(path)
        log.info(f"Loaded field layout from {path}")
        return layout
    except Exception as e:
        log.error(f"Failed to load field layout from {path}: {e}")
        return None