from dataclasses import dataclass, field
from typing import Any, Dict, Optional
import numpy as np

@dataclass
class CamCtx:
    name: str
    camera: Any                   # cscore.UsbCamera
    x_resolution: int = 0
    y_resolution: int = 0

    # stream objects
    image_source: Any = None      # cscore.CvSource
    cvstream: Any = None          # cscore.MjpegServer
    http_camera: Any = None       # cscore.HttpCamera
    sink: Any = None              # cscore.CvSink

    # vision pipeline + options
    pipeline: Any = None          # SpartanOverlay
    find_tags: bool = True
    find_colors: bool = False
    colors: list[str] = field(default_factory=lambda: ["orange"])
    greyscale: bool = False
    orientation: Dict[str, float] = field(default_factory=dict)
    intrinsics: Optional[Dict[str, float]] = None
    distortions: Optional[list[float]] = None
    use_distortions: bool = False
    max_tag_distance: float = 3.0

    # stream policy
    stream_fps: int = 16
    stream_max_width: int = 640
    processed_port: int = 1186
    reduce_bandwidth: bool = True

    # NT
    table_name: str = ""
    nt: Dict[str, Any] = field(default_factory=dict)

    # stats
    img_buf: Optional[np.ndarray] = None
    success_counter: int = 0
    failure_counter: int = 0
    previous_time: float = 0.0
    previous_counts: int = 0
    fps: float = 0.0
    last_image_send: float = 0.0
