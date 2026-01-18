import time, cv2
import json
from cscore import CvSource, MjpegServer, VideoMode, HttpCamera, CameraServer

def build_stream(ctx, compression_if_wide: int = 30):
    vm = ctx.camera.getVideoMode()
    ctx.x_resolution, ctx.y_resolution = vm.width, vm.height

    # Determine actual output resolution to match push_frame logic
    w, h = ctx.x_resolution, ctx.y_resolution
    if getattr(ctx, "reduce_bandwidth", False) and w > ctx.stream_max_width:
        w = int(ctx.stream_max_width)
        h = int(h * ctx.stream_max_width / ctx.x_resolution)

    ctx.image_source = CvSource(
        f"{ctx.name} CV Image Source",
        VideoMode.PixelFormat.kBGR,  # was kMJPEG
        w, h, ctx.stream_fps
    )
    ctx.cvstream = MjpegServer(f"{ctx.name} CV Image Stream", ctx.processed_port)
    if ctx.x_resolution > 300:
        ctx.cvstream.getProperty("compression").set(compression_if_wide)
    ctx.cvstream.setSource(ctx.image_source)
    CameraServer.addCamera(ctx.image_source)

    ctx.http_camera = HttpCamera(
        f"{ctx.name} Processed",
        f"http://127.0.0.1:{ctx.processed_port}/?action=stream",
        HttpCamera.HttpCameraKind.kMJPGStreamer
    )
    CameraServer.addCamera(ctx.http_camera)

def push_frame(ctx, frame):
    now = time.time()
    if now - ctx.last_image_send < 1.0 / ctx.stream_fps:
        return
    if ctx.reduce_bandwidth and frame.shape[1] > ctx.stream_max_width:
        h, w = frame.shape[:2]
        w2 = int(ctx.stream_max_width); h2 = int(h * w2 / w)
        frame = cv2.resize(frame, (w2, h2))
    ctx.image_source.putFrame(frame)
    ctx.last_image_send = now

def build_raw_stream(name: str, camera, port: int, stream_cfg: dict | None = None):
    srv = CameraServer.addServer(f"{name}_raw", int(port))
    srv.setSource(camera)
    if stream_cfg:
        srv.setConfigJson(json.dumps(stream_cfg))
    return srv