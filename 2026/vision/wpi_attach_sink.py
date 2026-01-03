import time, numpy as np
from cscore import CameraServer

def attach_sink(ctx):
    cs = CameraServer
    ctx.sink = cs.getVideo(camera=ctx.camera)
    ctx.img_buf = np.zeros((ctx.y_resolution or 480, ctx.x_resolution or 640, 3), dtype=np.uint8)