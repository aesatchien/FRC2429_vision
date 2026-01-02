import time, numpy as np
from cscore import CameraServer

def attach_sink(ctx):
    cs = CameraServer
    ctx.sink = cs.getVideo(camera=ctx.camera)
    ctx.img_buf = np.zeros((ctx.y_resolution or 480, ctx.x_resolution or 640, 3), dtype=np.uint8)

def tick(nt_global, ntinst, ctx, training: bool, debug: bool, push_frame_fn):
    # 1) capture
    ts_img, img = ctx.sink.grabFrame(ctx.img_buf)
    if ts_img <= 0:
        ctx.failure_counter += 1
        return

    # 2) process
    res, tags = ctx.pipeline.process(
        img.copy(),
        method="size",
        training=training,
        debug=debug,
        find_tags=ctx.find_tags,
        draw_overlay=True,
        find_colors=ctx.find_colors,
        cam_orientation=ctx.orientation,
        use_distortions=ctx.use_distortions
    )

    # 3) NT update
    ts = nt_global["timestamp"].get()
    ctx.nt["timestamp"].set(ts)

    # targets (colors + tags)
    keys = list(ctx.colors)
    if "tags" not in keys:
        keys.append("tags")
    for key in keys:
        tgt = res.get(key, {})
        ctx.nt["targets"][key]["targets"].set(tgt.get("targets", 0))
        if tgt.get("targets", 0) > 0:
            ctx.nt["targets"][key]["id"].set(tgt.get("ids", [0])[0])
            ctx.nt["targets"][key]["distance"].set(tgt.get("distances", [0])[0])
            ctx.nt["targets"][key]["rotation"].set(tgt.get("rotations", [0])[0])
            ctx.nt["targets"][key]["strafe"].set(tgt.get("strafes", [0])[0])
        else:
            for f in ["id","distance","rotation","strafe"]:
                ctx.nt["targets"][key][f].set(0)

    # tag poses (2 slots)
    if len(tags) > 0:
        keys = list(tags.keys())
        for i in range(2):
            if i < len(keys):
                k = keys[i]; d = tags[k]
                ctx.nt["tag_poses"][i].set([ts, d["id"], d["tx"], d["ty"], d["tz"], d["rx"], d["ry"], d["rz"]])
            else:
                ctx.nt["tag_poses"][i].set([ts] + [0]*7)
    else:
        for i in range(2): ctx.nt["tag_poses"][i].set([ts] + [0]*7)

    ntinst.flush()

    # 4) stream
    push_frame_fn(ctx, ctx.pipeline.image)

    # 5) stats
    ctx.success_counter += 1
    if ctx.success_counter % 30 == 0:
        now = time.time()
        dt = max(now - ctx.previous_time, 1e-3)
        ctx.fps = (ctx.success_counter - ctx.previous_counts) / dt
        ctx.previous_counts, ctx.previous_time = ctx.success_counter, now
        ctx.nt["frames"].set(ctx.success_counter)
        if "fps" in ctx.nt:
            ctx.nt["fps"].set(ctx.fps)