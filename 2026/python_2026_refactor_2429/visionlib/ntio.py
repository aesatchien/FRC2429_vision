from ntcore import PubSubOptions

def init_global_flags(ntinst):
    cams = ntinst.getTable("Cameras")
    training_sub = cams.getBooleanTopic("_training").subscribe(False)
    debug_sub    = cams.getBooleanTopic("_debug").subscribe(False)
    ts_sub       = ntinst.getDoubleTopic("/SmartDashboard/_timestamp").subscribe(0)
    return {"training": training_sub, "debug": debug_sub, "timestamp": ts_sub}

def init_cam_entries(ntinst, ctx):
    t = ntinst.getTable(ctx.table_name)
    ctx.nt["timestamp"]   = t.getDoubleTopic("_timestamp").publish()
    ctx.nt["frames"]      = t.getDoubleTopic("_frames").publish()
    ctx.nt["fps"] = t.getDoubleTopic("_fps").publish()
    ctx.nt["connections_pub"] = t.getDoubleTopic("_connections").publish(PubSubOptions(keepDuplicates=True))
    ctx.nt["connections_sub"] = t.getDoubleTopic("_connections").subscribe(0)
    ctx.nt["colors"]      = t.getStringArrayTopic("colors").publish()
    ctx.nt["colors"].set(ctx.colors)

    # per target color + tags
    ctx.nt["targets"] = {}
    keys = list(ctx.colors)
    if "tags" not in keys:
        keys.append("tags")
    for key in keys:
        ctx.nt["targets"][key] = {
            "id":       t.getDoubleTopic(f"{key}/id").publish(),
            "targets":  t.getDoubleTopic(f"{key}/targets").publish(),
            "distance": t.getDoubleTopic(f"{key}/distance").publish(),
            "rotation": t.getDoubleTopic(f"{key}/rotation").publish(),
            "strafe":   t.getDoubleTopic(f"{key}/strafe").publish(),
        }
    # tag pose slots
    ctx.nt["tag_poses"] = [
        t.getDoubleArrayTopic("poses/tag1").publish(),
        t.getDoubleArrayTopic("poses/tag2").publish()
    ]
