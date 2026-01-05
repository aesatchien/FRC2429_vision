from ntcore import PubSubOptions

def init_global_flags(ntinst):
    cams = ntinst.getTable("Cameras")
    
    # Publish global flags so they appear on the dashboard
    # We keep the publishers in the return dict to prevent garbage collection
    train_pub = cams.getBooleanTopic("_training").publish()
    train_pub.set(False)
    
    debug_pub = cams.getBooleanTopic("_debug").publish()
    debug_pub.set(False)

    # Use an Entry for _training_box so it is writable from Glass/Dashboard
    box_entry = cams.getEntry("_training_box")
    box_entry.setDefaultDoubleArray([0.5, 0.5])

    training_sub = cams.getBooleanTopic("_training").subscribe(False)
    debug_sub    = cams.getBooleanTopic("_debug").subscribe(False)
    # ts_sub       = ntinst.getDoubleTopic("/SmartDashboard/_timestamp").subscribe(0)
    return {
        "training": training_sub, 
        "debug": debug_sub, 
        "training_box": box_entry,
        # "timestamp": ts_sub,
        "_pubs": [train_pub, debug_pub]
    }

def init_cam_entries(ntinst, ctx):
    t = ntinst.getTable(ctx.table_name)
    ctx.nt["frames"]      = t.getIntegerTopic("_frames").publish()
    # ctx.nt["timestamp"]   = t.getIntegerTopic("_timestamp").publish()
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
    # Only force updates on the 'targets' count for latency checks to save bandwidth
    heartbeat_options = PubSubOptions(keepDuplicates=True)
    for key in keys:
        ctx.nt["targets"][key] = {
            "id":       t.getDoubleTopic(f"{key}/id").publish(),
            "targets":  t.getDoubleTopic(f"{key}/targets").publish(heartbeat_options),
            "distance": t.getDoubleTopic(f"{key}/distance").publish(),
            "rotation": t.getDoubleTopic(f"{key}/rotation").publish(),
            "strafe":   t.getDoubleTopic(f"{key}/strafe").publish(),
        }
    # tag pose slots
    ctx.nt["tag_poses"] = [
        t.getDoubleArrayTopic("poses/tag1").publish(),
        t.getDoubleArrayTopic("poses/tag2").publish()
    ]

def update_cam_entries(ctx, tags, colors, ntinst):
    """
    Shared logic to update NetworkTables from detection results.
    Used by both threaded_pipeline.py (production) and local_tester.py (dev).
    """
    # Update Colors
    for key in ctx.colors:
        tgt = colors.get(key, {})
        ctx.nt["targets"][key]["targets"].set(tgt.get("targets", 0))
        if tgt.get("targets", 0) > 0:
            ids = tgt.get("ids", [])
            dists = tgt.get("distances", [])
            rots = tgt.get("rotations", [])
            strafes = tgt.get("strafes", [])
            ctx.nt["targets"][key]["id"].set(ids[0] if ids else 0)
            ctx.nt["targets"][key]["distance"].set(dists[0] if dists else 0)
            ctx.nt["targets"][key]["rotation"].set(rots[0] if rots else 0)
            ctx.nt["targets"][key]["strafe"].set(strafes[0] if strafes else 0)
        else:
            ctx.nt["targets"][key]["id"].set(0)
            ctx.nt["targets"][key]["distance"].set(0)
            ctx.nt["targets"][key]["rotation"].set(0)
            ctx.nt["targets"][key]["strafe"].set(0)

    # Update Tags Summary
    tag_count = len(tags)
    ctx.nt["targets"]["tags"]["targets"].set(tag_count)
    
    if tag_count > 0:
        best_tag = min(tags.values(), key=lambda x: x['dist'])
        ctx.nt["targets"]["tags"]["id"].set(best_tag['id'])
        ctx.nt["targets"]["tags"]["distance"].set(best_tag['dist'])
        ctx.nt["targets"]["tags"]["rotation"].set(best_tag.get('rotation', 0))
        ctx.nt["targets"]["tags"]["strafe"].set(best_tag.get('strafe', 0))
    else:
        ctx.nt["targets"]["tags"]["id"].set(0)
        ctx.nt["targets"]["tags"]["distance"].set(0)
        ctx.nt["targets"]["tags"]["rotation"].set(0)
        ctx.nt["targets"]["tags"]["strafe"].set(0)

    # Update Tag Poses (Odometry)
    odo_tags = [t for t in tags.values() if t.get("in_layout", False)]
    if len(odo_tags) > 0:
        for i in range(2):
            if i < len(odo_tags):
                d = odo_tags[i]
                ctx.nt["tag_poses"][i].set([d["id"], d["tx"], d["ty"], d["tz"], d["rx"], d["ry"], d["rz"]])
            else:
                ctx.nt["tag_poses"][i].set([0]*7)
    else:
        for i in range(2): ctx.nt["tag_poses"][i].set([0]*7)

    ntinst.flush()
