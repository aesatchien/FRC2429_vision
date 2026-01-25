import math
import time

def _wrap_pi(a: float) -> float:
    # wrap to [-pi, pi)
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a

class MultiTagResidualLogger:
    """
    Rate-limited logger for comparing single-tag robot poses vs the multi_tag robot pose.
    Keyed by the closest tag-pair (by single-tag 'dist') to avoid spam.
    """
    def __init__(self, min_period_s: float = 1.0, max_keys: int = 64):
        self.min_period_s = float(min_period_s)
        self.max_keys = int(max_keys)
        self._last_print = {}  # key -> monotonic time

    def dump(self, results: dict, label: str = "") -> None:
        """
        results: dict from TagDetector.detect(), containing tagXX entries and optionally 'multi_tag'
        """
        if not isinstance(results, dict):
            return
        if "multi_tag" not in results:
            return

        multi = results.get("multi_tag", {})
        if not multi or not multi.get("in_layout", False):
            return

        # collect single tags that have a robot pose (in_layout True)
        singles = []
        for k, v in results.items():
            if not isinstance(v, dict):
                continue
            if k == "multi_tag":
                continue
            if v.get("in_layout", False) and "tx" in v and "rz" in v:
                singles.append(v)

        if len(singles) < 2:
            return

        # pick the two closest tags (by single-tag distance-to-tag, not robot pose)
        singles_sorted = sorted(singles, key=lambda t: float(t.get("dist", 1e9)))
        pair_ids = tuple(sorted((int(singles_sorted[0]["id"]), int(singles_sorted[1]["id"]))))

        # Calculate delta between the two closest tags
        t1, t2 = singles_sorted[0], singles_sorted[1]
        t1x, t1y, t1z = float(t1.get("tx", 0)), float(t1.get("ty", 0)), float(t1.get("tz", 0))
        t2x, t2y, t2z = float(t2.get("tx", 0)), float(t2.get("ty", 0)), float(t2.get("tz", 0))
        dt_x, dt_y, dt_z = t1x - t2x, t1y - t2y, t1z - t2z
        dt_dist = math.sqrt(dt_x*dt_x + dt_y*dt_y + dt_z*dt_z)

        now = time.monotonic()
        last = self._last_print.get(pair_ids, -1e9)
        if (now - last) < self.min_period_s:
            return

        # simple LRU cap
        if len(self._last_print) >= self.max_keys:
            # drop oldest key
            oldest_k = min(self._last_print.items(), key=lambda kv: kv[1])[0]
            self._last_print.pop(oldest_k, None)

        self._last_print[pair_ids] = now

        mx, my, mz = float(multi.get("tx", 0.0)), float(multi.get("ty", 0.0)), float(multi.get("tz", 0.0))
        myaw = float(multi.get("rz", 0.0))  # radians in your pipeline
        myaw_deg = math.degrees(_wrap_pi(myaw))
        tvec = multi.get('tvec', [0.0, 0.0, 0.0])
        rvec = multi.get('rvec', [0.0, 0.0, 0.0])
        rms = multi.get('pnp_rms_px', float('nan'))

        # compute residuals vs each single-tag pose
        rows = []
        for s in singles:
            sx, sy, sz = float(s.get("tx", 0.0)), float(s.get("ty", 0.0)), float(s.get("tz", 0.0))
            syaw = float(s.get("rz", 0.0))
            dx, dy, dz = (sx - mx), (sy - my), (sz - mz)
            dpos = math.sqrt(dx*dx + dy*dy + dz*dz)
            dyaw = _wrap_pi(syaw - myaw)
            rows.append({
                "id": int(s["id"]),
                "dpos": dpos,
                "dx": dx, "dy": dy, "dz": dz,
                "dyaw_deg": math.degrees(dyaw),
                "single_dist": float(s.get("dist", float("nan"))),
                "tx": sx, "ty": sy,"tz": sz,
                'rms': s.get("pnp_rms_px", -1)
            })

        rows.sort(key=lambda r: r["dpos"])

        hdr = f"\n[multi_resid]{' ' + label if label else ''} pair={pair_ids} " \
              f"multi(xyz)=({mx:+.3f},{my:+.3f},{mz:+.3f}) yaw={myaw_deg:+.1f}deg \n" \
              f"   tvec: {tvec[0,0]:.3f},{tvec[1,0]:.3f},{tvec[2,0]:.3f}  rvec: {rvec[0,0]:.3f},{rvec[1,0]:.3f},{rvec[2,0]:.3f} RMS {rms:.1f}\n" \
              f"   Tag Delta (tag{t1['id']} - tag{t2['id']}): dist={dt_dist:.3f}m xyz=({dt_x:+.3f}, {dt_y:+.3f}, {dt_z:+.3f})"
        print(hdr)
        for r in rows[:8]:
            print(
                f"  tag{r['id']:02d}: dpos={r['dpos']:.3f}m"
                f"dxyz=({r['dx']:+.3f},{r['dy']:+.3f},{r['dz']:+.3f}) "
                f"dyaw={r['dyaw_deg']:+.1f}deg single_tag_dist={r['single_dist']:.2f}m "
                f"WPI XYZ=({r['tx']:+.3f},{r['ty']:+.3f},{r['tz']:+.3f})  RMS:{rms:.2f} "
            )
