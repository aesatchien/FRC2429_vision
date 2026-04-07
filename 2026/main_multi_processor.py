1#!/usr/bin/env python3
import sys, os, time, signal, subprocess, argparse, logging, threading, re
from pathlib import Path
from ntcore import NetworkTableInstance
from vision.wpi_config import load_vision_cfg, select_profile
from vision import wpi_rio

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("launcher")
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent if HERE.name == "bin" else HERE
CAM_WORKER_MODULE = "vision.camera_node"

# Supervisor-side FPS watchdog: if a worker has produced zero FPS for this many
# seconds while still running, the supervisor force-kills it for a restart.
# This is the belt-and-suspenders backstop for when the worker's internal watchdog
# cannot fire (e.g. the process is alive but permanently stuck).
SUPERVISOR_FPS_TIMEOUT_S = 30

def cpu_map_for_profile(profile, reserve_cores):
    mapping = {}
    for cam in profile.get("cameras", []):
        name = cam["name"]
        cpu = cam.get("cpu")
        mapping[name] = cpu
    return mapping

def validate_cams_against_frc(frc_json):
    if not wpi_rio.readConfig(frc_json):
        raise RuntimeError(f"could not read {frc_json}")
    return [c.name for c in wpi_rio.cameraConfigs]

def spawn(proc_cmd, log_path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    f = open(log_path, "ab", buffering=0)
    p = subprocess.Popen(
        proc_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        cwd=HERE, text=True, bufsize=1
    )
    return {"p": p, "log": f, "cmd": proc_cmd, "log_path": log_path, "restart_count": 0}

def start_reader(name, child, stats):
    # parse: "<name>: 39.9fps  S:123  F:4"
    pat = re.compile(
        rf"{re.escape(name)}\s*:\s*(?P<fps>\d+(?:\.\d+)?)\s*fps.*?S:(?P<S>\d+)\s+F:(?P<F>\d+)",
        re.I,
    )
    def _run():
        for line in child["p"].stdout:
            try:
                child["log"].write(line.encode() if isinstance(line, str) else line)
            except Exception:
                pass
            m = pat.search(line)
            if m:
                d = stats.setdefault(name, {})
                d["fps_out"] = float(m.group("fps"))
                d["S"] = int(m.group("S"))
                d["F"] = int(m.group("F"))
    t = threading.Thread(target=_run, name=f"reader-{name}", daemon=True)
    t.start()
    child["reader"] = t


def terminate(child, grace=2.0):
    try:
        child["p"].terminate()
        t0 = time.time()
        while time.time() - t0 < grace:
            if child["p"].poll() is not None: break
            time.sleep(0.05)
        if child["p"].poll() is None: child["p"].kill()
    except Exception:
        pass
    try:
        child["log"].flush(); child["log"].close()
    except Exception:
        pass

def kill_existing_processes():
    # Kill any existing camera worker processes (zombies)
    # We do NOT stop the service here anymore to prevent self-termination loops.
    # If running manually, YOU must stop the service first: sudo systemctl stop runCamera
    for proc in ["vision.camera_node", "camera_node.py", "main_single_processor.py"]:
        try:
            subprocess.run(["pkill", "-f", proc], check=False)
        except Exception:
            pass
    time.sleep(1.0) # Allow time for cleanup

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vision", default="/boot/vision.json")
    ap.add_argument("--frc", default="/boot/frc.json")
    ap.add_argument("--logdir", default=str(ROOT / "logs"))
    ap.add_argument("--no-validate", action="store_true")
    ap.add_argument("--autorestart", action="store_true")
    ap.add_argument("--team", type=int, default=2429)
    ap.add_argument("--ip", type=str, default='10.24.29.2')
    args = ap.parse_args()

    # 1. Clean up environment
    # kill_other_launchers()
    kill_existing_processes()

    # host profile
    vcfg = load_vision_cfg(args.vision)
    prof = select_profile(vcfg)
    cam_names = [c["name"] for c in prof.get("cameras", [])]
    
    log.info(f"Selected profile: '{prof.get('role', 'unknown')}' with {len(cam_names)} cameras")
    if not cam_names:
        log.warning("No cameras found in this profile. Exiting.")
        return

    # optional frc.json validation
    if not args.no_validate:
        have = validate_cams_against_frc(args.frc)
        missing = [n for n in cam_names if n not in have]
        if missing: raise SystemExit(f"Profile cameras not in {args.frc}: {missing} (have {have})")

    # NT client for _fps subscriptions (works on-robot; harmless if no server)
    nt = NetworkTableInstance.getDefault()
    log.info("starting network client")
    nt.startClient4("launcher")
    nt.setServer(args.ip)
    # nt.setServerTeam(args.team)
    nt.startDSClient()
    
    # Pass the IP to the worker processes so they connect to the same server
    os.environ["NT_SERVER"] = args.ip

    fps_nt = {}
    for c in prof.get("cameras", []):
        tname = c.get("table_name", f"Cameras/{c['name']}")
        fps_nt[c["name"]] = nt.getTable(tname).getDoubleTopic("_fps").subscribe(0.0)

    # spawn workers; also parse stdout for FPS as fallback
    reserve = prof.get("reserve_cores", [0])
    cpu_map = cpu_map_for_profile(prof, reserve)
    children, stats = {}, {}
    for cam in prof.get("cameras", []):
        name = cam["name"]; cpu = cpu_map.get(name)
        cmd = [sys.executable, "-m", CAM_WORKER_MODULE, "--cam", name, "--vision", args.vision, "--frc", args.frc]
        if cpu is not None: cmd += ["--cpu", str(cpu)]
        log_path = Path(args.logdir) / f"camera-{name}.log"
        child = spawn(cmd, log_path)
        children[f"cam:{name}"] = child
        start_reader(name, child, stats)
        log.info(f"started {name} cpu={cpu} -> {log_path}")
        time.sleep(1.0) # Stagger startup to prevent USB bandwidth race conditions

    # Per-camera time of last nonzero FPS reading (initialized to now for startup grace).
    last_nonzero_fps = {name: time.time() for name in cam_names}

    # signals
    stopping = {"flag": False}
    def _stop(sig, frm):
        if stopping["flag"]: return
        stopping["flag"] = True
        log.info("stopping...")
        for k in list(children.keys()):
            terminate(children[k])
        sys.exit(0)
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    # supervise + one-line FPS
    last_print = 0.0
    while True:
        for key, ch in list(children.items()):
            rc = ch["p"].poll()
            if rc is None: continue
            print("") # Clear the status line so the log isn't overwritten
            log.warning(f"{key} exited rc={rc}")
            ch["log"].flush(); ch["log"].close()
            if args.autorestart and not stopping["flag"]:
                # rc==2: worker's own fatal signal; supervisor_killed: we force-killed
                # it because FPS was dead.  Both count as acquisition failures.
                if rc == 2 or ch.get("supervisor_killed"):
                    ch["restart_count"] += 1
                    if ch["restart_count"] >= 2:
                        log.error(f"{key} has fatal-restarted {ch['restart_count']} times in a row — rebooting")
                        os.system("sudo reboot")
                else:
                    ch["restart_count"] = 0  # clean exit or config error; reset streak
                time.sleep(0.5)
                new = spawn(ch["cmd"], ch["log_path"])
                new["restart_count"] = ch["restart_count"]  # carry the count forward
                children[key] = new
                cam_name = key.split("cam:", 1)[1]
                start_reader(cam_name, new, stats)
                last_nonzero_fps[cam_name] = time.time()  # reset watchdog timer for fresh start
                log.info(f"restarted {key} (restart_count={new['restart_count']})")
            else:
                del children[key]

        now = time.time()
        if now - last_print >= 2.0:
            parts = []
            for name in cam_names:
                # FPS: prefer NT if present, else stdout
                try:
                    v_nt = fps_nt[name].get()
                except Exception:
                    v_nt = 0.0
                fps_val = v_nt if (v_nt and v_nt > 0.0) else stats.get(name, {}).get("fps_out", 0.0)
                S = stats.get(name, {}).get("S", 0)
                F = stats.get(name, {}).get("F", 0)
                parts.append(f"{name}:{fps_val:0.1f}fps S:{S} F:{F}")

                # Supervisor FPS watchdog: force-kill a running worker that has
                # produced no output for too long (worker's internal watchdog may be
                # stuck if grabFrame blocks inside WPILib's reconnect loop).
                key = f"cam:{name}"
                if fps_val > 0.0:
                    last_nonzero_fps[name] = now
                elif args.autorestart and not stopping["flag"]:
                    ch = children.get(key)
                    if ch and ch["p"].poll() is None:  # still running but silent
                        dead_secs = now - last_nonzero_fps[name]
                        if dead_secs > SUPERVISOR_FPS_TIMEOUT_S:
                            log.warning(
                                f"{key} zero FPS for {dead_secs:.0f}s — supervisor forcing restart"
                            )
                            ch["supervisor_killed"] = True
                            terminate(ch)

            print("  ".join(parts), end="\r", flush=True)
            last_print = now

        time.sleep(0.1)

if __name__ == "__main__":
    main()
