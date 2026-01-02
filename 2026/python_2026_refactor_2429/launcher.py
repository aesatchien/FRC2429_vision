#!/usr/bin/env python3
import sys, os, time, signal, subprocess, argparse, logging, threading, re
from pathlib import Path
from ntcore import NetworkTableInstance
from visionlib.config_io import load_vision_cfg, select_profile
from visionlib import frc_io

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("launcher")
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent if HERE.name == "bin" else HERE
CAM_WORKER = str(HERE / "camera_worker.py")

def cpu_map_for_profile(profile, reserve_cores):
    n = os.cpu_count() or 4
    avail = [c for c in range(n) if c not in set(reserve_cores)]
    mapping = {}
    i = 0
    for cam in profile.get("cameras", []):
        name = cam["name"]
        cpu = cam.get("cpu")
        if cpu is None:
            cpu = avail[i % len(avail)] if avail else None
            i += 1
        mapping[name] = cpu
    return mapping

def validate_cams_against_frc(frc_json):
    frc_io.configFile = frc_json
    if not frc_io.readConfig():
        raise RuntimeError(f"could not read {frc_io.configFile}")
    return [c.name for c in frc_io.cameraConfigs]

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
        rf"{re.escape(name)}\s*:\s*(?P<fps>\d+(?:\.\d+)?)\s*fps\s+S:(?P<S>\d+)\s+F:(?P<F>\d+)",
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

    # host profile
    vcfg = load_vision_cfg(args.vision)
    prof = select_profile(vcfg)
    cam_names = [c["name"] for c in prof.get("cameras", [])]

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
        cmd = [sys.executable, CAM_WORKER, "--cam", name, "--vision", args.vision, "--frc", args.frc]
        if cpu is not None: cmd += ["--cpu", str(cpu)]
        log_path = Path(args.logdir) / f"camera-{name}.log"
        child = spawn(cmd, log_path)
        children[f"cam:{name}"] = child
        start_reader(name, child, stats)
        log.info(f"started {name} cpu={cpu} -> {log_path}")

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
                time.sleep(0.5)
                new = spawn(ch["cmd"], ch["log_path"])
                children[key] = new
                # restart reader
                name = key.split("cam:",1)[1]
                start_reader(name, new, stats)
                log.info(f"restarted {key}")
            else:
                del children[key]

        now = time.time()
        if now - last_print >= 1.0:
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
            print("  ".join(parts), end="\r", flush=True)
            last_print = now

        time.sleep(0.1)

if __name__ == "__main__":
    main()
