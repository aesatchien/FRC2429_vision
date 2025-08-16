#!/usr/bin/env python3
# launcher.py  (no fusion)
import sys, os, time, json, signal, subprocess, argparse, logging
from pathlib import Path

from config_io import load_vision_cfg, select_profile
from frc_io import readConfig as frc_read, cameraConfigs

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("launcher")

HERE = Path(__file__).resolve().parent
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
    if not frc_read(frc_json):
        raise RuntimeError(f"could not read {frc_json}")
    return [c.name for c in cameraConfigs]

def spawn(proc_cmd, log_path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    f = open(log_path, "ab", buffering=0)
    p = subprocess.Popen(proc_cmd, stdout=f, stderr=subprocess.STDOUT, cwd=HERE)
    return {"p": p, "log": f, "cmd": proc_cmd, "log_path": log_path, "restart_count": 0}

def terminate(child, grace=2.0):
    try:
        child["p"].terminate()
        t0 = time.time()
        while time.time() - t0 < grace:
            if child["p"].poll() is not None: break
            time.sleep(0.05)
        if child["p"].poll() is None:
            child["p"].kill()
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
    ap.add_argument("--logdir", default=str(HERE / "logs"))
    ap.add_argument("--no-validate", action="store_true", help="skip name check vs frc.json")
    ap.add_argument("--autorestart", action="store_true", help="restart crashed workers")
    args = ap.parse_args()

    vcfg = load_vision_cfg(args.vision)
    prof = select_profile(vcfg)

    reserve = prof.get("reserve_cores", [0])  # keep core0 free by default
    cpu_map = cpu_map_for_profile(prof, reserve)

    if not args.no_validate:
        frc_names = validate_cams_against_frc(args.frc)
        want = [c["name"] for c in prof.get("cameras", [])]
        missing = [n for n in want if n not in frc_names]
        if missing:
            raise SystemExit(f"Profile cameras not in {args.frc}: {missing} (have {frc_names})")

    children = {}

    # spawn camera workers
    for cam in prof.get("cameras", []):
        name = cam["name"]
        cpu = cpu_map.get(name)
        cmd = [sys.executable, CAM_WORKER, "--cam", name, "--vision", args.vision, "--frc", args.frc]
        if cpu is not None: cmd += ["--cpu", str(cpu)]
        log_path = Path(args.logdir) / f"camera-{name}.log"
        children[f"cam:{name}"] = spawn(cmd, log_path)
        log.info(f"started cam {name} cpu={cpu} -> {log_path}")

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

    # supervise
    while True:
        for key, ch in list(children.items()):
            rc = ch["p"].poll()
            if rc is None: continue
            log.warning(f"{key} exited rc={rc}")
            ch["log"].flush(); ch["log"].close()
            if args.autorestart and not stopping["flag"]:
                time.sleep(0.5)
                children[key] = spawn(ch["cmd"], ch["log_path"])
                children[key]["restart_count"] = ch["restart_count"] + 1
                log.info(f"restarted {key} (count={children[key]['restart_count']})")
            else:
                del children[key]
        time.sleep(0.25)

if __name__ == "__main__":
    main()
