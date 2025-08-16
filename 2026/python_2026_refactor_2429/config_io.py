import json, socket, subprocess
from typing import Any, Dict

VISION_CONFIG_FILE = "/boot/vision.json"

def _local_ips() -> set[str]:
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0]); s.close()
    except: pass
    try:
        host = socket.gethostname()
        _,_, addrs = socket.gethostbyname_ex(host)
        ips.update(a for a in addrs if ":" not in a)
    except: pass
    try:
        out = subprocess.run(["ip","-4","addr"], capture_output=True, text=True).stdout
        for tok in out.split():
            if tok.count(".")==3 and "/" in tok: ips.add(tok.split("/")[0])
    except: pass
    return ips

def load_vision_cfg(path: str = VISION_CONFIG_FILE) -> Dict[str, Any]:
    with open(path, "rt", encoding="utf-8") as f:
        return json.load(f)

def select_profile(cfg: Dict[str, Any]) -> Dict[str, Any]:
    hosts = cfg.get("hosts", {})
    for ip in _local_ips():
        if ip in hosts: return hosts[ip]
    hn = socket.gethostname()
    if hn in hosts: return hosts[hn]
    default = cfg.get("default")
    if not default: raise RuntimeError("No vision profile matched and no default provided.")
    return default
