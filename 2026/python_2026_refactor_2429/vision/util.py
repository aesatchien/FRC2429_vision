import time, sys, threading, signal, logging

log = logging.getLogger("vision")

stop_flags = []

def run_in_thread(fn):
    def wrapper(*args, **kwargs):
        stop = threading.Event()
        def target(): fn(*args, **kwargs, stop_flag=stop)
        t = threading.Thread(target=target, name=fn.__name__)
        t.start(); stop_flags.append(stop); return stop
    return wrapper

def handle_sigint(sig, frame):
    log.info("SIGINT received. Stopping workers...")
    for s in stop_flags: s.set()
    time.sleep(0.2)
    sys.exit(0)

signal.signal(signal.SIGINT, handle_sigint)

def nudge_brightness(cam, value):
    try:
        cam.setBrightness(int(value) + 1); time.sleep(0.25)
        cam.setBrightness(int(value))
    except Exception as e:
        log.debug(f"brightness nudge failed: {e}")
