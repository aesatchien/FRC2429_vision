import subprocess
import sys
import argparse

# Usage examples:
# python3 set_exposure.py 150       # Sets manual mode, then exposure 150
# python3 set_exposure.py --manual  # Sets manual mode only

parser = argparse.ArgumentParser(description="Set camera exposure using v4l2-ctl")
parser.add_argument("value", type=int, nargs="?", help="Absolute exposure value (e.g. 100-5000)")
parser.add_argument("--manual", action="store_true", help="Set mode to Manual Exposure (1)")
args = parser.parse_args()

devices = ["/dev/video0", "/dev/video2"]

for path in devices:
    print(f"Configuring {path}...")

    if args.manual or args.value is not None:
        print("  -> Setting Manual Mode")
        subprocess.run(["v4l2-ctl", "-d", path, "--set-ctrl=exposure_auto=1"], check=False)
        
        if args.value is not None:
            print(f"  -> Setting Exposure to {args.value}")
            subprocess.run(["v4l2-ctl", "-d", path, f"--set-ctrl=exposure_time_absolute={args.value}"], check=False)
    
    else:
        print("  -> No action specified. Pass a value or --manual.")