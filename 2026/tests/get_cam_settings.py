import os
import ctypes
import fcntl
import re
import sys

# --- V4L2 Constants & Macros ---
_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_DIRBITS = 2
_IOC_READ = 2
_IOC_WRITE = 1


def _IOWR(type_val, nr, size_obj):
    size = ctypes.sizeof(size_obj)
    return ((_IOC_READ | _IOC_WRITE) << 30) | \
        (size << 16) | \
        (type_val << 8) | \
        (nr << 0)


def _IOR(type_val, nr, size_obj):
    size = ctypes.sizeof(size_obj)
    return (_IOC_READ << 30) | \
        (size << 16) | \
        (type_val << 8) | \
        (nr << 0)


V4L2_CTRL_FLAG_NEXT_CTRL = 0x80000000
V4L2_CTRL_FLAG_DISABLED = 0x0001
V4L2_CTRL_TYPE_CTRL_CLASS = 6


# --- C Structures ---
class v4l2_capability(ctypes.Structure):
    _fields_ = [
        ('driver', ctypes.c_char * 16),
        ('card', ctypes.c_char * 32),
        ('bus_info', ctypes.c_char * 32),
        ('version', ctypes.c_uint32),
        ('capabilities', ctypes.c_uint32),
        ('device_caps', ctypes.c_uint32),
        ('reserved', ctypes.c_uint32 * 3),
    ]


class v4l2_queryctrl(ctypes.Structure):
    _fields_ = [
        ('id', ctypes.c_uint32),
        ('type', ctypes.c_uint32),
        ('name', ctypes.c_char * 32),
        ('minimum', ctypes.c_int32),
        ('maximum', ctypes.c_int32),
        ('step', ctypes.c_int32),
        ('default_value', ctypes.c_int32),
        ('flags', ctypes.c_uint32),
        ('reserved', ctypes.c_uint32 * 2),
    ]


class v4l2_control(ctypes.Structure):
    _fields_ = [('id', ctypes.c_uint32), ('value', ctypes.c_int32)]


VIDIOC_QUERYCAP = _IOR(ord('V'), 0, v4l2_capability)
VIDIOC_QUERYCTRL = _IOWR(ord('V'), 36, v4l2_queryctrl)
VIDIOC_G_CTRL = _IOWR(ord('V'), 27, v4l2_control)


# --- Helper Functions ---

def get_path_id(target_dev):
    """
    Scans /dev/v4l/by-path to find the symlink pointing to target_dev.
    Returns the filename of the symlink (e.g. platform-xhci-hcd.11...)
    """
    by_path_dir = "/dev/v4l/by-path"
    if not os.path.exists(by_path_dir):
        return "No Path Info"

    target_real = os.path.realpath(target_dev)

    for fname in os.listdir(by_path_dir):
        full_path = os.path.join(by_path_dir, fname)
        if os.path.realpath(full_path) == target_real:
            return fname
    return "Unknown Path"


def get_device_info(dev_path):
    info = {'card': 'Unknown', 'driver': 'Unknown'}
    if not os.path.exists(dev_path):
        return info

    try:
        fd = os.open(dev_path, os.O_RDWR, 0)
        cap = v4l2_capability()
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap)
        info['card'] = cap.card.decode('utf-8', errors='ignore')
        info['driver'] = cap.driver.decode('utf-8', errors='ignore')
        os.close(fd)
    except OSError:
        pass

    return info


def sanitize_name(name_bytes):
    raw = name_bytes.decode('utf-8', errors='ignore').strip()
    raw = raw.lower()
    raw = re.sub(r'[^a-z0-9]+', '_', raw)
    return raw.strip('_')


def get_cam_data(device_path):
    data = {}
    if not os.path.exists(device_path):
        return {}

    try:
        fd = os.open(device_path, os.O_RDWR, 0)
    except OSError:
        return {}

    try:
        query = v4l2_queryctrl()
        query.id = V4L2_CTRL_FLAG_NEXT_CTRL | 0

        while True:
            try:
                fcntl.ioctl(fd, VIDIOC_QUERYCTRL, query)
            except OSError:
                break

            # Skip disabled controls and Class Headers
            if not (query.flags & V4L2_CTRL_FLAG_DISABLED) and query.type != V4L2_CTRL_TYPE_CTRL_CLASS:
                ctrl = v4l2_control()
                ctrl.id = query.id
                name_str = sanitize_name(query.name)
                try:
                    fcntl.ioctl(fd, VIDIOC_G_CTRL, ctrl)
                    data[query.id] = (name_str, ctrl.value)
                except OSError:
                    data[query.id] = (name_str, "Err")

            query.id |= V4L2_CTRL_FLAG_NEXT_CTRL
    finally:
        os.close(fd)
    return data


def main():
    dev0 = "/dev/video0"
    dev2 = "/dev/video2"

    # 1. Gather Basic Info
    info0 = get_device_info(dev0)
    info2 = get_device_info(dev2)

    # 2. Resolve Paths and Diff them
    path0 = get_path_id(dev0)
    path2 = get_path_id(dev2)

    # Simple logic to strip common prefix to make the ID shorter/readable
    # If paths are identical (impossible for diff ports) or empty, show full
    common_prefix_len = 0
    if path0 and path2 and path0 != "Unknown Path" and path2 != "Unknown Path":
        # Find common prefix length
        for i, (c1, c2) in enumerate(zip(path0, path2)):
            if c1 != c2:
                break
            common_prefix_len = i

    # We strip the prefix, but if it strips everything (identical strings), we keep full.
    # We also keep the last few chars of the prefix (like "usb-") for context if we can.
    disp_path0 = path0[common_prefix_len:] if common_prefix_len < len(path0) else path0
    disp_path2 = path2[common_prefix_len:] if common_prefix_len < len(path2) else path2

    # 3. Gather Controls
    data0 = get_cam_data(dev0)
    data2 = get_cam_data(dev2)

    all_ids = set(data0.keys()) | set(data2.keys())
    sorted_ids = sorted(list(all_ids))

    # 4. Print Table (Widened Columns to 20 chars)
    # 32 (Name) + 3 ( | ) + 20 (Video0) + 3 ( | ) + 20 (Video2) = 78 chars
    print("\n" + "=" * 78)
    print(f"{'DEVICE INFO':<32} | {'VIDEO0':^20} | {'VIDEO2':^20}")
    print("=" * 78)
    print(f"{'Model':<32} | {info0['card'][:20]:^20} | {info2['card'][:20]:^20}")
    # Display the Unique ID (Path Suffix)
    print(f"{'Unique Path ID':<32} | {disp_path0[:20]:^20} | {disp_path2[:20]:^20}")
    print("-" * 78)
    print(f"{'CONTROL NAME':<32} | {'VAL':^20} | {'VAL':^20}")
    print("-" * 78)

    for uid in sorted_ids:
        row0 = data0.get(uid, (None, "-"))
        row2 = data2.get(uid, (None, "-"))

        display_name = row0[0] if row0[0] else row2[0]
        val0 = row0[1]
        val2 = row2[1]

        print(f"{display_name:<32} | {str(val0):^20} | {str(val2):^20}")


if __name__ == "__main__":
    main()