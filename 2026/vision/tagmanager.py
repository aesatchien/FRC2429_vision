# make a tag filter that averages tags if they are coming quickly
import numpy as np
import time


class TagFilter:
    def __init__(self, max_dt=1.0, max_averages=20, max_std=0.1):
        self.current_time = time.time()
        self.previous_time = self.current_time
        self.max_dt = max_dt
        self.max_averages = max_averages
        self.max_std = max_std
        self.max_translation = 0.05 # 0.1 # arbitryary for now, if moving 1 m/s you'd move .02 m/frame at 50FPS
        self.max_rotation = 0.1  # 0.05  #  harder - radians and we spin really fast - like 6.28/s so
        self.is_stable = False

        # Use fixed-size arrays initialized with NaN for efficiency
        self.history = {key: np.full(max_averages, np.nan) for key in ["tx", "ty", "tz", "rx", "ry", "rz"]}
        self.index = 0  # Rolling index to track position

    def get_values(self, index):
        """Retrieve [tx, ty, tz, rx, ry, rz] at a given history index safely."""
        if self.index == 0:
            raise ValueError("No data has been recorded yet.")
        return [float(self.history[key][index % self.max_averages]) for key in ["tx", "ty", "tz", "rx", "ry", "rz"]]

    def reset(self, tx, ty, tz, rx, ry, rz):
        new_values = np.array([tx, ty, tz, rx, ry, rz])
        for key in self.history:
            self.history[key].fill(np.nan)  # Reset all values to NaN
            self.history[key][0] = new_values[["tx", "ty", "tz", "rx", "ry", "rz"].index(key)]
        self.index = 1  # Reset index to start fresh

    def update(self, tx, ty, tz, rx, ry, rz):
        # if we're stable, average the tag values, otherwise reset and return current values
        self.current_time = time.time()
        dt = self.current_time - self.previous_time

        new_values = np.array([tx, ty, tz, rx, ry, rz])

        # If max time delta is exceeded, reset history to only the latest values
        if dt > self.max_dt or self.index == 0:
            self.reset(tx, ty, tz, rx, ry, rz)
            # print(".", end=" ")
        else:
            # Rolling update without reallocation - see if we moved too much
            txi, tyi, tzi, rxi, ryi, rzi = self.get_values(self.index - 1)
            d_translation = np.sqrt((tx - txi) ** 2 + (ty - tyi) ** 2 + (tz - tzi) ** 2)
            d_rotation = np.sqrt((rx - rxi) ** 2 + (ry - ryi) ** 2 + (rz - rzi) ** 2)
            if d_translation > self.max_translation or d_rotation > self.max_rotation:
                # we're probably moving, so reset
                self.reset(tx, ty, tz, rx, ry, rz)
            else:
                for key in self.history:
                    self.history[key][self.index % self.max_averages] = new_values[
                        ["tx", "ty", "tz", "rx", "ry", "rz"].index(key)]
                    # if self.index %500==2:
                    #     std_devs = np.array([np.nanstd(self.history[key]) for key in self.history])
                    #     print(f"{self.index} stdev:{std_devs[0]:.3e}", end=" ")
                self.index += 1
        # Compute averages ignoring NaN values
        averages = np.array([np.nanmean(self.history[key]) for key in self.history])

        # Compute standard deviation and check stability
        std_devs = np.array([np.nanstd(self.history[key]) for key in self.history])
        self.is_stable = np.all(std_devs < self.max_std)

        self.previous_time = self.current_time
        return tuple(map(float, averages))


class TagManager:
    def __init__(self, max_dt=1.0, max_averages=10, max_std=0.1):
        self.filters = {tag_id: TagFilter(max_dt, max_averages, max_std) for tag_id in range(1, 23)}

    def update(self, tag_id, tx, ty, tz, rx, ry, rz):
        if tag_id in self.filters:
            return self.filters[tag_id].update(tx, ty, tz, rx, ry, rz)
        else:
            raise ValueError(f"Invalid tag ID: {tag_id}")
