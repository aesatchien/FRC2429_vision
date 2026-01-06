# make a tag filter that averages tags if they are coming quickly
import numpy as np
import time


class TagFilter:
    def __init__(self, max_dt=1.0, max_averages=20, max_std=0.1):
        self.max_dt = max_dt
        self.max_averages = max_averages
        self.max_std = max_std
        
        # Thresholds: If motion exceeds this per frame, reset the filter (don't average)
        self.max_translation = 0.05 # meters
        self.max_rotation = 0.1     # radians
        
        self.is_stable = False
        self.prev_time = 0.0

        # History buffer: [Rows=Samples, Cols=6 (tx,ty,tz,rx,ry,rz)]
        self.history = np.zeros((max_averages, 6), dtype=np.float64)
        self.head = 0  # Write pointer
        self.size = 0  # Number of valid samples
        self.last_raw = np.zeros(6) # Keep track of last raw input for motion check

    def reset(self, values, now):
        self.head = 0
        self.size = 1
        self.history[0] = values
        self.last_raw = values
        self.prev_time = now
        self.is_stable = True # Single point is technically stable

    def update(self, tx, ty, tz, rx, ry, rz):
        now = time.time()
        dt = now - self.prev_time
        new_values = np.array([tx, ty, tz, rx, ry, rz])

        # 1. Check Time Timeout
        if dt > self.max_dt:
            self.reset(new_values, now)
            return new_values

        # 2. Check Motion Threshold (vs last raw input)
        if self.size > 0:
            delta = np.abs(new_values - self.last_raw)
            dist_tr = np.linalg.norm(delta[:3])
            dist_rot = np.linalg.norm(delta[3:])
            
            if dist_tr > self.max_translation or dist_rot > self.max_rotation:
                self.reset(new_values, now)
                return new_values

        # 3. Add to History
        self.history[self.head] = new_values
        self.head = (self.head + 1) % self.max_averages
        if self.size < self.max_averages:
            self.size += 1
        
        self.last_raw = new_values
        self.prev_time = now

        # 4. Compute Average & Stability
        valid_data = self.history[:self.size] if self.size < self.max_averages else self.history
        
        # Standard deviation check
        std_devs = np.std(valid_data, axis=0)
        self.is_stable = np.all(std_devs < self.max_std)

        return np.mean(valid_data, axis=0)

class TagManager:
    def __init__(self, max_dt=1.0, max_averages=10, max_std=0.1):
        self.params = (max_dt, max_averages, max_std)
        self.filters = {}

    def update(self, tag_id, tx, ty, tz, rx, ry, rz):
        if tag_id not in self.filters:
            self.filters[tag_id] = TagFilter(*self.params)
        return self.filters[tag_id].update(tx, ty, tz, rx, ry, rz)

    def process(self, tags: dict) -> dict:
        """
        Process a dictionary of tag results (from TagDetector), applying temporal
        smoothing to any tags that are part of the field layout.
        Returns a new dictionary with smoothed poses.
        """
        out = {}
        for key, tag in tags.items():
            # Only filter tags that have a valid field pose (in_layout=True)
            if tag.get("in_layout"):
                # Update filter
                res = self.update(
                    tag['id'], 
                    tag['tx'], tag['ty'], tag['tz'], 
                    tag['rx'], tag['ry'], tag['rz']
                )
                
                # Copy tag data and overwrite pose with smoothed values
                # res is a numpy array, convert to float for safety
                new_tag = tag.copy()
                new_tag['tx'] = float(res[0])
                new_tag['ty'] = float(res[1])
                new_tag['tz'] = float(res[2])
                new_tag['rx'] = float(res[3])
                new_tag['ry'] = float(res[4])
                new_tag['rz'] = float(res[5])
                out[key] = new_tag
            else:
                # Pass through practice tags or invalid tags untouched
                out[key] = tag
        return out
