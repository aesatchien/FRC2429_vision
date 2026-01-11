# make a tag filter that averages tags if they are coming quickly
import numpy as np
import time


class TagFilter:
    def __init__(self, max_dt=0.5, max_averages=10, max_std=0.05):
        self.max_dt = max_dt
        self.max_averages = max_averages
        self.max_std = max_std
        
        # Rejection Thresholds: Points outside this deviation from the median are ignored
        # unless they persist (indicating real motion).
        self.reject_dist = 0.5  # meters (large jump = glitch)
        self.reject_rot = 0.5   # radians

        self.is_stable = False
        self.prev_time = 0.0

        # History buffer: [Rows=Samples, Cols=6 (tx,ty,tz,rx,ry,rz)]
        self.history = np.zeros((max_averages, 6), dtype=np.float64)
        self.head = 0  # Write pointer
        self.size = 0  # Number of valid samples
        
        # Consistency counter for fast motion / teleport detection
        self.outlier_count = 0
        self.max_outliers = 2 # Accept new position after 2 consistent "outliers"

    def reset(self, values, now):
        self.head = 1 % self.max_averages
        self.size = 1
        self.history[0] = values
        self.prev_time = now
        self.outlier_count = 0
        self.is_stable = False

    def update(self, tx, ty, tz, rx, ry, rz):
        now = time.time()
        dt = now - self.prev_time
        new_values = np.array([tx, ty, tz, rx, ry, rz])

        # 1. Check Time Timeout
        if dt > self.max_dt:
            self.reset(new_values, now)
            return new_values

        # 2. Outlier / Motion Logic
        if self.size > 0:
            # Compare against the current median (robust center)
            # We use the last calculated median or the last raw if size is small
            valid_hist = self.history[:self.size] if self.size < self.max_averages else self.history
            current_ref = np.median(valid_hist, axis=0)

            delta = np.abs(new_values - current_ref)
            dist_tr = np.linalg.norm(delta[:3]) # Euclidean dist for translation
            dist_rot = np.linalg.norm(delta[3:])
            
            if dist_tr > self.reject_dist or dist_rot > self.reject_rot:
                self.outlier_count += 1
                if self.outlier_count > self.max_outliers:
                    # It's not a glitch, we actually moved there. Reset filter to snap to new pos.
                    self.reset(new_values, now)
                    return new_values
                else:
                    # Ignore this frame, return previous valid result (suppress glitch)
                    return current_ref
            else:
                self.outlier_count = 0

        # 3. Add to History
        self.history[self.head] = new_values
        self.head = (self.head + 1) % self.max_averages
        if self.size < self.max_averages:
            self.size += 1
        
        self.prev_time = now

        # 4. Compute Average & Stability
        valid_data = self.history[:self.size] if self.size < self.max_averages else self.history
        
        # Standard deviation check
        std_devs = np.std(valid_data, axis=0)
        self.is_stable = np.all(std_devs < self.max_std)

        # If sitting still (stable), return SMOOTHED value
        # Translation: Median is robust against noise if moving and having outliers
        # t_avg = np.median(valid_data[:, :3], axis=0)
        # Translation: Mean is statistically better for stationary Gaussian noise
        t_avg = np.mean(valid_data[:, :3], axis=0)

        # Rotation: Circular Mean (handles wrapping 179 -> -179 correctly)
        # Convert to complex numbers: exp(i * angle)
        rots = valid_data[:, 3:]
        complex_rots = np.exp(1j * rots)
        mean_complex = np.mean(complex_rots, axis=0)
        r_avg = np.angle(mean_complex)
        
        return np.concatenate((t_avg, r_avg))

class TagManager:
    def __init__(self, max_dt=1.0, max_averages=10, max_std=0.1):
        self.params = (max_dt, max_averages, max_std)
        self.filters = {}

    def update(self, tag_id, tx, ty, tz, rx, ry, rz):
        if tag_id not in self.filters:
            self.filters[tag_id] = TagFilter(*self.params)
        return self.filters[tag_id].update(tx, ty, tz, rx, ry, rz)

    def process(self, tags: dict, averaging_enabled: bool = False) -> dict:
        """
        Process a dictionary of tag results (from TagDetector), applying temporal
        smoothing to any tags that are part of the field layout.
        Returns a new dictionary with smoothed poses.
        """
        out = {}
        for key, tag in tags.items():
            # Only filter tags that have a valid field pose (in_layout=True)
            # Skip smoothing for multi-tag result (ID -1) as it is already aggregated
            if tag.get("in_layout") and tag['id'] != -1:
                # Always update filter to keep history current
                res = self.update(
                    tag['id'], 
                    tag['tx'], tag['ty'], tag['tz'], 
                    tag['rx'], tag['ry'], tag['rz']
                )
                
                # Copy tag data and overwrite pose with smoothed values
                # res is a numpy array, convert to float for safety
                new_tag = tag.copy()
                
                if averaging_enabled:
                    # Overwrite with smoothed values
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
