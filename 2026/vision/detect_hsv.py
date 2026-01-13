import cv2
import numpy as np
import math
from vision.hsv_config import get_hsv_config, get_target_dimensions

class HSVDetector:
    def __init__(self, camera_model):
        self.cam = camera_model
        self.fov_h = self.cam.get_fov()

    def process(self, image, color, training=False, train_box=None):
        # 1. Get Config
        cfg = get_hsv_config(color, self.cam.width, self.cam.height)
        if not cfg: return {}

        # 2. Pre-process
        if cfg.get('_blur_radius', 0) > 0:
            image = cv2.blur(image, (cfg['_blur_radius'], cfg['_blur_radius']))
            
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 3. Threshold
        lower = np.array([cfg['_hsv_threshold_hue'][0], cfg['_hsv_threshold_saturation'][0], cfg['_hsv_threshold_value'][0]])
        upper = np.array([cfg['_hsv_threshold_hue'][1], cfg['_hsv_threshold_saturation'][1], cfg['_hsv_threshold_value'][1]])
        
        stats = {}
        if training:
            stats = self._calc_training_stats(hsv, self.cam.width, self.cam.height, train_box)
            if stats:
                lower = np.array([stats['hue'][0], stats['sat'][0], stats['val'][0]])
                upper = np.array([stats['hue'][1], stats['sat'][1], stats['val'][1]])

        mask = cv2.inRange(hsv, lower, upper)
        
        # 4. Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 5. Filter
        filtered = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            if area < cfg.get('_filter_contours_min_area', 0): continue
            if w < cfg.get('_filter_contours_min_width', 0): continue
            ratio = w / h
            if ratio < cfg.get('_filter_contours_min_ratio', 0): continue
            if ratio > cfg.get('_filter_contours_max_ratio', 100): continue
            
            # Y-limit check
            ignore_y = cfg.get('ignore_y', [9999, 9999])
            if y > ignore_y[0]: continue
            
            filtered.append(c)
            
        # 6. Sort (Size)
        filtered.sort(key=cv2.contourArea, reverse=True)
        
        # 7. Calculate Attributes for Top Target
        results = {
            'targets': len(filtered),
            'contours': filtered,
            'ids': [], 'distances': [], 'strafes': [], 'rotations': [], 'heights': [],
            'cx': [], 'cy': []
        }
        
        target_width_m = get_target_dimensions(color)
        
        for i, c in enumerate(filtered):
            x, y, w, h = cv2.boundingRect(c)
            
            # Distance Calc
            # distance = object_width / (2 * tan(width_in_fov * fov / 2))
            width_fraction = w / self.cam.width
            distance = target_width_m / (2 * math.tan(width_fraction * math.radians(self.fov_h) / 2.0))
            
            # Rotation/Strafe
            cx = x + w/2
            cy = y + h/2
            
            # Map cx from [0, width] to [-1, 1]
            norm_x = (2.0 * cx / self.cam.width) - 1.0
            rotation = -norm_x * (self.fov_h / 2.0)
            strafe = math.sin(math.radians(rotation)) * distance
            
            results['ids'].append(0) # Color targets don't have specific IDs
            results['distances'].append(distance)
            results['rotations'].append(rotation)
            results['strafes'].append(strafe)
            results['heights'].append(h)
            results['cx'].append(cx)
            results['cy'].append(cy)
            
        # 8. Training Stats (if requested)
        if training and stats:
            results['training_stats'] = stats
            
        return results

    def _calc_training_stats(self, hsv_img, w, h, train_box=None):
        # Sample center box
        cw, ch = 10, 20
        if train_box and len(train_box) >= 2:
            cx = int(w * train_box[0])
            cy = int(h * train_box[1])
        else:
            cx, cy = w // 2, h // 2
        sample = hsv_img[cy-ch:cy+ch, cx-cw:cx+cw, :]
        
        if sample.size == 0: return {}
        
        hue = sample[:,:,0]
        sat = sample[:,:,1]
        val = sample[:,:,2]
        
        # Simple stats
        return {
            'hue': [float(np.min(hue)), float(np.max(hue))],
            'sat': [float(np.min(sat)), float(np.max(sat))],
            'val': [float(np.min(val)), float(np.max(val))]
        }





