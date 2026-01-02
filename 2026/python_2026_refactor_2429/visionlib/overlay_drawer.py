import cv2
import numpy as np

def get_solidity(contour):
    rect = cv2.boundingRect(contour)
    x, y, w, h = rect
    box_area = w * h
    area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = 100 * area / hull_area if hull_area > 0 else 0
    box_fill = 100 * area / box_area if box_area > 0 else 0
    return box_fill, solidity

def draw_overlays(image, tag_results, color_results, ctx, training=False, debug=False):
    """
    Draws visual overlays on the image based on detection results.
    This runs in the Stream thread, decoupled from detection.
    """
    h_res, w_res = image.shape[:2]
    
    # Color definitions
    primary_colors = {'yellow': (0, 255, 0), 'purple':(0, 0, 255), 'green':(255, 255, 255), 'orange': (0, 165, 255)}
    secondary_colors = {'yellow': (255, 255, 0), 'purple':(255, 0, 255), 'green':(255, 255, 255), 'orange': (0, 200, 255)}
    text_colors = {'yellow': (0, 255, 255), 'purple':(255, 255, 0), 'green':(255, 255, 255), 'orange': (0, 255, 255)}

    # 1. Draw Color Targets
    for color_name, res in color_results.items():
        if color_name == 'training_stats': continue
        
        contours = res.get('contours', [])
        
        # Draw contours and bounding boxes
        for ix, contour in enumerate(contours):
            if ix == 0:
                box_color = primary_colors.get(color_name, (255, 255, 127))
                thickness = 2
            else:
                box_color = secondary_colors.get(color_name, (255, 255, 127))
                thickness = 1

            rect = cv2.boundingRect(contour)
            x, y, w, h = rect

            if debug or training:
                cv2.drawContours(image, contours, ix, box_color, thickness)
                
                # Draw debug stats
                text_color = text_colors.get(color_name, (255, 255, 127))
                box_fill, solidity = get_solidity(contour)
                cv2.putText(image, f'bf {int(box_fill)}, sol {int(solidity)}', (int(x), int(y)-17), 1, 1, text_color, 1)
                
                # Draw dimensions
                cv2.putText(image, f'{w:02d},{h:02d}', (int(x+w), int(y+w)), 1, 1.5, text_color, 1)
            else:
                cv2.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), box_color, thickness)

        # Draw detailed text for primary target
        if len(contours) > 0:
            dist = res.get('distances', [0])[0]
            strafe = res.get('strafes', [0])[0]
            height = res.get('heights', [0])[0]
            rot = res.get('rotations', [0])[0]
            
            text = f"Dist: {dist:3.2f} Str: {strafe:2.1f} H: {height:2.0f} Rot: {rot:+2.0f} deg"
            cv2.putText(image, text, (int(0.02 * w_res), 27), 1, 0.9, (255, 255, 0), 1)

            # Draw target lines
            camera_shift = 0 # TODO: get from ctx if needed
            cv2.line(image, (int(0.4 * w_res + camera_shift), int(0.9 * h_res)),
                     (int(0.4 * w_res + camera_shift), int(0.14 * h_res)), (0, 255, 0), 2)
            cv2.line(image, (int(0.6 * w_res + camera_shift), int(0.9 * h_res)),
                     (int(0.6 * w_res + camera_shift), int(0.14 * h_res)), (0, 255, 0), 2)
        else:
             # Draw grey lines if no target
             camera_shift = 0
             cv2.line(image, (int(0.4 * w_res + camera_shift), int(0.9 * h_res)),
                      (int(0.4 * w_res + camera_shift), int(0.14 * h_res)), (127, 127, 127), 1)
             cv2.line(image, (int(0.6 * w_res + camera_shift), int(0.9 * h_res)),
                      (int(0.6 * w_res + camera_shift), int(0.14 * h_res)), (127, 127, 127), 1)

    # 2. Draw AprilTags
    if len(tag_results) > 0:
        for t_id, t_data in tag_results.items():
            # Draw Polygon
            if 'corners' in t_data:
                corners = np.array(t_data['corners']).reshape((-1, 1, 2)).astype(dtype=np.int32)
                color = (255, 75, 0) if int(t_id) > 12 else (0, 0, 255) # 2025 colors
                cv2.polylines(image, [corners], isClosed=True, color=color, thickness=2)

            # Center point
            cx = int(t_data.get('cx', 0))
            cy = int(t_data.get('cy', 0))
            
            # Draw ID
            cv2.putText(image, f'{t_id:2d}', (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Draw 3D Axes if we have pose data and intrinsics
            if ctx.intrinsics and ctx.distortions and 'tx' in t_data:
                try:
                    if 'rvec' in t_data and 'tvec' in t_data:
                        rvec = np.array(t_data['rvec'], dtype=np.float64)
                        tvec = np.array(t_data['tvec'], dtype=np.float64)
                        
                        K = np.array([
                            [ctx.intrinsics['fx'], 0, ctx.intrinsics['cx']],
                            [0, ctx.intrinsics['fy'], ctx.intrinsics['cy']],
                            [0, 0, 1]
                        ], dtype=np.float64)
                        
                        dist = np.array(ctx.distortions, dtype=np.float64)
                        cv2.drawFrameAxes(image, K, dist, rvec, tvec, 0.15) # 0.15m axis length
                except Exception:
                    pass # Avoid crashing stream on math errors

    # 3. Draw HUD (Top Bar)
    # Black bar
    cv2.rectangle(image, (0, 0), (w_res, 35), (0, 0, 0), -1)
    
    # Info text
    info_text_color = (0, 255, 255)
    target_text_color = (255, 255, 0)
    
    target_count_msg = ""
    for color_name in ctx.colors:
        if color_name in color_results:
            cnt = color_results[color_name].get('targets', 0)
            target_count_msg += f"{color_name[0].upper()}:{cnt} "
    
    tag_cnt = len(tag_results)
    target_count_msg += f"T:{tag_cnt}"
    
    cv2.putText(image, target_count_msg, (int(0.7 * w_res), 13), 1, 0.9, target_text_color, 1)
    cv2.putText(image, f"{ctx.name} {ctx.colors}", (2, 10), 1, 0.9, info_text_color, 1)

    # 4. Training Stats Overlay
    if training and 'training_stats' in color_results:
        stats = color_results['training_stats']
        hue = stats.get('hue', [0,0])
        sat = stats.get('sat', [0,0])
        val = stats.get('val', [0,0])
        
        hue_msg = f"hue min max: {hue[0]:.0f} {hue[1]:.0f}"
        sat_msg = f"sat min max: {sat[0]:.0f} {sat[1]:.0f}"
        val_msg = f"val min max: {val[0]:.0f} {val[1]:.0f}"
        
        cv2.putText(image, hue_msg, (2, 21), 1, 0.8, (0, 255, 200), 1)
        cv2.putText(image, sat_msg, (2, 32), 1, 0.8, (0, 255, 200), 1)
        cv2.putText(image, val_msg, (2, 43), 1, 0.8, (0, 255, 200), 1)

    return image