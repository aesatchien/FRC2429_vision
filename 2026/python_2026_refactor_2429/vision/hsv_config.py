def get_hsv_config(color, x_resolution, y_resolution):
    """
    Returns the HSV threshold and contour filtering configuration for a given color.
    """
    cfg = {}

    if color == 'orange':  # 2024 orange rings
        cfg['_hsv_threshold_hue'] = [0, 10]
        cfg['_hsv_threshold_saturation'] = [50, 254]
        cfg['_hsv_threshold_value'] = [10, 254]
        cfg['_blur_radius'] = 3
        cfg['_filter_contours_max_ratio'] = 10
        cfg['_filter_contours_min_ratio'] = 2
        cfg['_filter_contours_min_area'] = 100.0
        cfg['_filter_contours_min_height'] = 15
        cfg['_filter_contours_min_width'] = 15
        cfg['_filter_contours_max_width'] = x_resolution // 2
        cfg['_filter_contours_max_height'] = y_resolution // 2
        cfg['_filter_contours_solidity'] = [10.0, 100.0]
        cfg['_filter_contours_box_fill'] = [10.0, 100]
        # above (y is 0 at top) or below this we ignore detections
        cfg['ignore_y'] = [0.3 * y_resolution, y_resolution]

    elif color == 'purple':  # 2023 purple cubes
        cfg['_hsv_threshold_hue'] = [117, 126]
        cfg['_hsv_threshold_saturation'] = [110, 255]
        cfg['_hsv_threshold_value'] = [130, 255]
        cfg['_blur_radius'] = 3
        cfg['_filter_contours_max_ratio'] = 2
        cfg['_filter_contours_min_ratio'] = 0.5
        cfg['_filter_contours_min_area'] = 100.0
        cfg['_filter_contours_min_height'] = 12
        cfg['_filter_contours_min_width'] = 12
        cfg['_filter_contours_max_width'] = 200
        cfg['_filter_contours_max_height'] = 200

    elif color == 'yellow':  # 2023 yellow cones
        cfg['_hsv_threshold_hue'] = [17, 23]
        cfg['_hsv_threshold_saturation'] = [150, 254]
        cfg['_hsv_threshold_value'] = [120, 254]
        cfg['_filter_contours_max_ratio'] = 3
        cfg['_filter_contours_min_ratio'] = 0.1
        cfg['_filter_contours_min_area'] = 2000.0
        cfg['_filter_contours_min_height'] = 30
        cfg['_filter_contours_min_width'] = 30
        cfg['_filter_contours_max_width'] = 200
        cfg['_filter_contours_max_height'] = 200

    elif color == 'yellow_balls':  # 2020 yellow balls
        cfg['_hsv_threshold_hue'] = [20, 30]
        cfg['_hsv_threshold_saturation'] = [128, 255]
        cfg['_hsv_threshold_value'] = [100, 255]

    elif color == 'blue':  # blue balls
        cfg['_hsv_threshold_hue'] = [114, 136]
        cfg['_hsv_threshold_saturation'] = [80, 255]
        cfg['_hsv_threshold_value'] = [80, 255]
        cfg['_filter_contours_solidity'] = [50.0, 100.0]
        cfg['_filter_contours_box_fill'] = [50.0, 95.0]
        cfg['_filter_contours_max_ratio'] = 2.0
        cfg['_filter_contours_min_area'] = 30.0
        cfg['_filter_contours_max_height'] = 60

    elif color == 'red':  # red balls
        cfg['_hsv_threshold_hue'] = [0, 10]
        cfg['_hsv_threshold_saturation'] = [150, 255]
        cfg['_hsv_threshold_value'] = [50, 255]
        cfg['_filter_contours_solidity'] = [50.0, 100.0]
        cfg['_filter_contours_box_fill'] = [50.0, 95.0]
        cfg['_filter_contours_max_ratio'] = 2.0
        cfg['_filter_contours_max_height'] = 60

    elif color == 'green':  # vision targets for 2023 are small squares
        cfg['_hsv_threshold_hue'] = [80, 88]
        cfg['_hsv_threshold_saturation'] = [110, 254]
        cfg['_hsv_threshold_value'] = [150, 254]
        cfg['_filter_contours_min_width'] = 5
        cfg['_filter_contours_min_height'] = 5
        cfg['_filter_contours_min_ratio'] = 0.2
        cfg['_filter_contours_max_ratio'] = 3

    else:
        print(f'No valid color provided: {color}')

    return cfg

def get_target_dimensions(color):
    """
    Returns the physical width (meters) of the target object for distance calculation.
    """
    # Conversions
    in_to_m = 0.0254
    
    if color == 'yellow': return 8 * in_to_m      # 2023 Cone base
    if color == 'purple': return 9.5 * in_to_m    # 2023 Cube (approx)
    if color == 'orange': return 14 * in_to_m     # 2024 Ring
    if color == 'blue':   return 9.5 * in_to_m    # 2022 Ball
    if color == 'red':    return 9.5 * in_to_m    # 2022 Ball
    if color == 'green':  return 2 * in_to_m      # Retro-reflective tape width
    
    return 9.5 * in_to_m # Default