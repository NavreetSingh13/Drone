import math

def get_single_axis_delta(value1, value2):
    """Returns the delta between two coordinates along one axis."""
    return value2 - value1

def point_in_rectangle(point, left, right, top, bottom):
    """Check if a point is inside the bounding box."""
    return left < point[0] < right and top < point[1] < bottom

def estimate_distance_from_box(box_height, image_height, known_person_height_m=1.7, focal_length_px=600):
    """
    Approximates the distance from bounding box height.
    Formula: Distance = (Real Height * Focal Length) / Pixel Height
    Calibrated on 640x480 video with person 1.7m tall.
    """
    if box_height <= 0:
        return 10.0  # Assume far away if no box
    return (known_person_height_m * focal_length_px) / box_height
