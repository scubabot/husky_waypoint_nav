# robot_math_utils.py
import math
from geometry_msgs.msg import Quaternion

def get_yaw_from_quaternion(quaternion_msg: Quaternion) -> float:
    """
    Calculates the yaw angle (rotation around the Z-axis) from a 
    ROS geometry_msgs.msg.Quaternion.
    Input is a geometry_msgs.msg.Quaternion.
    Returns the yaw angle in radians (-pi to +pi).
    """
    if not isinstance(quaternion_msg, Quaternion):
        # You could raise a TypeError or log a warning/error
        print(f"Warning: Expected geometry_msgs.msg.Quaternion, got {type(quaternion_msg)}")
        # Attempt to proceed if it has x,y,z,w attributes (duck typing)
        # For robustness, it's better to ensure the type or structure.
        # For this example, we'll assume it has the necessary attributes if not the exact type.

    x = quaternion_msg.x
    y = quaternion_msg.y
    z = quaternion_msg.z
    w = quaternion_msg.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return yaw

def normalize_angle_radians(angle_rad: float) -> float:
    """
    Normalizes an angle in radians to the range [-pi, pi].
    """
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad

def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Calculates the Euclidean distance between two 2D points.
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
