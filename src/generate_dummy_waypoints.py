#!/usr/bin/env python
import os

waypoint_path = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/optitrack_waypoints.txt")
dummy_waypoints = [
    "1.0 0.0 0.0\n",
    "2.0 1.0 0.0\n",
    "3.0 0.0 0.0\n"
]

if not os.path.exists(waypoint_path) or os.stat(waypoint_path).st_size == 0:
    with open(waypoint_path, 'w') as f:
        f.writelines(dummy_waypoints)
    print("[Dummy Waypoints] Created optitrack_waypoints.txt with default points.")
else:
    print("[Dummy Waypoints] File already exists and is not empty.")
