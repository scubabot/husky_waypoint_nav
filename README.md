# Notes on Changes
NOW THERE ARE TWO LAUNCH FILES, FIRST MAKE SURE THE COLLECTOR RUNS, THEN THE NAVIGATOR. ALSO, MAKE SURE HUKSY_NAVIGATON AND HUSKY_CONTROL PROPERLY EDITED. NOT DEFAULTS. 
THE EKF PART OF THE PACKAGE IS REMOVED AS OF NOW. YOU DONT NEED THE LOCALIZATION.YAML FILE OR THE POSE_COVARIANCE.PY FILE. 

OBVIOUSLY A FEW OF THESE FILES NEED TO BE REMOVED AND CLEANED UP. 

THIS ASSUMES YOU HAVE CLEARPATH PACKAGES INSTALLED AT CATKIN_WS/SRC

# How to launch

First, launch natnet node for optitrack:

	`roslaunch natnet_ros_cpp gui_natnet_ros.launch`


To to launch move_base with static map and  DWA planner:

	`roslaunch husky_waypoint_nav with_map_husky_nav.launch`


To launch move_base without static map and TrejectoryPlanner:

	`roslaunch husky_waypoint_nav no_map_husky_nav.launch`


To launch the collection: 

	`rosrun husky_waypoint_nav collect_optitrack_waypoints.py`

To launch navigator:
	`rosrun husky_waypoint_nav optitrack_waypoint_navigator.py`
