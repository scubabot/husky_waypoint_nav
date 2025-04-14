# Notes on Changes
This package takes move_base out of the waypoint goals, and uses a custom velocity controller based on [--insert refernce]

We still need to figure out how to make the waypoint reached status more accurate

# How to launch

First, launch natnet node for optitrack:

	`roslaunch natnet_ros_cpp gui_natnet_ros.launch`


To launch the collection: 

	rosrun husky_waypoint_nav collect_optitrack_waypoints.py

To launch navigator:

	rosrun husky_waypoint_nav optitrack_waypoint_navigator.py

# Debugging Tips

## Time issues due to offline errors
If you run into time sync errors, run the following when connected to internet:

	`sudo timedatectl set-ntp true`
AND
	`sudo hwclock --systohc`


## URDF issues:


## Costmap issues:


