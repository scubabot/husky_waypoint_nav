# Notes on Changes
This package takes move_base out of the waypoint goals, and uses a custom velocity controller based on [--insert refernce]

We still need to figure out how to make the waypoint reached status more accurate

# How to launch

### First, launch natnet node for optitrack:

	roslaunch natnet_ros_cpp gui_natnet_ros.launch


### To launch the collection: 

	rosrun husky_waypoint_nav collect_optitrack_waypoints.py


### To launch navigator there are two options:

#### For the P controller, launch the default:

	roslaunch husky_waypoint_nav navigator.launch

#### For the PD controller, launch with the following argument:

	roslaunch husky_waypoint_nav navigator.launch controller_select:=pd

##### If you want custom gains, use the following arguments:

	roslaunch husky_waypoint_nav navigator.launch controller_select:=pd Kp_angular:=1.8 Kd_angular:=0.08 Kp_linear:=0.6


# Debugging Tips

## Time issues due to offline errors
If you run into time sync errors, run the following when connected to internet:

	sudo timedatectl set-ntp true
AND
	sudo hwclock --systohc


