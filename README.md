# Notes on Changes
Latest waypoints and logs are ready for FCRAR paper. Need to post-process. 

# How to launch

### First, launch natnet node for optitrack:
Make sure to set world_frame as "map"

	roslaunch natnet_ros_cpp gui_natnet_ros.launch

## Important:

Make sure to launch rviz and make sure that when you launch move the robot, the tf of map and base_link are moving how you expect. For me: 
- Red is x-axis, and moves the robot forward (front face is the no button face). 
- Green is y-axis, points to the left of x-axis if x is forward. 
- Blue is z-axis, pointing up. 

If any of these look wrong, go back into Motive Software and configure Rigid Body to align properly with axis. 

### To launch the collection: 

	rosrun husky_waypoint_nav collect_optitrack_waypoints.py


### To launch navigator there are two options:

#### For the P controller, launch the default:

	roslaunch husky_waypoint_nav p_navigator.launch

#### For the PD controller, launch with the following argument:

	roslaunch husky_waypoint_nav pd_navigator.launch

##### If you want custom gains, use the following arguments:

	roslaunch husky_waypoint_nav navigator.launch controller Kp_angular:=1.8 Kd_angular:=0.08 Kp_linear:=0.6


# Debugging Tips

## Time issues due to offline errors
If you run into time sync errors, run the following when connected to internet:

	sudo timedatectl set-ntp true
AND
	sudo hwclock --systohc


