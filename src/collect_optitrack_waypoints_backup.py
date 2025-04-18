#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped # Import PoseStamped
from sensor_msgs.msg import Joy
# import tf # No longer using TF
# import tf.transformations # No longer using TF
import math # Keep math for quaternion_to_euler
import os
import glob
import datetime # Keep for timestamped log files

class WaypointCollector:
    def __init__(self):
        rospy.init_node('collect_optitrack_waypoints', anonymous=True)

        # Set and create waypoint directory
        waypoint_dir = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        os.makedirs(waypoint_dir, exist_ok=True)

        # Auto-increment filename like optitrack_waypoints_01.txt
        existing = glob.glob(os.path.join(waypoint_dir, "optitrack_waypoints_*.txt"))
        indices = []
        for f in existing:
            try:
                num = int(os.path.basename(f).split('_')[-1].split('.')[0])
                indices.append(num)
            except ValueError:
                continue
        next_index = max(indices + [0]) + 1
        self.file_path = os.path.join(waypoint_dir, f"optitrack_waypoints_{next_index:02d}.txt")

        # --- Parameters ---
        # Note: Parameter names might need adjustment depending on your launch file structure
        # Using defaults here. Consider using ~param_name for private parameters.
        self.collect_button = rospy.get_param("~collect_button_num", 2)  # Default: Square
        self.end_button = rospy.get_param("~end_button_num", 0)          # Default: Cross
        self.collect_button_sym = rospy.get_param("~collect_button_sym", "square")
        self.end_button_sym = rospy.get_param("~end_button_sym", "cross")
        self.pose_topic = rospy.get_param("~pose_topic", "/natnet_ros/base_link/pose") # Parameter for pose topic

        rospy.loginfo(f"Using pose topic: {self.pose_topic}")
        rospy.loginfo(f"Collect button index: {self.collect_button} ({self.collect_button_sym})")
        rospy.loginfo(f"End button index: {self.end_button} ({self.end_button_sym})")

        # --- State Variables ---
        self.num_waypoints = 0
        # REMOVED: self.tf_listener = tf.TransformListener()
        self.latest_pose = None # Variable to store the latest pose message
        self.last_collect_state = False
        self.last_end_state = False

        # --- File Handling ---
        try:
            self.file = open(self.file_path, "w")
            # Write header (optional, but good practice)
            self.file.write("# Waypoints collected using PoseStamped topic\n")
            self.file.write("# Format: x y yaw_radians\n")
            rospy.loginfo("[INFO] Saving OptiTrack waypoints to: %s", self.file_path)
        except IOError:
            rospy.logerr("[ERROR] Unable to open waypoint file: %s", self.file_path)
            rospy.signal_shutdown("File error")
            return

        # --- Subscribers ---
        rospy.Subscriber("/joy_teleop/joy", Joy, self.joy_callback) # Assuming this topic is correct now
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback) # Subscribe to the pose topic

        rospy.loginfo("[INFO] Waiting for the first pose message on %s...", self.pose_topic)
        while self.latest_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1) # Wait until the callback receives the first message

        if not rospy.is_shutdown():
            rospy.loginfo("[INFO] First pose received. Ready to collect waypoints: [%s] to collect, [%s] to finish.",
                          self.collect_button_sym, self.end_button_sym)
        else:
             rospy.logwarn("Shutdown requested before first pose was received.")


    def pose_callback(self, data):
        """Callback function to store the latest pose."""
        self.latest_pose = data

    def joy_callback(self, data):
        """Callback function for joystick button presses."""
        # --- Check if pose data is available ---
        if self.latest_pose is None:
             # Log only occasionally if buttons are pressed before pose is ready
             rospy.logwarn_throttle(2.0, "No pose received yet, cannot save waypoint.")
             return # Don't process buttons if we don't have a pose

        collect_now = data.buttons[self.collect_button]
        end_now = data.buttons[self.end_button]

        # Rising edge detection for collect button
        if collect_now and not self.last_collect_state:
            rospy.loginfo("[LOG] [%s] Collect button pressed", self.collect_button_sym)
            self.save_current_pose() # Call save function

        # Rising edge detection for end button
        if end_now and not self.last_end_state:
            rospy.loginfo("[LOG] [%s] End button pressed", self.end_button_sym)
            rospy.loginfo("[LOG] Saved %d waypoint(s).", self.num_waypoints)
            self.file.close()
            rospy.loginfo("Waypoint file closed.")
            rospy.signal_shutdown("Waypoint collection complete")

        # Update last button states
        self.last_collect_state = collect_now
        self.last_end_state = end_now

    def save_current_pose(self):
        """Saves the current pose (from the latest message) to the file."""
        if self.latest_pose is None:
            rospy.logwarn("[ERROR] Attempted to save pose, but no pose data available!")
            return

        # Extract data from the stored PoseStamped message
        pose_msg = self.latest_pose.pose
        x = pose_msg.position.x
        y = pose_msg.position.y
        orientation_q = pose_msg.orientation

        # Calculate yaw from the quaternion
        try:
            # Using the existing helper function
            _, _, yaw = self.quaternion_to_euler(
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            )
        except Exception as e:
             rospy.logerr(f"Error converting quaternion to Euler angles: {e}")
             return # Don't save if conversion fails

        # Write to file
        try:
            self.file.write(f"{x:.3f} {y:.3f} {yaw:.3f}\n")
            self.file.flush() # Ensure data is written immediately
            self.num_waypoints += 1
            rospy.loginfo("[INFO] Waypoint saved: [x: %.2f, y: %.2f, yaw: %.2f rad (%.1f deg)]",
                          x, y, yaw, math.degrees(yaw))
        except IOError as e:
             rospy.logerr(f"Error writing waypoint to file: {e}")


    def quaternion_to_euler(self, x, y, z, w):
        """Convert a quaternion into euler angles (roll, pitch, yaw)."""
        # (This function remains the same as before)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

# Main execution block
if __name__ == '__main__':
    try:
        collector = WaypointCollector()
        rospy.spin() # Keep node alive until shutdown signal
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint collector node interrupted.")
    except Exception as e:
         rospy.logerr(f"An unexpected error occurred in the collector: {e}")
         import traceback
         traceback.print_exc()
    finally:
         # Ensure file is closed if node exits unexpectedly (though on_shutdown is preferred)
         # Accessing self.file here might fail if __init__ didn't complete
         # Relying on normal exit or Ctrl+C calling the shutdown hook implicitly
         rospy.loginfo("Waypoint collector shutting down.")
