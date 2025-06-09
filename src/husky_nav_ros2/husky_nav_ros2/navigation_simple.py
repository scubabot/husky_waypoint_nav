import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String as StringMsg
import math
import time # For dt in PD controller
import os
import sys
from .robot_math_utils import get_yaw_from_quaternion, normalize_angle_radians, distance_2d

WAYPOINTS_FILENAME = "waypoints.csv"

class WaypointNavigatorPD(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_pd_node')
        self.get_logger().info('PD Waypoint Navigator initializing...')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/a200_1046/cmd_vel', 10)
        self.status_publisher = self.create_publisher(StringMsg, '/navigation_status', 10)

        # Subscriber
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/base_link/pose', # IMPORTANT: Your robot's localized pose topic
            self.pose_callback,
            10
        )
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_yaw = 0.0
        self.pose_is_initialized = False

        # Waypoint data
        self.target_waypoints = []
        self.current_waypoint_index = -1

        # --- PD Controller & Navigation Parameters (CRITICAL TO TUNE!) ---
        self.goal_xy_tolerance = 0.08  # meters (how close to get to (x,y))
        self.goal_final_theta_tolerance = 0.08 # radians (for final alignment)
        self.approach_heading_tolerance = 0.2 # radians (looser tolerance while approaching (x,y))

        self.max_linear_speed = 0.15      # m/s
        self.max_angular_speed = 0.30     # rad/s
        
        # Proportional Gains
        self.linear_Kp = 0.5
        self.angular_Kp = 0.9
        
        # Derivative Gains
        self.linear_Kd = 0.15
        self.angular_Kd = 0.1

        # Variables for PD controller
        self.previous_distance_error = 0.0
        self.previous_heading_error_to_point = 0.0 # For approaching (x,y)
        self.previous_final_heading_error = 0.0    # For final theta alignment
        self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9 # Initialize with current time in seconds

        # Navigation States
        self.NAV_STATE_IDLE = "IDLE"
        self.NAV_STATE_LOADING_WAYPOINTS = "LOADING_WAYPOINTS" # Transient
        self.NAV_STATE_ALIGNING_TO_POINT = "ALIGNING_TO_POINT" # Aligning to drive towards (x,y)
        self.NAV_STATE_MOVING_TO_POINT = "MOVING_TO_POINT"     # Moving towards (x,y)
        self.NAV_STATE_REACHED_POINT_XY = "REACHED_POINT_XY"   # Transient: (x,y) reached
        self.NAV_STATE_ALIGNING_FINAL_ORIENTATION = "ALIGNING_FINAL_ORIENTATION" # Aligning to waypoint's theta
        self.NAV_STATE_WAYPOINT_COMPLETE = "WAYPOINT_COMPLETE" # Transient
        self.NAV_STATE_ALL_WAYPOINTS_DONE = "ALL_WAYPOINTS_DONE"
        self.current_nav_state = self.NAV_STATE_LOADING_WAYPOINTS # Start by loading

        self.load_waypoints_from_file() # Load waypoints at startup

        if self.target_waypoints:
            self.current_waypoint_index = 0
            self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT # Start with first waypoint
            self.log_and_publish_status(f"Loaded {len(self.target_waypoints)} waypoints. Targeting waypoint 0.")
        else:
            self.current_nav_state = self.NAV_STATE_IDLE
            self.log_and_publish_status("No waypoints loaded. Navigator is idle.")

        self.control_loop_timer = self.create_timer(0.05, self.navigation_control_loop) # 20 Hz loop
        self.get_logger().info('Node initialized. Control loop started.')

    def pose_callback(self, msg: PoseStamped):
        self.current_robot_x = msg.pose.position.x
        self.current_robot_y = msg.pose.position.y
        raw_yaw = get_yaw_from_quaternion(msg.pose.orientation)
        self.current_robot_yaw = normalize_angle_radians(raw_yaw)
        
        if not self.pose_is_initialized:
            self.pose_is_initialized = True
            self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9 # Re-init time for first PD calc
            self.get_logger().info(f"Initial robot pose: x={self.current_robot_x:.2f}, y={self.current_robot_y:.2f}, yaw={self.current_robot_yaw:.2f}")

    def load_waypoints_from_file(self):
        self.target_waypoints = []
        full_path = os.path.abspath(WAYPOINTS_FILENAME)
        if not os.path.exists(full_path):
            self.get_logger().warn(f"Waypoint file not found: {full_path}")
            return
        try:
            with open(WAYPOINTS_FILENAME, 'r') as f:
                for line_number, line in enumerate(f):
                    line = line.strip()
                    if not line or line.startswith("#"): continue
                    try:
                        parts = line.split(',')
                        x = float(parts[0])
                        y = float(parts[1])
                        theta = normalize_angle_radians(float(parts[2])) # Ensure loaded theta is normalized
                        self.target_waypoints.append({'x': x, 'y': y, 'theta': theta})
                    except (IndexError, ValueError) as e:
                        self.get_logger().warn(f"Skipping malformed line {line_number+1} in {WAYPOINTS_FILENAME}: '{line}'. Error: {e}")
            self.get_logger().info(f"Loaded {len(self.target_waypoints)} waypoints from {full_path}")
        except IOError as e:
            self.get_logger().error(f"Could not read waypoints from file {full_path}: {e}")

    def navigation_control_loop(self):
        if not self.pose_is_initialized:
            self.log_and_publish_status("Waiting for initial robot pose...")
            return

        # Calculate dt for PD controller
        current_time = self.get_clock().now().nanoseconds / 1e9 # seconds
        dt = current_time - self.last_time_error_calculated
        if dt <= 0: # Avoid division by zero or negative dt if time jumps
            dt = 1e-9 # A very small positive number
        self.last_time_error_calculated = current_time

        # --- Handle states where no active waypoint or all done ---
        if self.current_nav_state == self.NAV_STATE_IDLE:
            self.log_and_publish_status("Navigator idle. No waypoints loaded or error occurred.")
            self.stop_robot()
            return
        if self.current_nav_state == self.NAV_STATE_ALL_WAYPOINTS_DONE:
            # self.log_and_publish_status("All waypoints successfully navigated.") # Logged once
            self.stop_robot()
            return
        if self.current_waypoint_index < 0 or self.current_waypoint_index >= len(self.target_waypoints):
            self.log_and_publish_status("Error: Invalid waypoint index or no waypoints. Setting to IDLE.")
            self.current_nav_state = self.NAV_STATE_IDLE
            self.stop_robot()
            return

        # --- Active Navigation ---
        current_goal = self.target_waypoints[self.current_waypoint_index]
        goal_x = current_goal['x']
        goal_y = current_goal['y']
        goal_theta = current_goal['theta'] # Target final orientation

        distance_to_goal_xy = distance_2d(self.current_robot_x, self.current_robot_y, goal_x, goal_y)
        angle_to_goal_point = math.atan2(goal_y - self.current_robot_y, goal_x - self.current_robot_x)
        heading_error_to_point = normalize_angle_radians(angle_to_goal_point - self.current_robot_yaw) # For approaching (x,y)
        final_heading_error = normalize_angle_radians(goal_theta - self.current_robot_yaw) # For final theta alignment

        twist_cmd = Twist()

        # --- State Machine Logic ---
        if self.current_nav_state == self.NAV_STATE_ALIGNING_TO_POINT:
            self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Aligning to point (x,y). Err: {heading_error_to_point:.2f} rad")
            if abs(heading_error_to_point) > self.approach_heading_tolerance:
                # PD for angular velocity
                angular_error_derivative = (heading_error_to_point - self.previous_heading_error_to_point) / dt
                twist_cmd.angular.z = (self.angular_Kp * heading_error_to_point) + \
                                      (self.angular_Kd * angular_error_derivative)
                twist_cmd.angular.z = max(min(twist_cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.get_logger().info(f"Wpt {self.current_waypoint_index}: Alignment to point (x,y) complete.")
                self.current_nav_state = self.NAV_STATE_MOVING_TO_POINT
                self.previous_distance_error = distance_to_goal_xy # Reset for moving state
                twist_cmd.angular.z = 0.0 # Stop rotation
            self.previous_heading_error_to_point = heading_error_to_point

        elif self.current_nav_state == self.NAV_STATE_MOVING_TO_POINT:
            self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Moving to point (x,y). Dist: {distance_to_goal_xy:.2f}m")
            if distance_to_goal_xy > self.goal_xy_tolerance:
                # If heading error becomes too large while moving, re-align
                if abs(heading_error_to_point) > self.approach_heading_tolerance * 1.5: # Wider tolerance
                    self.get_logger().info(f"Wpt {self.current_waypoint_index}: Re-aligning to point (x,y) during move.")
                    self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
                    self.previous_heading_error_to_point = heading_error_to_point # Reset for align state
                    self.stop_robot() # Stop linear before turning
                    return # Exit and re-evaluate in next loop

                # PD for linear velocity
                linear_error_derivative = (distance_to_goal_xy - self.previous_distance_error) / dt
                # Note: derivative for distance is tricky, as error decreases.
                # A simpler approach for linear might be just P, or P with a velocity ramp-down.
                # For now, let's try P for linear, and PD for angular correction during move.
                twist_cmd.linear.x = self.linear_Kp * distance_to_goal_xy
                # twist_cmd.linear.x = (self.linear_Kp * distance_to_goal_xy) - \
                #                      (self.linear_Kd * linear_error_derivative) # Negative Kd if error is distance
                twist_cmd.linear.x = max(min(twist_cmd.linear.x, self.max_linear_speed), 0.0)

                # PD for angular velocity (path tracking)
                angular_error_derivative = (heading_error_to_point - self.previous_heading_error_to_point) / dt
                twist_cmd.angular.z = (self.angular_Kp * heading_error_to_point * 0.7) + \
                                      (self.angular_Kd * angular_error_derivative * 0.5) # Dampen during fwd
                twist_cmd.angular.z = max(min(twist_cmd.angular.z, self.max_angular_speed*0.7), -self.max_angular_speed*0.7)
            else:
                self.get_logger().info(f"Wpt {self.current_waypoint_index}: Point (x,y) reached.")
                self.current_nav_state = self.NAV_STATE_REACHED_POINT_XY # Transient state
                self.stop_robot() # Stop movement before final alignment
                self.previous_final_heading_error = final_heading_error # Init for next state
                return # Process next state in next loop
            self.previous_distance_error = distance_to_goal_xy
            self.previous_heading_error_to_point = heading_error_to_point

        elif self.current_nav_state == self.NAV_STATE_REACHED_POINT_XY: # Just a transition
            self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Reached (x,y). Preparing for final orientation.")
            self.current_nav_state = self.NAV_STATE_ALIGNING_FINAL_ORIENTATION
            self.previous_final_heading_error = final_heading_error # Ensure it's set for the first D calc
            # No command here, next loop will handle ALIGNING_FINAL_ORIENTATION

        elif self.current_nav_state == self.NAV_STATE_ALIGNING_FINAL_ORIENTATION:
            self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Aligning final orientation. Err: {final_heading_error:.2f} rad")
            if abs(final_heading_error) > self.goal_final_theta_tolerance:
                twist_cmd.linear.x = 0.0 # No linear movement
                # PD for angular velocity
                angular_error_derivative = (final_heading_error - self.previous_final_heading_error) / dt
                twist_cmd.angular.z = (self.angular_Kp * final_heading_error) + \
                                      (self.angular_Kd * angular_error_derivative)
                twist_cmd.angular.z = max(min(twist_cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.get_logger().info(f"Wpt {self.current_waypoint_index}: Final orientation complete.")
                self.current_nav_state = self.NAV_STATE_WAYPOINT_COMPLETE # Transient
                self.stop_robot()
                return # Process next state in next loop
            self.previous_final_heading_error = final_heading_error
        
        elif self.current_nav_state == self.NAV_STATE_WAYPOINT_COMPLETE: # Just a transition
            self.log_and_publish_status(f"Waypoint {self.current_waypoint_index} fully completed.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.target_waypoints):
                self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT # Move to next waypoint
                # Reset previous errors for the new waypoint approach
                self.previous_heading_error_to_point = 0.0 
                self.previous_distance_error = 0.0 
                self.log_and_publish_status(f"Targeting next waypoint {self.current_waypoint_index}.")
            else:
                self.log_and_publish_status("All waypoints successfully navigated!")
                self.current_nav_state = self.NAV_STATE_ALL_WAYPOINTS_DONE
            # No command here, next loop will handle new state

        self.cmd_vel_publisher.publish(twist_cmd)

    def stop_robot(self):
        stop_twist = Twist()
        self.cmd_vel_publisher.publish(stop_twist)

    def log_and_publish_status(self, status_text: str):
        self.get_logger().info(status_text) # Log to console
        status_msg = StringMsg()
        status_msg.data = f"WptIdx: {self.current_waypoint_index}, State: {self.current_nav_state}, Msg: {status_text}"
        self.status_publisher.publish(status_msg) # Publish to topic

    def on_shutdown(self):
        self.get_logger().info("PD Waypoint Navigator shutting down...")
        self.stop_robot()
        self.get_logger().info("Robot stopped. Shutdown complete.")

def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    navigator_node = WaypointNavigatorPD()
    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        navigator_node.get_logger().info('Keyboard interrupt, shutting down navigator.')
    finally:
        navigator_node.on_shutdown()
        if rclpy.ok(): navigator_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
