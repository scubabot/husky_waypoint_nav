#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry # Not used for control
import math
# import numpy as np # Not used
# import copy # Not used
import os, glob
import traceback # For error handling
import datetime # Needed for timestamped log files

class HuskyOptiTrackNavigatorPD:
    def __init__(self):
        rospy.init_node('husky_optitrack_navigator_pd', anonymous=True) # Simplified node name

        # --- Get Controller Type Parameter ---
        # Reads the '~controller_type' private parameter set in the launch file.
        # Defaults to 'PD_Controller' if not specified.
        self.controller_type = rospy.get_param("~controller_type", "PD_Controller")
        rospy.loginfo(f"Initializing navigator with controller type: {self.controller_type}")
        # --- End Parameter ---

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.update_pose)

        # Robot state
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0 # Radians
        self.pose_received = False

        # Controller tuning - Using Parameters
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_turning_speed = rospy.get_param("~max_turning_speed", 1.0)
        self.Kp_angular = rospy.get_param("~Kp_angular", 1.5) # Proportional gain for angular control
        self.Kd_angular = rospy.get_param("~Kd_angular", 0.1) # Derivative gain for angular control
        self.Kp_linear = rospy.get_param("~Kp_linear", 0.5)   # Proportional gain for linear control
        self.dist_tolerance = rospy.get_param("~distance_tolerance", 0.10) # Distance tolerance (meters)
        # --- Angle Tolerances (Parameterized) ---
        # Tolerance for switching from turn to move (degrees)
        self.align_tolerance_deg = rospy.get_param("~align_tolerance_deg", 5.0)
        self.align_tolerance_rad = math.radians(self.align_tolerance_deg) # Convert to RADIANS
        # Tolerance for final alignment (degrees)
        self.final_align_tolerance_deg = rospy.get_param("~final_align_tolerance_deg", 3.0)
        self.final_align_tolerance_rad = math.radians(self.final_align_tolerance_deg) # Convert to RADIANS
        # --- End Angle Tolerances ---
        self.control_rate = rospy.get_param("~control_rate", 60) # Control loop frequency (Hz)
        self.rate = rospy.Rate(self.control_rate)


        # PD Controller State Variables
        self.prev_yaw_error = 0.0 # Radians
        self.last_time = None # Initialize after first pose

        # Command smoothing
        self.enable_smoothing = rospy.get_param("~enable_smoothing", False) # Make smoothing configurable
        self.prev_cmd = Twist()

        # --- Logging Setup ---
        self.log_file = None
        # Get log directory from parameter, default to standard location
        self.log_dir = os.path.expanduser(rospy.get_param("~log_dir", "~/catkin_ws/src/husky_waypoint_nav/logs"))
        self.setup_logging() # Call setup_logging *after* controller_type is defined
        # Register the cleanup function to be called on node shutdown
        rospy.on_shutdown(self.cleanup)
        # --- End Logging Setup ---

        # Load waypoints
        self.waypoints = self.load_waypoints() # Stores yaw in radians
        if not self.waypoints:
             rospy.logerr("No waypoints loaded. Shutting down.")
             rospy.signal_shutdown("No waypoints loaded.")
             return

        rospy.loginfo("Waiting for the first pose update...")
        while not self.pose_received and not rospy.is_shutdown():
            self.rate.sleep()
        # Initialize time AFTER first pose is received to avoid large initial dt
        if self.last_time is None: # Ensure it's only set once
             self.last_time = rospy.Time.now()
        rospy.loginfo("First pose received. Starting navigation.")

        self.move_through_waypoints()

    # --- ADDED Logging Methods ---
    def setup_logging(self):
        """Creates the log directory and opens the log file for writing."""
        try:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
                rospy.loginfo(f"Created log directory: {self.log_dir}")

            # Sanitize controller_type string
            safe_controller_type = "".join(c if c.isalnum() else '_' for c in self.controller_type)
            safe_controller_type = '_'.join(filter(None, safe_controller_type.split('_')))
            if not safe_controller_type: safe_controller_type = "unknown_controller"

            # Readable timestamp
            timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            log_filename = f"{safe_controller_type}_{timestamp_str}.csv"
            log_filepath = os.path.join(self.log_dir, log_filename)

            self.log_file = open(log_filepath, 'w')
            # ADDING P/D terms to header
            header = "timestamp,current_pose_x,current_pose_y,current_pose_yaw," \
                     "target_wp_index,goal_x,goal_y,goal_yaw," \
                     "cmd_linear_x,cmd_angular_z," \
                     "distance_error,heading_error_rad," \
                     "p_term_angular,d_term_angular\n" # Added PD terms
            self.log_file.write(header)
            rospy.loginfo(f"Logging data to: {log_filepath}")

        except IOError as e:
            rospy.logerr(f"Failed to setup logging: {e}")
            self.log_file = None
            rospy.signal_shutdown("Logging setup failed")
        except OSError as e:
             rospy.logerr(f"Failed to create log directory {self.log_dir}: {e}")
             self.log_file = None
             rospy.signal_shutdown("Logging setup failed")

    def log_data(self, wp_index, goal_x, goal_y, goal_yaw, cmd_linear_x, cmd_angular_z, distance_error, heading_error_rad, p_term=0.0, d_term=0.0):
        """Writes a line of data to the log file, including PD terms."""
        # Ensure goal_yaw is a float (might be None if not specified in waypoint)
        goal_yaw_log = goal_yaw if goal_yaw is not None else float('nan')

        if self.log_file:
            try:
                timestamp = rospy.Time.now().to_sec()
                # ADDING P/D terms to log line
                log_line = f"{timestamp:.4f},{self.current_pose_x:.4f},{self.current_pose_y:.4f},{self.current_pose_yaw:.4f}," \
                           f"{wp_index},{goal_x:.4f},{goal_y:.4f},{goal_yaw_log:.4f}," \
                           f"{cmd_linear_x:.4f},{cmd_angular_z:.4f}," \
                           f"{distance_error:.4f},{heading_error_rad:.4f}," \
                           f"{p_term:.4f},{d_term:.4f}\n" # Added PD terms
                self.log_file.write(log_line)
            except IOError as e:
                rospy.logwarn_throttle(10, f"Failed to write to log file: {e}")

    def cleanup(self):
        """Closes the log file."""
        rospy.loginfo("Cleaning up and closing log file...")
        if self.log_file:
            try:
                self.log_file.flush()
                self.log_file.close()
                rospy.loginfo("Log file closed.")
                self.log_file = None
            except IOError as e:
                rospy.logerr(f"Error closing log file: {e}")
    # --- END Logging Methods ---

    def update_pose(self, data):
        """Callback function for updating the robot's pose."""
        self.current_pose_x = data.pose.position.x
        self.current_pose_y = data.pose.position.y
        # Yaw is stored in RADIANS
        self.current_pose_yaw = self.quaternion_to_euler(
            data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        )
        if not self.pose_received:
            # Initialize time only once when the first pose is received
            if self.last_time is None:
                 self.last_time = rospy.Time.now()
            self.pose_received = True

    def quaternion_to_euler(self, x, y, z, w):
        """Converts quaternion to Euler yaw angle (radians)."""
        t3 = +2.0 * (w * z + x * y); t4 = +1.0 - 2.0 * (y * y + z * z); return math.atan2(t3, t4)

    def load_waypoints(self):
        """Loads waypoints (X Y Yaw_radians) and stores yaw in RADIANS."""
        # Use parameters for directory and prefix
        wp_dir_param = rospy.get_param("~waypoint_dir", "~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        wp_prefix = rospy.get_param("~waypoint_prefix", "optitrack_waypoints_")
        wp_dir = os.path.expanduser(wp_dir_param)
        waypoints = []
        try:
            files = sorted(glob.glob(os.path.join(wp_dir, f"{wp_prefix}*.txt")))
            if not files: rospy.logerr(f"[ERROR] No waypoint files matching '{wp_prefix}*.txt' found in: {wp_dir}"); return []
            latest_file = files[-1]
            with open(latest_file, 'r') as f:
                for i, line in enumerate(f):
                    if line.strip().startswith('#') or not line.strip(): continue
                    try:
                        parts = list(map(float, line.strip().split()))
                        if len(parts) >= 3:
                            x, y, yaw_rad = parts # Assume file has radians
                            waypoints.append((x, y, yaw_rad))
                        elif len(parts) == 2:
                             x, y = parts; waypoints.append((x, y, None)) # Store None if no yaw
                             rospy.logwarn(f"Waypoint {i+1} has no yaw specified.")
                        else: rospy.logwarn(f"Skipping malformed line {i+1}: {line.strip()}")
                    except (ValueError, IndexError): rospy.logwarn(f"Skipping invalid line {i+1}: {line.strip()}")
            rospy.loginfo(f"[INFO] Loaded {len(waypoints)} waypoints from {latest_file}")
            return waypoints
        except Exception as e: rospy.logerr(f"Failed to load waypoints: {e}"); return []

    # --- Attribution Comment ---
    # The normalize_angle function below is adapted from
    # code commonly found in robotics navigation examples, including
    # repositories like https://github.com/sciyen/NCKUES-SelfDriving.
    # Please refer to the original source(s) for specific license terms if applicable.
    # --- End Attribution Comment ---
    def normalize_angle(self, angle):
        """Normalize an angle (in radians) to the range [-pi, pi]."""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def reset_pid_state(self):
        """Resets the PID controller's error states and prev command."""
        self.prev_yaw_error = 0.0 # Resetting this is key for D-term handling on first cycle
        self.prev_cmd = Twist()
        # No log message needed here

    def calculate_pd_angular_control(self, current_yaw_rad, target_yaw_rad, dt, is_first_cycle=False):
        """Calculates the PD control output (radians/sec) using radian inputs.
           Returns: angular_vel_clamped (rad/s), yaw_error_rad (rad), p_term, d_term
        """
        if dt <= 1e-6:
             yaw_error_rad = self.normalize_angle(target_yaw_rad - current_yaw_rad)
             rospy.logwarn_throttle(1.0, f"PD Calc: dt too small ({dt:.5f}), returning 0 command.")
             # Return zero command but still calculate P term for logging consistency
             p_term = self.Kp_angular * yaw_error_rad
             return 0.0, yaw_error_rad, p_term, 0.0 # Return 0 command and 0 D-term

        Kp = self.Kp_angular
        Kd = self.Kd_angular

        yaw_error_rad = self.normalize_angle(target_yaw_rad - current_yaw_rad)
        p_term = Kp * yaw_error_rad
        d_term = 0.0
        if not is_first_cycle: # Disable D on first cycle after reset
            # Normalize angle difference for derivative? Usually not needed if dt is small,
            # but can prevent large spikes if yaw wraps around.
            # error_diff = self.normalize_angle(yaw_error_rad - self.prev_yaw_error)
            # derivative_error = error_diff / dt
            derivative_error = (yaw_error_rad - self.prev_yaw_error) / dt # Simpler version
            d_term = Kd * derivative_error

        angular_vel = p_term + d_term
        angular_vel_clamped = max(min(angular_vel, self.max_turning_speed), -self.max_turning_speed)
        self.prev_yaw_error = yaw_error_rad # Store current error in radians

        # Log error in degrees for readability, but calculation uses radians
        # rospy.loginfo_throttle(1.0, f"PD Calc: P={p_term:.2f}, D={d_term:.2f} "
        #                        f"| Err={math.degrees(yaw_error_rad):.1f} | dt={dt:.4f} | First={is_first_cycle} | Cmd={angular_vel_clamped:.2f}")

        # --- MODIFIED RETURN ---
        return angular_vel_clamped, yaw_error_rad, p_term, d_term # Return command, error, and terms

    def move_through_waypoints(self):
        """Main control loop using Turn-Then-Move logic with PD turning (Radian Tolerance)."""
        for i, (goal_x, goal_y, goal_yaw_rad) in enumerate(self.waypoints):
            wp_index = i + 1 # Use 1-based index for logging
            waypoint_label = f"WAYPOINT {wp_index}/{len(self.waypoints)}"
            rospy.loginfo(f"[{waypoint_label}] Navigating to ({goal_x:.2f}, {goal_y:.2f}) "
                          f"{f'with target yaw {math.degrees(goal_yaw_rad):.1f}°' if goal_yaw_rad is not None else '(position only)'}")

            self.reset_pid_state() # Reset prev_error for PD
            first_cycle_waypoint = True # Flag to handle first D-term calculation

            # --- Movement Loop (Turn-Then-Move) ---
            while not rospy.is_shutdown():
                if not self.pose_received or self.last_time is None: self.rate.sleep(); continue
                current_time = rospy.Time.now(); dt = (current_time - self.last_time).to_sec()

                # Calculate state
                distance = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                if distance < self.dist_tolerance:
                    rospy.loginfo(f"[{waypoint_label}] Reached position (Dist: {distance:.3f}).")
                    break # Exit loop, proceed to final alignment

                # Calculate error in RADIANS for PD controller
                yaw_to_goal_rad = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x)
                # Calculate PD command based on current error, disable D on first cycle
                # --- Capture P/D terms ---
                angular_vel_pd, yaw_error_rad, p_term_move, d_term_move = self.calculate_pd_angular_control(
                    self.current_pose_yaw, yaw_to_goal_rad, dt, first_cycle_waypoint
                )
                first_cycle_waypoint = False # D-term is valid after first calculation

                if dt <= 1e-6: # Check dt validity *after* first cycle flag is handled
                    rospy.logwarn_throttle(1.0, f"[Move Loop] dt ({dt:.5f}) too small, skipping cycle.")
                    self.last_time = current_time; self.rate.sleep(); continue

                # --- Apply Turn-Then-Move Logic ---
                cmd = Twist()
                # Check tolerance using RADIANS
                if abs(yaw_error_rad) > self.align_tolerance_rad:
                    # Turn in place using PD controller
                    # rospy.loginfo_throttle(2.0, f"[{waypoint_label}] Turning: YawErr = {math.degrees(yaw_error_rad):.1f} deg")
                    cmd.linear.x = 0.0
                    cmd.angular.z = angular_vel_pd # Use calculated PD command
                else:
                    # Move forward (aligned)
                    # rospy.loginfo_throttle(2.0, f"[{waypoint_label}] Moving Forward: YawErr = {math.degrees(yaw_error_rad):.1f} deg, Dist = {distance:.2f} m")
                    cmd.angular.z = angular_vel_pd # Apply small corrections while moving
                    linear_vel = self.Kp_linear * distance
                    cmd.linear.x = min(linear_vel, self.max_speed)

                # --- Smoothing (Applied AFTER Turn/Move decision) ---
                if self.enable_smoothing:
                    smoothed_linear_x = 0.8 * self.prev_cmd.linear.x + 0.2 * cmd.linear.x
                    smoothed_angular_z = 0.8 * self.prev_cmd.angular.z + 0.2 * cmd.angular.z
                    final_cmd = Twist()
                    final_cmd.linear.x = smoothed_linear_x
                    final_cmd.angular.z = smoothed_angular_z
                else:
                    final_cmd = cmd

                # --- Log Data ---
                self.log_data(
                    wp_index=wp_index,
                    goal_x=goal_x,
                    goal_y=goal_y,
                    goal_yaw=goal_yaw_rad, # Log final target yaw (RADIANS or None)
                    cmd_linear_x=final_cmd.linear.x,
                    cmd_angular_z=final_cmd.angular.z,
                    distance_error=distance,
                    heading_error_rad=yaw_error_rad, # Log error towards position (RADIANS)
                    p_term=p_term_move, # Log P term from move phase calc
                    d_term=d_term_move  # Log D term from move phase calc
                )
                # --- End Log Data ---

                # Publish and Store
                self.velocity_publisher.publish(final_cmd)
                self.prev_cmd = final_cmd
                self.last_time = current_time
                self.rate.sleep()
            # --- End of Movement Loop ---

            # Stop robot before final alignment phase
            self.velocity_publisher.publish(Twist())
            self.prev_cmd = Twist()
            rospy.sleep(0.2)

            # --- Final Yaw Alignment Loop ---
            if goal_yaw_rad is not None:
                rospy.loginfo(f"[{waypoint_label}] Phase 3: Aligning to final yaw: {math.degrees(goal_yaw_rad):.1f}°")
                self.reset_pid_state() # Reset prev_error for PD alignment
                first_align_cycle = True

                while not rospy.is_shutdown():
                    if not self.pose_received or self.last_time is None: self.rate.sleep(); continue
                    current_time = rospy.Time.now(); dt = (current_time - self.last_time).to_sec()

                    # Calculate error in RADIANS for PD controller
                    yaw_error_final_rad = self.normalize_angle(goal_yaw_rad - self.current_pose_yaw)

                    # Check tolerance using RADIANS
                    if abs(yaw_error_final_rad) <= self.final_align_tolerance_rad:
                        rospy.loginfo(f"[{waypoint_label}] Final yaw alignment complete.")
                        break # Exit alignment loop

                    # Calculate PD command, disable D on first cycle
                    # --- Capture P/D terms ---
                    angular_vel_pd, _, p_term_align, d_term_align = self.calculate_pd_angular_control(
                        self.current_pose_yaw, goal_yaw_rad, dt, first_align_cycle
                    )
                    first_align_cycle = False

                    if dt <= 1e-6: self.last_time = current_time; self.rate.sleep(); continue

                    cmd = Twist(); cmd.linear.x = 0.0; cmd.angular.z = angular_vel_pd

                    # Apply Smoothing
                    if self.enable_smoothing:
                        smoothed_angular_z = 0.8 * self.prev_cmd.angular.z + 0.2 * cmd.angular.z
                        final_cmd = Twist()
                        final_cmd.angular.z = smoothed_angular_z
                    else:
                        final_cmd = cmd

                    # --- Log Data (Alignment Phase) ---
                    self.log_data(
                        wp_index=wp_index,
                        goal_x=goal_x, # Current goal position
                        goal_y=goal_y,
                        goal_yaw=goal_yaw_rad, # Final target yaw (RADIANS)
                        cmd_linear_x=final_cmd.linear.x, # Will be 0
                        cmd_angular_z=final_cmd.angular.z,
                        distance_error=math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y), # Log current distance
                        heading_error_rad=yaw_error_final_rad, # Log error relative to final yaw (RADIANS)
                        p_term=p_term_align, # Log P term from align phase calc
                        d_term=d_term_align  # Log D term from align phase calc
                    )
                    # --- End Log Data ---

                    # Publish and Store
                    self.velocity_publisher.publish(final_cmd)
                    self.prev_cmd = final_cmd
                    self.last_time = current_time

                    # Log error in degrees for readability
                    # rospy.loginfo_throttle(1.0, f"[{waypoint_label} Final Align] YawErr: {math.degrees(yaw_error_final_rad):.1f}°, CmdAng: {final_cmd.angular.z:.2f}")
                    self.rate.sleep()

                self.velocity_publisher.publish(Twist()) # Stop robot
                self.prev_cmd = Twist()
                rospy.sleep(0.2)
            else:
                 rospy.loginfo(f"[{waypoint_label}] Phase 3: No final yaw specified, skipping.")
            # --- End of Final Yaw Alignment ---

            rospy.loginfo(f"[{waypoint_label}] Completed.")

        rospy.loginfo("[DONE] Finished all waypoints.")
        self.velocity_publisher.publish(Twist())
        # No rospy.spin() needed if __main__ doesn't call it

if __name__ == '__main__':
    navigator = None
    try:
        navigator = HuskyOptiTrackNavigatorPD() # Use the correct class name
        # If the node should persist after finishing, add rospy.spin() here
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {traceback.format_exc()}")
    finally:
        # Ensure robot stops on exit
        if rospy.core.is_initialized() and not rospy.core.is_shutdown():
            publisher = None
            if navigator and hasattr(navigator, 'velocity_publisher'): publisher = navigator.velocity_publisher
            else: # Fallback publisher
                 try: pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1); rospy.sleep(0.2); publisher = pub
                 except Exception: rospy.logerr("Failed to create fallback publisher.")
            if publisher:
                 rospy.loginfo("Publishing zero velocity on exit."); publisher.publish(Twist()); rospy.sleep(0.1)
        rospy.loginfo("Node shutting down.")

