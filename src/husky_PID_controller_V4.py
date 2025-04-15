#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import math
import os, glob
import traceback

class HuskyOptiTrackNavigatorPID:
    def __init__(self):
        # --- Node and Pub/Sub ---
        rospy.init_node('husky_optitrack_navigator_v12', anonymous=True) # Version 12
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.update_pose)

        # --- Robot State ---
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0
        self.pose_received = False

        # --- Controller Tuning ---
        # Gains for Rotation (Phases 1, 3 & Timeout Realign)
        self.Kp_angular_rot = 0.8
        self.Ki_angular_rot = 0.0 # Keep Ki=0 for now
        self.Kd_angular_rot = 0.05
        # Gains for Movement Correction (Phase 2) - NOT USED
        # self.Kp_angular_move = 0.1 # Set to 0 or remove
        # self.Ki_angular_move = 0.0
        # self.Kd_angular_move = 0.0

        # Linear Gain
        self.Kp_linear = 0.5

        # Limits & Tolerances
        self.max_speed = 0.4
        self.max_turning_speed = 1.0 # Allow faster rotation in Phase 1/3
        self.dist_tolerance = 0.10 # Target distance tolerance
        self.yaw_final_tolerance_rad = math.radians(2.0) # For Phase 3
        # Tolerance for FINISHING Phase 1 alignment - Keep tight
        self.yaw_align_tolerance_rad = math.radians(5.0) # 5 degrees
        # Tolerance for finishing forced realignment
        self.realign_tolerance_rad = math.radians(5.0)

        # Timeout for Phase 2 (seconds) - TUNE THIS
        self.phase2_timeout_duration = rospy.Duration(20.0)
        # Timeout for the realignment itself
        self.realign_timeout_duration = rospy.Duration(10.0)
        # Progress check threshold for timeout (e.g., must reduce distance by 10% of initial)
        self.timeout_progress_threshold = 0.1 # 10% reduction required

        # Linear Acceleration Limit
        self.max_linear_acceleration = 0.15

        # PID State
        self.integral_yaw_error = 0.0
        self.prev_yaw_error = 0.0
        self.last_time = None

        # Command Storage
        self.prev_cmd = Twist()

        # Rate
        self.rate = rospy.Rate(60)

        # --- Load Waypoints & Start ---
        self.waypoints = self.load_waypoints()
        if not self.waypoints: rospy.signal_shutdown("No waypoints loaded."); return
        rospy.loginfo("Waiting for the first pose update...")
        while not self.pose_received and not rospy.is_shutdown(): self.rate.sleep()
        self.last_time = rospy.Time.now()
        rospy.loginfo("First pose received. Initializing time. Starting navigation.")
        self.move_through_waypoints()

    # update_pose, quaternion_to_euler, load_waypoints, normalize_angle remain the same
    def update_pose(self, data):
        self.current_pose_x = data.pose.position.x; self.current_pose_y = data.pose.position.y
        self.current_pose_yaw = self.quaternion_to_euler(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        self.pose_received = True
    def quaternion_to_euler(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y); t4 = +1.0 - 2.0 * (y * y + z * z); return math.atan2(t3, t4)
    def load_waypoints(self):
        wp_dir = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints"); files = sorted(glob.glob(os.path.join(wp_dir, "optitrack_waypoints_*.txt")))
        if not files: rospy.logerr(f"[ERROR] No waypoint files found in: {wp_dir}"); return []
        latest_file = files[-1]; waypoints = []
        try:
            with open(latest_file, 'r') as f:
                for i, line in enumerate(f):
                    if line.strip().startswith('#') or not line.strip(): continue
                    try:
                        parts = list(map(float, line.strip().split())); x, y = parts[0], parts[1]
                        yaw_rad = math.radians(parts[2]) if len(parts) >= 3 else None; waypoints.append((x, y, yaw_rad))
                    except (ValueError, IndexError): rospy.logwarn(f"Skipping malformed line {i+1}: {line.strip()}")
            rospy.loginfo(f"[INFO] Loaded {len(waypoints)} waypoints from {latest_file}"); return waypoints
        except Exception as e: rospy.logerr(f"Failed to load waypoints: {e}"); return []
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def reset_pid_state(self):
        """Resets the PID controller's integral/error states and prev command."""
        self.integral_yaw_error = 0.0
        self.prev_yaw_error = 0.0 # Resetting this is key for D-term handling on first cycle
        self.prev_cmd = Twist()
        # No log message here, called frequently

    def calculate_pid_angular_control(self, current_yaw, target_yaw, dt, is_first_cycle=False):
        """Calculates the PID control output for angular velocity (Phases 1, 3, Realign)."""
        # NOTE: This uses ROTATION gains. is_first_cycle disables D-term initially.
        if dt <= 1e-6:
             yaw_error = self.normalize_angle(target_yaw - current_yaw)
             rospy.logwarn_throttle(1.0, f"PID Rot Calc: dt too small ({dt:.5f}), returning 0 command.")
             return 0.0, yaw_error

        Kp = self.Kp_angular_rot
        Ki = self.Ki_angular_rot # Currently 0
        Kd = self.Kd_angular_rot

        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        p_term = Kp * yaw_error
        i_term = 0.0 # Ki is 0
        # if abs(Ki) > 1e-9: ... # Integral logic omitted for brevity as Ki=0

        d_term = 0.0
        if not is_first_cycle: # Disable D on first cycle
            derivative_error = (yaw_error - self.prev_yaw_error) / dt
            d_term = Kd * derivative_error

        angular_vel = p_term + i_term + d_term
        angular_vel_clamped = max(min(angular_vel, self.max_turning_speed), -self.max_turning_speed)
        self.prev_yaw_error = yaw_error

        rospy.loginfo_throttle(0.5, f"PID Rot: P={p_term:.2f}, D={d_term:.2f} "
                               f"| Err={math.degrees(yaw_error):.1f} | dt={dt:.4f} | First={is_first_cycle} | Cmd={angular_vel_clamped:.2f}")
        return angular_vel_clamped, yaw_error

    def apply_linear_acceleration_limit(self, target_linear_vel, dt):
        """Applies acceleration limit to the target linear velocity."""
        if dt <= 1e-6: return self.prev_cmd.linear.x
        max_dv = self.max_linear_acceleration * dt
        current_linear_cmd = self.prev_cmd.linear.x
        if target_linear_vel > current_linear_cmd: linear_vel_cmd = min(target_linear_vel, current_linear_cmd + max_dv)
        elif target_linear_vel < current_linear_cmd: linear_vel_cmd = max(target_linear_vel, current_linear_cmd - max_dv)
        else: linear_vel_cmd = target_linear_vel
        linear_vel_cmd = max(0.0, min(linear_vel_cmd, self.max_speed))
        return linear_vel_cmd

    def perform_realignment(self, target_yaw, waypoint_label):
        """ Rotates the robot in place towards a target yaw using rotation gains. """
        rospy.logwarn(f"[{waypoint_label}] Performing forced realignment towards {math.degrees(target_yaw):.1f}°...")
        self.reset_pid_state() # Reset PID for this task
        first_realign_cycle = True
        realign_start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if not self.pose_received or self.last_time is None: self.rate.sleep(); continue
            current_time = rospy.Time.now(); dt = (current_time - self.last_time).to_sec()

            # Timeout for realignment itself
            if current_time - realign_start_time > self.realign_timeout_duration:
                rospy.logerr(f"[{waypoint_label}] REALIGNMENT TIMED OUT!")
                break

            # Calculate PID using rotation gains, disable D on first cycle
            angular_vel_cmd, yaw_error = self.calculate_pid_angular_control(
                self.current_pose_yaw, target_yaw, dt, is_first_cycle=first_realign_cycle
            )
            first_realign_cycle = False

            if dt <= 1e-6:
                self.last_time = current_time; self.rate.sleep(); continue

            cmd = Twist(); cmd.linear.x = 0.0; cmd.angular.z = angular_vel_cmd
            self.velocity_publisher.publish(cmd)
            self.prev_cmd = cmd; self.last_time = current_time
            rospy.loginfo_throttle(1.0,f"[{waypoint_label} Realigning] YawErr: {math.degrees(yaw_error):.1f}°")

            if abs(yaw_error) < self.realign_tolerance_rad:
                rospy.loginfo(f"[{waypoint_label}] Forced realignment complete (YawErr: {math.degrees(yaw_error):.1f}°).")
                break
            self.rate.sleep()

        self.velocity_publisher.publish(Twist()) # Stop after realignment attempt
        rospy.sleep(0.2)

    def move_through_waypoints(self):
        """Main control loop using 'Align (PID), Move (No Angular Correction), Align (PID)' strategy."""
        for i, (goal_x, goal_y, goal_yaw_rad) in enumerate(self.waypoints):
            waypoint_label = f"WAYPOINT {i+1}/{len(self.waypoints)}"
            rospy.loginfo(f"[{waypoint_label}] Navigating to ({goal_x:.2f}, {goal_y:.2f}) "
                          f"{f'with target yaw {math.degrees(goal_yaw_rad):.1f}°' if goal_yaw_rad is not None else '(position only)'}")

            initial_distance_wp = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
            if initial_distance_wp < self.dist_tolerance:
                 rospy.loginfo(f"[{waypoint_label}] Already at goal position (Dist: {initial_distance_wp:.3f}). Skipping Phases 1 & 2.")
            else:
                # --- Phase 1: Align towards Goal Position (using Rotation PID Gains) ---
                rospy.loginfo(f"[{waypoint_label}] Phase 1: Aligning towards goal (Dist: {initial_distance_wp:.2f})...")
                self.reset_pid_state()
                first_phase1_cycle = True

                while not rospy.is_shutdown():
                    if not self.pose_received or self.last_time is None: self.rate.sleep(); continue
                    current_time = rospy.Time.now(); dt = (current_time - self.last_time).to_sec()

                    # Calculate PID using rotation gains, disable D on first cycle
                    target_heading = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x)
                    angular_vel_cmd, yaw_error = self.calculate_pid_angular_control(
                        self.current_pose_yaw, target_heading, dt, is_first_cycle=first_phase1_cycle
                    )
                    first_phase1_cycle = False

                    if dt <= 1e-6: self.last_time = current_time; self.rate.sleep(); continue

                    cmd = Twist(); cmd.linear.x = 0.0; cmd.angular.z = angular_vel_cmd
                    self.velocity_publisher.publish(cmd)
                    self.prev_cmd = cmd; self.last_time = current_time
                    rospy.loginfo_throttle(1.0,f"[{waypoint_label} Phase 1] Aligning YawErr: {math.degrees(yaw_error):.1f}°")

                    if abs(yaw_error) < self.yaw_align_tolerance_rad: # Use 5 degrees
                        rospy.loginfo(f"[{waypoint_label} Phase 1] Aligned within {math.degrees(self.yaw_align_tolerance_rad):.1f}° (YawErr: {math.degrees(yaw_error):.1f}°).")
                        self.velocity_publisher.publish(Twist()); rospy.sleep(0.2)
                        break # Exit Phase 1 loop
                    self.rate.sleep()

                # --- Phase 2: Move towards Goal Position (NO Angular Correction) ---
                rospy.loginfo(f"[{waypoint_label}] Phase 2: Moving towards goal (No angular correction)...")
                phase2_start_time = rospy.Time.now()
                # Initialize min_dist_phase2 correctly BEFORE the loop starts
                min_dist_phase2 = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                first_phase2_cycle = True # Flag for dt stabilization

                while not rospy.is_shutdown():
                    current_time = rospy.Time.now()
                    # --- Timeout Check (Based on Lack of Progress) ---
                    if current_time - phase2_start_time > self.phase2_timeout_duration:
                        # We need the current distance to check progress accurately
                        current_distance_for_timeout_check = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                        progress_ratio = (initial_distance_wp - min_dist_phase2) / initial_distance_wp if initial_distance_wp > 0 else 1.0
                        if progress_ratio < self.timeout_progress_threshold:
                            rospy.logerr(f"[{waypoint_label} Phase 2] TIMEOUT ({self.phase2_timeout_duration.to_sec()}s) & No Progress! "
                                         f"(MinDist: {min_dist_phase2:.2f} vs Initial: {initial_distance_wp:.2f}). Realigning.")
                            self.velocity_publisher.publish(Twist()); rospy.sleep(0.1)
                            current_yaw_to_goal = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x)
                            self.perform_realignment(current_yaw_to_goal, waypoint_label)
                            break # Exit Phase 2 loop after timeout and realignment attempt
                        else:
                            # Timeout occurred, but progress was made, reset timer and continue
                            rospy.logwarn(f"[{waypoint_label} Phase 2] Timeout threshold reached, but progress was made. Resetting timer.")
                            phase2_start_time = current_time # Reset timer
                            # Reset min distance tracking to current distance
                            min_dist_phase2 = current_distance_for_timeout_check # Use current distance


                    # --- Standard Loop Logic ---
                    if not self.pose_received or self.last_time is None: self.rate.sleep(); continue
                    dt = (current_time - self.last_time).to_sec()

                    # Skip first cycle dt calculation, but still need to update time
                    if first_phase2_cycle:
                        rospy.loginfo("[%s Phase 2] Skipping first cycle actions.", waypoint_label)
                        self.last_time = current_time; first_phase2_cycle = False; self.rate.sleep(); continue
                    if dt <= 1e-6: self.last_time = current_time; self.rate.sleep(); continue

                    distance = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                    min_dist_phase2 = min(min_dist_phase2, distance) # Update minimum distance seen

                    if distance < self.dist_tolerance:
                        rospy.loginfo(f"[{waypoint_label} Phase 2] Reached goal (Dist: {distance:.3f}).")
                        self.velocity_publisher.publish(Twist()); rospy.sleep(0.2)
                        break # Exit Phase 2 loop

                    # --- Linear Velocity Control ---
                    target_linear_vel = min(self.Kp_linear * distance, self.max_speed)
                    final_linear_cmd = self.apply_linear_acceleration_limit(target_linear_vel, dt)

                    # --- Angular Velocity Control (DISABLED) ---
                    angular_vel_cmd = 0.0
                    # Keep track of error just for logging
                    target_heading_phase2 = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x)
                    yaw_error_correction = self.normalize_angle(target_heading_phase2 - self.current_pose_yaw)

                    # --- Combine and Publish ---
                    cmd = Twist(); cmd.linear.x = final_linear_cmd; cmd.angular.z = angular_vel_cmd
                    self.velocity_publisher.publish(cmd)

                    # Logging
                    log_msg = (f"[{waypoint_label} Phase 2] Dist: {distance:.2f}, TargetHdg: {math.degrees(target_heading_phase2):.1f}°, "
                               f"YawErr: {math.degrees(yaw_error_correction):.1f}°, LinCmd: {cmd.linear.x:.2f}, AngCmd: {cmd.angular.z:.2f}")
                    rospy.loginfo_throttle(1.0, log_msg)

                    self.prev_cmd = cmd; self.last_time = current_time
                    self.rate.sleep()
                # --- End of Phase 2 ---

            # --- Phase 3: Final Yaw Alignment ---
            if goal_yaw_rad is not None:
                rospy.loginfo(f"[{waypoint_label}] Phase 3: Aligning to final yaw: {math.degrees(goal_yaw_rad):.1f}°")
                self.reset_pid_state() # Reset before final alignment
                first_phase3_cycle = True # Flag for dt stabilization
                while not rospy.is_shutdown():
                    if not self.pose_received or self.last_time is None: self.rate.sleep(); continue
                    current_time = rospy.Time.now(); dt = (current_time - self.last_time).to_sec()

                    # Calculate PID using rotation gains, disable D on first cycle
                    angular_vel_cmd, yaw_error_final = self.calculate_pid_angular_control(
                        self.current_pose_yaw, goal_yaw_rad, dt, is_first_cycle=first_phase3_cycle
                    )
                    first_phase3_cycle = False

                    if dt <= 1e-6:
                        self.last_time = current_time; self.rate.sleep(); continue

                    cmd = Twist(); cmd.linear.x = 0.0; cmd.angular.z = angular_vel_cmd
                    self.velocity_publisher.publish(cmd)
                    self.prev_cmd = cmd; self.last_time = current_time
                    rospy.loginfo_throttle(1.0, f"[{waypoint_label} Phase 3] Final Align YawErr: {math.degrees(yaw_error_final):.1f}°")

                    if abs(yaw_error_final) < self.yaw_final_tolerance_rad:
                        rospy.loginfo(f"[{waypoint_label} Phase 3] Final yaw alignment complete.")
                        self.velocity_publisher.publish(Twist()); rospy.sleep(0.2)
                        break
                    self.rate.sleep()
            else:
                 rospy.loginfo(f"[{waypoint_label}] Phase 3: No final yaw specified, skipping.")
            # --- End of Phase 3 ---

            rospy.loginfo(f"[{waypoint_label}] Completed.")

        rospy.loginfo("[DONE] Finished all waypoints.")
        self.velocity_publisher.publish(Twist())

if __name__ == '__main__':
    # ... (main execution block remains the same) ...
    navigator = None
    try:
        navigator = HuskyOptiTrackNavigatorPID() # Make sure class name matches
        rospy.spin()
    except rospy.ROSInterruptException: rospy.loginfo("ROS node interrupted.")
    except Exception as e: rospy.logerr(f"An unexpected error occurred: {traceback.format_exc()}")
    finally: # Ensure robot stops on exit
        if rospy.core.is_initialized() and not rospy.core.is_shutdown():
            publisher = None
            if navigator and hasattr(navigator, 'velocity_publisher'): publisher = navigator.velocity_publisher
            else: # Fallback publisher
                 try: pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1); rospy.sleep(0.2); publisher = pub
                 except Exception: rospy.logerr("Failed to create fallback publisher.")
            if publisher:
                 rospy.loginfo("Publishing zero velocity on exit."); publisher.publish(Twist()); rospy.sleep(0.1)
        rospy.loginfo("Node shutting down.")