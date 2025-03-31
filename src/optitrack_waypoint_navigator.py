#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

struct Waypoint {
  double x, y, yaw;
};

std::vector<Waypoint> waypoints;
int current_wp_index = 0;
bool paused = false;
bool pose_received = false;
geometry_msgs::Pose current_pose;

int pause_button = 3;
int resume_button = 1;
bool loop_mode = false;

ros::Publisher cmd_vel_pub;

std::vector<Waypoint> loadWaypoints(const std::string& filepath) {
  std::vector<Waypoint> wps;
  std::ifstream file(filepath);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream ss(line);
    Waypoint wp;
    if (ss >> wp.x >> wp.y >> wp.yaw)
      wps.push_back(wp);
  }
  return wps;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (msg->buttons[pause_button] == 1) {
    paused = true;
    ROS_WARN("Navigation paused.");
  } else if (msg->buttons[resume_button] == 1) {
    paused = false;
    ROS_INFO("Navigation resumed.");
  }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_pose = msg->pose;
  pose_received = true;
}

double getYaw(const geometry_msgs::Quaternion& q) {
  tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "optitrack_waypoint_navigator");
  ros::NodeHandle nh;

  // Load params
  std::string path_param;
  nh.param<std::string>("/outdoor_waypoint_nav/coordinates_file", path_param, "/config/optitrack_waypoints.txt");
  std::string path_abs = ros::package::getPath("husky_waypoint_nav") + path_param;

  nh.param("/outdoor_waypoint_nav/pause_button_num", pause_button, 3);
  nh.param("/outdoor_waypoint_nav/resume_button_num", resume_button, 1);
  nh.param("/outdoor_waypoint_nav/loop_mode", loop_mode, false);

  waypoints = loadWaypoints(path_abs);
  if (waypoints.empty()) {
    ROS_ERROR("No waypoints loaded.");
    return 1;
  }

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber joy_sub = nh.subscribe("/joy", 10, joyCallback);
  ros::Subscriber pose_sub = nh.subscribe("/natnet_ros/Husky/pose", 10, poseCallback);

  ros::Rate rate(10);
  while (ros::ok() && current_wp_index < waypoints.size()) {
    ros::spinOnce();
    if (paused || !pose_received) {
      rate.sleep();
      continue;
    }

    const Waypoint& wp = waypoints[current_wp_index];
    double dx = wp.x - current_pose.position.x;
    double dy = wp.y - current_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Threshold to consider the waypoint reached
    if (dist < 0.2) {
      ROS_INFO("Reached waypoint %d", current_wp_index);
      current_wp_index++;
      if (current_wp_index >= waypoints.size()) {
        if (loop_mode) {
          current_wp_index = 0;
          ROS_INFO("Looping waypoints.");
        } else {
          ROS_INFO("All waypoints complete.");
          break;
        }
      }
      continue;
    }

    double yaw = getYaw(current_pose.orientation);
    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle - yaw;

    // Normalize angle error to [-pi, pi]
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.4 * std::cos(angle_error);
    cmd.angular.z = 1.0 * angle_error;
    cmd_vel_pub.publish(cmd);

    rate.sleep();
  }

  // Stop the robot
  geometry_msgs::Twist stop;
  cmd_vel_pub.publish(stop);

  return 0;
}
