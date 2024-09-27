// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef PATH_TO_TRAJECTORY__PATH_TO_TRAJECTORY_HPP_
#define PATH_TO_TRAJECTORY__PATH_TO_TRAJECTORY_HPP_

// #include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class PathToTrajectory : public rclcpp::Node {
 public:
  using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

 public:
  PathToTrajectory();

 private:
  // Publisher
  rclcpp::Publisher<Trajectory>::sharedPtr trajectory_pub_;

  // Subscription
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  nav_msgs::msg::Odometry odometry_;

  // Client

  // Server

  // Param
  std::string base_path_;
  int downsample_rate_;
  int margin_;

  // function
  void load_csv(void load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::PoseStamped>& point););

  // variable
  std::vector<geometry_msgs::msg::msg::Pose> base_points_;
  bool odom_flag_;



  // void callback(const PathWithLaneId::SharedPtr msg);
  // rclcpp::Subscription<PathWithLaneId>::SharedPtr sub_;
  // rclcpp::Publisher<Trajectory>::SharedPtr pub_;
};

#endif  // PATH_TO_TRAJECTORY__PATH_TO_TRAJECTORY_HPP_
