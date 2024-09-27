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

#include "path_to_trajectory/path_to_trajectory.hpp"
#include <fstream>
#include <sstream>
#include <string>

PathToTrajectory::PathToTrajectory() : Node("path_to_trajectory_node"), odom_flag_(false)
{
  RCLCPP_INFO(this->get_logger(), "================ Path To Trajectory ==================");

  this->declare_parameter("base_path", std::string("base_path"));
  base_path_ = this->get_parameter("base_path").as_string();

  this->declare_parameter("downsample_rate", 0);
  downsample_rate_ = this->get_parameter("downsample_rate").as_int();

  this->declare_parameter("margin", 10);
  margin_ = this->get_parameter("margin").as_int();

  RCLCPP_INFO(this->get_logger(), "base_path: %s", base_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "downsample_rate: %d", downsample_rate_);
  RCLCPP_INFO(this->get_logger(), "margin: %d", margin_);

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 1,
    std::bind(&PathToTrajectory::odometry_callback, this, std::placeholders::_1));

  trajectory_pub_ = this->create_publisher<Trajectory>("/planning/scenario_planning/trajectory", 1);

  load_csv(base_path_, downsample_rate_, base_points_);
}

void PathToTrajectory::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odometry_ = *msg;

  Trajectory trajectory;
  trajectory.header.stamp = this->now();
  trajectory.header.frame_id = "map";

  double start_x = odometry_.pose.pose.position.x;
  double start_y = odometry_.pose.pose.position.y;
  double min_dist = std::numeric_limits<double>::infinity();
  int index = 0;

  if (!odom_flag_) {
    for (int i = 0; i < base_points_.size(); i++) {
      double dist = std::sqrt(
        std::pow(base_points_[i].position.x - start_x, 2) +
        std::pow(base_points_[i].position.y - start_y, 2)
      );

      if (dist < min_dist) {
        min_dist = dist;
        index = i;
      }
    }

    std::vector<geometry_msgs::msg::Pose> points;
    for (int i = index; i < base_points_.size(); i++) {
      points.push_back(base_points_[i]);
    }

    for (int i = 0; i < index; i++) {
      points.push_back(base_points_[i]);
    }

    odom_flag_ = true;

    base_points_ = std::move(points);

  } else {
    for (int i = 0; i < margin_; i++) {
      double dist = std::sqrt(
        std::pow(base_points_[i].position.x - start_x, 2) +
        std::pow(base_points_[i].position.y - start_y, 2)
      );

      if (dist < min_dist) {
        min_dist = dist;
        index = i;
      }
    }

    base_points_.erase(base_points_.begin(), base_points_.begin() + index);
  }

  for (const auto& pose: base_points_) {
    TrajectoryPoint point;
    point.pose = pose;
    point.longitudinal_velocity_mps = 20.0;
    trajectory.points.push_back(point);
  }

  trajectory_pub_->publish(trajectory);

}

void PathToTrajectory::load_csv(std::string csv_path, int downsample_rate, std::vector<geometry_msgs::msg::Pose>& point)
{
  RCLCPP_INFO(this->get_logger(), "--------------- Load CSV %s ---------------", csv_path.c_str());

  std::ifstream file(csv_path);
  std::string line;
  int line_count = 0;
  if (!file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Failed to open CSV file");
  } else {
    RCLCPP_INFO(this->get_logger(), "Reading CSV file");
    std::getline(file, line);
    while (std::getline(file, line))
    {
      line_count++;
      
      if (line_count % downsample_rate != 0)
      {
        continue;
      }

      std::stringstream ss(line);
      std::string x, y, z, x_quat, y_quat, z_quat, w_quat;
      std::getline(ss, x, ',');
      std::getline(ss, y, ',');
      std::getline(ss, z, ',');
      std::getline(ss, x_quat, ',');
      std::getline(ss, y_quat, ',');
      std::getline(ss, z_quat, ',');
      std::getline(ss, w_quat, ',');

      geometry_msgs::msg::Pose pose;
      pose.position.x = std::stof(x);
      pose.position.y = std::stof(y);
      pose.position.z = 43.1;
      pose.orientation.x = std::stof(x_quat);
      pose.orientation.y = std::stof(y_quat);
      pose.orientation.z = std::stof(z_quat);
      pose.orientation.w = std::stof(w_quat);

      point.push_back(pose);
    }
    file.close();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu points", point.size());
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
