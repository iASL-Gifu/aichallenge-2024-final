#ifndef GPS_MODULE__TRUE_GPS_MODULE_HPP_
#define GPS_MODULE__TRUE_GPS_MODULE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <deque>

class PoseComparisonNode : public rclcpp::Node {
public:
  PoseComparisonNode();

private:
  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_pose_with_cov_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

  // Subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Client

  // Server

  // Param
  bool debug_;
  float scale_pos_cov_x_;
  float scale_pos_cov_y_;
  float scale_pos_cov_z_;
  float scale_ori_cov_0_;
  float scale_ori_cov_1_;
  float scale_ori_cov_2_;

  // Function
  void compare_poses(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& old_pose,
                       const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& new_pose);

  // Variable
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_queue_;
};

#endif