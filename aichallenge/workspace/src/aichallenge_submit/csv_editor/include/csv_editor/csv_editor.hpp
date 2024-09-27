#ifndef CSV_EDITOR__CSV_EDITOR_HPP_
#define CSV_EDITOR__CSV_EDITOR_HPP_

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "custom_msgs/srv/set_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>

class CsvEditor : public rclcpp::Node {
  public:
    using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

    static constexpr size_t MAX_POINTS = 10000;

    CsvEditor();

  private:
    // Publisher

    // Subscription

    // Client
    rclcpp::Client<custom_msgs::srv::SetTrajectory>::SharedPtr set_trajectory_client_;
    void set_trajectory_request();

    // Server

    // Param
    std::string base_path_;
    int downsample_rate_;

    // function
    void load_csv(std::string csv_path, int downsample_rate);

    // variable
    std::array<TrajectoryPoint, MAX_POINTS> points_;
    size_t point_count_;
};

#endif
