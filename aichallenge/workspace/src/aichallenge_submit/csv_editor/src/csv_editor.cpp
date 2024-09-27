#include "csv_editor/csv_editor.hpp"
#include <fstream>
#include <sstream>
#include <string>

CsvEditor::CsvEditor() : Node("csv_editor"), point_count_(0)  // point_count_を初期化
{
  RCLCPP_INFO(this->get_logger(), "================ Csv Editor ==================");

  this->declare_parameter("base_path", std::string("base_path"));
  base_path_ = this->get_parameter("base_path").as_string();

  this->declare_parameter("downsample_rate", 0);
  downsample_rate_ = this->get_parameter("downsample_rate").as_int();

  RCLCPP_INFO(this->get_logger(), "base_path: %s", base_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "downsample_rate: %d", downsample_rate_);

  set_trajectory_client_ = this->create_client<custom_msgs::srv::SetTrajectory>("/set_trajectory");

  load_csv(base_path_, downsample_rate_);
}

void CsvEditor::load_csv(std::string csv_path, int downsample_rate)
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

      if (point_count_ >= MAX_POINTS) {
        RCLCPP_WARN(this->get_logger(), "Reached maximum number of points (%zu)", MAX_POINTS);
        break;  // 最大数に達したらループを抜ける
      }

      std::stringstream ss(line);
      std::string x, y, z, x_quat, y_quat, z_quat, w_quat, speed;
      std::getline(ss, x, ',');
      std::getline(ss, y, ',');
      std::getline(ss, z, ',');
      std::getline(ss, x_quat, ',');
      std::getline(ss, y_quat, ',');
      std::getline(ss, z_quat, ',');
      std::getline(ss, w_quat, ',');
      std::getline(ss, speed, ',');

      geometry_msgs::msg::Pose pose;
      pose.position.x = std::stof(x);
      pose.position.y = std::stof(y);
      pose.position.z = 43.1;
      pose.orientation.x = std::stof(x_quat);
      pose.orientation.y = std::stof(y_quat);
      pose.orientation.z = std::stof(z_quat);
      pose.orientation.w = std::stof(w_quat);

      TrajectoryPoint point;
      point.pose = pose;
      point.longitudinal_velocity_mps = std::stof(speed);
      points_[point_count_] = point;  // std::arrayに格納
      point_count_++;  // カウントを増やす
    }
    file.close();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu points", point_count_);

  set_trajectory_request();
}

void CsvEditor::set_trajectory_request() {
  auto request = std::make_shared<custom_msgs::srv::SetTrajectory::Request>();

  // std::arrayをそのまま代入
  request->points = points_;

  using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::SetTrajectory>::SharedFuture;

  // レスポンス処理
  auto response_callback = [this](ServiceResponseFuture future) {
    RCLCPP_INFO(this->get_logger(), "Set Trajectory");
  };

  set_trajectory_client_->async_send_request(request, response_callback);
  RCLCPP_INFO(this->get_logger(), "Send Trajectory");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvEditor>());
  rclcpp::shutdown();
  return 0;
}
