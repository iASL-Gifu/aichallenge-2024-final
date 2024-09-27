#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>  // 追加

class SectionTimerNode : public rclcpp::Node
{
public:
  SectionTimerNode()
  : Node("section_timer_node"), is_started_(false)
  {
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<double>("radius", 5.0);

    debug_ = this->get_parameter("debug").as_bool();
    radius_ = this->get_parameter("radius").as_double();

    if (debug_) {
      RCLCPP_INFO(this->get_logger(), "Debug mode is ON. Checking if YAML parameters are loaded...");
    }

    // セクション情報のロード
    for (int i = 1; i <= 12; ++i) {
      std::string section_name = "sections.section_" + std::to_string(i);

      this->declare_parameter<std::vector<double>>(section_name + ".start", {0.0, 0.0});
      this->declare_parameter<std::vector<double>>(section_name + ".end", {0.0, 0.0});

      std::vector<double> start;
      std::vector<double> end;

      if (this->get_parameter(section_name + ".start", start) && this->get_parameter(section_name + ".end", end)) {
        if (debug_) {
          RCLCPP_INFO(this->get_logger(), "Section %d:", i);
          RCLCPP_INFO(this->get_logger(), "  Start: [%.2f, %.2f]", start[0], start[1]);
          RCLCPP_INFO(this->get_logger(), "  End:   [%.2f, %.2f]", end[0], end[1]);
        }
        sections_.emplace_back(std::make_pair(start, end));
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameters for section %d", i);
      }
    }

    // Odomトピックのサブスクライバ
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", 1, std::bind(&SectionTimerNode::odometry_callback, this, std::placeholders::_1));

    // Markerパブリッシャの作成
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("~/output/visualization", 10);

    // Float32MultiArrayパブリッシャの作成
    time_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/output/section_times", 10);

    // Start地点のマーカーを可視化
    publish_start_markers();

    // セクションタイム配列を初期化（13要素: 12セクション + 1ラップタイム）
    section_times_.resize(sections_.size() + 1, 0.0);
  }

private:
  void publish_start_markers()
  {
    for (size_t i = 0; i < sections_.size(); ++i) {
      const auto& start = sections_[i].first;

      // Markerメッセージの設定
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";  // 使用するフレームID
      marker.header.stamp = this->now();
      marker.ns = "start_points";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;  // マーカーの形状
      marker.action = visualization_msgs::msg::Marker::ADD;

      // 座標の設定
      marker.pose.position.x = start[0];
      marker.pose.position.y = start[1];
      marker.pose.position.z = 43.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // スケールの設定
      marker.scale.x = 4.0;  // 半径1mの球
      marker.scale.y = 4.0;
      marker.scale.z = 4.0;

      // 色の設定（緑色）
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;  // アルファ値

      // マーカーのパブリッシュ
      marker_publisher_->publish(marker);
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (!is_started_) {
      if (distance(x, y, sections_[current_section_].first[0], sections_[current_section_].first[1]) < radius_) {
        RCLCPP_INFO(this->get_logger(), "Started section %d timer", current_section_ + 1);
        start_time_ = this->now();

        // 最初のセクションに到達したらラップタイマーを開始
        if (current_section_ == 0) {
          lap_start_time_ = this->now();
        }

        is_started_ = true;
      }
    } else {
      if (distance(x, y, sections_[current_section_].second[0], sections_[current_section_].second[1]) < radius_) {
        auto end_time = this->now();
        auto duration = end_time - start_time_;
        RCLCPP_INFO(this->get_logger(), "Finished section %d. Time: %.2f seconds",
                    current_section_ + 1, duration.seconds());

        // セクションタイムを更新
        section_times_[current_section_] = duration.seconds();

        // セクションタイムをパブリッシュ
        publish_section_times(false);  // ラップ終了時でないのでfalse

        is_started_ = false;
        current_section_++;
        if (current_section_ >= sections_.size()) {
          // 1ラップ終了時に合計タイムを計算して表示
          auto lap_end_time = this->now();
          auto lap_duration = lap_end_time - lap_start_time_;
          RCLCPP_INFO(this->get_logger(), "Lap completed! Total lap time: %.2f seconds", lap_duration.seconds());

          // ラップタイムを最後のインデックスにセット
          section_times_.back() = lap_duration.seconds();

          // ラップタイムを含めたパブリッシュ
          publish_section_times(true);  // ラップ終了なのでtrue

          current_section_ = 0;  // 最初のセクションに戻る
        }
      }
    }
  }

  // セクションタイムのパブリッシュ
  void publish_section_times(bool is_lap_completed)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = section_times_;  // セクションタイムとラップタイムを含める
    time_publisher_->publish(msg);
  }

  double distance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr time_publisher_;  // タイムパブリッシャ
  std::vector<std::pair<std::vector<double>, std::vector<double>>> sections_;
  std::vector<float> section_times_;  // セクションタイムとラップタイムを保持する配列（サイズは13）
  rclcpp::Time start_time_;
  rclcpp::Time lap_start_time_;
  bool is_started_;
  int current_section_ = 0;
  double radius_;
  bool debug_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SectionTimerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}