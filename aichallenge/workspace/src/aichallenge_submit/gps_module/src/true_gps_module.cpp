#include "gps_module/true_gps_module.hpp"

PoseComparisonNode::PoseComparisonNode() : Node("pose_comparison_node")
{
    // パラメータの宣言とデフォルト値
    this->declare_parameter<bool>("debug", false);
    debug_ = this->get_parameter("debug").as_bool();
    scale_pos_cov_x_ = this->declare_parameter<float>("scale_position_covariance_x");
    scale_pos_cov_y_ = this->declare_parameter<float>("scale_position_covariance_y");
    scale_pos_cov_z_ = this->declare_parameter<float>("scale_position_covariance_z");
    scale_ori_cov_0_ = this->declare_parameter<float>("scale_orientation_covariance_0");
    scale_ori_cov_1_ = this->declare_parameter<float>("scale_orientation_covariance_1");
    scale_ori_cov_2_ = this->declare_parameter<float>("scale_orientation_covariance_2");

    // サブスクリプションの設定
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "~/input/pose_with_covariance", 10, std::bind(&PoseComparisonNode::pose_callback, this, std::placeholders::_1));

    // PoseWithCovarianceStampedメッセージ用のパブリッシャー
    publisher_pose_with_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/pose_with_covariance", 10);
    
    // Poseメッセージ用のパブリッシャー
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/pose", 10);
}

void PoseComparisonNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // キューに新しいメッセージを追加し、2つを超える場合は古いメッセージを削除
    if (pose_queue_.size() >= 2) {
        pose_queue_.pop_front();
    }
    pose_queue_.push_back(msg);

    // キューに2つのメッセージが揃ったら比較を行う
    if (pose_queue_.size() == 2) {
        compare_poses(pose_queue_[0], pose_queue_[1]);
    }
}

void PoseComparisonNode::compare_poses(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& old_pose,
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& new_pose)
{
    // 位置の変化を確認
    bool position_change = 
        old_pose->pose.pose.position.x != new_pose->pose.pose.position.x ||
        old_pose->pose.pose.position.y != new_pose->pose.pose.position.y ||
        old_pose->pose.pose.position.z != new_pose->pose.pose.position.z;

    // 向き（四元数）の変化を確認
    bool orientation_change = 
        old_pose->pose.pose.orientation.x != new_pose->pose.pose.orientation.x ||
        old_pose->pose.pose.orientation.y != new_pose->pose.pose.orientation.y ||
        old_pose->pose.pose.orientation.z != new_pose->pose.pose.orientation.z ||
        old_pose->pose.pose.orientation.w != new_pose->pose.pose.orientation.w;

    // 位置または向きが変化した場合にイベントをトリガー
    if (position_change || orientation_change) {
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), "Pose has changed!");
        }

        // PoseWithCovarianceStampedメッセージのパブリッシュ
        auto new_pose_modify_cov = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*new_pose);
        new_pose_modify_cov->header.stamp = this->now();

        //Pose Cov
        new_pose_modify_cov->pose.covariance[0] = new_pose->pose.covariance[0] * scale_pos_cov_x_;            
        new_pose_modify_cov->pose.covariance[7] = new_pose->pose.covariance[7] * scale_pos_cov_y_;
        // Keep Z 0.0
        // new_pose_modify_cov->pose.covariance[14] = new_pose->pose.covariance[14] * scale_pos_cov_z_;

        // Orientation Cov, nature 0.1 0.1 1.0
        new_pose_modify_cov->pose.covariance[22] = new_pose->pose.covariance[22] * scale_ori_cov_0_;       
        new_pose_modify_cov->pose.covariance[29] = new_pose->pose.covariance[29] * scale_ori_cov_1_;
        new_pose_modify_cov->pose.covariance[35] = new_pose->pose.covariance[35] * scale_ori_cov_2_;

        publisher_pose_with_cov_->publish(*new_pose_modify_cov);

        // Poseメッセージのパブリッシュ
        auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_msg->header = new_pose->header;
        pose_msg->pose = new_pose->pose.pose;
        publisher_pose_->publish(*pose_msg);
    } else {
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), "Pose not changed!");
        }
    }

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseComparisonNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
