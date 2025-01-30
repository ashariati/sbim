//
// Created by armon on 1/30/20.
//
// Updated 1/30/25
//

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class PoseToTFNode : public rclcpp::Node {

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:

    PoseToTFNode() : Node("pose_to_tf") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/keyframe", 1, std::bind(&PoseToTFNode::callback, this, std::placeholders::_1));
    }


    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = pose_msg->header.stamp;
        // transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = pose_msg->header.frame_id;
        transform.child_frame_id = "vehicle";

        transform.transform.rotation = pose_msg->pose.orientation;
        transform.transform.translation.x = pose_msg->pose.position.x;
        transform.transform.translation.y = pose_msg->pose.position.y;
        transform.transform.translation.z = pose_msg->pose.position.z;

        tf_broadcaster_->sendTransform(transform);
    }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseToTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
