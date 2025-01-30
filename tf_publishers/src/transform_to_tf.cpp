//
// Created by armon on 1/30/20.
//

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <eigen3/Eigen/Dense>

class TransformToTFNode : public rclcpp::Node {

private:
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_;
    bool flip_transform_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:

    TransformToTFNode() : Node("transform_to_tf") {

        this->declare_parameter<bool>("flip_transform", false);
        flip_transform_ = this->get_parameter("flip_transform").as_bool();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "/transform", 1, std::bind(&TransformToTFNode::callback, this, std::placeholders::_1));
    }

    void callback(const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg) {

        Eigen::Isometry3d G = tf2::transformToEigen(*transform_msg);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = transform_msg->header.stamp;
        // transform.header.stamp = ros::Time::now();

        if (flip_transform_) {
            G = G.inverse();
            transform = tf2::eigenToTransform(G);
            transform.header.frame_id = transform_msg->child_frame_id;
            transform.child_frame_id = transform_msg->header.frame_id;

        } else {
            transform = tf2::eigenToTransform(G);
            transform.header.frame_id = transform_msg->header.frame_id;
            transform.child_frame_id = transform_msg->child_frame_id;
        }

        tf_broadcaster_->sendTransform(transform);

    }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformToTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
