#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTFNode : public rclcpp::Node {

    private:

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    public:

        OdomToTFNode() : Node("odom_to_tf") {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/odometry", 1, std::bind(&OdomToTFNode::callback, this, std::placeholders::_1));
        }

        void callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = odom_msg->header.stamp;
            transform.header.frame_id = odom_msg->header.frame_id;
            transform.child_frame_id = "vehicle";

            transform.transform.rotation = odom_msg->pose.pose.orientation;
            transform.transform.translation.x = odom_msg->pose.pose.position.x;
            transform.transform.translation.y = odom_msg->pose.pose.position.y;
            transform.transform.translation.z = odom_msg->pose.pose.position.z;

            tf_broadcaster_->sendTransform(transform);

        }


};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdomToTFNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
