//
// Created by armon on 1/31/20.
//

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <eigen3/Eigen/Dense>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TransformStamped> Policy;

class CompassToTFNode : public rclcpp::Node {

    private:
        bool is_first_;

        message_filters::Subscriber<geometry_msgs::msg::TransformStamped> compass_transform_subscriber_;
        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> keyframe_pose_subscriber_;
        message_filters::Synchronizer<Policy> sync_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> stf_broadcaster_;

    public:
        CompassToTFNode() : Node("compass_to_tf"), is_first_(true),
        compass_transform_subscriber_(this, "/compass_transform", rmw_qos_profile_default),
        keyframe_pose_subscriber_(this, "/keyframe", rmw_qos_profile_default),
        sync_(Policy(10), keyframe_pose_subscriber_, compass_transform_subscriber_) {

            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            stf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            sync_.registerCallback(std::bind(&CompassToTFNode::callback, this, std::placeholders::_1, std::placeholders::_2));
        }

        void callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg,
                const geometry_msgs::msg::TransformStamped::ConstSharedPtr &transform_msg) {

            Eigen::Isometry3d G_ws;
            tf2::fromMsg(pose_msg->pose, G_ws);

            Eigen::Isometry3d G_cs = tf2::transformToEigen(*transform_msg);

            Eigen::Isometry3d G_wc = G_ws * G_cs.inverse();

            geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(G_wc);
            transform.header.stamp = pose_msg->header.stamp;
            transform.header.frame_id = pose_msg->header.frame_id;
            transform.child_frame_id = transform_msg->header.frame_id;
            tf_broadcaster_->sendTransform(transform);

            if (is_first_) {
                transform.header.frame_id = pose_msg->header.frame_id;
                transform.child_frame_id = "building";
                stf_broadcaster_->sendTransform(transform);
                is_first_ = false;
            }
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CompassToTFNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
