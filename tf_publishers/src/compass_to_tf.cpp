//
// Created by armon on 1/31/20.
//


#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TransformStamped> Policy;

class CompassToTFNode {

private:

    ros::NodeHandle nh_;
    bool is_first_;

    message_filters::Subscriber<geometry_msgs::TransformStamped> compass_transform_subscriber_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> keyframe_pose_subscriber_;
    message_filters::Synchronizer<Policy> sync_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster stf_broadcaster_;

public:

    CompassToTFNode() : nh_("~"), is_first_(true),
                        compass_transform_subscriber_(nh_, "/compass_transform", 1),
                        keyframe_pose_subscriber_(nh_, "/keyframe", 1),
                        sync_(Policy(10), keyframe_pose_subscriber_, compass_transform_subscriber_) {

        sync_.registerCallback(boost::bind(&CompassToTFNode::callback, this, _1, _2));

    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg,
                  const geometry_msgs::TransformStamped::ConstPtr &transform_msg) {

        Eigen::Isometry3d G_ws;
        tf::poseMsgToEigen(pose_msg->pose, G_ws);

        Eigen::Isometry3d G_cs;
        tf::transformMsgToEigen(transform_msg->transform, G_cs);

        Eigen::Isometry3d G_wc = G_ws * G_cs.inverse();

        // debug
        // Eigen::Matrix3d R_wc = G_ws.rotation() * G_cs.rotation().transpose();
        // std::cout << G_ws.matrix() << std::endl;
        // std::cout << G_cs.matrix() << std::endl;
        // std::cout << transform_msg->transform << std::endl;
        // std::cout << std::endl;

        geometry_msgs::TransformStamped transform;
        tf::transformEigenToMsg(G_wc, transform.transform);
        transform.header.stamp = pose_msg->header.stamp;
        transform.header.frame_id = pose_msg->header.frame_id;
        transform.child_frame_id = transform_msg->header.frame_id;
        tf_broadcaster_.sendTransform(transform);

        if (is_first_) {

            transform.header.frame_id = pose_msg->header.frame_id;
            transform.child_frame_id = "building";
            stf_broadcaster_.sendTransform(transform);
            is_first_ = false;

        }

    }


};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "compass_to_tf");

    CompassToTFNode compass_to_tf_node;

    ros::spin();

}