//
// Created by armon on 1/20/20.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTFNode {

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

public:

    OdomToTFNode() : nh_("~") {
        sub_ = nh_.subscribe<nav_msgs::Odometry>("/odometry", 1, boost::bind(&OdomToTFNode::callback, this, _1));
    }

    void callback(const nav_msgs::Odometry::ConstPtr odom_msg) {

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = odom_msg->header.stamp;
        transform.header.frame_id = odom_msg->header.frame_id;
        transform.child_frame_id = "vehicle";

        transform.transform.rotation = odom_msg->pose.pose.orientation;
        transform.transform.translation.x = odom_msg->pose.pose.position.x;
        transform.transform.translation.y = odom_msg->pose.pose.position.y;
        transform.transform.translation.z = odom_msg->pose.pose.position.z;

        tf_broadcaster_.sendTransform(transform);

    }


};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "odom_to_tf");

    OdomToTFNode odom_to_tf_node;

    ros::spin();

}