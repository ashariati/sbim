//
// Created by armon on 1/30/20.
//

//
// Created by armon on 1/20/20.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

class PoseToTFNode {

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

public:

    PoseToTFNode() : nh_("~") {
        sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/keyframe", 1,
                                                         boost::bind(&PoseToTFNode::callback, this, _1));
    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = pose_msg->header.stamp;
        // transform.header.stamp = ros::Time::now();
        transform.header.frame_id = pose_msg->header.frame_id;
        transform.child_frame_id = "vehicle";

        transform.transform.rotation = pose_msg->pose.orientation;
        transform.transform.translation.x = pose_msg->pose.position.x;
        transform.transform.translation.y = pose_msg->pose.position.y;
        transform.transform.translation.z = pose_msg->pose.position.z;

        tf_broadcaster_.sendTransform(transform);

    }


};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "pose_to_tf");

    PoseToTFNode pose_to_tf_node;

    ros::spin();

}