//
// Created by armon on 1/30/20.
//

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>

class TransformToTFNode {

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool flip_transform_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

public:

    TransformToTFNode() : nh_("~") {
        sub_ = nh_.subscribe<geometry_msgs::TransformStamped>("/transform", 1,
                                                              boost::bind(&TransformToTFNode::callback, this, _1));

        nh_.param<bool>("flip_transform", flip_transform_, false);
    }

    void callback(const geometry_msgs::TransformStamped::ConstPtr &transform_msg) {

        Eigen::Isometry3d G;
        tf::transformMsgToEigen(transform_msg->transform, G);

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = transform_msg->header.stamp;
        // transform.header.stamp = ros::Time::now();

        if (flip_transform_) {
            G = G.inverse();

            transform.header.frame_id = transform_msg->child_frame_id;
            transform.child_frame_id = transform_msg->header.frame_id;

        } else {

            transform.header.frame_id = transform_msg->header.frame_id;
            transform.child_frame_id = transform_msg->child_frame_id;

        }

        tf::transformEigenToMsg(G, transform.transform);


        tf_broadcaster_.sendTransform(transform);

    }


};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "transform_to_tf");

    TransformToTFNode transform_to_tf_node;

    ros::spin();

}