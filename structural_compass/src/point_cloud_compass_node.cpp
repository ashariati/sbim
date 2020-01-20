//
// Created by armon on 1/16/20.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

#include <structural_compass/structural_compass.h>
#include <structural_compass/PrincipalDirections.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, nav_msgs::Odometry> Policy;

template<typename Compass>
class PointCloudCompassNode {

public:

    PointCloudCompassNode() : nh_("~"),
                              pc_sub_(nh_, "/scan_aggregator/aggregate_scan", 1),
                              odom_sub_(nh_, "/odometry", 500),
                              pc_sync_(Policy(100), pc_sub_, odom_sub_) {

        compass_ = std::make_unique<Compass>();
        pc_sync_.registerCallback(boost::bind(&PointCloudCompassNode::callback, this, _1, _2));
        pd_pub_ = nh_.advertise<structural_compass::PrincipalDirections>("principal_directions", 10);

    }

    void callback(const PointCloud::ConstPtr &cloud_msg, const nav_msgs::Odometry::ConstPtr &odom_msg) {

        Eigen::Isometry3d G_wc;
        tf::poseMsgToEigen(odom_msg->pose.pose, G_wc);

        Eigen::Vector3f gravity;
        gravity << -G_wc(2, 0), -G_wc(2, 1), -G_wc(2, 2);

        Eigen::Matrix3f R_cg;
        std::vector<Eigen::Vector3f> directions;
        R_cg = compass_->principalDirections(*cloud_msg, gravity, directions);
        publish(R_cg, directions, odom_msg->header.stamp);

    }

    void publish(const Eigen::Matrix3f &R, const std::vector<Eigen::Vector3f> &directions, const ros::Time stamp) {

        geometry_msgs::Quaternion q_cg;
        tf::quaternionEigenToMsg(Eigen::Quaternionf(R).cast<double>(), q_cg);

        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "vehicle";
        transform.header.stamp = stamp;
        transform.child_frame_id = "compass";
        transform.transform.rotation = q_cg;
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        tf_broadcaster_.sendTransform(transform);

        structural_compass::PrincipalDirections principal_directions;
        principal_directions.header.frame_id = "compass";
        principal_directions.header.stamp = stamp;
        for (auto &d : directions) {
            geometry_msgs::Vector3 v;
            tf::vectorEigenToMsg(d.cast<double>(), v);
            principal_directions.directions.push_back(v);
        }
        pd_pub_.publish(principal_directions);

    }

private:

    ros::NodeHandle nh_;

    std::unique_ptr<Compass> compass_;

    message_filters::Subscriber<PointCloud> pc_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Synchronizer<Policy> pc_sync_;

    ros::Publisher pd_pub_;
    ros::Publisher rot_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point_cloud_compass_node");

    PointCloudCompassNode<structural_compass::EntropyCompass<PointCloud>> compass_node;

    ros::spin();
}

