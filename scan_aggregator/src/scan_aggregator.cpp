//
// Created by armon on 1/14/20.
//
#include <deque>
#include <tuple>
#include <iostream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, nav_msgs::Odometry> Policy;

class ScanAggregator {

    ros::NodeHandle nh_;
    float frequency_;
    std::string cloud_frame_id_;
    std::string pose_frame_id_;

    Eigen::Isometry3f G_sv_;

    message_filters::Subscriber<PointCloud> pc_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Synchronizer<Policy> sync_;
    ros::Publisher cloud_pub_;
    ros::Publisher keyframe_pub_;

    std::deque<std::tuple<PointCloud::Ptr, Eigen::Isometry3d>> cloud_buffer_;


public:

    ScanAggregator() : nh_("~"), pc_sub_(nh_, "/scan", 20),
                       odom_sub_(nh_, "/odometry", 100),
                       sync_(Policy(20), ScanAggregator::pc_sub_, ScanAggregator::odom_sub_),
                       G_sv_(Eigen::Isometry3f::Identity()) {

        nh_.param<std::string>("cloud_frame_id", cloud_frame_id_, "vehicle");
        nh_.param<std::string>("pose_frame_id", pose_frame_id_, "local");

        float duration;
        nh_.param<float>("duration", duration, 0.2);
        frequency_ = 1.0 / duration;

        std::vector<double> calibration_parameters;
        nh_.param<std::vector<double>>("sensor_calibration", calibration_parameters,
                                       {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                        1.0});

        Eigen::Matrix3f R_sv;
        R_sv << calibration_parameters[0], calibration_parameters[1], calibration_parameters[2],
                calibration_parameters[4], calibration_parameters[5], calibration_parameters[6],
                calibration_parameters[8], calibration_parameters[9], calibration_parameters[10];
        Eigen::Vector3f t_sv;
        t_sv << calibration_parameters[3], calibration_parameters[7], calibration_parameters[11];
        G_sv_.translate(t_sv);
        G_sv_.rotate(R_sv);

        sync_.registerCallback(boost::bind(&ScanAggregator::callback, this, _1, _2));
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aggregate_scan", 10);
        keyframe_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("keyframe", 10);
    }

    void loop() {

        ros::Rate rate(frequency_);
        while (ros::ok()) {

            rate.sleep();
            ros::spinOnce();

            if (cloud_buffer_.empty()) {
                continue;
            }

            // ROS_INFO("Queue size = %d", static_cast<int>(cloud_buffer_.size()));

            Eigen::Isometry3d G0 = std::get<1>(cloud_buffer_.back());
            Eigen::Isometry3d G_inv = G0.inverse();
            PointCloud vehicle_aggregate_cloud;
            for (auto &e : cloud_buffer_) {
                PointCloud c;
                auto ci = std::get<0>(e);
                Eigen::Isometry3d Gi = std::get<1>(e);
                pcl::transformPointCloud(*ci, c, (G_inv * Gi).cast<float>());
                vehicle_aggregate_cloud += c;
            }

            ros::Time now = ros::Time::now();

            sensor_msgs::PointCloud2 cloud_out;
            pcl::toROSMsg(vehicle_aggregate_cloud, cloud_out);
            cloud_out.header.stamp = now;
            cloud_out.header.frame_id = cloud_frame_id_;

            geometry_msgs::PoseStamped keyframe;
            keyframe.header.stamp = now;
            keyframe.header.frame_id = pose_frame_id_;
            tf::poseEigenToMsg(G0, keyframe.pose);

            cloud_pub_.publish(cloud_out);
            keyframe_pub_.publish(keyframe);

            cloud_buffer_.clear();

        }

    }

    void callback(const PointCloud::ConstPtr &cloud_msg, const nav_msgs::Odometry::ConstPtr &odom_msg) {
        Eigen::Isometry3d G;
        tf::poseMsgToEigen(odom_msg->pose.pose, G);

        PointCloud::Ptr vehicle_cloud(new PointCloud());
        pcl::transformPointCloud(*cloud_msg, *vehicle_cloud, G_sv_.inverse());
        cloud_buffer_.emplace_back(vehicle_cloud, G);

    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "scan_aggregator");

    ScanAggregator scan_aggregator;
    scan_aggregator.loop();

    return 0;
}

