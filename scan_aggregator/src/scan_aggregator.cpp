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
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, nav_msgs::Odometry> Policy;

class ScanAggregator {

public:

    ScanAggregator() : nh_("~"), pc_sub_(nh_, "/puck/scan_transformed_corrected", 10),
        odom_sub_(nh_, "/odometry", 100),
        sync_(Policy(20), ScanAggregator::pc_sub_, ScanAggregator::odom_sub_)
    {
        float duration;
        float overlap_period;
        float scanner_frequency;

        nh_.param<float>("duration", duration, 0.2);
        nh_.param<float>("overlap_period", overlap_period, 0.0);
        nh_.param<float>("scanner_frequency", scanner_frequency, 80.0);

        max_buffer_size_ = static_cast<int>(round(scanner_frequency * duration));
        overlap_size_ = std::min(static_cast<int>(round(scanner_frequency * overlap_period)), max_buffer_size_);

        sync_.registerCallback(boost::bind(&ScanAggregator::callback, this, _1, _2));
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aggregate_scan", 20);
    }

    void callback(const PointCloud::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        Eigen::Isometry3d G;
        tf::poseMsgToEigen(odom_msg->pose.pose, G);

        PointCloud::Ptr vehicle_cloud(new PointCloud());
        *vehicle_cloud = *cloud_msg;
        cloud_buffer_.emplace_back(vehicle_cloud, G.cast<float>());

        if (cloud_buffer_.size() < max_buffer_size_)
            return;

        auto G_inv = G.cast<float>().inverse();
        PointCloud vehicle_aggregate_cloud;
        for (auto &e : cloud_buffer_) {
            PointCloud c;
            auto ci = std::get<0>(e);
            auto G0 = std::get<1>(e);
            pcl::transformPointCloud(*ci, c, G_inv * G0);
            vehicle_aggregate_cloud += c;
        }

        sensor_msgs::PointCloud2 cloud_out;
        pcl::toROSMsg(vehicle_aggregate_cloud, cloud_out);
        cloud_out.header.stamp = odom_msg->header.stamp;
        cloud_out.header.frame_id = cloud_msg->header.frame_id;
        pub_.publish(cloud_out);

        while (cloud_buffer_.size() > overlap_size_)
            cloud_buffer_.pop_front();

    }

private:

    ros::NodeHandle nh_;
    int max_buffer_size_;
    int overlap_size_;

    message_filters::Subscriber<PointCloud> pc_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Synchronizer<Policy> sync_;
    ros::Publisher pub_;

    std::deque<std::tuple<PointCloud::Ptr, Eigen::Isometry3f>> cloud_buffer_;

};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "scan_aggregator");

    ScanAggregator scan_aggregator;

    ros::spin();

    return 0;
}

