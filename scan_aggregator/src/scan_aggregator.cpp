//
// Created by armon on 1/14/20.
//
#include <deque>
#include <tuple>
#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, nav_msgs::msg::Odometry> Policy;

class ScanAggregator : public rclcpp::Node {

    float frequency_;
    std::string cloud_frame_id_;
    std::string pose_frame_id_;

    Eigen::Isometry3f G_sv_;

    std::shared_ptr<message_filters::Subscriber<PointCloud>> pc_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr keyframe_pub_;

    std::deque<std::tuple<PointCloud::Ptr, Eigen::Isometry3d>> cloud_buffer_;

public:

    ScanAggregator() : Node("scan_aggregator"), G_sv_(Eigen::Isometry3f::Identity()) {

        this->declare_parameter<std::string>("cloud_frame_id", "vehicle");
        this->declare_parameter<std::string>("pose_frame_id", "local");

        this->declare_parameter<float>("duration", 0.2);
        this->declare_parameter<std::vector<double>>("sensor_calibration",
            {1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0});

        cloud_frame_id_ = this->get_parameter("cloud_frame_id").as_string();
        pose_frame_id_ = this->get_parameter("pose_frame_id").as_string();
        float duration = this->get_parameter("duration").as_double();
        frequency_ = 1.0 / duration;

        std::vector<double> calibration_parameters = this->get_parameter("sensor_calibration").as_double_array();
        Eigen::Matrix3f R_sv;
        R_sv << calibration_parameters[0], calibration_parameters[1], calibration_parameters[2],
                calibration_parameters[4], calibration_parameters[5], calibration_parameters[6],
                calibration_parameters[8], calibration_parameters[9], calibration_parameters[10];
        Eigen::Vector3f t_sv;
        t_sv << calibration_parameters[3], calibration_parameters[7], calibration_parameters[11];
        G_sv_.translate(t_sv);
        G_sv_.rotate(R_sv);

        pc_sub_ = std::make_shared<message_filters::Subscriber<PointCloud>>(this, "/scan");
        odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/odometry");

        sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(20), *pc_sub_, *odom_sub_);
        sync_->registerCallback(std::bind(&ScanAggregator::callback, this, std::placeholders::_1, std::placeholders::_2));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aggregate_scan", 10);
        keyframe_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("keyframe", 10);
    }

    void loop() {

        rclcpp::Rate rate(frequency_);
        while (rclcpp::ok()) {

            rate.sleep();
            rclcpp::spin_some(this->get_node_base_interface());


            if (cloud_buffer_.empty()) {
                continue;
            }

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

            auto now = this->now();

            sensor_msgs::msg::PointCloud2 cloud_out;
            pcl::toROSMsg(vehicle_aggregate_cloud, cloud_out);
            cloud_out.header.stamp = now;
            cloud_out.header.frame_id = cloud_frame_id_;

            geometry_msgs::msg::PoseStamped keyframe;
            keyframe.header.stamp = now;
            keyframe.header.frame_id = pose_frame_id_;
            keyframe.pose = tf2::toMsg(G0);

            cloud_pub_->publish(cloud_out);
            keyframe_pub_->publish(keyframe);

            cloud_buffer_.clear();

        }

    }

    void callback(const PointCloud::ConstPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
        Eigen::Isometry3d G;
        tf2::fromMsg(odom_msg->pose.pose, G);

        PointCloud::Ptr vehicle_cloud(new PointCloud());
        pcl::transformPointCloud(*cloud_msg, *vehicle_cloud, G_sv_.inverse());
        cloud_buffer_.emplace_back(vehicle_cloud, G);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanAggregator>();
    node->loop();
    rclcpp::shutdown();
    return 0;
}

