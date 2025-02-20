#include <deque>
#include <tuple>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

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
#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> Policy;

class ScanAggregator : public rclcpp::Node {
public:
    ScanAggregator() : Node("scan_aggregator"), G_sv_(Eigen::Isometry3f::Identity()) {
        this->declare_parameter<std::string>("cloud_frame_id", "vehicle");
        this->declare_parameter<std::string>("pose_frame_id", "local");

        this->declare_parameter<double>("duration", 0.2);
        this->declare_parameter<std::vector<double> >("sensor_calibration",
                                                      {
                                                          1.0, 0.0, 0.0, 0.0,
                                                          0.0, 1.0, 0.0, 0.0,
                                                          0.0, 0.0, 1.0, 0.0,
                                                          0.0, 0.0, 0.0, 1.0
                                                      });

        cloud_frame_id_ = this->get_parameter("cloud_frame_id").as_string();
        pose_frame_id_ = this->get_parameter("pose_frame_id").as_string();
        double duration = this->get_parameter("duration").as_double();

        std::vector<double> calibration_parameters = this->get_parameter("sensor_calibration").as_double_array();
        Eigen::Matrix3f R_sv;
        R_sv << calibration_parameters[0], calibration_parameters[1], calibration_parameters[2],
                calibration_parameters[4], calibration_parameters[5], calibration_parameters[6],
                calibration_parameters[8], calibration_parameters[9], calibration_parameters[10];
        Eigen::Vector3f t_sv;
        t_sv << calibration_parameters[3], calibration_parameters[7], calibration_parameters[11];
        G_sv_.translate(t_sv);
        G_sv_.rotate(R_sv);

        pc_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2> >(this, "/scan");
        odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry> >(this, "/odometry");

        sync_ = std::make_shared<message_filters::Synchronizer<Policy> >(Policy(5), *pc_sub_, *odom_sub_);
        sync_->registerCallback(
            std::bind(&ScanAggregator::callback, this, std::placeholders::_1, std::placeholders::_2));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aggregate_scan",
            //rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
            100);
        keyframe_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("keyframe", 10);

        timer_ = this->create_wall_timer(std::chrono::duration<double>(duration),
                                         std::bind(&ScanAggregator::loop, this));
    }

    void loop() {
        if (cloud_buffer_.empty()) {
            return;
        }

        Eigen::Isometry3d G0 = std::get<1>(cloud_buffer_.back());
        Eigen::Isometry3d G_inv = G0.inverse();

        PointCloud::Ptr vehicle_aggregate_cloud(new PointCloud());
        for (auto &e: cloud_buffer_) {
            PointCloud::Ptr ci_transformed(new PointCloud());
            auto ci = std::get<0>(e);
            Eigen::Isometry3d Gi = std::get<1>(e);
            pcl::transformPointCloud(*ci, *ci_transformed, (G_inv * Gi).cast<float>());
            *vehicle_aggregate_cloud += *ci_transformed;
        }

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(vehicle_aggregate_cloud);
        sor.setLeafSize(0.05f, 0.05f, 0.5f);  // Adjust leaf size as needed
        sor.filter(*vehicle_aggregate_cloud);

        auto now = this->now();

        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg(*vehicle_aggregate_cloud, cloud_out);
        cloud_out.header.stamp = now;
        cloud_out.header.frame_id = cloud_frame_id_;
        cloud_pub_->publish(cloud_out);

        // RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 size: %ld bytes", cloud_out.data.size());

        geometry_msgs::msg::PoseStamped keyframe;
        keyframe.header.stamp = now;
        keyframe.header.frame_id = pose_frame_id_;
        keyframe.pose = tf2::toMsg(G0);
        keyframe_pub_->publish(keyframe);

        cloud_buffer_.clear();

        // static auto last_time = this->now();
        // auto curr_time = this->now();
        // RCLCPP_INFO(this->get_logger(), "Loop Interval: %f s", (curr_time - last_time).seconds());
        // last_time = curr_time;
    }

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                  const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
        Eigen::Isometry3d G;
        tf2::fromMsg(odom_msg->pose.pose, G);

        PointCloud pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);

        PointCloud::Ptr vehicle_cloud(new PointCloud());
        pcl::transformPointCloud(pcl_cloud, *vehicle_cloud, G_sv_.inverse());
        cloud_buffer_.emplace_back(vehicle_cloud, G);

        // RCLCPP_INFO(this->get_logger(), "Cloud buffer size: %ld", cloud_buffer_.size());
    }

private:
    std::string cloud_frame_id_;
    std::string pose_frame_id_;

    Eigen::Isometry3f G_sv_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2> > pc_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry> > odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<Policy> > sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr keyframe_pub_;

    std::deque<std::tuple<PointCloud::Ptr, Eigen::Isometry3d> > cloud_buffer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanAggregator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

