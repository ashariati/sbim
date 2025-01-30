//
// Created by armon on 3/5/20.
//

#include <mutex>
#include <unordered_map>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sbim_msgs/msg/trajectory.hpp>

// #include <scene_parsing/layout_extractor.h>

typedef pcl::PointXYZ PointT;

class CloudTransformerNode : public rclcpp::Node {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sbim_msgs::msg::Trajectory>::SharedPtr traj_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex cloud_mutex_;
    std::deque<pcl::PointCloud<PointT> > cloud_queue_;

    std::mutex traj_mutex_;
    std::unordered_map<uint64_t, Eigen::Isometry3f> pose_at_time_;

    int frequency_;
    int cloud_window_;
    std::string frame_id_;

public:
    ~CloudTransformerNode() = default;

    CloudTransformerNode()
        : Node("cloud_transformer_node"),
          frequency_(10),
          cloud_window_(20) {
        this->declare_parameter<int>("frequency", 10);
        this->declare_parameter<int>("cloud_window", 20);
        this->declare_parameter<std::string>("frame_id", "building");

        frequency_ = this->get_parameter("frequency").as_int();
        cloud_window_ = this->get_parameter("cloud_window").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);

        traj_sub_ = this->create_subscription<sbim_msgs::msg::Trajectory>(
            "/trajectory", 1, std::bind(&CloudTransformerNode::traj_callback, this, std::placeholders::_1));
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/transformed_scan", 1, std::bind(&CloudTransformerNode::cloud_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / frequency_),
            std::bind(&CloudTransformerNode::loop, this));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        pcl::PointCloud<PointT> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        cloud_queue_.push_back(cloud);
        if (cloud_queue_.size() > cloud_window_) {
            cloud_queue_.pop_front();
        }
    }

    void traj_callback(const sbim_msgs::msg::Trajectory::ConstSharedPtr trajectory) {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        for (auto &pose_stamped: trajectory->poses) {
            Eigen::Isometry3d G;
            tf2::fromMsg(pose_stamped.pose, G);

            uint64_t key = pcl_conversions::toPCL(pose_stamped.header.stamp);
            pose_at_time_[key] = G.cast<float>();
        }
    }

    void loop() {
        pcl::PointCloud<PointT> out_cloud;

        std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
        std::lock_guard<std::mutex> traj_lock(traj_mutex_);

        for (auto &cloud: cloud_queue_) {
            uint64_t key = cloud.header.stamp;

            if (!pose_at_time_.count(key)) {
                continue;
            }

            pcl::PointCloud<PointT> transformed_cloud;
            Eigen::Isometry3f G = pose_at_time_[key];
            pcl::transformPointCloud(cloud, transformed_cloud, G);

            out_cloud += transformed_cloud;
        }

        out_cloud.header.frame_id = frame_id_;

        sensor_msgs::msg::PointCloud2 out_cloud_msg;
        pcl::toROSMsg(out_cloud, out_cloud_msg);

        cloud_pub_->publish(out_cloud_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudTransformerNode>());
    rclcpp::shutdown();
    return 0;
}
