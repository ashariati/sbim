//
// Created by armon on 3/5/20.
//

#include <mutex>
#include <unordered_map>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <eigen_conversions/eigen_msg.h>

#include <sbim_msgs/Trajectory.h>

#include <scene_parsing/layout_extractor.h>

typedef pcl::PointXYZ PointT;

class CloudTransformerNode {

    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Subscriber pc_sub_;
    ros::Subscriber traj_sub_;

    std::mutex cloud_mutex_;
    std::deque<pcl::PointCloud<PointT>> cloud_queue_;

    std::mutex traj_mutex_;
    std::unordered_map<uint64_t, Eigen::Isometry3f> pose_at_time_;

    int frequency_;
    int cloud_window_;
    std::string frame_id_;

public:

    ~CloudTransformerNode() = default;

    CloudTransformerNode() : nh_("~"),
                             frequency_(10),
                             cloud_window_(1) {

        nh_.param<int>("frequency", frequency_, 10);
        nh_.param<int>("cloud_window", cloud_window_, 20);
        nh_.param<std::string>("frame_id", frame_id_, "building");

        cloud_pub_ = nh_.advertise<pcl::PointCloud<PointT>>("cloud", 10);

        traj_sub_ = nh_.subscribe<sbim_msgs::Trajectory>("/trajectory", 1,
                                                         boost::bind(&CloudTransformerNode::traj_callback, this, _1));
        pc_sub_ = nh_.subscribe<pcl::PointCloud<PointT>>("/transformed_scan", 1,
                                                         boost::bind(&CloudTransformerNode::cloud_callback, this, _1));

    }

    void cloud_callback(const pcl::PointCloud<PointT>::ConstPtr &cloud) {

        cloud_mutex_.lock();
        cloud_queue_.push_back(*cloud);
        if (cloud_queue_.size() > cloud_window_) {
            cloud_queue_.pop_front();
        }
        cloud_mutex_.unlock();

    }

    void traj_callback(const sbim_msgs::Trajectory::ConstPtr &trajectory) {

        traj_mutex_.lock();
        for (auto &pose_stamped : trajectory->poses) {
            Eigen::Isometry3d G;
            tf::poseMsgToEigen(pose_stamped.pose, G);

            uint64_t key = pcl_conversions::toPCL(pose_stamped.header.stamp);
            pose_at_time_[key] = G.cast<float>();
        }
        traj_mutex_.unlock();

    }

    void loop() {

        ros::Rate rate(frequency_);
        while (ros::ok()) {

            rate.sleep();
            ros::spinOnce();

            pcl::PointCloud<PointT> out_cloud;

            cloud_mutex_.lock();
            traj_mutex_.lock();

            for (auto &cloud : cloud_queue_) {
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

            cloud_mutex_.unlock();
            traj_mutex_.unlock();

            cloud_pub_.publish(out_cloud);
        }

    }

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "cloud_transformer_node");

    CloudTransformerNode cloud_transformer_node;
    cloud_transformer_node.loop();

    return 0;

}
