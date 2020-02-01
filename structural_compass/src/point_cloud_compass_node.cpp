//
// Created by armon on 1/16/20.
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>

#include <sbim_msgs/PrincipalDirections.h>
#include <structural_compass/structural_compass.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, geometry_msgs::PoseStamped> Policy;
// typedef message_filters::sync_policies::ExactTime<PointCloud, geometry_msgs::PoseStamped> Policy;

template<typename Compass>
class PointCloudCompassNode {

public:

    PointCloudCompassNode() : nh_("~"),
                              pc_sub_(nh_, "/scan", 1),
                              pose_sub_(nh_, "/pose", 1),
                              pc_sync_(Policy(10), pc_sub_, pose_sub_) {

        nh_.param<float>("frequency", frequency_, 10.0);
        nh_.param<int>("queue_size", queue_size_, 1);

        compass_ = std::make_unique<Compass>();
        pc_sync_.registerCallback(boost::bind(&PointCloudCompassNode::callback, this, _1, _2));
        pd_pub_ = nh_.advertise<sbim_msgs::PrincipalDirections>("principal_directions", 10);
        rot_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("compass_transform", 10);

    }

    void loop() {

        ros::Rate rate(frequency_);
        while (ros::ok()) {

            rate.sleep();
            ros::spinOnce();

            if (message_queue_.size() < 1) {
                continue;
            }

            auto message = message_queue_.front();
            message_queue_.pop_front();

            PointCloud P_s = std::get<0>(message);

            geometry_msgs::PoseStamped pose_msg = std::get<1>(message);
            Eigen::Isometry3d G_ws;
            tf::poseMsgToEigen(pose_msg.pose, G_ws);
            Eigen::Matrix3f R_ws = G_ws.rotation().cast<float>();

            Eigen::Vector3f gravity;
            gravity << -R_ws(2, 0), -R_ws(2, 1), -R_ws(2, 2);

            Eigen::Matrix3f R_cs;
            std::vector<Eigen::Vector3f> directions;
            R_cs = compass_->principalDirections(P_s, R_ws, gravity, directions);

            publish(R_cs, directions, pose_msg.header.stamp);

        }
    }

    void callback(const PointCloud::ConstPtr &cloud_msg, const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

        message_queue_.emplace_back(*cloud_msg, *pose_msg);

        while (message_queue_.size() > queue_size_) {
            message_queue_.pop_front();
        }

    }

    void publish(const Eigen::Matrix3f &R, const std::vector<Eigen::Vector3f> &directions, const ros::Time stamp) {

        Eigen::Isometry3f G = Eigen::Isometry3f::Identity();
        G.rotate(R);

        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "compass";
        transform.header.stamp = stamp;
        transform.child_frame_id = "vehicle";
        tf::transformEigenToMsg(G.cast<double>(), transform.transform);
        rot_pub_.publish(transform);

        sbim_msgs::PrincipalDirections principal_directions;
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
    float frequency_;
    int queue_size_;

    std::deque<std::tuple<PointCloud, geometry_msgs::PoseStamped>> message_queue_;

    std::unique_ptr<Compass> compass_;

    message_filters::Subscriber<PointCloud> pc_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    message_filters::Synchronizer<Policy> pc_sync_;

    ros::Publisher pd_pub_;
    ros::Publisher rot_pub_;

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point_cloud_compass_node");

    PointCloudCompassNode<structural_compass::EntropyCompass<PointCloud>> compass_node;
    compass_node.loop();

    return 0;
}

