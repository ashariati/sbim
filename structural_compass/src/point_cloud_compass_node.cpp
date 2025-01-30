//
// Created by armon on 1/16/20.
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <deque>
#include <memory>
#include <sbim_msgs/msg/principal_direction_array.hpp>
#include <structural_compass/structural_compass.h>

using namespace std::chrono_literals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, geometry_msgs::msg::PoseStamped> Policy;

class PointCloudCompassNode : public rclcpp::Node {

    private:
        message_filters::Subscriber<PointCloud> pc_sub_;
        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
        message_filters::Synchronizer<Policy> pc_sync_;

        rclcpp::Publisher<sbim_msgs::msg::PrincipalDirectionArray>::SharedPtr pd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr rot_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

        bool manhattan_world_;

        float frequency_;
        int queue_size_;
        std::deque<std::tuple<PointCloud, geometry_msgs::msg::PoseStamped>> message_queue_;

        std::unique_ptr<structural_compass::EntropyCompass> compass_;

    public:
        PointCloudCompassNode() : Node("point_cloud_compass_node"),
        pc_sub_(this, "/scan", rmw_qos_profile_sensor_data),
        pose_sub_(this, "/pose", rmw_qos_profile_sensor_data),
        pc_sync_(Policy(10), pc_sub_, pose_sub_) {


            this->declare_parameter("frequency", 10.0);
            this->declare_parameter("queue_size", 1);
            this->declare_parameter("manhattan_world", false);

            frequency_ = this->get_parameter("frequency").as_double();
            queue_size_ = this->get_parameter("queue_size").as_int();
            manhattan_world_ = this->get_parameter("manhattan_world").as_bool();


            compass_ = std::make_unique<structural_compass::EntropyCompass>();
            pc_sync_.registerCallback(std::bind(&PointCloudCompassNode::callback, this, std::placeholders::_1, std::placeholders::_2));

            pd_pub_ = this->create_publisher<sbim_msgs::msg::PrincipalDirectionArray>("principal_directions", 10);
            rot_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("compass_transform", 10);
            pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_scan", 10);

            RCLCPP_INFO(this->get_logger(), "PointCloudCompassNode initialized.");

        }

        void loop() {

            rclcpp::Rate rate(frequency_);
            while (rclcpp::ok()) {

                rate.sleep();
                rclcpp::spin_some(this->get_node_base_interface());

                if (message_queue_.empty()) {
                    continue;
                }

                auto message = message_queue_.front();
                message_queue_.pop_front();

                PointCloud P_s = std::get<0>(message);
                geometry_msgs::msg::PoseStamped pose_msg = std::get<1>(message);

                Eigen::Isometry3d G_ws;
                tf2::fromMsg(pose_msg.pose, G_ws);
                Eigen::Matrix3f R_ws = G_ws.rotation().cast<float>();

                Eigen::Vector3f gravity;
                gravity << -R_ws(2, 0), -R_ws(2, 1), -R_ws(2, 2);

                Eigen::Matrix3f R_cs;
                std::vector<Eigen::Vector3f> directions;
                R_cs = compass_->principalDirections(P_s, R_ws, gravity, directions);

                if (manhattan_world_) {
                    directions.resize(3);
                }

                Eigen::Isometry3f G_cs = Eigen::Isometry3f::Identity();
                G_cs.rotate(R_cs);

                PointCloud P_c;
                pcl::transformPointCloud(P_s, P_c, G_cs);

                publish(P_c, R_cs, directions, pose_msg.header.stamp);

            }
        }

        void callback(const PointCloud::ConstPtr &cloud_msg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg) {

            message_queue_.emplace_back(*cloud_msg, *pose_msg);

            if (message_queue_.size() > queue_size_) {
                message_queue_.pop_front();
            }

        }

        void publish(const PointCloud &P, const Eigen::Matrix3f &R, const std::vector<Eigen::Vector3f> &directions, rclcpp::Time stamp) {

            Eigen::Isometry3f G = Eigen::Isometry3f::Identity();
            G.rotate(R);

            geometry_msgs::msg::TransformStamped transform;
            transform.header.frame_id = "compass";
            transform.header.stamp = stamp;
            transform.child_frame_id = "vehicle";

            transform.transform.translation.x = G.translation().x();
            transform.transform.translation.y = G.translation().y();
            transform.transform.translation.z = G.translation().z();

            Eigen::Quaternionf quat(G.rotation());
            transform.transform.rotation.x = quat.x();
            transform.transform.rotation.y = quat.y();
            transform.transform.rotation.z = quat.z();
            transform.transform.rotation.w = quat.w();

            // tf2::toMsg(G, transform.transform);
            // transform.transform = tf2::toMsg(static_cast<Eigen::Isometry3d>(G.cast<double>()));
            // transform.transform = tf2::toMsg(G.cast<double>());
            rot_pub_->publish(transform);

            sbim_msgs::msg::PrincipalDirectionArray principal_directions;
            principal_directions.header.frame_id = "compass";
            principal_directions.header.stamp = stamp;
            for (auto &d : directions) {
                geometry_msgs::msg::Vector3 v;
                v.x = d.x();
                v.y = d.y();
                v.z = d.z();
                principal_directions.directions.push_back(v);
            }
            pd_pub_->publish(principal_directions);

            sensor_msgs::msg::PointCloud2 cloud_out;
            pcl::toROSMsg(P, cloud_out);
            cloud_out.header.frame_id = "compass";
            cloud_out.header.stamp = stamp;
            pc_pub_->publish(cloud_out);


        }

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudCompassNode>();
    node->loop();
    rclcpp::shutdown();
    return 0;
}

