#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <sbim_msgs/msg/principal_direction_array.hpp>
#include <sbim_msgs/msg/principal_plane_array.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <scene_parsing/plane_detector.h>
#include <tf2_eigen/tf2_eigen.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
    sbim_msgs::msg::PrincipalDirectionArray> Policy;

class PlaneDetectorNode : public rclcpp::Node {
public:
    PlaneDetectorNode()
        : Node("plane_detector_node"),
          pc_sub_(this, "/scan", rmw_qos_profile_default),
          pd_sub_(this, "/principal_directions", rmw_qos_profile_default),
          sync_(Policy(10), pc_sub_, pd_sub_),
          plane_detector_(),
          plane_count_(0),
          frequency_(10),
          queue_size_(1),
          scan_range_(10.0),
          min_intensity_(1000) {
        this->declare_parameter<int>("frequency", 10);
        this->declare_parameter<int>("queue_size", 1);
        this->declare_parameter<float>("scan_range", 10.0);
        this->declare_parameter<int>("min_intensity", 1000);

        this->get_parameter("frequency", frequency_);
        this->get_parameter("queue_size", queue_size_);
        this->get_parameter("scan_range", scan_range_);
        this->get_parameter("min_intensity", min_intensity_);

        sync_.registerCallback(std::bind(&PlaneDetectorNode::callback, this, std::placeholders::_1,
                                         std::placeholders::_2));
        pub_ = this->create_publisher<sbim_msgs::msg::PrincipalPlaneArray>("planes", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / frequency_),
            std::bind(&PlaneDetectorNode::loop, this));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                  const sbim_msgs::msg::PrincipalDirectionArray::ConstSharedPtr &directions_msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        message_queue_.emplace_back(cloud, *directions_msg);

        if (message_queue_.size() > queue_size_) {
            message_queue_.pop_front();
        }
    }

    void loop() {
        if (message_queue_.empty()) {
            return;
        }

        auto message = message_queue_.front();
        message_queue_.pop_front();

        pcl::PointCloud<pcl::PointXYZ> P = std::get<0>(message);
        sbim_msgs::msg::PrincipalDirectionArray pd_msg = std::get<1>(message);

        std::vector<Eigen::Vector3f> directions;
        for (auto e: pd_msg.directions) {
            Eigen::Vector3d v;
            tf2::fromMsg(e, v);
            directions.emplace_back(v.cast<float>());
        }

        sbim_msgs::msg::PrincipalPlaneArray principal_planes;
        principal_planes.header.frame_id = pd_msg.header.frame_id;
        principal_planes.header.stamp = pd_msg.header.stamp;
        int direction_index = 0;
        for (auto v: directions) {
            std::vector<float> offsets;
            std::vector<double> intensities;
            plane_detector_.scanDirection<pcl::PointXYZ>(P, v, scan_range_, min_intensity_, offsets, intensities);

            for (size_t i = 0; i < offsets.size(); ++i) {
                shape_msgs::msg::Plane plane;
                plane.coef[0] = v[0];
                plane.coef[1] = v[1];
                plane.coef[2] = v[2];
                plane.coef[3] = -offsets[i];

                std_msgs::msg::Float64 plane_intensity;
                plane_intensity.data = intensities[i];

                std_msgs::msg::String plane_label;
                plane_label.data = std::to_string(direction_index);

                std_msgs::msg::String plane_id;
                plane_id.data = std::to_string(plane_count_);

                sbim_msgs::msg::PrincipalPlane principal_plane;
                principal_plane.plane = plane;
                principal_plane.intensity = plane_intensity;
                principal_plane.label = plane_label;
                principal_plane.id = plane_id;

                principal_planes.planes.push_back(principal_plane);
                plane_count_ += 1;
            }

            direction_index += 1;
        }

        pub_->publish(principal_planes);
    }

    rclcpp::Publisher<sbim_msgs::msg::PrincipalPlaneArray>::SharedPtr pub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
    message_filters::Subscriber<sbim_msgs::msg::PrincipalDirectionArray> pd_sub_;
    message_filters::Synchronizer<Policy> sync_;
    rclcpp::TimerBase::SharedPtr timer_;

    int frequency_;
    int queue_size_;
    float scan_range_;
    int min_intensity_;
    std::deque<std::tuple<pcl::PointCloud<pcl::PointXYZ>, sbim_msgs::msg::PrincipalDirectionArray> > message_queue_;
    PlaneDetector plane_detector_;
    size_t plane_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
