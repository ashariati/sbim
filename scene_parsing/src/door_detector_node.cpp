#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sbim_msgs/msg/trajectory.hpp>
#include <sbim_msgs/msg/principal_plane_array.hpp>
#include <sbim_msgs/msg/door.hpp>
#include <sbim_msgs/msg/door_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <scene_parsing/door_detector.h>

typedef pcl::PointXYZ PointT;
typedef message_filters::sync_policies::ApproximateTime<sbim_msgs::msg::Trajectory, sbim_msgs::msg::PrincipalPlaneArray>
Policy;

class DoorDetectorNode : public rclcpp::Node {
public:
    DoorDetectorNode()
        : Node("door_detector_node"),
          frequency_(5),
          queue_size_(1),
          traj_sub_(this, "/planar_slam_node/trajectory", rmw_qos_profile_default),
          plane_sub_(this, "/planar_slam_node/layout_planes", rmw_qos_profile_default),
          sync_(Policy(10), traj_sub_, plane_sub_) {
        this->declare_parameter<int>("queue_size", 1);
        this->declare_parameter<int>("frequency", 5);
        this->declare_parameter<double>("distance_threshold", 0.03);
        this->declare_parameter<double>("min_door_intensity", 1.8);
        this->declare_parameter<double>("min_door_prominence", 1.8);

        this->get_parameter("queue_size", queue_size_);
        this->get_parameter("frequency", frequency_);
        this->get_parameter("distance_threshold", detector_params_.distance_threshold);
        this->get_parameter("min_door_intensity", detector_params_.min_peak_intensity);
        this->get_parameter("min_door_prominence", detector_params_.min_peak_prominence);

        sync_.registerCallback(std::bind(&DoorDetectorNode::slam_callback, this, std::placeholders::_1,
                                         std::placeholders::_2));

        pub_ = this->create_publisher<sbim_msgs::msg::DoorArray>("doors", 10);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/compass_cloud_transformer/cloud", 1,
            std::bind(&DoorDetectorNode::pc_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / frequency_),
            std::bind(&DoorDetectorNode::loop, this));
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
        pcl::PointCloud<PointT> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        pc_message_queue_.push_back(cloud);

        if (pc_message_queue_.size() > queue_size_) {
            pc_message_queue_.pop_front();
        }
    }

    void slam_callback(const sbim_msgs::msg::Trajectory::ConstSharedPtr trajectory,
                       const sbim_msgs::msg::PrincipalPlaneArray::ConstSharedPtr layout_planes) {
        slam_message_queue_.emplace_back(*trajectory, *layout_planes);

        if (slam_message_queue_.size() > queue_size_) {
            slam_message_queue_.pop_front();
        }
    }

    void loop() {
        scene_parsing::DoorDetector door_detector(detector_params_);

        if (slam_message_queue_.empty() || pc_message_queue_.empty()) {
            return;
        }

        auto slam_message = slam_message_queue_.front();
        sbim_msgs::msg::Trajectory trajectory = std::get<0>(slam_message);
        sbim_msgs::msg::PrincipalPlaneArray plane_array = std::get<1>(slam_message);
        pcl::PointCloud<PointT> point_cloud = pc_message_queue_.front();
        slam_message_queue_.pop_front();
        pc_message_queue_.pop_front();

        std::vector<std::vector<double> > points;
        for (auto &pose: trajectory.poses) {
            std::vector<double> p = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
            points.push_back(p);
        }

        std::vector<std::vector<Eigen::Vector3f> > door_extents;
        std::vector<std::string> plane_ids;
        std::vector<std::string> detection_types;
        for (auto &plane: plane_array.planes) {
            // skip z-planes
            if (plane.label.data == "0") {
                continue;
            }

            std::vector<double> p = {
                plane.plane.coef[0], plane.plane.coef[1], plane.plane.coef[2], plane.plane.coef[3]
            };

            size_t num_cloud_doors = door_detector.detectDoorsFromCloud(point_cloud, p, door_extents);
            for (size_t i = 0; i < num_cloud_doors; ++i) {
                plane_ids.push_back(plane.id.data);
                detection_types.emplace_back("cloud");
            }

            // size_t num_crossing_doors = door_detector.detectDoorsFromCrossing(points, p, door_extents);
            // for (size_t i = 0; i < num_crossing_doors; ++i) {
            //     plane_ids.push_back(plane.id.data);
            //     detection_types.emplace_back("crossing");
            // }
        }

        sbim_msgs::msg::DoorArray door_array_msg;
        for (size_t j = 0; j < door_extents.size(); ++j) {
            sbim_msgs::msg::Door door_msg;
            door_msg.header = plane_array.header;
            door_msg.plane_id.data = plane_ids[j];
            door_msg.detection_type.data = detection_types[j];
            for (auto &v: door_extents[j]) {
                geometry_msgs::msg::Point point;
                point.x = v[0];
                point.y = v[1];
                point.z = v[2];
                door_msg.vertices.push_back(point);
            }
            door_array_msg.doors.push_back(door_msg);
        }

        pub_->publish(door_array_msg);
    }

    rclcpp::Publisher<sbim_msgs::msg::DoorArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    message_filters::Subscriber<sbim_msgs::msg::Trajectory> traj_sub_;
    message_filters::Subscriber<sbim_msgs::msg::PrincipalPlaneArray> plane_sub_;
    message_filters::Synchronizer<Policy> sync_;
    rclcpp::TimerBase::SharedPtr timer_;

    int frequency_;
    int queue_size_;
    scene_parsing::DetectorParams detector_params_;
    std::deque<std::tuple<sbim_msgs::msg::Trajectory, sbim_msgs::msg::PrincipalPlaneArray> > slam_message_queue_;
    std::deque<pcl::PointCloud<PointT> > pc_message_queue_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DoorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
