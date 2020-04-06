//
// Created by armon on 3/10/20.
//

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sbim_msgs/Trajectory.h>
#include <sbim_msgs/PrincipalPlaneArray.h>
#include <sbim_msgs/Door.h>
#include <sbim_msgs/DoorArray.h>
#include <pcl_ros/point_cloud.h>

#include <scene_parsing/door_detector.h>

typedef pcl::PointXYZ PointT;
typedef message_filters::sync_policies::ApproximateTime<sbim_msgs::Trajectory, sbim_msgs::PrincipalPlaneArray> Policy;

class DoorDetectorNode {

    ros::NodeHandle nh_;

    ros::Publisher pub_;
    ros::Subscriber pc_sub_;

    message_filters::Subscriber<sbim_msgs::Trajectory> traj_sub_;
    message_filters::Subscriber<sbim_msgs::PrincipalPlaneArray> plane_sub_;
    message_filters::Synchronizer<Policy> sync_;

    int frequency_;
    scene_parsing::DetectorParams detector_params_;

    int queue_size_;
    std::deque<std::tuple<sbim_msgs::Trajectory, sbim_msgs::PrincipalPlaneArray>> slam_message_queue_;
    std::deque<pcl::PointCloud<PointT>> pc_message_queue_;

public:

    ~DoorDetectorNode() = default;

    DoorDetectorNode(DoorDetectorNode &node) = delete;

    DoorDetectorNode() : nh_("~"),
                         frequency_(5),
                         queue_size_(1),
                         traj_sub_(nh_, "/planar_slam_node/trajectory", 1),
                         plane_sub_(nh_, "/planar_slam_node/layout_planes", 1),
                         sync_(Policy(10), traj_sub_, plane_sub_) {

        nh_.param<int>("queue_size", queue_size_, 1);
        nh_.param<int>("frequency", frequency_, 5);
        nh_.param<double>("distance_threshold", detector_params_.distance_threshold, 0.03);
        nh_.param<double>("min_door_intensity", detector_params_.min_peak_intensity, 1.8);
        nh_.param<double>("min_door_prominence", detector_params_.min_peak_prominence, 1.8);

        sync_.registerCallback(boost::bind(&DoorDetectorNode::slam_callback, this, _1, _2));

        pub_ = nh_.advertise<sbim_msgs::DoorArray>("doors", 10);
        pc_sub_ = nh_.subscribe<pcl::PointCloud<PointT>>("/compass_cloud_transformer/cloud", 1,
                                                         boost::bind(&DoorDetectorNode::pc_callback, this, _1));

    }

    void pc_callback(const pcl::PointCloud<PointT>::ConstPtr &cloud) {

        std::cout << "pc here" << std::endl;

        pc_message_queue_.push_back(*cloud);

        if (pc_message_queue_.size() > queue_size_) {
            pc_message_queue_.pop_front();
        }

    }

    void slam_callback(const sbim_msgs::Trajectory::ConstPtr &trajectory,
                       const sbim_msgs::PrincipalPlaneArray::ConstPtr &layout_planes) {

        std::cout << "slam here" << std::endl;

        slam_message_queue_.emplace_back(*trajectory, *layout_planes);

        if (slam_message_queue_.size() > queue_size_) {
            slam_message_queue_.pop_front();
        }

    }

    void loop() {

        scene_parsing::DoorDetector door_detector(detector_params_);

        ros::Rate rate(frequency_);
        while (ros::ok()) {

            rate.sleep();
            ros::spinOnce();

            if (slam_message_queue_.empty()) {
                continue;
            }

            if (pc_message_queue_.empty()) {
                continue;
            }

            // next message
            auto slam_message = slam_message_queue_.front();
            sbim_msgs::Trajectory trajectory = std::get<0>(slam_message);
            sbim_msgs::PrincipalPlaneArray plane_array = std::get<1>(slam_message);
            pcl::PointCloud<PointT> point_cloud = pc_message_queue_.front();
            slam_message_queue_.pop_front();
            pc_message_queue_.pop_front();

            std::vector<std::vector<double>> points;
            for (auto &pose : trajectory.poses) {
                std::vector<double> p = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
                points.push_back(p);
            }

            std::vector<std::vector<Eigen::Vector3f>> door_extents;
            std::vector<std::string> plane_ids;
            std::vector<std::string> detection_types;
            for (auto &plane : plane_array.planes) {

                // skip z-planes
                if (plane.label.data == "0") {
                    continue;
                }

                std::vector<double> p = {plane.plane.coef[0], plane.plane.coef[1], plane.plane.coef[2],
                                         plane.plane.coef[3]};

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

            sbim_msgs::DoorArray door_array_msg;
            for (size_t j = 0; j < door_extents.size(); ++j) {
                sbim_msgs::Door door_msg;
                door_msg.header = plane_array.header;
                door_msg.plane_id.data = plane_ids[j];
                door_msg.detection_type.data = detection_types[j];
                for (auto &v : door_extents[j]) {
                    geometry_msgs::Point point;
                    point.x = v[0];
                    point.y = v[1];
                    point.z = v[2];
                    door_msg.vertices.push_back(point);
                }
                door_array_msg.doors.push_back(door_msg);
            }

            pub_.publish(door_array_msg);

        }

    }

};


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "door_detector_node");

    DoorDetectorNode door_detector_node;
    door_detector_node.loop();

    return 0;
}
