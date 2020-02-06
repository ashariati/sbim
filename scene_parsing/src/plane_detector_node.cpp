//
// Created by armon on 2/1/20.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <sbim_msgs/PrincipalDirectionArray.h>
#include <sbim_msgs/PrincipalPlaneArray.h>

#include <scene_parsing/plane_detector.h>
#include <eigen_conversions/eigen_msg.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, sbim_msgs::PrincipalDirectionArray> Policy;

class PlaneDetectorNode {

public:

    ~PlaneDetectorNode() = default;

    PlaneDetectorNode() : nh_("~"),
                          pc_sub_(nh_, "/scan", 1),
                          pd_sub_(nh_, "/principal_directions", 1),
                          sync_(Policy(10), pc_sub_, pd_sub_),
                          plane_detector_(),
                          plane_count_(0),
                          frequency_(10),
                          queue_size_(1),
                          scan_range_(10.0),
                          min_intensity_(1000) {

        nh_.param<int>("frequency", frequency_, 10);
        nh_.param<int>("queue_size", queue_size_, 1);
        nh_.param<float>("scan_range", scan_range_, 10.0);
        nh_.param<int>("min_intensity", min_intensity_, 1000);

        sync_.registerCallback(boost::bind(&PlaneDetectorNode::callback, this, _1, _2));
        pub_ = nh_.advertise<sbim_msgs::PrincipalPlaneArray>("planes", 10);

    }

    void
    callback(const PointCloud::ConstPtr &point_cloud, const sbim_msgs::PrincipalDirectionArray::ConstPtr &directions) {

        message_queue_.emplace_back(*point_cloud, *directions);

        if (message_queue_.size() > queue_size_) {
            message_queue_.pop_front();
        }

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

            PointCloud P = std::get<0>(message);

            sbim_msgs::PrincipalDirectionArray pd_msg = std::get<1>(message);
            std::vector<Eigen::Vector3f> directions;
            for (auto e : pd_msg.directions) {
                Eigen::Vector3d v;
                tf::vectorMsgToEigen(e, v);
                directions.emplace_back(v.cast<float>());
            }

            sbim_msgs::PrincipalPlaneArray principal_planes;
            principal_planes.header.frame_id = pd_msg.header.frame_id;
            principal_planes.header.stamp = pd_msg.header.stamp;
            int direction_index = 0;
            for (auto v : directions) {

                std::vector<float> offsets;
                std::vector<double> intensities;
                plane_detector_.scanDirection(P, v, scan_range_, min_intensity_,
                                              offsets, intensities);

                for (size_t i = 0; i < offsets.size(); ++i) {


                    shape_msgs::Plane plane;
                    plane.coef[0] = v[0];
                    plane.coef[1] = v[1];
                    plane.coef[2] = v[2];
                    plane.coef[3] = offsets[i];

                    std_msgs::Float64 plane_intensity;
                    plane_intensity.data = intensities[i];

                    std_msgs::String plane_label;
                    plane_label.data = std::to_string(direction_index);

                    std_msgs::String plane_id;
                    plane_id.data = std::to_string(plane_count_);

                    sbim_msgs::PrincipalPlane principal_plane;
                    principal_plane.plane = plane;
                    principal_plane.intensity = plane_intensity;
                    principal_plane.label = plane_label;
                    principal_plane.id = plane_id;

                    principal_planes.planes.push_back(principal_plane);

                    plane_count_ += 1;

                }

                direction_index += 1;
            }

            pub_.publish(principal_planes);

        }

    }

private:

    ros::NodeHandle nh_;
    message_filters::Subscriber<PointCloud> pc_sub_;
    message_filters::Subscriber<sbim_msgs::PrincipalDirectionArray> pd_sub_;
    message_filters::Synchronizer<Policy> sync_;
    ros::Publisher pub_;

    int frequency_{};
    int queue_size_{};
    std::deque<std::tuple<PointCloud, sbim_msgs::PrincipalDirectionArray>> message_queue_;

    PlaneDetector<PointCloud> plane_detector_;

    size_t plane_count_;
    int min_intensity_{};
    float scan_range_{};

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "plane_detector_node");

    PlaneDetectorNode plane_detector_node;
    plane_detector_node.loop();

    return 0;

}
