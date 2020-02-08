//
// Created by armon on 2/7/20.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>

#include <sbim_msgs/PrincipalPlaneArray.h>

#include <vision_msgs/Detection3DArray.h>
#include <sbim_msgs/LayoutSegmentArray.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, sbim_msgs::PrincipalPlaneArray> Policy;

class LayoutExtractorNode {

    ros::NodeHandle nh_;
    ros::Publisher object_pub_;
    ros::Publisher segment_pub_;
    message_filters::Subscriber<PointCloud> pc_sub_;
    message_filters::Subscriber<sbim_msgs::PrincipalPlaneArray> plane_sub_;
    message_filters::Synchronizer<Policy> sync_;

    std::deque<std::tuple<PointCloud, sbim_msgs::PrincipalPlaneArray>> message_queue_;

    int frequency_;
    int queue_size_;

public:

    ~LayoutExtractorNode() = default;

    LayoutExtractorNode() : nh_("~"),
                            pc_sub_(nh_, "/transformed_scan", 1),
                            plane_sub_(nh_, "/layout_planes", 1),
                            sync_(Policy(10), pc_sub_, plane_sub_),
                            frequency_(10),
                            queue_size_(1) {

        nh_.param<int>("frequency", frequency_, 10);
        nh_.param<int>("queue_size", queue_size_, 1);

        segment_pub_ = nh_.advertise<sbim_msgs::LayoutSegmentArray>("layout_segments", 10);
        object_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("objects", 10);

        sync_.registerCallback(boost::bind(&LayoutExtractorNode::callback, this, _1, _2));

    }

    void callback(const PointCloud::ConstPtr &cloud, const sbim_msgs::PrincipalPlaneArray::ConstPtr &layout_planes) {

        message_queue_.emplace_back(*cloud, *layout_planes);

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

        }

    }

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "layout_extractor_node");

    LayoutExtractorNode layout_extractor_node;
    layout_extractor_node.loop();

    return 0;

}
