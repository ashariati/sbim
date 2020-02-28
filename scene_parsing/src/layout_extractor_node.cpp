//
// Created by armon on 2/7/20.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sbim_msgs/PrincipalPlaneArray.h>

#include <vision_msgs/Detection3DArray.h>
#include <sbim_msgs/LayoutSegmentArray.h>

#include <scene_parsing/layout_extractor.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<PointCloud, sbim_msgs::PrincipalPlaneArray> Policy;

class LayoutExtractorNode {

    ros::NodeHandle nh_;
    ros::Publisher object_pub_;
    ros::Publisher segment_pub_;
    ros::Publisher cloud_pub_;
    message_filters::Subscriber<PointCloud> pc_sub_;
    message_filters::Subscriber<sbim_msgs::PrincipalPlaneArray> plane_sub_;
    message_filters::Synchronizer<Policy> sync_;

    std::deque<std::pair<PointCloud, sbim_msgs::PrincipalPlaneArray>> message_queue_;

    int frequency_;
    int queue_size_;

    layout_extractor::Params params_;
    double filter_leaf_size_;

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

        nh_.param<double>("distance_threshold", params_.distance_threshold, 0.03);
        nh_.param<double>("cluster_tolerance", params_.cluster_tolerance, 0.05);
        nh_.param<int>("min_cluster_size", params_.min_cluster_size, 1000);
        nh_.param<int>("max_cluster_size", params_.max_cluster_size, -1);

        nh_.param<double>("filter_leaf_size", filter_leaf_size_, 0.02);

        object_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("objects", 10);
        segment_pub_ = nh_.advertise<sbim_msgs::LayoutSegmentArray>("layout_segments", 10);
        cloud_pub_ = nh_.advertise<PointCloud>("cloud_segments", 10);

        sync_.registerCallback(boost::bind(&LayoutExtractorNode::callback, this, _1, _2));

    }

    void callback(const PointCloud::ConstPtr &cloud, const sbim_msgs::PrincipalPlaneArray::ConstPtr &layout_planes) {

        message_queue_.emplace_back(*cloud, *layout_planes);

        if (message_queue_.size() > queue_size_) {
            message_queue_.pop_front();
        }

    }

    void loop() {

        layout_extractor::LayoutExtractor extractor(params_);

        ros::Rate rate(frequency_);
        while (ros::ok()) {

            rate.sleep();
            ros::spinOnce();

            if (message_queue_.empty()) {
                continue;
            }

            // next message
            auto message = message_queue_.front();
            message_queue_.pop_front();
            PointCloud point_cloud = message.first;
            sbim_msgs::PrincipalPlaneArray plane_array = message.second;

            // filter point cloud
            PointCloud filtered_cloud = extractor.filterPointCloud(point_cloud, filter_leaf_size_);
            // PointCloud filtered_cloud = point_cloud;

            // for each plane
            for (auto p: plane_array.planes) {

                std::vector<double> plane_coeff = {p.plane.coef[0], p.plane.coef[1], p.plane.coef[2], p.plane.coef[3]};

                // extract layout segments
                std::vector<PointCloud> plane_segments;
                plane_segments = extractor.extractSegmentsAtPlane(filtered_cloud, plane_coeff);

                // publish
                for (auto segment : plane_segments) {
                    segment.header.stamp = point_cloud.header.stamp;
                    segment.header.frame_id = point_cloud.header.frame_id;
                    cloud_pub_.publish(segment);
                }

            }


        }

    }

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "layout_extractor_node");

    LayoutExtractorNode layout_extractor_node;
    layout_extractor_node.loop();

    return 0;

}

