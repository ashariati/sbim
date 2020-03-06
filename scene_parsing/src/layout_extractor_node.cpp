//
// Created by armon on 2/7/20.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <sbim_msgs/PrincipalPlaneArray.h>

#include <vision_msgs/Detection3DArray.h>
#include <sbim_msgs/LayoutSegmentArray.h>

#include <scene_parsing/layout_extractor.h>

typedef pcl::PointXYZ PointT;
typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<PointT>, sbim_msgs::PrincipalPlaneArray> Policy;

class LayoutExtractorNode {

    ros::NodeHandle nh_;
    ros::Publisher object_pub_;
    ros::Publisher object_cloud_pub_;
    ros::Publisher segment_pub_;
    ros::Publisher segment_cloud_pub_;
    message_filters::Subscriber<pcl::PointCloud<PointT>> pc_sub_;
    message_filters::Subscriber<sbim_msgs::PrincipalPlaneArray> plane_sub_;
    message_filters::Synchronizer<Policy> sync_;

    std::deque<std::pair<pcl::PointCloud<PointT>, sbim_msgs::PrincipalPlaneArray>> message_queue_;

    int frequency_;
    int queue_size_;

    layout_extractor::ExtractorParams extractor_params_;
    double boundary_alpha_;
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

        nh_.param<double>("distance_threshold", extractor_params_.distance_threshold, 0.03);
        nh_.param<double>("cluster_tolerance", extractor_params_.cluster_tolerance, 0.05);
        nh_.param<int>("min_cluster_size", extractor_params_.min_cluster_size, 1000);
        nh_.param<int>("max_cluster_size", extractor_params_.max_cluster_size, -1);

        nh_.param<double>("filter_leaf_size", filter_leaf_size_, 0.02);

        object_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("objects", 10);
        object_cloud_pub_ = nh_.advertise<pcl::PointCloud<PointT>>("object_cloud", 10);
        segment_pub_ = nh_.advertise<sbim_msgs::LayoutSegmentArray>("layout_segments", 10);
        segment_cloud_pub_ = nh_.advertise<pcl::PointCloud<PointT>>("cloud_segments", 10);

        sync_.registerCallback(boost::bind(&LayoutExtractorNode::callback, this, _1, _2));

    }

    void callback(const pcl::PointCloud<PointT>::ConstPtr &cloud,
                  const sbim_msgs::PrincipalPlaneArray::ConstPtr &layout_planes) {

        message_queue_.emplace_back(*cloud, *layout_planes);

        if (message_queue_.size() > queue_size_) {
            message_queue_.pop_front();
        }

    }

    void loop() {

        layout_extractor::LayoutExtractor extractor(extractor_params_);
        layout_extractor::LayoutSummarizer summarizer;

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
            pcl::PointCloud<PointT> point_cloud = message.first;
            sbim_msgs::PrincipalPlaneArray plane_array = message.second;

            // filter point cloud
            pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
            if (filter_leaf_size_ > 0) {
                *filtered_cloud = extractor.filterPointCloud(point_cloud, filter_leaf_size_);
            } else {
                *filtered_cloud = point_cloud;
            }

            pcl::PointCloud<PointT> all_cloud_segments;
            all_cloud_segments.header = point_cloud.header;

            std::vector<int> all_inliers;

            // instantiate message
            sbim_msgs::LayoutSegmentArray layout_segment_array;

            // for each plane
            for (auto &p: plane_array.planes) {

                std::vector<double> plane_coeff = {p.plane.coef[0], p.plane.coef[1], p.plane.coef[2], p.plane.coef[3]};

                // extract layout segments
                std::vector<pcl::PointCloud<PointT>> cloud_segments;
                std::vector<int> inliers = extractor.extractSegmentsAtPlane(*filtered_cloud, plane_coeff,
                                                                            cloud_segments);

                // save inliers
                all_inliers.reserve(all_inliers.size() + inliers.size());
                all_inliers.insert(all_inliers.end(), inliers.begin(), inliers.end());

                // convert to and save summarized layout segments
                if (p.label.data == "0") {

                    for (auto &s : cloud_segments) {
                        sbim_msgs::LayoutSegment layout_segment;
                        layout_segment.header.frame_id = plane_array.header.frame_id;
                        layout_segment.header.stamp = plane_array.header.stamp;
                        layout_segment.plane_id = p.id;
                        layout_segment.label = p.label;

                        std::vector<std::vector<double>> segment_vertices = summarizer.ellipsoidSummary(s);
                        layout_segment.vertices = verticesToPointArray(segment_vertices);

                        layout_segment_array.layout_segments.push_back(layout_segment);
                    }

                } else {

                    for (auto &s : cloud_segments) {
                        sbim_msgs::LayoutSegment layout_segment;
                        layout_segment.header.frame_id = plane_array.header.frame_id;
                        layout_segment.header.stamp = plane_array.header.stamp;
                        layout_segment.plane_id = p.id;
                        layout_segment.label = p.label;

                        std::vector<std::vector<double>> segment_vertices = summarizer.rectangleSummary(s);
                        layout_segment.vertices = verticesToPointArray(segment_vertices);

                        layout_segment_array.layout_segments.push_back(layout_segment);
                    }

                }

                // cloud segments
                for (auto &segment : cloud_segments) {
                    all_cloud_segments += segment;
                }

            }

            // remove repeated inliers
            std::sort(all_inliers.begin(), all_inliers.end());
            auto ip = std::unique(all_inliers.begin(), all_inliers.end());
            all_inliers.resize(std::distance(all_inliers.begin(), ip));

            // filter for objects
            pcl::PointCloud<PointT> object_cloud;
            pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
            inliers_ptr->indices = all_inliers;
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(filtered_cloud);
            extract.setNegative(true);
            extract.setIndices(inliers_ptr);
            extract.filter(object_cloud);
            object_cloud.header = point_cloud.header;


            // publish
            object_cloud_pub_.publish(object_cloud);
            segment_pub_.publish(layout_segment_array);
            segment_cloud_pub_.publish(all_cloud_segments);

        }

    }

    static std::vector<geometry_msgs::Point> verticesToPointArray(const std::vector<std::vector<double>> &vertices) {

        std::vector<geometry_msgs::Point> points;
        for (auto &v : vertices) {
            geometry_msgs::Point point;
            point.x = v[0];
            point.y = v[1];
            point.z = v[2];
            points.push_back(point);
        }

        return points;
    }

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "layout_extractor_node");

    LayoutExtractorNode layout_extractor_node;
    layout_extractor_node.loop();

    return 0;

}

