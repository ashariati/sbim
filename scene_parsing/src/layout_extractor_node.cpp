#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <sbim_msgs/msg/principal_plane_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <sbim_msgs/msg/layout_segment_array.hpp>

#include <scene_parsing/layout_extractor.h>

typedef pcl::PointXYZ PointT;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
    sbim_msgs::msg::PrincipalPlaneArray> Policy;

class LayoutExtractorNode : public rclcpp::Node {
public:
    LayoutExtractorNode()
        : Node("layout_extractor_node"),
          pc_sub_(this, "/transformed_scan", rmw_qos_profile_default),
          plane_sub_(this, "/layout_planes", rmw_qos_profile_default),
          sync_(Policy(10), pc_sub_, plane_sub_),
          frequency_(10),
          queue_size_(1) {
        this->declare_parameter<int>("frequency", 10);
        this->declare_parameter<int>("queue_size", 1);

        this->declare_parameter<double>("distance_threshold", 0.03);
        this->declare_parameter<double>("cluster_tolerance", 0.05);
        this->declare_parameter<int>("min_cluster_size", 1000);
        this->declare_parameter<int>("max_cluster_size", -1);

        this->declare_parameter<double>("filter_leaf_size", 0.02);

        this->get_parameter("frequency", frequency_);
        this->get_parameter("queue_size", queue_size_);

        this->get_parameter("distance_threshold", extractor_params_.distance_threshold);
        this->get_parameter("cluster_tolerance", extractor_params_.cluster_tolerance);
        this->get_parameter("min_cluster_size", extractor_params_.min_cluster_size);
        this->get_parameter("max_cluster_size", extractor_params_.max_cluster_size);

        this->get_parameter("filter_leaf_size", filter_leaf_size_);

        object_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("objects", 10);
        object_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_cloud", 10);
        segment_pub_ = this->create_publisher<sbim_msgs::msg::LayoutSegmentArray>("layout_segments", 10);
        segment_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_segments", 10);

        sync_.registerCallback(std::bind(&LayoutExtractorNode::callback, this, std::placeholders::_1,
                                         std::placeholders::_2));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / frequency_),
            std::bind(&LayoutExtractorNode::loop, this));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
                  const sbim_msgs::msg::PrincipalPlaneArray::ConstSharedPtr layout_planes_msg) {
        pcl::PointCloud<PointT> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        message_queue_.emplace_back(cloud, *layout_planes_msg);

        if (message_queue_.size() > queue_size_) {
            message_queue_.pop_front();
        }
    }

    void loop() {
        layout_extractor::LayoutExtractor extractor(extractor_params_);
        layout_extractor::LayoutSummarizer summarizer;

        if (message_queue_.empty()) {
            return;
        }

        // next message
        auto message = message_queue_.front();
        message_queue_.pop_front();
        pcl::PointCloud<PointT> point_cloud = message.first;
        sbim_msgs::msg::PrincipalPlaneArray plane_array = message.second;

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
        sbim_msgs::msg::LayoutSegmentArray layout_segment_array;

        // for each plane
        for (auto &p: plane_array.planes) {
            std::vector<double> plane_coeff = {p.plane.coef[0], p.plane.coef[1], p.plane.coef[2], p.plane.coef[3]};

            // extract layout segments
            std::vector<pcl::PointCloud<PointT> > cloud_segments;
            std::vector<int> inliers = extractor.extractSegmentsAtPlane(*filtered_cloud, plane_coeff,
                                                                        cloud_segments);

            // save inliers
            all_inliers.reserve(all_inliers.size() + inliers.size());
            all_inliers.insert(all_inliers.end(), inliers.begin(), inliers.end());

            // convert to and save summarized layout segments
            if (p.label.data == "0") {
                for (auto &s: cloud_segments) {
                    sbim_msgs::msg::LayoutSegment layout_segment;
                    layout_segment.header.frame_id = plane_array.header.frame_id;
                    layout_segment.header.stamp = plane_array.header.stamp;
                    layout_segment.plane_id = p.id;
                    layout_segment.label = p.label;

                    std::vector<std::vector<double> > segment_vertices = summarizer.ellipsoidSummary(s);
                    layout_segment.vertices = verticesToPointArray(segment_vertices);

                    layout_segment_array.layout_segments.push_back(layout_segment);
                }
            } else {
                for (auto &s: cloud_segments) {
                    sbim_msgs::msg::LayoutSegment layout_segment;
                    layout_segment.header.frame_id = plane_array.header.frame_id;
                    layout_segment.header.stamp = plane_array.header.stamp;
                    layout_segment.plane_id = p.id;
                    layout_segment.label = p.label;

                    std::vector<std::vector<double> > segment_vertices = summarizer.rectangleSummary(s);
                    layout_segment.vertices = verticesToPointArray(segment_vertices);

                    layout_segment_array.layout_segments.push_back(layout_segment);
                }
            }

            // cloud segments
            for (auto &segment: cloud_segments) {
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
        sensor_msgs::msg::PointCloud2 object_cloud_msg;
        pcl::toROSMsg(object_cloud, object_cloud_msg);
        object_cloud_pub_->publish(object_cloud_msg);

        segment_pub_->publish(layout_segment_array);

        sensor_msgs::msg::PointCloud2 all_cloud_segments_msg;
        pcl::toROSMsg(all_cloud_segments, all_cloud_segments_msg);
        segment_cloud_pub_->publish(all_cloud_segments_msg);
    }

    static std::vector<geometry_msgs::msg::Point> verticesToPointArray(
        const std::vector<std::vector<double> > &vertices) {
        std::vector<geometry_msgs::msg::Point> points;
        for (const auto &v: vertices) {
            geometry_msgs::msg::Point point;
            point.x = v[0];
            point.y = v[1];
            point.z = v[2];
            points.push_back(point);
        }
        return points;
    }

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr object_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_cloud_pub_;
    rclcpp::Publisher<sbim_msgs::msg::LayoutSegmentArray>::SharedPtr segment_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segment_cloud_pub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
    message_filters::Subscriber<sbim_msgs::msg::PrincipalPlaneArray> plane_sub_;
    message_filters::Synchronizer<Policy> sync_;
    rclcpp::TimerBase::SharedPtr timer_;

    int frequency_;
    int queue_size_;
    layout_extractor::ExtractorParams extractor_params_;
    double filter_leaf_size_;
    std::deque<std::pair<pcl::PointCloud<PointT>, sbim_msgs::msg::PrincipalPlaneArray> > message_queue_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LayoutExtractorNode>());
    rclcpp::shutdown();
    return 0;
}
