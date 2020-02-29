//
// Created by armon on 2/29/20.
//

#include <ros/ros.h>

#include <sbim_msgs/LayoutSegmentArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <sbim_visualizations/segment_visualization.h>

class LayoutSegmentVisualization {

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::string marker_ns_;

public:

    ~LayoutSegmentVisualization() = default;

    LayoutSegmentVisualization() : nh_("~"), marker_ns_("") {

        nh_.param<std::string>("marker_ns", marker_ns_, "layout_segments");

        sub_ = nh_.subscribe<sbim_msgs::LayoutSegmentArray>("/layout_segments", 1,
                                                            boost::bind(&LayoutSegmentVisualization::callback,
                                                                        this, _1));
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("layout_segment_viz", 0);
    }

    void callback(const sbim_msgs::LayoutSegmentArray::ConstPtr &layout_segments) {

        visualization_msgs::MarkerArray marker_array;
        size_t id = 0;
        for (auto &s : layout_segments->layout_segments) {

            visualization_msgs::Marker marker;
            marker.header.frame_id = s.header.frame_id;
            marker.header.stamp = ros::Time();
            marker.ns = marker_ns_;
            marker.id = id;
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;

            std::vector<std::vector<double>> vertices;
            for (auto &vertex : s.vertices) {
                std::vector<double> v = {vertex.x, vertex.y, vertex.z};
                vertices.push_back(v);
            }

            std::vector<std::vector<double>> triangles = sbim_visualizations::convexToTriangles(vertices);
            for (auto &t : triangles) {
                geometry_msgs::Point pt;
                pt.x = t[0];
                pt.y = t[1];
                pt.z = t[2];
                marker.points.push_back(pt);
            }

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            if (s.label.data == "0") {
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.75;
                marker.color.b = 1.0;
            } else {
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }

            marker_array.markers.push_back(marker);

            id += 1;
        }

        pub_.publish(marker_array);

    }

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "layout_segment_visualization");

    LayoutSegmentVisualization layout_segment_visualization;

    ros::spin();

    return 0;
}

