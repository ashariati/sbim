//
// Created by armon on 3/2/20.
//

#include <ros/ros.h>

#include <sbim_msgs/FloorplanArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <sbim_visualizations/convex.h>

class FloorplanVisualization {

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::string marker_ns_;
    float boundary_height_;

public:

    ~FloorplanVisualization() = default;

    FloorplanVisualization() : nh_("~"), marker_ns_("") {

        nh_.param<std::string>("marker_ns", marker_ns_, "floorplan");
        nh_.param<float>("boundary_height", boundary_height_, 2.5);

        sub_ = nh_.subscribe<sbim_msgs::FloorplanArray>("/floorplan", 1,
                                                        boost::bind(&FloorplanVisualization::callback,
                                                               this, _1));
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("floorplan_viz", 0);
    }

    void callback(const sbim_msgs::FloorplanArray::ConstPtr &floorplan_array) {

        visualization_msgs::MarkerArray marker_array;

        size_t id = 0;
        for (auto &floorplan : floorplan_array->floorplans) {

            visualization_msgs::Marker floorplan_marker;
            floorplan_marker.header.frame_id = floorplan_array->header.frame_id;
            floorplan_marker.header.stamp = ros::Time();
            floorplan_marker.ns = marker_ns_;
            floorplan_marker.id = id;
            floorplan_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            floorplan_marker.action = visualization_msgs::Marker::ADD;

            for (auto &node : floorplan.nodes) {

                std::vector<std::vector<double>> vertices;
                for (auto &vertex : node.vertices) {
                    std::vector<double> v = {vertex.x, vertex.y, vertex.z};
                    vertices.push_back(v);
                }

                sbim_visualizations::sortVerticesCCW(vertices);

                std::vector<std::vector<double>> triangles = sbim_visualizations::convexToTriangles(vertices);
                for (auto &t : triangles) {
                    geometry_msgs::Point pt;
                    pt.x = t[0];
                    pt.y = t[1];
                    pt.z = t[2];
                    floorplan_marker.points.push_back(pt);
                }
            }

            floorplan_marker.pose.orientation.x = 0.0;
            floorplan_marker.pose.orientation.y = 0.0;
            floorplan_marker.pose.orientation.z = 0.0;
            floorplan_marker.pose.orientation.w = 1.0;

            floorplan_marker.scale.x = 1.0;
            floorplan_marker.scale.y = 1.0;
            floorplan_marker.scale.z = 1.0;

            floorplan_marker.color.a = 1.0;
            floorplan_marker.color.r = 0.0;
            floorplan_marker.color.g = 1.0;
            floorplan_marker.color.b = 0.0;

            marker_array.markers.push_back(floorplan_marker);

            id += 1;

            visualization_msgs::Marker boundary_marker;
            boundary_marker.header.frame_id = floorplan_array->header.frame_id;
            boundary_marker.header.stamp = ros::Time();
            boundary_marker.ns = marker_ns_;
            boundary_marker.id = id;
            boundary_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            boundary_marker.action = visualization_msgs::Marker::ADD;

            for (auto &e : floorplan.edges) {

                if (e.boundary.empty()) {
                    continue;
                }

                std::vector<double> x4 = {e.boundary[0].x, e.boundary[0].y, e.boundary[0].z};
                std::vector<double> x3 = {e.boundary[1].x, e.boundary[1].y, e.boundary[1].z};
                std::vector<double> x1 = {e.boundary[0].x, e.boundary[0].y, e.boundary[0].z + boundary_height_};
                std::vector<double> x2 = {e.boundary[1].x, e.boundary[1].y, e.boundary[1].z + boundary_height_};

                std::vector<std::vector<double>> vertices;
                vertices.push_back(x1);
                vertices.push_back(x2);
                vertices.push_back(x3);
                vertices.push_back(x4);

                std::vector<std::vector<double>> triangles = sbim_visualizations::convexToTriangles(vertices);
                for (auto &t : triangles) {
                    geometry_msgs::Point pt;
                    pt.x = t[0];
                    pt.y = t[1];
                    pt.z = t[2];
                    boundary_marker.points.push_back(pt);
                }

            }

            boundary_marker.pose.orientation.x = 0.0;
            boundary_marker.pose.orientation.y = 0.0;
            boundary_marker.pose.orientation.z = 0.0;
            boundary_marker.pose.orientation.w = 1.0;

            boundary_marker.scale.x = 1.0;
            boundary_marker.scale.y = 1.0;
            boundary_marker.scale.z = 1.0;

            boundary_marker.color.a = 0.6;
            boundary_marker.color.r = 1.0;
            boundary_marker.color.g = 0.0;
            boundary_marker.color.b = 0.0;

            marker_array.markers.push_back(boundary_marker);

            id += 1;

        }

        pub_.publish(marker_array);

    }


};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "floorplan_visualization");

    FloorplanVisualization floorplan_visualization;

    ros::spin();

    return 0;
}

