//
// Created by armon on 3/2/20.
//

#include <unordered_map>

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
    float graph_height_;

public:

    ~FloorplanVisualization() = default;

    FloorplanVisualization() : nh_("~"), marker_ns_("") {

        nh_.param<std::string>("marker_ns", marker_ns_, "floorplan");
        nh_.param<float>("boundary_height", boundary_height_, 2.5);
        nh_.param<float>("graph_height", graph_height_, 1.25);

        sub_ = nh_.subscribe<sbim_msgs::FloorplanArray>("/floorplan", 1,
                                                        boost::bind(&FloorplanVisualization::callback,
                                                               this, _1));
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("floorplan_viz", 0);
    }

    void callback(const sbim_msgs::FloorplanArray::ConstPtr &floorplan_array) {

        visualization_msgs::MarkerArray marker_array;

        size_t id = 0;
        for (auto &floorplan : floorplan_array->floorplans) {

            visualization_msgs::Marker boundary_marker = boundaryMarker(floorplan);
            boundary_marker.header.frame_id = floorplan_array->header.frame_id;
            boundary_marker.id = id;
            id += 1;

            visualization_msgs::Marker floorplan_marker = floorplanMarker(floorplan);
            floorplan_marker.header.frame_id = floorplan_array->header.frame_id;
            floorplan_marker.id = id;
            id += 1;

            visualization_msgs::Marker nodes_marker;
            nodes_marker.header.frame_id = floorplan_array->header.frame_id;
            nodes_marker.id = id;
            id += 1;

            visualization_msgs::Marker edges_marker;
            edges_marker.header.frame_id = floorplan_array->header.frame_id;
            edges_marker.id = id;
            id += 1;

            graphMarker(floorplan, nodes_marker, edges_marker);

            marker_array.markers.push_back(boundary_marker);
            marker_array.markers.push_back(floorplan_marker);

            marker_array.markers.push_back(nodes_marker);
            marker_array.markers.push_back(edges_marker);

        }

        pub_.publish(marker_array);

    }

    void graphMarker(const sbim_msgs::Floorplan &floorplan, visualization_msgs::Marker &nodes_marker,
                     visualization_msgs::Marker &edges_marker) {

        nodes_marker.header.stamp = ros::Time();
        nodes_marker.ns = marker_ns_;
        nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD;

        edges_marker.header.stamp = ros::Time();
        edges_marker.ns = marker_ns_;
        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD;

        for (auto &node : floorplan.nodes) {

            auto n = static_cast<double>(node.vertices.size());

            std::vector<double> mean = {0.0, 0.0, 0.0};
            for (auto &vertex : node.vertices) {
                mean[0] += vertex.x;
                mean[1] += vertex.y;
                mean[2] += vertex.z;
            }
            mean[0] = mean[0] / n;
            mean[1] = mean[1] / n;
            mean[2] = mean[2] / n;

            geometry_msgs::Point mu;
            mu.x = mean[0];
            mu.y = mean[1];
            mu.z = mean[2] + graph_height_;

            nodes_marker.points.push_back(mu);

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 0.0;
            nodes_marker.colors.push_back(color);

        }

        for (auto &edges : floorplan.edges) {

            size_t u = edges.u.data;
            size_t v = edges.v.data;

            edges_marker.points.push_back(nodes_marker.points[u]);
            edges_marker.points.push_back(nodes_marker.points[v]);

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 0.0;
            edges_marker.colors.push_back(color);
            edges_marker.colors.push_back(color);
        }

        nodes_marker.pose.position.x = 0.0;
        nodes_marker.pose.position.y = 0.0;
        nodes_marker.pose.position.z = 0.0;
        nodes_marker.pose.orientation.x = 0.0;
        nodes_marker.pose.orientation.y = 0.0;
        nodes_marker.pose.orientation.z = 0.0;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.5;
        nodes_marker.scale.y = 0.5;
        nodes_marker.scale.z = 0.5;

        edges_marker.pose.position.x = 0.0;
        edges_marker.pose.position.y = 0.0;
        edges_marker.pose.position.z = 0.0;
        edges_marker.pose.orientation.x = 0.0;
        edges_marker.pose.orientation.y = 0.0;
        edges_marker.pose.orientation.z = 0.0;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.2;

    }

    visualization_msgs::Marker floorplanMarker(const sbim_msgs::Floorplan &floorplan) {

        visualization_msgs::Marker floorplan_marker;
        floorplan_marker.header.stamp = ros::Time();
        floorplan_marker.ns = marker_ns_;
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

                std_msgs::ColorRGBA color;
                color.a = 1.0;
                color.r = 0.0;
                color.g = 1.0;
                color.b = 0.0;
                floorplan_marker.colors.push_back(color);
            }

        }

        floorplan_marker.pose.orientation.x = 0.0;
        floorplan_marker.pose.orientation.y = 0.0;
        floorplan_marker.pose.orientation.z = 0.0;
        floorplan_marker.pose.orientation.w = 1.0;

        floorplan_marker.scale.x = 1.0;
        floorplan_marker.scale.y = 1.0;
        floorplan_marker.scale.z = 1.0;

        return floorplan_marker;
    }

    visualization_msgs::Marker boundaryMarker(const sbim_msgs::Floorplan &floorplan) {

        visualization_msgs::Marker boundary_marker;
        boundary_marker.header.stamp = ros::Time();
        boundary_marker.ns = marker_ns_;
        boundary_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        boundary_marker.action = visualization_msgs::Marker::ADD;

        std::set<std::pair<size_t, size_t>> shared_edges;
        for (auto &e : floorplan.edges) {

            if (e.shared_edge.empty()) {
                continue;
            }

            shared_edges.insert(std::make_pair(e.shared_edge[0].data, e.shared_edge[1].data));
        }

        for (auto &node : floorplan.nodes) {

            std::unordered_map<size_t, size_t> index_map;
            for (size_t i = 0; i < node.index_map_keys.size(); ++i) {
                size_t keys = node.index_map_keys[i].data;
                size_t values = node.index_map_values[i].data;
                index_map[keys] = values;
            }

            for (auto &e : node.edges) {

                size_t u = e.u.data;
                size_t v = e.v.data;

                std::pair<size_t, size_t> edge_pair = std::make_pair(index_map[u], index_map[v]);

                if (shared_edges.count(edge_pair)) {
                    continue;
                }

                std::vector<double> x1 = {node.vertices[u].x, node.vertices[u].y,
                                          node.vertices[u].z + boundary_height_};
                std::vector<double> x2 = {node.vertices[v].x, node.vertices[v].y,
                                          node.vertices[v].z + boundary_height_};
                std::vector<double> x3 = {node.vertices[v].x, node.vertices[v].y, node.vertices[v].z};
                std::vector<double> x4 = {node.vertices[u].x, node.vertices[u].y, node.vertices[u].z};

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

                    std_msgs::ColorRGBA color;
                    color.a = 1.0;
                    color.r = 1.0;
                    color.g = 0.0;
                    color.b = 0.0;
                    boundary_marker.colors.push_back(color);
                }

            }

        }

        boundary_marker.pose.orientation.x = 0.0;
        boundary_marker.pose.orientation.y = 0.0;
        boundary_marker.pose.orientation.z = 0.0;
        boundary_marker.pose.orientation.w = 1.0;

        boundary_marker.scale.x = 1.0;
        boundary_marker.scale.y = 1.0;
        boundary_marker.scale.z = 1.0;

        return boundary_marker;
    }


};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "floorplan_visualization");

    FloorplanVisualization floorplan_visualization;

    ros::spin();

    return 0;
}

