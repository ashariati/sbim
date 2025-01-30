#include <rclcpp/rclcpp.hpp>
#include <sbim_msgs/msg/door_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sbim_visualizations/convex.h>

class DoorVisualization : public rclcpp::Node {

private:

    rclcpp::Subscription<sbim_msgs::msg::DoorArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

    std::string marker_ns_;

public:

    ~DoorVisualization() = default;

    DoorVisualization() : Node("door_visualization"), marker_ns_("") {

        this->declare_parameter<std::string>("marker_ns", "doors");
        this->get_parameter("marker_ns", marker_ns_);

        sub_ = this->create_subscription<sbim_msgs::msg::DoorArray>(
            "/doors", 1, std::bind(&DoorVisualization::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("door_viz", 10);
    }

    void callback(const sbim_msgs::msg::DoorArray::SharedPtr door_array) {

        visualization_msgs::msg::MarkerArray marker_array;
        size_t id = 0;
        for (auto &d : door_array->doors) {

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = d.header.frame_id;
            marker.header.stamp = this->now();
            marker.ns = marker_ns_;
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            std::vector<std::vector<double>> vertices;
            for (auto &vertex : d.vertices) {
                std::vector<double> v = {vertex.x, vertex.y, vertex.z};
                vertices.push_back(v);
            }

            std::vector<std::vector<double>> triangles = sbim_visualizations::convexToTriangles(vertices);
            for (auto &t : triangles) {
                geometry_msgs::msg::Point pt;
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

            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);

            id += 1;
        }

        pub_->publish(marker_array);
    }
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DoorVisualization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
