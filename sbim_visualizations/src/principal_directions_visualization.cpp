#include <rclcpp/rclcpp.hpp>
#include <sbim_msgs/msg/principal_direction_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PrincipalDirectionsVisualization : public rclcpp::Node {

public:

    ~PrincipalDirectionsVisualization() = default;

    PrincipalDirectionsVisualization() : Node("principal_directions_visualization") {
        sub_ = this->create_subscription<sbim_msgs::msg::PrincipalDirectionArray>(
            "/principal_directions", 1, std::bind(&PrincipalDirectionsVisualization::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("principal_direction_viz", 10);
    }

    void callback(const sbim_msgs::msg::PrincipalDirectionArray::SharedPtr directions) {

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (auto d : directions->directions) {

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = directions->header.frame_id;
            marker.header.stamp = this->now();
            marker.ns = "principal_directions";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            geometry_msgs::msg::Point start;
            start.x = 0.0;
            start.y = 0.0;
            start.z = 0.0;
            geometry_msgs::msg::Point end;
            end.x = d.x / 2;
            end.y = d.y / 2;
            end.z = d.z / 2;
            marker.points.push_back(start);
            marker.points.push_back(end);

            marker.scale.x = 0.05;
            marker.scale.y = 0.1;
            marker.scale.z = 0.0;

            marker.color.a = 0.8;
            marker.color.r = 1.0;
            marker.color.g = 0.75;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);

            id += 1;
        }

        pub_->publish(marker_array);
    }

private:

    rclcpp::Subscription<sbim_msgs::msg::PrincipalDirectionArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

};


int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PrincipalDirectionsVisualization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
