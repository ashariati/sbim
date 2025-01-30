#include <rclcpp/rclcpp.hpp>
#include <sbim_msgs/msg/principal_plane_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sbim_visualizations/plane_visualization.h>
#include <eigen3/Eigen/Dense>

class PrincipalPlanesVisualization : public rclcpp::Node {

private:

    rclcpp::Subscription<sbim_msgs::msg::PrincipalPlaneArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

    std::string marker_ns_;
    float r_;
    float g_;
    float b_;

    float plane_length_;
    float plane_height_;
    float plane_width_;

public:

    ~PrincipalPlanesVisualization() = default;

    PrincipalPlanesVisualization() : Node("principal_planes_visualization"), marker_ns_(""), r_(0), g_(0), b_(0),
                                     plane_length_(0), plane_height_(0), plane_width_(0) {

        this->declare_parameter<std::string>("marker_ns", "principal_planes");
        this->declare_parameter<float>("red", 1.0);
        this->declare_parameter<float>("green", 0.75);
        this->declare_parameter<float>("blue", 0.0);
        this->declare_parameter<float>("plane_length", 500);
        this->declare_parameter<float>("plane_height", 0.2);
        this->declare_parameter<float>("plane_width", 4.0);

        this->get_parameter("marker_ns", marker_ns_);
        this->get_parameter("red", r_);
        this->get_parameter("green", g_);
        this->get_parameter("blue", b_);
        this->get_parameter("plane_length", plane_length_);
        this->get_parameter("plane_height", plane_height_);
        this->get_parameter("plane_width", plane_width_);

        sub_ = this->create_subscription<sbim_msgs::msg::PrincipalPlaneArray>(
            "/planes", 1, std::bind(&PrincipalPlanesVisualization::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("principal_planes_viz", 10);
    }

    void callback(const sbim_msgs::msg::PrincipalPlaneArray::SharedPtr planes) {

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (auto p : planes->planes) {

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = planes->header.frame_id;
            marker.header.stamp = this->now();
            marker.ns = marker_ns_;
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            std::vector<Eigen::Vector3f> triangles;
            if (p.label.data != "0") {
                triangles = sbim_visualizations::planeTriangles(
                        Eigen::Vector4f(p.plane.coef[0], p.plane.coef[1], p.plane.coef[2], p.plane.coef[3]),
                        plane_length_, plane_height_);
            } else {
                triangles = sbim_visualizations::planeTriangles(
                        Eigen::Vector4f(p.plane.coef[0], p.plane.coef[1], p.plane.coef[2], p.plane.coef[3]),
                        plane_width_, plane_width_);
            }

            for (auto t : triangles) {
                geometry_msgs::msg::Point pt;
                pt.x = t[0];
                pt.y = t[1];
                pt.z = t[2];
                marker.points.push_back(pt);

                std_msgs::msg::ColorRGBA color;
                color.a = 1.0;
                color.r = r_;
                color.g = g_;
                color.b = b_;
                marker.colors.push_back(color);
            }

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker_array.markers.push_back(marker);

            id += 1;
        }

        pub_->publish(marker_array);
    }
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PrincipalPlanesVisualization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
