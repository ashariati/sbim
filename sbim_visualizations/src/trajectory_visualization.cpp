#include <rclcpp/rclcpp.hpp>
#include <sbim_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class TrajectoryVisualization : public rclcpp::Node {

private:

    rclcpp::Subscription<sbim_msgs::msg::Trajectory>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;

public:

    ~TrajectoryVisualization() = default;

    TrajectoryVisualization() : Node("trajectory_visualization") {

        sub_ = this->create_subscription<sbim_msgs::msg::Trajectory>(
            "/trajectory", 1, std::bind(&TrajectoryVisualization::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("trajectory_visualization", 10);
    }

    void callback(const sbim_msgs::msg::Trajectory::SharedPtr trajectory) {

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = trajectory->poses[0].header.frame_id;
        pose_array.header.stamp = this->now();
        for (auto p : trajectory->poses) {
            pose_array.poses.push_back(p.pose);
        }

        pub_->publish(pose_array);
    }
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryVisualization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
