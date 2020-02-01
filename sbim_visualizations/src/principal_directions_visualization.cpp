//
// Created by armon on 2/1/20.
//

#include <ros/ros.h>
#include <sbim_msgs/PrincipalDirections.h>
#include <visualization_msgs/MarkerArray.h>

class PrincipalDirectionsVisualization {

public:

    ~PrincipalDirectionsVisualization() = default;

    PrincipalDirectionsVisualization() : nh_("~") {
        sub_ = nh_.subscribe<sbim_msgs::PrincipalDirections>("/principal_directions", 1,
                                                             boost::bind(&PrincipalDirectionsVisualization::callback,
                                                                         this, _1));
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("principal_direction_viz", 0);
    }

    void callback(const sbim_msgs::PrincipalDirections::ConstPtr &directions) {

        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        for (auto d : directions->directions) {

            visualization_msgs::Marker marker;
            marker.header.frame_id = directions->header.frame_id;
            marker.header.stamp = ros::Time();
            marker.ns = "principal_directions";
            marker.id = id;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            geometry_msgs::Point start;
            start.x = 0.0;
            start.y = 0.0;
            start.z = 0.0;
            geometry_msgs::Point end;
            start.x = d.x / 2;
            start.y = d.y / 2;
            start.z = d.z / 2;
            marker.points.push_back(end);
            marker.points.push_back(start);

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

        pub_.publish(marker_array);

    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

};


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "pincipal_directions_visualization");

    PrincipalDirectionsVisualization principal_directions_visualization;

    ros::spin();

    return 0;
}
