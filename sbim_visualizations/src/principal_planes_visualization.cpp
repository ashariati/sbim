//
// Created by armon on 2/5/20.
//

#include <ros/ros.h>

#include <sbim_msgs/PrincipalPlaneArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <sbim_visualizations/plane_visualization.h>

class PrincipalPlanesVisualization {

public:

    ~PrincipalPlanesVisualization() = default;

    PrincipalPlanesVisualization() : nh_("~") {
        sub_ = nh_.subscribe<sbim_msgs::PrincipalPlaneArray>("/principal_planes", 1,
                                                             boost::bind(&PrincipalPlanesVisualization::callback,
                                                                         this, _1));
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("principal_planes_viz", 0);
    }

    void callback(const sbim_msgs::PrincipalPlaneArray::ConstPtr &planes) {

        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        for (auto p : planes->planes) {

            visualization_msgs::Marker marker;
            marker.header.frame_id = planes->header.frame_id;
            marker.header.stamp = ros::Time();
            marker.ns = "principal_planes";
            marker.id = id;
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;

            std::vector<Eigen::Vector3f> triangles = sbim_visualizations::planeTriangles(
                    Eigen::Vector4f(p.plane.coef[0], p.plane.coef[1], p.plane.coef[2], p.plane.coef[3]),
                    10.0, 3.0);
            for (auto t : triangles) {
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

            marker.color.a = 1.0;
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

    ros::init(argc, argv, "principal_planes_visualization");

    PrincipalPlanesVisualization principal_planes_visualization;

    ros::spin();

    return 0;
}

