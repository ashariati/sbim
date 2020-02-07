//
// Created by armon on 2/5/20.
//

#include <ros/ros.h>

#include <sbim_msgs/PrincipalPlaneArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <sbim_visualizations/plane_visualization.h>

class PrincipalPlanesVisualization {

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::string marker_ns_;
    float a_;
    float r_;
    float g_;
    float b_;

public:

    ~PrincipalPlanesVisualization() = default;

    PrincipalPlanesVisualization() : nh_("~"), marker_ns_(""), a_(0), r_(0), g_(0), b_(0) {

        nh_.param<std::string>("marker_ns", marker_ns_, "principal_planes");
        nh_.param<float>("alpha", a_, 1.0);
        nh_.param<float>("red", r_, 1.0);
        nh_.param<float>("green", g_, 0.75);
        nh_.param<float>("blue", b_, 0.0);

        sub_ = nh_.subscribe<sbim_msgs::PrincipalPlaneArray>("/planes", 1,
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
            marker.ns = marker_ns_;
            marker.id = id;
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;

            float width = (p.label.data != "0") ? 10.0 : 4.0;
            std::vector<Eigen::Vector3f> triangles = sbim_visualizations::planeTriangles(
                    Eigen::Vector4f(p.plane.coef[0], p.plane.coef[1], p.plane.coef[2], p.plane.coef[3]),
                    width, 4.0);
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

            marker.color.a = a_;
            marker.color.r = r_;
            marker.color.g = g_;
            marker.color.b = b_;

            marker_array.markers.push_back(marker);

            id += 1;
        }

        pub_.publish(marker_array);

    }

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "principal_planes_visualization");

    PrincipalPlanesVisualization principal_planes_visualization;

    ros::spin();

    return 0;
}

