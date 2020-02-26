//
// Created by armon on 2/26/20.
//

#include <ros/ros.h>

#include <sbim_msgs/Trajectory.h>
#include <geometry_msgs/PoseArray.h>

class TrajectoryVisualization {

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

public:

    ~TrajectoryVisualization() = default;

    TrajectoryVisualization() : nh_("~") {

        sub_ = nh_.subscribe<sbim_msgs::Trajectory>("/trajectory", 1,
                                                    boost::bind(&TrajectoryVisualization::callback,
                                                                this, _1));
        pub_ = nh_.advertise<geometry_msgs::PoseArray>("trajectory_visualization", 0);
    }

    void callback(const sbim_msgs::Trajectory::ConstPtr &trajectory) {

        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = trajectory->poses[0].header.frame_id;
        pose_array.header.stamp = ros::Time();
        for (auto p : trajectory->poses) {
            pose_array.poses.push_back(p.pose);
        }

        pub_.publish(pose_array);

    }

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "trajector_visualization");

    TrajectoryVisualization trajectory_visualization;

    ros::spin();

    return 0;
}

