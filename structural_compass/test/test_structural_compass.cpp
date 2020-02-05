//
// Created by armon on 1/22/20.
//

#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include <math.h>
#include <string>
#include <fstream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <structural_compass/structural_compass.h>

// void visualizeCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
//     std::cout << "Cloud size: " << cloud->points.size() << std::endl;
//     pcl::visualization::CloudViewer viewer("Cloud Viewer");
//     viewer.showCloud(cloud);
//     while (!viewer.wasStopped()) {
//     }
// }

void loadPointCloud(const std::string &file, pcl::PointCloud<pcl::PointXYZ> &cloud) {

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, cloud) == -1) {
        throw std::runtime_error("loadPointCloud: couldn't read file");
    }

}

Eigen::Vector3f gravityVectorFromFile(const std::string &file, const int index) {

    std::ifstream infile(file.c_str());

    std::string line;
    for (size_t i = 0; i <= index; ++i) {
        std::getline(infile, line);
    }

    std::istringstream ss(line);
    float x, y, z;
    ss >> x >> y >> z;

    Eigen::Vector3f gravity;
    gravity << x, y, z;

    return gravity;

}

TEST_CASE("Entropy Compass", "[EntropyCompass]") {

    int cloud_id = 123;

    std::string vector_file = std::string("/home/armon/Research/Data/exyn_building_scans/gravity_vectors.txt");
    Eigen::Vector3f gravity = gravityVectorFromFile(vector_file, cloud_id);

    std::string cloud_file = std::string("/home/armon/Research/Data/exyn_building_scans/pointclouds_indexed/") +
                             std::string(3, '0') + std::to_string(cloud_id) + std::string(".pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    loadPointCloud(cloud_file, *cloud_ptr);

    structural_compass::EntropyCompass compass;

    SECTION("principal directions") {

        Eigen::Matrix3f R;
        std::vector<Eigen::Vector3f> principal_directions;
        R = compass.principalDirections(*cloud_ptr, Eigen::Matrix3f::Identity(), gravity, principal_directions);

        // visualizeCloud(cloud_ptr);

    }
}

TEST_CASE("My Angle Experiments", "[My Experiments]") {

    float angles[] = {-M_PI, -3 * M_PI_4, -M_PI_2, -M_PI_4, 0, M_PI_4, M_PI_2, 3 * M_PI_4, M_PI};

    for (auto a : angles) {

        Eigen::AngleAxisf ax = Eigen::AngleAxisf(a, Eigen::Vector3f::UnitZ());

        Eigen::Matrix3f R = ax.toRotationMatrix();

        Eigen::Vector3f ea = R.eulerAngles(2, 1, 0);

        std::cout << "true angle: " << a << std::endl;
        std::cout << "euler angles: " << ea.transpose() << std::endl;
        std::cout << "acos: " << std::acos(R(0, 0)) << std::endl;
        std::cout << "asin: " << std::asin(R(1, 0)) << std::endl;
        std::cout << "atan2: " << std::atan2(R(1, 0), R(0, 0)) << std::endl;
        std::cout << std::endl;

    }


}

