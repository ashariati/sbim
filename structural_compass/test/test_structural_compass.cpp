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

#define private public

#include <structural_compass/structural_compass.h>

void visualizeCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
    std::cout << "Cloud size: " << cloud->points.size() << std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }
}

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

TEST_CASE("Entropy Compass Utilities", "[EntropyCompass]") {

    structural_compass::EntropyCompass<pcl::PointCloud<pcl::PointXYZ>> compass;

    SECTION("computing histogram entropies") {

        Eigen::ArrayXf h1(7);
        h1 << 4, 6, 3, 8, 3, 6, 7;

        auto H = compass.histogramEntropy(h1);
        REQUIRE(std::abs(H - 1.884) < 1e-4);

    }

    SECTION("computing histogram counts") {

        std::vector<float> points{6.3615, 2.3843, 18.7966, 12.9110, 9.5893, 12.7863, 10.8943, 12.9462, 10.8777,
                                  14.4209};
        Eigen::ArrayXf edges(11);
        edges << 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20;

        Eigen::ArrayXf gt_counts(10);
        gt_counts << 0, 1, 0, 1, 1, 2, 3, 1, 0, 1;

        auto counts = compass.histogramCounts(points, edges);

        auto diff = (counts - gt_counts).abs();
        REQUIRE(diff.sum() < 1e-4);

    }

}

TEST_CASE("Entropy Compass", "[EntropyCompass]") {

    int cloud_id = 123;

    std::string vector_file = std::string("/home/armon/Research/Data/exyn_building_scans/gravity_vectors.txt");
    Eigen::Vector3f gravity = gravityVectorFromFile(vector_file, cloud_id);
    std::cout << gravity << std::endl;

    std::string cloud_file = std::string("/home/armon/Research/Data/exyn_building_scans/pointclouds_indexed/") +
                             std::string(3, '0') + std::to_string(cloud_id) + std::string(".pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    loadPointCloud(cloud_file, *cloud_ptr);

    structural_compass::EntropyCompass<pcl::PointCloud<pcl::PointXYZ>> compass;

    SECTION("principal directions") {

        Eigen::Matrix3f R;
        std::vector<Eigen::Vector3f> principal_directions;
        R = compass.principalDirections(*cloud_ptr, gravity, principal_directions);

        visualizeCloud(cloud_ptr);

    }
}

