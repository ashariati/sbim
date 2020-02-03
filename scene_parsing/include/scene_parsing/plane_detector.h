//
// Created by armon on 2/1/20.
//

#ifndef SRC_PLANE_DETECTOR_H
#define SRC_PLANE_DETECTOR_H

#include <eigen3/Eigen/Core>
#include <scene_parsing/signal_1d.h>

template<typename PointCloud>
class PlaneDetector {

public:

    ~PlaneDetector() = default;

    void scanDirection(const PointCloud &point_cloud, const Eigen::Vector3f &direction,
                       std::vector<float> &offsets, std::vector<double> &intensities) const;

};

template<typename PointCloud>
void PlaneDetector<PointCloud>::scanDirection(const PointCloud &point_cloud, const Eigen::Vector3f &direction,
                                              std::vector<float> &offsets, std::vector<double> &intensities) const {

    Eigen::MatrixXf P = point_cloud.getMatrixXfMap();
    Eigen::VectorXf distances = P * direction;

    offsets.push_back(0.0);
    intensities.push_back(100.0);

}


#endif //SRC_PLANE_DETECTOR_H
