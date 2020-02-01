//
// Created by armon on 2/1/20.
//

#ifndef SRC_PLANE_DETECTOR_H
#define SRC_PLANE_DETECTOR_H

#include <eigen3/Eigen/Core>

template<typename PointCloud>
class PlaneDetector {

public:

    ~PlaneDetector() = default;

    void scan(const PointCloud &point_cloud, const Eigen::Vector3f &direction, std::vector<float> &offsets) const;

};

template<typename PointCloud>
void PlaneDetector<PointCloud>::scan(const PointCloud &point_cloud, const Eigen::Vector3f &direction,
                                     std::vector<float> &offsets) const {

    offsets.push_back(0.0);

}


#endif //SRC_PLANE_DETECTOR_H
