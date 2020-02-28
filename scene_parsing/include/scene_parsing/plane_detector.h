//
// Created by armon on 2/1/20.
//

#ifndef SRC_PLANE_DETECTOR_H
#define SRC_PLANE_DETECTOR_H

#include <eigen3/Eigen/Core>
#include <scene_parsing/signal_1d.h>

class PlaneDetector {

public:

    ~PlaneDetector() = default;

    template<typename PointT>
    void scanDirection(const pcl::PointCloud<PointT> &point_cloud, const Eigen::Vector3f &direction, float range,
                       int min_intensity,
                       std::vector<float> &offsets, std::vector<double> &intensities) const;

};

template<typename PointT>
void
PlaneDetector::scanDirection(const pcl::PointCloud<PointT> &point_cloud, const Eigen::Vector3f &direction, float range,
                             int min_intensity,
                             std::vector<float> &offsets, std::vector<double> &intensities) const {

    Eigen::Vector4f direction_hom = Eigen::Vector4f::Zero();
    direction_hom.head(3) = direction;

    Eigen::MatrixXf P = point_cloud.getMatrixXfMap();
    Eigen::VectorXf d = P.transpose() * direction_hom;

    std::vector<float> distances;
    for (size_t i = 0; i < d.rows(); ++i) {
        if (!isnan(d[i])) {
            distances.push_back(d[i]);
        }
    }

    std::vector<int> counts = signal_1d::histogram_counts<float>(distances, -range, range, 0.05);

    std::vector<float> peak_locs;
    std::vector<double> x(counts.begin(), counts.end());
    signal_1d::find_peaks<double>(x, min_intensity, min_intensity, intensities, peak_locs);

    for (auto i : peak_locs) {
        offsets.push_back(((0.05 * i) + 0.025) - range);
    }

}


#endif //SRC_PLANE_DETECTOR_H
