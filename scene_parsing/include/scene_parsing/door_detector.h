//
// Created by armon on 3/10/20.
//

#ifndef SRC_DOOR_DETECTOR_H
#define SRC_DOOR_DETECTOR_H

#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <scene_parsing/signal_1d.h>

namespace scene_parsing {

    struct DetectorParams {
        double distance_threshold;
        double min_peak_prominence;
        double min_peak_intensity;
        double standard_door_height = 2.0;
        double standard_door_width = 0.9;
    };

    class DoorDetector {

        DetectorParams params_;

        const std::vector<double> door_filter_ = {0.0019, 0.0083, 0.0252, 0.0586, 0.1109, 0.1800, 0.2632, 0.3577,
                                                  0.4553,
                                                  0.5385, 0.5834, 0.5688, 0.4812, 0.3232, 0.1140, -0.1140, -0.3232,
                                                  -0.4812, -0.5688, -0.5834, -0.5405, -0.4636, -0.3829, -0.3218,
                                                  -0.2909,
                                                  -0.2909, -0.3218, -0.3829, -0.4636, -0.5405, -0.5834, -0.5688,
                                                  -0.4812,
                                                  -0.3232, -0.1140, 0.1140, 0.3232, 0.4812, 0.5688, 0.5834, 0.5385,
                                                  0.4553,
                                                  0.3577, 0.2632, 0.1800, 0.1109, 0.0586, 0.0252, 0.0083, 0.0019};

    public:

        explicit DoorDetector(scene_parsing::DetectorParams &params) : params_(params) {}

        template<typename PointT>
        size_t detectDoorsFromCloud(const pcl::PointCloud<PointT> &point_cloud, const std::vector<double> &plane,
                                    std::vector<std::vector<Eigen::Vector3f>> &doors) const {

            typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
            *cloud_ptr = point_cloud;

            Eigen::VectorXf plane_coefficients(4);
            plane_coefficients << plane[0], plane[1], plane[2], plane[3];

            // points along plane
            typename pcl::PointCloud<PointT>::Ptr layout_cloud(new pcl::PointCloud<PointT>);
            std::vector<int> inliers;
            pcl::SampleConsensusModelPlane<PointT> sac_model_plane(cloud_ptr);
            sac_model_plane.selectWithinDistance(plane_coefficients, params_.distance_threshold, inliers);
            typename pcl::PointCloud<PointT>::Ptr projected_cloud(new pcl::PointCloud<PointT>);
            sac_model_plane.projectPoints(inliers, plane_coefficients, *projected_cloud);
            pcl::copyPointCloud(*projected_cloud, inliers, *layout_cloud);

            if (inliers.empty()) {
                return 0;
            }

            // remove outliers
            typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
            // pcl::StatisticalOutlierRemoval<PointT> outlier_removal;
            // outlier_removal.setInputCloud(projected_cloud);
            // outlier_removal.setMeanK(3);
            // outlier_removal.setStddevMulThresh(1.0);
            // outlier_removal.filter(*filtered_cloud);
            filtered_cloud = projected_cloud;

            // get height
            PointT min_p, max_p;
            pcl::getMinMax3D(*filtered_cloud, min_p, max_p);
            double z_ref = min_p.z;

            // crop
            pcl::PointCloud<PointT> cropped_cloud;
            pcl::PassThrough<PointT> pass_through;
            pass_through.setInputCloud(filtered_cloud);
            pass_through.setFilterFieldName("z");
            pass_through.setFilterLimits(z_ref, z_ref + params_.standard_door_height);
            pass_through.filter(cropped_cloud);

            // construct line
            Eigen::Vector3f line;
            line << plane[0], plane[1], (plane[2] * z_ref + plane[3]);
            line = line / std::sqrt(plane[0] * plane[0] + plane[1] * plane[1]);

            // re-parameterize line
            double mag = -line[2];
            Eigen::Vector3f p_0;
            Eigen::Vector3f n_hat;
            p_0 << mag * line[0], mag * line[1], z_ref;
            n_hat << -line[1], line[0], 0;

            // compute distances along line projection
            std::vector<double> distances;
            for (auto &p : cropped_cloud.points) {
                if (!pcl::isFinite(p)) {
                    continue;
                }
                distances.push_back(n_hat[0] * (p.x - p_0[0]) + n_hat[1] * (p.y - p_0[1]));
            }

            // find door peak locations
            double min_d = *std::min_element(distances.begin(), distances.end());
            double max_d = *std::max_element(distances.begin(), distances.end());
            std::vector<int> histogram = signal_1d::histogram_counts(distances, min_d, max_d, 0.05);
            double max_count = *std::max_element(histogram.begin(), histogram.end());
            std::vector<double> dhistogram = std::vector<double>(histogram.size(), 0);
            for (size_t i = 0; i < histogram.size(); ++i) {
                dhistogram[i] = static_cast<double>(histogram[i]) / max_count;
            }
            std::vector<double> door_signal = signal_1d::filter(dhistogram, door_filter_);
            std::vector<double> peak_intensity;
            std::vector<float> peak_location;
            signal_1d::find_peaks(door_signal, params_.min_peak_intensity, params_.min_peak_prominence,
                                  peak_intensity, peak_location);
            std::vector<double> door_centers;
            for (auto f : peak_location) {
                door_centers.push_back(((0.05 * f) + 0.025) + min_d);
            }

            // transform to interval
            Eigen::Vector3f p_1;
            p_1 << p_0[0], p_0[1], z_ref + params_.standard_door_height;
            for (auto c : door_centers) {
                std::vector<Eigen::Vector3f> door_extent = {(c + params_.standard_door_width / 2) * n_hat + p_1,
                                                            (c - params_.standard_door_width / 2) * n_hat + p_1,
                                                            (c - params_.standard_door_width / 2) * n_hat + p_0,
                                                            (c + params_.standard_door_width / 2) * n_hat + p_0};
                doors.push_back(door_extent);
            }

            return door_centers.size();

        }

        size_t
        detectDoorsFromCrossing(const std::vector<std::vector<double>> &trajectory, const std::vector<double> &plane,
                                std::vector<std::vector<Eigen::Vector3f>> &doors) const {

            bool sign = std::signbit(trajectory[0][0] * plane[0] +
                                     trajectory[0][1] * plane[1] +
                                     trajectory[0][2] * plane[2] +
                                     plane[3]);

            for (auto &point : trajectory) {

                double p_dist = point[0] * plane[0] + point[1] * plane[1] + point[2] * plane[2] + plane[3];
                bool p_sign = std::signbit(p_dist);

                if (p_sign != sign) {

                    double z_ref = point[2] - params_.standard_door_height / 2;

                    // construct line
                    Eigen::Vector3f line;
                    line << plane[0], plane[1], (plane[2] * z_ref + plane[3]);
                    line = line / std::sqrt(plane[0] * plane[0] + plane[1] * plane[1]);

                    // re-parameterize line
                    double mag = -line[2];
                    Eigen::Vector3f p_0;
                    Eigen::Vector3f n_hat;
                    p_0 << mag * line[0], mag * line[1], z_ref;
                    n_hat << -line[1], line[0], 0;

                    Eigen::Vector3f p_1;
                    p_1 << p_0[0], p_0[1], z_ref + params_.standard_door_height;

                    double c = n_hat[0] * (point[0] - p_0[0]) +
                               n_hat[1] * (point[1] - p_0[1]) +
                               n_hat[2] * (point[2] - p_0[2]);

                    std::vector<Eigen::Vector3f> door_extent = {(c + params_.standard_door_width / 2) * n_hat + p_1,
                                                                (c - params_.standard_door_width / 2) * n_hat + p_1,
                                                                (c - params_.standard_door_width / 2) * n_hat + p_0,
                                                                (c + params_.standard_door_width / 2) * n_hat + p_0};

                    doors.push_back(door_extent);

                }
            }

            return 1;

        }


    };

}


#endif //SRC_DOOR_DETECTOR_H
