//
// Created by armon on 1/16/20.
//

#ifndef SRC_STRUCTURAL_COMPASS_H
#define SRC_STRUCTURAL_COMPASS_H

#include <math.h>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Core>

namespace structural_compass {

    template<typename PointCloud>
    class EntropyCompass {

    private:

        std::vector<float> searchSpace() const;

        float cloudEntropy(const PointCloud &point_cloud) const;

        float histogramEntropy(Eigen::ArrayXf &histogram) const;

    public:

        ~EntropyCompass() = default;
        Eigen::Matrix3f principalDirections(const PointCloud &point_cloud, const Eigen::Vector3f &gravity,
                                            std::vector<Eigen::Vector3f> &directions);

    };

    template<typename PointCloud>
    Eigen::Matrix3f
    EntropyCompass<PointCloud>::principalDirections(const PointCloud &point_cloud, const Eigen::Vector3f &gravity,
                                                    std::vector<Eigen::Vector3f> &directions) {

        // initial gravity-aligned orthonormal bases
        Eigen::Vector3f v3 = -gravity.normalized();
        Eigen::Vector3f v1;
        v1 << -v3[1], v3[0], 0.0;
        Eigen::Vector3f v2;
        v2 = v3.cross(v1);
        Eigen::Matrix4f M;
        M << v1[0], v2[0], v3[0], 0,
                v1[1], v2[1], v3[1], 0,
                v1[2], v2[2], v3[2], 0,
                0, 0, 0, 1;
        Eigen::Isometry3f G_cg(M);

        PointCloud P_g;
        pcl::transformPointCloud(point_cloud, P_g, G_cg.inverse());

        // find entropy-minimizing rotation
        PointCloud P_th;
        Eigen::Isometry3f R_th = Eigen::Isometry3f::Identity();
        std::vector<float> objective;
        auto search_space = searchSpace();
        for (auto theta : search_space) {
            R_th.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud(P_g, P_th, R_th);
            objective.push_back(cloudEntropy(P_th));
        }
        int opt_index = std::distance(objective.begin(), std::min_element(objective.begin(), objective.end()));
        float theta_opt = search_space[opt_index];
        Eigen::Isometry3f G_opt = Eigen::Isometry3f::Identity();
        G_opt.rotate(Eigen::AngleAxisf(theta_opt, Eigen::Vector3f::UnitZ()));
        Eigen::Isometry3f G_compass = (G_opt * G_cg.inverse()).inverse();

        // directions.push_back(v3);
        // directions.push_back(v2);
        // directions.push_back(v1);

        return G_compass.rotation();

    }

    template<typename PointCloud>
    std::vector<float> EntropyCompass<PointCloud>::searchSpace() const {

        float start = -M_PI_2;
        float stop = M_PI_2;
        float step = 0.5 * (M_PI / 180.0);

        std::vector<float> arange;
        for (float i = start; i < stop; i += step) {
            arange.push_back(i);
        }

        return arange;
    }

    template<typename PointCloud>
    float EntropyCompass<PointCloud>::cloudEntropy(const PointCloud &point_cloud) const {

        const long kNumBins = 1000;

        Eigen::ArrayXf e_data = Eigen::ArrayXf::LinSpaced(kNumBins + 1, -50.0, 50.0);
        std::vector<float> edges = std::vector<float>(e_data.data(), e_data.data() + e_data.size());

        Eigen::ArrayXf x_bins = Eigen::ArrayXf::Zero(edges.size() - 1);
        Eigen::ArrayXf y_bins = Eigen::ArrayXf::Zero(edges.size() - 1);

        for (auto p : point_cloud.points) {

            if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y))
                continue;

            int first_x_greater = std::distance(edges.begin(), std::lower_bound(edges.begin(), edges.end(), p.x));
            int first_y_greater = std::distance(edges.begin(), std::lower_bound(edges.begin(), edges.end(), p.y));

            bool out_of_range_x = ((first_x_greater == 0) ||
                                   ((first_x_greater == edges.size()) && p.x > edges[first_x_greater]));
            bool out_of_range_y = ((first_y_greater == 0) ||
                                   ((first_y_greater == edges.size()) && p.y > edges[first_y_greater]));

            if (!out_of_range_x) {
                x_bins[first_x_greater - 1]++;
            }

            if (!out_of_range_y) {
                y_bins[first_y_greater - 1]++;
            }

        }

        float H_x = histogramEntropy(x_bins);
        float H_y = histogramEntropy(y_bins);

        return H_x + H_y;
    }

    template<typename PointCloud>
    float
    EntropyCompass<PointCloud>::histogramEntropy(Eigen::ArrayXf &histogram) const {

        Eigen::ArrayXf p = histogram / histogram.sum();
        Eigen::ArrayXf logp = p.log();
        return -(static_cast<float>((p * logp).sum()));
    }

}

#endif //SRC_STRUCTURAL_COMPASS_H
