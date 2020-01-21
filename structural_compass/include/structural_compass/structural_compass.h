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

        std::vector<float> searchSpace() const;

        float cloudEntropy(const PointCloud &point_cloud) const;

        Eigen::VectorXf histogramCounts(const std::vector<float> &points, const Eigen::VectorXf &edges) const;

        float histogramEntropy(Eigen::VectorXf &histogram) const;

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
        std::vector<float> objective;
        auto search_space = searchSpace();
        for (auto theta : search_space) {
            Eigen::Isometry3f R_th = Eigen::Isometry3f::Identity();
            R_th.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

            PointCloud P;
            pcl::transformPointCloud(P_g, P, R_th);

            objective.push_back(cloudEntropy(P));
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

        const float res = 0.03;

        pcl::PointXYZ min_point{};
        pcl::PointXYZ max_point{};
        pcl::getMinMax3D(point_cloud, min_point, max_point);

        Eigen::VectorXf x_edges = Eigen::VectorXf::LinSpaced(static_cast<long>((max_point.x - min_point.x) / res),
                                                             min_point.x, max_point.x);
        Eigen::VectorXf y_edges = Eigen::VectorXf::LinSpaced(static_cast<long>((max_point.y - min_point.y) / res),
                                                             min_point.y, max_point.y);

        std::vector<float> x_points;
        std::vector<float> y_points;
        for (auto p : point_cloud.points) {
            if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y))
                continue;
            x_points.push_back(p.x);
            y_points.push_back(p.y);
        }

        Eigen::VectorXf x_bins;
        x_bins = histogramCounts(x_points, x_edges);
        Eigen::VectorXf y_bins;
        y_bins = histogramCounts(y_points, y_edges);

        float H_x = histogramEntropy(x_bins);
        float H_y = histogramEntropy(y_bins);

        return H_x + H_y;
    }

    template<typename PointCloud>
    Eigen::VectorXf
    EntropyCompass<PointCloud>::histogramCounts(const std::vector<float> &points, const Eigen::VectorXf &edges) const {

        Eigen::VectorXf bins = Eigen::VectorXf::Zero(edges.size() - 1);
        for (size_t i = 0; i < bins.size(); ++i) {

            auto e_min = edges[i];
            auto e_max = edges[i + 1];

            for (auto p : points) {
                if (p < e_min || p > e_max)
                    continue;
                bins[i]++;
            }

        }

        return bins;
    }

    template<typename PointCloud>
    float
    EntropyCompass<PointCloud>::histogramEntropy(Eigen::VectorXf &histogram) const {

        Eigen::VectorXf p = histogram / histogram.sum();
        Eigen::VectorXf logp = p.array().log();
        return -(static_cast<float>(p.adjoint() * logp));
    }

}

#endif //SRC_STRUCTURAL_COMPASS_H
