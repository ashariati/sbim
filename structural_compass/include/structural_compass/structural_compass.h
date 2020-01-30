//
// Created by armon on 1/16/20.
//

#ifndef SRC_STRUCTURAL_COMPASS_H
#define SRC_STRUCTURAL_COMPASS_H

#include <math.h>
#include <limits>
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

        class SimpleKF {

            float r_;
            float q_;

            float sigma_max_;

        public:
            float mu_;
            float sigma_;

            SimpleKF() = delete;

            SimpleKF(float mu, float sigma, float r, float q, float sigma_max = std::numeric_limits<float>::max()) {
                mu_ = mu;
                sigma_ = sigma;
                r_ = r;
                q_ = q;
                sigma_max_ = sigma_max;
            }

            ~SimpleKF() = default;

            void predict() {
                sigma_ = sigma_ + r_;
                sigma_ = std::min(sigma_, sigma_max_);
            }

            void update(float x) {
                float K = sigma_ / (sigma_ + q_);
                mu_ = mu_ + K * (x - mu_);
                sigma_ = (1 - K) * sigma_;
            }

        } simple_kf_;

    public:

        EntropyCompass() : simple_kf_(0.0, M_PI / 6, 0.1 * (M_PI / 180.0), 1.0 * (M_PI / 180.0), M_PI_2) {}
        ~EntropyCompass() = default;

        Eigen::Matrix3f principalDirections(const PointCloud &point_cloud, const Eigen::Isometry3f &G_ws,
                                            const Eigen::Vector3f &gravity,
                                            std::vector<Eigen::Vector3f> &directions);

    };

    template<typename PointCloud>
    Eigen::Matrix3f
    EntropyCompass<PointCloud>::principalDirections(const PointCloud &point_cloud, const Eigen::Isometry3f &G_ws,
                                                    const Eigen::Vector3f &gravity,
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
        Eigen::Isometry3f G_sg(M);

        PointCloud P_g;
        pcl::transformPointCloud(point_cloud, P_g, G_sg.inverse());

        // find entropy-minimizing rotational correction
        PointCloud P_c;
        Eigen::Isometry3f R_cg = Eigen::Isometry3f::Identity();
        std::vector<float> objective;
        auto search_space = searchSpace(); // NOTE: ERROR -- This search space is w.r.t the arbitrary G_sg starting point..
        for (auto theta_cg : search_space) {
            R_cg.rotate(Eigen::AngleAxisf(theta_cg, Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud(P_g, P_c, R_cg);
            objective.push_back(cloudEntropy(P_c));
        }
        int opt_index = std::distance(objective.begin(), std::min_element(objective.begin(), objective.end()));
        float theta_cg_opt = search_space[opt_index];

        // directions.push_back(v3);
        // directions.push_back(v2);
        // directions.push_back(v1);

        Eigen::Isometry3f G_cg = Eigen::Isometry3f::Identity();
        G_cg.rotate(Eigen::AngleAxisf(theta_cg_opt, Eigen::Vector3f::UnitZ()));
        Eigen::Isometry3f G_cs = (G_cg * G_sg.inverse());

        return G_cs.rotation();

    }

    template<typename PointCloud>
    std::vector<float> EntropyCompass<PointCloud>::searchSpace() const {

        const int kNumSamples = 60;

        // float start = -M_PI_2;
        // float stop = M_PI_2;
        // float step = 3.0 * (M_PI / 180.0);
        float start = simple_kf_.mu_ - (3 * simple_kf_.sigma_);
        float stop = simple_kf_.mu_ + (3 * simple_kf_.sigma_);
        float step = (stop - start) / kNumSamples;

        std::vector<float> arange;
        float val = start;
        for (int i = 0; i <= kNumSamples; ++i) {
            arange.push_back(val);
            val += step;
        }

        return arange;
    }

    template<typename PointCloud>
    float EntropyCompass<PointCloud>::cloudEntropy(const PointCloud &point_cloud) const {

        const long kNumBins = 100;

        Eigen::ArrayXf e_data = Eigen::ArrayXf::LinSpaced(kNumBins + 1, -10.0, 10.0);
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

        int num_points = histogram.sum();
        assert(num_points != 0);
        Eigen::ArrayXf p = histogram / num_points;
        Eigen::ArrayXf logp = p.log();
        logp = logp.isFinite().select(logp, 0);
        return -(p * logp).sum();
    }

}

#endif //SRC_STRUCTURAL_COMPASS_H
