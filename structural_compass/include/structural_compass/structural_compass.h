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

        bool is_initialized_;

        Eigen::Matrix3f R_ws_;
        Eigen::Matrix3f R_sg_;
        Eigen::Matrix3f R_cg_;

        class YawTracker {

            float r_;
            float q_;

            float sigma_max_;

        public:
            float theta_;
            float sigma_;

            YawTracker() = delete;

            YawTracker(float mu, float sigma, float r, float q, float sigma_max = std::numeric_limits<float>::max()) {
                theta_ = mu;
                sigma_ = sigma;
                r_ = r;
                q_ = q;
                sigma_max_ = sigma_max;
            }

            ~YawTracker() = default;

            void predict(float dtheta) {
                theta_ = theta_ + dtheta;
                sigma_ = sigma_ + r_;
                sigma_ = std::min(sigma_, sigma_max_);
            }

            void update(float theta) {
                float K = sigma_ / (sigma_ + q_);
                theta_ = theta_ + K * (theta - theta_);
                sigma_ = (1 - K) * sigma_;
            }

        } yaw_tracker_;

        std::vector<float> searchSpace(float theta, float radius, const int num_samples = 60) const;
        float cloudEntropy(const PointCloud &point_cloud) const;
        float histogramEntropy(Eigen::ArrayXf &histogram) const;

        Eigen::Matrix3f gravityAlignedFrame(const Eigen::Vector3f &gravity) const;

        float zAxisRotationToYaw(const Eigen::Matrix3f &R_z) const;

        Eigen::Matrix3f minEntropySearch(const PointCloud &P_g, const std::vector<float> &search_space) const;


    public:

        EntropyCompass() : yaw_tracker_(0.0, M_PI_4, 0.1 * (M_PI / 180.0), 1.0 * (M_PI / 180.0), M_PI_4) {
            is_initialized_ = false;
        }

        ~EntropyCompass() = default;

        Eigen::Matrix3f principalDirections(const PointCloud &point_cloud, const Eigen::Matrix3f &R_ws,
                                            const Eigen::Vector3f &gravity,
                                            std::vector<Eigen::Vector3f> &directions);

    };

    template<typename PointCloud>
    Eigen::Matrix3f
    EntropyCompass<PointCloud>::principalDirections(const PointCloud &point_cloud, const Eigen::Matrix3f &R_ws,
                                                    const Eigen::Vector3f &gravity,
                                                    std::vector<Eigen::Vector3f> &directions) {

        // intermediary gravity-aligned frame
        Eigen::Matrix3f R_sg = gravityAlignedFrame(gravity);

        // transform point cloud to gravity-aligned frame
        Eigen::Isometry3f G_gs = Eigen::Isometry3f::Identity();
        G_gs.rotate(R_sg.transpose());
        PointCloud P_g;
        pcl::transformPointCloud(point_cloud, P_g, G_gs);

        if (is_initialized_) {

            // rotation from current sensor frame to last sensor frame
            Eigen::Matrix3f R_ss = R_ws_.transpose() * R_ws;

            // rotation from current to last gravity-aligned frame
            Eigen::Matrix3f R_gg = R_sg_.transpose() * R_ss * R_sg;

            // predict current compass orientation w.r.t. current gravity-aligned frame
            Eigen::Matrix3f R_cg_hat = R_cg_ * R_gg;
            auto theta_hat = zAxisRotationToYaw(R_cg_hat);
            yaw_tracker_.predict(theta_hat - yaw_tracker_.theta_);

        }

        // search for true compass orientation around estimate
        auto search_space = searchSpace(yaw_tracker_.theta_, 2 * yaw_tracker_.sigma_);
        Eigen::Matrix3f R_cg = minEntropySearch(P_g, search_space);

        // save results for next iteration
        auto theta = zAxisRotationToYaw(R_cg);
        yaw_tracker_.update(theta);
        R_ws_ = R_ws;
        R_sg_ = R_sg;
        R_cg_ = R_cg;


        // // directions.push_back(v3);
        // // directions.push_back(v2);
        // // directions.push_back(v1);

        return R_cg * R_sg.transpose();

    }

    template<typename PointCloud>
    std::vector<float> EntropyCompass<PointCloud>::searchSpace(float theta, float radius, const int num_samples) const {

        float start = theta - radius;
        float stop = theta + radius;
        float step = (stop - start) / num_samples;

        std::vector<float> arange;
        float val = start;
        for (int i = 0; i <= num_samples; ++i) {
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

    template<typename PointCloud>
    Eigen::Matrix3f EntropyCompass<PointCloud>::gravityAlignedFrame(const Eigen::Vector3f &gravity) const {

        Eigen::Vector3f v3 = -gravity.normalized();
        Eigen::Vector3f v1;
        v1 << -v3[1], v3[0], 0.0;
        Eigen::Vector3f v2;
        v2 = v3.cross(v1);
        Eigen::Matrix3f R_sg;
        R_sg << v1[0], v2[0], v3[0],
                v1[1], v2[1], v3[1],
                v1[2], v2[2], v3[2];

        return R_sg;
    }

    template<typename PointCloud>
    float EntropyCompass<PointCloud>::zAxisRotationToYaw(const Eigen::Matrix3f &R_z) const {
        return std::asin(R_z(0, 0));
    }

    template<typename PointCloud>
    Eigen::Matrix3f
    EntropyCompass<PointCloud>::minEntropySearch(const PointCloud &P_g, const std::vector<float> &search_space) const {

        // find entropy-minimizing rotational correction
        PointCloud P_c;
        std::vector<float> objective;
        for (auto theta_cg : search_space) {

            Eigen::Isometry3f G_cg = Eigen::Isometry3f::Identity();
            G_cg.rotate(Eigen::AngleAxisf(theta_cg, Eigen::Vector3f::UnitZ()));

            pcl::transformPointCloud(P_g, P_c, G_cg);
            objective.push_back(cloudEntropy(P_c));
        }
        int opt_index = std::distance(objective.begin(), std::min_element(objective.begin(), objective.end()));
        float theta_cg_opt = search_space[opt_index];

        Eigen::Matrix3f R_cg_opt = Eigen::AngleAxisf(theta_cg_opt, Eigen::Vector3f::UnitZ()).toRotationMatrix();

        return R_cg_opt;
    }

}

#endif //SRC_STRUCTURAL_COMPASS_H
