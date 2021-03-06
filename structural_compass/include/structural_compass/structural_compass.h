//
// Created by armon on 1/16/20.
//

#ifndef SRC_STRUCTURAL_COMPASS_H
#define SRC_STRUCTURAL_COMPASS_H

#include <cmath>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Core>

namespace structural_compass {

    class EntropyCompass {

    private:

        bool is_initialized_;

        Eigen::Matrix3f R_ws_;
        Eigen::Matrix3f R_gs_;
        Eigen::Matrix3f R_cg_;

        class YawTracker {

            double r_;
            double q_;

        public:
            double theta_;
            double sigma_;

            YawTracker() = delete;

            YawTracker(double mu, double sigma, double r, double q) {
                theta_ = mu;
                sigma_ = sigma;
                r_ = r;
                q_ = q;
            }

            ~YawTracker() = default;

            void predict(double dtheta) {
                theta_ = theta_ + dtheta;
                sigma_ = sigma_ + r_;
                sigma_ = std::min(sigma_, M_PI_4);
            }

            void update(double theta) {
                double K = sigma_ / (sigma_ + q_);
                theta_ = theta_ + K * (theta - theta_);
                sigma_ = (1 - K) * sigma_;
            }

        } yaw_tracker_;

        std::vector<float> searchSpace(float theta, float radius, const int num_samples = 60) const;

        template<typename PointT>
        float cloudEntropy(const pcl::PointCloud<PointT> &point_cloud) const;

        float histogramEntropy(const Eigen::ArrayXf &histogram) const;

        Eigen::Matrix3f gravityAlignedFrame(const Eigen::Vector3f &gravity) const;

        float zAxisRotationToYaw(const Eigen::Matrix3f &R_z) const;

        template<typename PointT>
        Eigen::Matrix3f
        minEntropySearch(const pcl::PointCloud<PointT> &P_g, const std::vector<float> &search_space) const;


    public:

        EntropyCompass() : yaw_tracker_(0.0, M_PI_4, 1.0 * (M_PI / 180.0), 1.0 * (M_PI / 180.0)) {
            is_initialized_ = false;
        }

        ~EntropyCompass() = default;

        template<typename PointT>
        Eigen::Matrix3f principalDirections(const pcl::PointCloud<PointT> &point_cloud, const Eigen::Matrix3f &R_ws,
                                            const Eigen::Vector3f &gravity,
                                            std::vector<Eigen::Vector3f> &directions);

    };

    template<typename PointT>
    Eigen::Matrix3f
    EntropyCompass::principalDirections(const pcl::PointCloud<PointT> &point_cloud, const Eigen::Matrix3f &R_ws,
                                        const Eigen::Vector3f &gravity,
                                        std::vector<Eigen::Vector3f> &directions) {

        // intermediary gravity-aligned frame
        Eigen::Matrix3f R_gs = gravityAlignedFrame(gravity);

        // transform point cloud to gravity-aligned frame
        Eigen::Isometry3f G_gs = Eigen::Isometry3f::Identity();
        G_gs.rotate(R_gs);
        pcl::PointCloud<PointT> P_g;
        pcl::transformPointCloud(point_cloud, P_g, G_gs);

        if (is_initialized_) {

            // rotation from current sensor frame to last sensor frame
            Eigen::Matrix3f R_ss = R_ws_.transpose() * R_ws;

            // rotation from current to last gravity-aligned frame
            Eigen::Matrix3f R_gg = R_gs_ * R_ss * R_gs.transpose();

            // predict current compass orientation w.r.t. current gravity-aligned frame
            Eigen::Matrix3f R_cg_hat = R_cg_ * R_gg;
            auto theta_hat = zAxisRotationToYaw(R_cg_hat);
            yaw_tracker_.predict(theta_hat - yaw_tracker_.theta_);

        } else {
            is_initialized_ = true;
        }

        // search for true compass orientation around estimate
        auto search_space = searchSpace(yaw_tracker_.theta_, 2 * yaw_tracker_.sigma_);
        Eigen::Matrix3f R_cg = minEntropySearch(P_g, search_space);

        // save results for next iteration
        auto theta = zAxisRotationToYaw(R_cg);
        yaw_tracker_.update(theta);
        R_ws_ = R_ws;
        R_gs_ = R_gs;
        R_cg_ = R_cg;

        // compass transform
        Eigen::Matrix3f R_cs = R_cg * R_gs;

        // principal directions
        Eigen::Vector3f vz{0.0, 0.0, 1.0};
        Eigen::Vector3f vx{1.0, 0.0, 0.0};
        Eigen::Vector3f vy{0.0, 1.0, 0.0};
        Eigen::Vector3f v4 = (vx + vy).normalized();
        Eigen::Vector3f v5 = (-vx + vy).normalized();

        directions.push_back(vz);
        directions.push_back(vx);
        directions.push_back(vy);
        directions.push_back(v4);
        directions.push_back(v5);

        return R_cs;

    }

    std::vector<float> EntropyCompass::searchSpace(float theta, float radius, const int num_samples) const {

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

    template<typename PointT>
    float EntropyCompass::cloudEntropy(const pcl::PointCloud<PointT> &point_cloud) const {

        float bin_width = 0.05;
        float radius = 10.0;
        int num_bins = std::ceil((2 * radius) / bin_width);

        Eigen::ArrayXf x_bins = Eigen::ArrayXf::Zero(num_bins);
        Eigen::ArrayXf y_bins = Eigen::ArrayXf::Zero(num_bins);

        for (auto p : point_cloud.points) {

            if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y))
                continue;

            bool in_range_x = (p.x < radius) && (p.x > -radius);
            bool in_range_y = (p.y < radius) && (p.y > -radius);

            if (!(in_range_x && in_range_y)) {
                continue;
            }

            float x_distance = (p.x - (-radius));
            float y_distance = (p.y - (-radius));
            int x_index = static_cast<int>(x_distance / bin_width);
            int y_index = static_cast<int>(y_distance / bin_width);

            x_bins[x_index] += 1;
            y_bins[y_index] += 1;

        }

        float H_x = histogramEntropy(x_bins);
        float H_y = histogramEntropy(y_bins);

        return H_x + H_y;
    }

    float EntropyCompass::histogramEntropy(const Eigen::ArrayXf &histogram) const {

        int num_points = histogram.sum();
        assert(num_points != 0);
        Eigen::ArrayXf p = histogram / num_points;
        Eigen::ArrayXf logp = p.log();
        logp = logp.isFinite().select(logp, 0);
        return -(p * logp).sum();
    }

    Eigen::Matrix3f EntropyCompass::gravityAlignedFrame(const Eigen::Vector3f &gravity) const {

        Eigen::Vector3f v3 = -gravity;
        v3 = v3.normalized();

        Eigen::Vector3f v1;
        v1 << v3[2], 0.0, -v3[0];
        v1 = v1.normalized();

        Eigen::Vector3f v2;
        v2 = v3.cross(v1);
        v2 = v2.normalized();

        Eigen::Matrix3f R_gs;
        R_gs << v1[0], v1[1], v1[2],
                v2[0], v2[1], v2[2],
                v3[0], v3[1], v3[2];

        return R_gs;
    }

    float EntropyCompass::zAxisRotationToYaw(const Eigen::Matrix3f &R_z) const {
        return std::atan2(R_z(1, 0), R_z(0, 0));
    }

    template<typename PointT>
    Eigen::Matrix3f
    EntropyCompass::minEntropySearch(const pcl::PointCloud<PointT> &P_g, const std::vector<float> &search_space) const {

        // find entropy-minimizing rotational correction
        pcl::PointCloud<PointT> P_c;
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
