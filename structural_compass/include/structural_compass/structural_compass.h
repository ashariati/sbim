//
// Created by armon on 1/16/20.
//

#ifndef SRC_STRUCTURAL_COMPASS_H
#define SRC_STRUCTURAL_COMPASS_H

#include <vector>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Core>

namespace structural_compass {

    template<typename PointCloud>
    class EntropyCompass {

    public:

        Eigen::Matrix3f principalDirections(const PointCloud &point_cloud, const Eigen::Vector3f &gravity,
                                            std::vector<Eigen::Vector3f> &directions);

        ~EntropyCompass() = default;

    };

    template<typename PointCloud>
    Eigen::Matrix3f
    EntropyCompass<PointCloud>::principalDirections(const PointCloud &point_cloud, const Eigen::Vector3f &gravity,
                                                    std::vector<Eigen::Vector3f> &directions) {

        // compute vectors to form orthonormal bases
        Eigen::Vector3f v3 = -gravity.normalized();
        Eigen::Vector3f v1;
        v1 << -v3[1], v3[0], 0.0;
        Eigen::Vector3f v2;
        v2 = v3.cross(v1);

        // build transform
        Eigen::Matrix4f M;
        M << v1[0], v2[0], v3[0], 0,
                v1[1], v2[1], v3[1], 0,
                v1[2], v2[2], v3[2], 0,
                0, 0, 0, 1;
        Eigen::Isometry3f G_cg(M);

        PointCloud P_g;
        pcl::transformPointCloud(point_cloud, P_g, G_cg);

        directions.push_back(v3);
        directions.push_back(v2);
        directions.push_back(v1);

        return G_cg.rotation();

    }

}

#endif //SRC_STRUCTURAL_COMPASS_H
