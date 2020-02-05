//
// Created by armon on 2/5/20.
//

#ifndef SRC_PLANE_VISUALIZATION_H
#define SRC_PLANE_VISUALIZATION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace sbim_visualizations {

    void rotateToNormalDirection(const std::vector<Eigen::Vector3f> &in_vertices, const Eigen::Vector3f &normal,
                                 std::vector<Eigen::Vector3f> &out_vertices) {

        Eigen::Vector3f x1 = normal;
        Eigen::Vector3f x2(-x1[1], x1[0], 0);
        Eigen::Vector3f x3 = normal.cross(x2);

        x1 = x1.normalized();
        x2 = x2.normalized();
        x3 = x3.normalized();

        Eigen::Matrix3f R;
        R << x1[0], x2[0], x3[0],
                x1[1], x2[1], x3[1],
                x1[2], x2[2], x3[2];

        for (auto v : in_vertices) {
            out_vertices.emplace_back(R * v);
        }

    }

    std::vector<Eigen::Vector3f> planeTriangles(Eigen::Matrix<float, 4, 1> coef, float width, float height) {

        std::vector<Eigen::Vector3f> base_vertices;

        base_vertices.emplace_back(coef[3], width / 2, height / 2);
        base_vertices.emplace_back(coef[3], -width / 2, -height / 2);
        base_vertices.emplace_back(coef[3], width / 2, -height / 2);
        base_vertices.emplace_back(coef[3], width / 2, height / 2);
        base_vertices.emplace_back(coef[3], -width / 2, height / 2);
        base_vertices.emplace_back(coef[3], -width / 2, -height / 2);

        std::vector<Eigen::Vector3f> vertices;
        Eigen::Vector3f normal = coef.segment(0, 2);
        rotateToNormalDirection(base_vertices, normal, vertices);

        return vertices;
    }

}

#endif //SRC_PLANE_VISUALIZATION_H
