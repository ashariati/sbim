//
// Created by armon on 2/29/20.
//

#ifndef SRC_SEGMENT_VISUALIZATION_H
#define SRC_SEGMENT_VISUALIZATION_H

#include <cmath>

namespace sbim_visualizations {

    class AngleLessXY {

        std::vector<double> mean_;

    public:

        ~AngleLessXY() = default;

        AngleLessXY() = delete;

        explicit AngleLessXY(std::vector<double> &mean) {
            mean_ = mean;
        }

        bool operator()(std::vector<double> xi, std::vector<double> xj) {
            double angle_xi = std::atan2(xi[1] - mean_[1], xi[0] - mean_[0]);
            double angle_xj = std::atan2(xj[1] - mean_[1], xj[0] - mean_[0]);
            return angle_xi < angle_xj;
        }

    };


    void sortVerticesCCW(std::vector<std::vector<double>> &vertices) {

        size_t n = vertices.size();

        std::vector<double> center = std::vector<double>(3, 0);
        for (auto &v : vertices) {
            center[0] += v[0];
            center[1] += v[1];
            center[2] += v[2];
        }
        center[0] = center[0] / static_cast<double>(n);
        center[1] = center[1] / static_cast<double>(n);
        center[2] = center[2] / static_cast<double>(n);

        AngleLessXY angle_less_xy = AngleLessXY(center);
        std::sort(vertices.begin(), vertices.end(), angle_less_xy);

    }

    /*
     * convextToTriangles
     *
     * Assumes vertices are ordered in counter clockwise fashion
     */

    std::vector<std::vector<double>>
    convexToTriangles(std::vector<std::vector<double>> &vertices) {

        size_t n = vertices.size();

        // compute mean of vertices
        double x_c = 0;
        double y_c = 0;
        double z_c = 0;
        for (auto &v : vertices) {
            x_c += v[0];
            y_c += v[1];
            z_c += v[2];
        }
        x_c = x_c / static_cast<double>(n);
        y_c = y_c / static_cast<double>(n);
        z_c = z_c / static_cast<double>(n);

        // create triangles
        std::vector<std::vector<double>> triangles;
        for (size_t i = 0; i < n - 1; ++i) {

            std::vector<double> v1 = {x_c, y_c, z_c};
            std::vector<double> v2 = vertices[i];
            std::vector<double> v3 = vertices[i + 1];
            triangles.push_back(v1);
            triangles.push_back(v2);
            triangles.push_back(v3);

        }

        // last one
        std::vector<double> v1 = {x_c, y_c, z_c};
        std::vector<double> v2 = vertices[n - 1];
        std::vector<double> v3 = vertices[0];
        triangles.push_back(v1);
        triangles.push_back(v2);
        triangles.push_back(v3);


        return triangles;

    }

}

#endif //SRC_SEGMENT_VISUALIZATION_H
