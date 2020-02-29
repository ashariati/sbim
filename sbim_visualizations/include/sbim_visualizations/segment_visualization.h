//
// Created by armon on 2/29/20.
//

#ifndef SRC_SEGMENT_VISUALIZATION_H
#define SRC_SEGMENT_VISUALIZATION_H

#endif //SRC_SEGMENT_VISUALIZATION_H

namespace sbim_visualizations {

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