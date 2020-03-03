//
// Created by armon on 1/22/20.
//

#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include <sbim_visualizations/convex.h>

TEST_CASE("Vertex Sort", "[Convex]") {

    std::vector<std::vector<double>> vertices;

    std::vector<double> v1 = {-3, 0, 0};
    std::vector<double> v2 = {-3, -2, 0};
    std::vector<double> v3 = {1, 4, 0};
    std::vector<double> v4 = {1, -2, 0};

    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    vertices.push_back(v4);

    for (auto &v : vertices) {
        std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }
    std::cout << std::endl;

    sbim_visualizations::sortVerticesCCW(vertices);

    for (auto &v : vertices) {
        std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }
    std::cout << std::endl;

}


