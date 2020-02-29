//
// Created by armon on 2/28/20.
//

#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <scene_parsing/layout_extractor.h>

TEST_CASE("linspace forward", "[Extractor]") {

    std::vector<double> points;
    points = layout_extractor::linspace(-3, 10, 7);

    for (auto p : points) {
        std::cout << p << std::endl;
    }

    REQUIRE(points.size() == 7);
    REQUIRE(points[0] == -3);
    REQUIRE(points[points.size() - 1] == 10);

}

TEST_CASE("linspace backward", "[Extractor]") {

    std::vector<double> points;
    points = layout_extractor::linspace(17, -4, 12);

    for (auto p : points) {
        std::cout << p << std::endl;
    }

    REQUIRE(points.size() == 12);
    REQUIRE(points[0] == 17);
    REQUIRE(points[points.size() - 1] == -4);

}

TEST_CASE("line_intersection", "[Extractor]") {

    std::vector<double> p1 = {0, 0};
    std::vector<double> p2 = {3, 3};
    std::vector<double> p3 = {2, 0};
    std::vector<double> p4 = {0, 1};

    std::vector<double> intersection = layout_extractor::line_intersection(p1, p2, p3, p4);

    std::cout << "px: " << intersection[0] << " py: " << intersection[1] << std::endl;

}

TEST_CASE("steiner_circle", "[Extractor]") {

    std::vector<std::vector<double>> ellipse = layout_extractor::steiner_ellipse(2, 1, 3);

    for (auto &p : ellipse) {
        std::cout << "px: " << p[0] << " py: " << p[1] << std::endl;
    }
    std::cout << std::endl;

}

TEST_CASE("rectangle", "[Extractor]") {

    std::vector<std::vector<double>> rect = layout_extractor::rectangle(2, 1);

    for (auto &p : rect) {
        std::cout << "px: " << p[0] << " py: " << p[1] << std::endl;
    }
    std::cout << std::endl;

}



