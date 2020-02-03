//
// Created by armon on 2/3/20.
//

#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <scene_parsing/signal_1d.h>

TEST_CASE("Histogram easy", "[Histogram]") {

    std::vector<double> signal{3.2787, 0.1786, 4.2456, 4.6700, 3.3937, 3.7887, 3.7157, 1.9611, 3.2774, 0.8559};

    std::vector<int> hist = signal_1d::histogram_counts<double>(signal, 0, 5, 1);

    std::vector<int> gt{2, 1, 0, 5, 2};

    REQUIRE(gt.size() == hist.size());
    REQUIRE(std::equal(hist.begin(), hist.end(), gt.begin()));
}

TEST_CASE("Histogram non-clean bin width", "[Histogram]") {

    std::vector<double> signal{4.2363, 0.1910, 1.6615, 0.2770, 0.5828, 4.9407, 4.1690, 1.9026, 5.7013, 0.2067};

    std::vector<int> hist = signal_1d::histogram_counts<double>(signal, 0, 6, 0.8);

    std::vector<int> gt{4, 0, 2, 0, 0, 2, 1, 1};

    REQUIRE(gt.size() == hist.size());
    REQUIRE(std::equal(hist.begin(), hist.end(), gt.begin()));
}

TEST_CASE("Histogram out-of-range min", "[Histogram]") {

    std::vector<double> signal{1.7550, 1.5262, 3.0621, 3.1808, 0.7475, 1.9591, 1.7823, 2.5853, 2.8375, 3.0187};

    std::vector<int> hist = signal_1d::histogram_counts<double>(signal, 1, 4, 1.0);

    std::vector<int> gt{4, 2, 3};

    REQUIRE(gt.size() == hist.size());
    REQUIRE(std::equal(hist.begin(), hist.end(), gt.begin()));
}

TEST_CASE("Histogram out-of-range max", "[Histogram]") {

    std::vector<double> signal{3.2787, 0.1786, 4.2456, 4.6700, 3.3937, 3.7887, 3.7157, 1.9611, 3.2774, 0.8559};

    std::vector<int> hist = signal_1d::histogram_counts<double>(signal, 0, 4, 1);

    std::vector<int> gt{2, 1, 0, 5};

    REQUIRE(gt.size() == hist.size());
    REQUIRE(std::equal(hist.begin(), hist.end(), gt.begin()));
}

TEST_CASE("Filter easy", "[Filter]") {

    std::vector<int> f{1, 2, 3, 4, 5, 6, 7, 8, 9};

    std::vector<int> y = signal_1d::filter(f, signal_1d::diff_kernel<int>());

    std::vector<int> gt{1, 1, 1, 1, 1, 1, 1, 1, 1, -9};

    REQUIRE(gt.size() == y.size());
    REQUIRE(std::equal(y.begin(), y.end(), gt.begin()));

}