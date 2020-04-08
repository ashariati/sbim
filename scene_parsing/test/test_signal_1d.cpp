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

    std::vector<int> y = signal_1d::conv(f, signal_1d::diff_kernel<int>());

    for (auto i : y) {
        std::cout << i << std::endl;
    }

    std::vector<int> gt{1, 1, 1, 1, 1, 1, 1, 1, 1, -9};

    REQUIRE(gt.size() == y.size());
    REQUIRE(std::equal(y.begin(), y.end(), gt.begin()));

}

TEST_CASE("Find peaks basic", "[Find Peaks]") {

    std::vector<double> f{8.4072, 2.5428, 8.1428, 2.4352, 9.2926, 3.4998, 1.9660, 2.5108, 6.1604, 4.7329};

    std::vector<double> mag;
    std::vector<float> loc;
    signal_1d::find_peaks<double>(f, 0, 0, mag, loc);

    // for (size_t i = 0; i < loc.size(); ++i) {
    //     std::cout << "loc: " << loc[i] << ", mag: " << mag[i] << std::endl;
    // }
    // std::cout << std::endl;

    std::vector<float> gt_loc{0, 2, 4, 8};
    std::vector<double> gt_mag{8.4072, 8.1428, 9.2926, 6.1604};

    REQUIRE(gt_loc.size() == loc.size());
    REQUIRE(std::equal(loc.begin(), loc.end(), gt_loc.begin()));
    REQUIRE(gt_mag.size() == mag.size());
    REQUIRE(std::equal(mag.begin(), mag.end(), gt_mag.begin()));

}

TEST_CASE("Find peaks negatives", "[Find Peaks]") {

    std::vector<double> f{-1.5928, -7.4572, -1.8572, -7.5648, -0.7074, -6.5002, -8.0340, -7.4892, -3.8396, -5.2671};

    std::vector<double> mag;
    std::vector<float> loc;
    signal_1d::find_peaks<double>(f, -10, 0, mag, loc);

    // for (size_t i = 0; i < loc.size(); ++i) {
    //     std::cout << "loc: " << loc[i] << ", mag: " << mag[i] << std::endl;
    // }
    // std::cout << std::endl;

    std::vector<float> gt_loc{0, 2, 4, 8};
    std::vector<double> gt_mag{-1.5928, -1.8572, -0.7074, -3.8396};

    REQUIRE(gt_loc.size() == loc.size());
    REQUIRE(std::equal(loc.begin(), loc.end(), gt_loc.begin()));
    REQUIRE(gt_mag.size() == mag.size());
    REQUIRE(std::equal(mag.begin(), mag.end(), gt_mag.begin()));

}

TEST_CASE("Find peaks with starting/ending zeros", "[Find Peaks]") {

    std::vector<double> f{0, 0, 0, 0.3517, 0.8308, 0.5853, 0.5497, 0.9172, 0.2858, 0.7572, 0, 0};

    std::vector<double> mag;
    std::vector<float> loc;
    signal_1d::find_peaks<double>(f, -10, 0, mag, loc);

    for (size_t i = 0; i < loc.size(); ++i) {
        std::cout << "loc: " << loc[i] << ", mag: " << mag[i] << std::endl;
    }
    std::cout << std::endl;

    std::vector<float> gt_loc{4, 7, 9};
    std::vector<double> gt_mag{0.8308, 0.9172, 0.7572};

    REQUIRE(gt_loc.size() == loc.size());
    REQUIRE(std::equal(loc.begin(), loc.end(), gt_loc.begin()));
    REQUIRE(gt_mag.size() == mag.size());
    REQUIRE(std::equal(mag.begin(), mag.end(), gt_mag.begin()));

}

TEST_CASE("Find peaks with prominence filter", "[Find Peaks]") {

    std::vector<double> f{0, 0, 0, 0.3517, 0.8308, 0.5853, 0.5497, 0.9172, 0.2858, 0.7572, 0, 0};

    std::vector<double> mag;
    std::vector<float> loc;
    signal_1d::find_peaks<double>(f, -10, 0.3243, mag, loc);

    for (size_t i = 0; i < loc.size(); ++i) {
        std::cout << "loc: " << loc[i] << ", mag: " << mag[i] << std::endl;
    }
    std::cout << std::endl;

    std::vector<float> gt_loc{7, 9};
    std::vector<double> gt_mag{0.9172, 0.7572};

    REQUIRE(gt_loc.size() == loc.size());
    REQUIRE(std::equal(loc.begin(), loc.end(), gt_loc.begin()));
    REQUIRE(gt_mag.size() == mag.size());
    REQUIRE(std::equal(mag.begin(), mag.end(), gt_mag.begin()));

}

TEST_CASE("Find peaks with end peak", "[Find Peaks]") {

    std::vector<double> f{0, 0, 0, 0.3517, 0.8308, 0.5853, 0.5497, 0.9172, 0.2858, 0.7572, 0, 10};

    std::vector<double> mag;
    std::vector<float> loc;
    signal_1d::find_peaks<double>(f, -10, 0, mag, loc);

    for (size_t i = 0; i < loc.size(); ++i) {
        std::cout << "loc: " << loc[i] << ", mag: " << mag[i] << std::endl;
    }
    std::cout << std::endl;

    std::vector<float> gt_loc{4, 7, 9, 11};
    std::vector<double> gt_mag{0.8308, 0.9172, 0.7572, 10};

    REQUIRE(gt_loc.size() == loc.size());
    REQUIRE(std::equal(loc.begin(), loc.end(), gt_loc.begin()));
    REQUIRE(gt_mag.size() == mag.size());
    REQUIRE(std::equal(mag.begin(), mag.end(), gt_mag.begin()));

}

TEST_CASE("Find peaks speed", "[Find Peaks]") {

    std::vector<int> counts;
    for (size_t i = 0; i < 200; ++i) {
        int c = std::rand() % 1000;
        counts.push_back(c);
        std::cout << c << std::endl;
    }

    std::vector<int> mag;
    std::vector<float> loc;
    signal_1d::find_peaks<int>(counts, -10, 0, mag, loc);

}




