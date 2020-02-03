//
// Created by armon on 2/3/20.
//

#ifndef SRC_SIGNAL_1D_H
#define SRC_SIGNAL_1D_H

#include <cmath>

namespace signal_1d {

    template<typename T>
    std::vector<int> histogram_counts(const std::vector<T> &signal, const T min, const T max, const T bin_width) {

        size_t num_bins = std::ceil((max - min) / bin_width);
        std::vector<int> bins = std::vector<int>(num_bins, 0);
        for (auto s : signal) {

            if (s > max || s < min) {
                continue;
            }

            int bin_idx = static_cast<int>((s - min) / bin_width);
            bins[bin_idx] += 1;
        }

        return bins;
    }

    template<typename T>
    std::vector<T> filter(const std::vector<T> &f, const std::vector<T> &g) {

        size_t m = f.size();
        size_t n = g.size();

        std::vector<T> y(m + n - 1, 0);
        for (size_t k = 0; k < y.size(); ++k) {

            intmax_t j0 = static_cast<intmax_t>(k) - static_cast<intmax_t>(n) + 1;
            for (size_t j = std::max(intmax_t(0), j0); j <= std::min(k, m); ++j) {

                y[k] += f[j] * g[k - j];

            }

        }

        return y;
    }

    template<typename T>
    std::vector<T> diff_kernel() {
        return std::vector<T>{1, -1};
    }

    template<typename T>
    void find_peaks(const std::vector<T> &signal) {

    }

}


#endif //SRC_SIGNAL_1D_H
