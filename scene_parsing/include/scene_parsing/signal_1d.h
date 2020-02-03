//
// Created by armon on 2/3/20.
//

#ifndef SRC_SIGNAL_1D_H
#define SRC_SIGNAL_1D_H

#include <math.h>

namespace signal_1d {

    template<typename T>
    std::vector<int> histogram(const std::vector<T> &signal, const T min, const T max, const T bin_width) {

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
    std::vector<T> conv(const std::vector<T> &signal, const std::vector<T> filter) {

        return std::vector<int>();
    }

    template<typename T>
    void find_peaks(const std::vector<T> &signal) {

    }

}


#endif //SRC_SIGNAL_1D_H
