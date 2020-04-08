//
// Created by armon on 2/3/20.
//

#ifndef SRC_SIGNAL_1D_H
#define SRC_SIGNAL_1D_H

#include <cmath>

namespace signal_1d {

    template<typename T>
    void print_vector(const std::vector<T> &vec) {
        for (auto &y : vec) {
            std::cout << y << " ";
        }
        std::cout << std::endl << std::endl;
    }

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
    std::vector<T> conv(const std::vector<T> &f, const std::vector<T> &g) {

        int m = f.size();
        int n = g.size();

        std::vector<T> y(m + n - 1, 0);
        for (int k = 0; k < y.size(); ++k) {

            int j0 = k - n + 1;
            for (int j = std::max(0, j0); j <= std::min(k, m - 1); ++j) {
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
    void find_peaks(const std::vector<T> &f, const T min_height, const T min_prominence,
                    std::vector<T> &magnitude, std::vector<float> &location) {

        // subtract min element from f for derivative
        T min_f = *std::min_element(f.begin(), f.end());
        std::vector<T> f_shift(f.size());
        std::transform(f.begin(), f.end(), f_shift.begin(), [min_f](T t) { return t - min_f; });

        // first derivative
        std::vector<T> df = conv<T>(f_shift, diff_kernel<T>());

        // ensures first of repeated value selected
        for (auto &di : df) {
            // make derivative values close to zero negative
            di = (std::fabs(di) < 1e-8) ? -1e-8 : di;
        }

        // find zero-crossing points of the derivative
        std::vector<T> peaks_valleys;
        std::vector<size_t> zero_crossing_ids;
        for (size_t i = 1; i < df.size(); ++i) {
            if (std::signbit(df[i] * df[i - 1])) {
                zero_crossing_ids.push_back(i - 1);
                peaks_valleys.push_back(f[i - 1]);
            }
        }

        // loop through peaks
        int start = (peaks_valleys[1] > peaks_valleys[0]) ? 1 : 0;
        for (int i = start; i < peaks_valleys.size(); i += 2) {

            T pi = peaks_valleys[i];

            // find lowest valley between fi and the first peak larger than fi on its left
            T left_ref = (i > 0) ? std::numeric_limits<T>::max() : min_f;
            for (int j = i - 1; j >= 0; j -= 2) {
                T vj = peaks_valleys[j];
                left_ref = std::min(left_ref, vj);
                if (peaks_valleys[j - 1] > pi) {
                    break;
                }
            }

            // find lowest valley between fi and the first peak larger than fi on its right
            T right_ref = (i + 1 < peaks_valleys.size()) ? std::numeric_limits<T>::max() : min_f;
            for (int j = i + 1; j < peaks_valleys.size(); j += 2) {
                T vj = peaks_valleys[j];
                right_ref = std::min(right_ref, vj);
                if (peaks_valleys[std::min(j + 1, static_cast<int>(peaks_valleys.size()) - 1)] > pi) {
                    break;
                }
            }

            // save if thresholds are satisfied
            T ref = std::max(left_ref, right_ref);
            if ((pi > ref + min_prominence) && (pi > min_height)) {
                magnitude.push_back(pi);
                location.push_back(zero_crossing_ids[i]);
            }

        }

    }

}


#endif //SRC_SIGNAL_1D_H
