#pragma once
// Small deterministic RNG for sensor noise.
//
// std::mt19937's output sequence is fixed by the standard, but the standard
// distributions (std::normal_distribution etc.) are NOT, so they give different
// numbers on MSVC vs libstdc++ vs libc++. This builds the distributions by hand
// on top of mt19937 so a given seed reproduces the same noise everywhere
// (up to last-bit differences in libm).

#include <cmath>
#include <cstdint>
#include <random>

namespace asv {

class Rng {
public:
    explicit Rng(std::uint32_t seed = 1) : gen_(seed) {}

    // Uniform in the open interval (0, 1): never exactly 0 (safe for log()).
    double uniform() {
        return (static_cast<double>(gen_()) + 0.5) / 4294967296.0;
    }

    double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

    // True with probability p.
    bool chance(double p) { return uniform() < p; }

    // Gaussian via Box-Muller (generates a pair, caches the spare).
    double normal(double mean = 0.0, double sigma = 1.0) {
        if (have_spare_) {
            have_spare_ = false;
            return mean + sigma * spare_;
        }
        constexpr double kTwoPi = 6.283185307179586;
        const double r = std::sqrt(-2.0 * std::log(uniform()));
        const double a = kTwoPi * uniform();
        spare_ = r * std::sin(a);
        have_spare_ = true;
        return mean + sigma * r * std::cos(a);
    }

    // Poisson (Knuth). Fine for the small means used for clutter.
    int poisson(double mean) {
        if (mean <= 0.0) return 0;
        const double limit = std::exp(-mean);
        int k = 0;
        double p = 1.0;
        do {
            ++k;
            p *= uniform();
        } while (p > limit);
        return k - 1;
    }

private:
    std::mt19937 gen_;
    double spare_ = 0.0;
    bool have_spare_ = false;
};

}  // namespace asv