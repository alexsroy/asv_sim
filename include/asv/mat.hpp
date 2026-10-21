#pragma once
// Minimal fixed-size dense matrices, just enough for the Kalman filter
// (up to 4x4). Header-only, no dependencies. Not a general linear algebra
// library: swap in Eigen if the project ever needs more.

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace asv {

template <int R, int C>
struct Mat {
    std::array<double, R * C> a{};  // row-major, zero-initialised

    double& operator()(int r, int c) { return a[r * C + c]; }
    double operator()(int r, int c) const { return a[r * C + c]; }

    static Mat identity() {
        static_assert(R == C, "identity needs a square matrix");
        Mat m;
        for (int i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }
};

using Vec2 = Mat<2, 1>;
using Vec4 = Mat<4, 1>;
using Mat2 = Mat<2, 2>;
using Mat4 = Mat<4, 4>;

template <int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& x, const Mat<K, C>& y) {
    Mat<R, C> out;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) {
            double s = 0.0;
            for (int k = 0; k < K; ++k) s += x(i, k) * y(k, j);
            out(i, j) = s;
        }
    return out;
}

template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& x, const Mat<R, C>& y) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) out.a[i] = x.a[i] + y.a[i];
    return out;
}

template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& x, const Mat<R, C>& y) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) out.a[i] = x.a[i] - y.a[i];
    return out;
}

template <int R, int C>
Mat<R, C> operator*(double s, const Mat<R, C>& x) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) out.a[i] = s * x.a[i];
    return out;
}

template <int R, int C>
Mat<C, R> transpose(const Mat<R, C>& x) {
    Mat<C, R> out;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) out(j, i) = x(i, j);
    return out;
}

// Gauss-Jordan inverse with partial pivoting. Throws std::runtime_error if the
// matrix is singular (pivot negligible relative to the largest entry).
template <int N>
Mat<N, N> inverse(Mat<N, N> m) {
    Mat<N, N> inv = Mat<N, N>::identity();

    double scale = 0.0;
    for (double v : m.a) scale = std::fmax(scale, std::fabs(v));
    if (scale == 0.0) throw std::runtime_error("inverse: zero matrix");

    for (int col = 0; col < N; ++col) {
        int pivot = col;
        for (int r = col + 1; r < N; ++r)
            if (std::fabs(m(r, col)) > std::fabs(m(pivot, col))) pivot = r;
        if (std::fabs(m(pivot, col)) <= 1e-13 * scale)
            throw std::runtime_error("inverse: singular matrix");

        if (pivot != col) {
            for (int c = 0; c < N; ++c) {
                std::swap(m(pivot, c), m(col, c));
                std::swap(inv(pivot, c), inv(col, c));
            }
        }

        const double d = m(col, col);
        for (int c = 0; c < N; ++c) {
            m(col, c) /= d;
            inv(col, c) /= d;
        }
        for (int r = 0; r < N; ++r) {
            if (r == col) continue;
            const double f = m(r, col);
            if (f == 0.0) continue;
            for (int c = 0; c < N; ++c) {
                m(r, c) -= f * m(col, c);
                inv(r, c) -= f * inv(col, c);
            }
        }
    }
    return inv;
}

}  // namespace asv