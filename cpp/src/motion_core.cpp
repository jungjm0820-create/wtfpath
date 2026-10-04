// motion_core — background multithreaded FK math for Motion Sculpt.
// Builds to motion_core_cpp.pyd (Windows) / .so (Linux/macOS) via pybind11.
// Python fallback (fk_numpy) mirrors these functions, so the addon runs
// without compiling; building only unlocks 60fps multi-frame throughput.
//
// Functions:
//   compose_globals(rest_rel[N,4,4], offsets[N,4,4], parents[N] int) -> globals[N,4,4]
//   skin_deform(verts[V,3], ghost_globals[G,B,4,4], weights[V,K], bone_idx[V,K],
//               rest_global_inv[B,4,4]) -> ghost_verts[G,V,3]
//   ccd_step(...) — single CCD iteration helper (full solver stays in Python
//   for clarity; move here once profiling demands it).

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

namespace py = pybind11;

using Mat4 = double[4][4];

static void mat_mul(const double A[4][4], const double B[4][4], double Out[4][4]) {
    double T[4][4] = {};
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < 4; ++k) {
            double a = A[i][k];
            for (int j = 0; j < 4; ++j) T[i][j] += a * B[k][j];
        }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) Out[i][j] = T[i][j];
}

// globals[i] = globals[parent[i]] @ rest_rel[i] @ offsets[i]  (root: I @ ...)
py::array_t<double> compose_globals(py::array_t<double> rest_rel,
                                    py::array_t<double> offsets,
                                    py::array_t<int> parents) {
    auto r = rest_rel.unchecked<3>();   // [N,4,4]
    auto o = offsets.unchecked<3>();
    auto p = parents.unchecked<1>();
    ssize_t N = r.shape(0);
    py::array_t<double> out({N, 4, 4});
    auto g = out.mutable_unchecked<3>();

    // depth order assumed: caller passes topo-sorted arrays (roots first).
    double tmp[4][4], gm[4][4], R[4][4], O[4][4];
    for (ssize_t i = 0; i < N; ++i) {
        for (int a = 0; a < 4; ++a)
            for (int b = 0; b < 4; ++b) { R[a][b] = r(i, a, b); O[a][b] = o(i, a, b); }
        mat_mul(R, O, tmp);
        int par = p(i);
        if (par < 0) {
            for (int a = 0; a < 4; ++a)
                for (int b = 0; b < 4; ++b) g(i, a, b) = tmp[a][b];
        } else {
            double P[4][4];
            for (int a = 0; a < 4; ++a)
                for (int b = 0; b < 4; ++b) P[a][b] = g(par, a, b);
            mat_mul(P, tmp, gm);
            for (int a = 0; a < 4; ++a)
                for (int b = 0; b < 4; ++b) g(i, a, b) = gm[a][b];
        }
    }
    return out;
}

// Linear-blend skinning across G ghost frames, multithreaded over ghosts.
// verts: [V,3], weights: [V,K], bone_idx: [V,K] int,
// ghosts: [G,B,4,4], rest_inv: [B,4,4]  -> [G,V,3]
py::array_t<double> skin_deform(py::array_t<double> verts,
                                py::array_t<double> ghosts,
                                py::array_t<double> weights,
                                py::array_t<int> bone_idx,
                                py::array_t<double> rest_inv) {
    auto V  = verts.unchecked<2>();
    auto G  = ghosts.unchecked<4>();
    auto W  = weights.unchecked<2>();
    auto BI = bone_idx.unchecked<2>();
    auto RI = rest_inv.unchecked<3>();
    ssize_t nV = V.shape(0), nG = G.shape(0), nB = G.shape(1), K = W.shape(1);

    py::array_t<double> out({nG, nV, 3});
    auto O = out.mutable_unchecked<3>();

    unsigned nthreads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> pool;
    auto work = [&](ssize_t g0, ssize_t g1) {
        double M[4][4], R[4][4], GM[4][4];
        for (ssize_t g = g0; g < g1; ++g) {
            for (ssize_t v = 0; v < nV; ++v) {
                double px = V(v, 0), py_ = V(v, 1), pz = V(v, 2);
                double ax = 0, ay = 0, az = 0;
                for (ssize_t k = 0; k < K; ++k) {
                    double w = W(v, k);
                    if (w == 0.0) continue;
                    int b = BI(v, k);
                    if (b < 0 || b >= nB) continue;
                    for (int a = 0; a < 4; ++a)
                        for (int c = 0; c < 4; ++c) {
                            GM[a][c] = G(g, b, a, c);
                            R[a][c]  = RI(b, a, c);
                        }
                    mat_mul(GM, R, M);
                    ax += w * (M[0][0]*px + M[0][1]*py_ + M[0][2]*pz + M[0][3]);
                    ay += w * (M[1][0]*px + M[1][1]*py_ + M[1][2]*pz + M[1][3]);
                    az += w * (M[2][0]*px + M[2][1]*py_ + M[2][2]*pz + M[2][3]);
                }
                O(g, v, 0) = ax; O(g, v, 1) = ay; O(g, v, 2) = az;
            }
        }
    };
    ssize_t per = (nG + nthreads - 1) / nthreads;
    for (unsigned t = 0; t < nthreads; ++t) {
        ssize_t g0 = t * per, g1 = std::min(nG, g0 + per);
        if (g0 >= g1) break;
        pool.emplace_back(work, g0, g1);
    }
    for (auto &th : pool) th.join();
    return out;
}

PYBIND11_MODULE(motion_core_cpp, m) {
    m.doc() = "Motion Sculpt FK math core (multithreaded)";
    m.def("compose_globals", &compose_globals, "Compose FK global matrices",
          py::arg("rest_rel"), py::arg("offsets"), py::arg("parents"));
    m.def("skin_deform", &skin_deform, "LBS skinning for onion ghosts",
          py::arg("verts"), py::arg("ghosts"), py::arg("weights"),
          py::arg("bone_idx"), py::arg("rest_inv"));
}
