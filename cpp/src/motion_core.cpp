// motion_core — background multithreaded FK math for Motion Sculpt.
// Builds to motion_core_cpp.pyd (Windows) / .so (Linux/macOS) via pybind11.
// Python fallback (fk_numpy) mirrors these functions, so the addon runs
// without compiling; building only unlocks 60fps multi-frame throughput.
//
// Functions:
//   compose_globals(rest_rel[N,4,4], offsets[N,4,4], parents[N] int) -> globals[N,4,4]
//   skin_deform(verts[V,3], ghost_globals[G,B,4,4], weights[V,K], bone_idx[V,K],
//               rest_global_inv[B,4,4]) -> ghost_verts[G,V,3]
//
// MSVC note: Windows MSVC has no POSIX `ssize_t`, so all indices use
// `py::ssize_t` (== Py_ssize_t, MSVC-safe). Do not reintroduce bare `ssize_t`.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

namespace py = pybind11;

static void mat_mul(const double A[4][4], const double B[4][4], double Out[4][4]) {
    double T[4][4] = {};
    for (int i = 0; i < 4; ++i) {
        for (int k = 0; k < 4; ++k) {
            const double a = A[i][k];
            for (int j = 0; j < 4; ++j) {
                T[i][j] += a * B[k][j];
            }
        }
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Out[i][j] = T[i][j];
        }
    }
}

// globals[i] = globals[parent[i]] @ rest_rel[i] @ offsets[i]  (root: I @ ...)
// Caller passes topo-sorted arrays (roots first).
py::array_t<double> compose_globals(py::array_t<double> rest_rel,
                                    py::array_t<double> offsets,
                                    py::array_t<int> parents) {
    if (rest_rel.ndim() != 3 || offsets.ndim() != 3 || parents.ndim() != 1) {
        throw std::invalid_argument("compose_globals: expected rest_rel[N,4,4], offsets[N,4,4], parents[N]");
    }
    auto r = rest_rel.unchecked<3>(); // [N,4,4]
    auto o = offsets.unchecked<3>();  // [N,4,4]
    auto p = parents.unchecked<1>();  // [N]

    const py::ssize_t n = r.shape(0);
    if (o.shape(0) != n || p.shape(0) != n) {
        throw std::invalid_argument("compose_globals: leading dimensions must match");
    }

    py::array_t<double> out({n, 4, 4});
    auto g = out.mutable_unchecked<3>();

    double tmp[4][4];
    double gm[4][4];
    double R[4][4];
    double O[4][4];
    for (py::ssize_t i = 0; i < n; ++i) {
        for (int a = 0; a < 4; ++a) {
            for (int b = 0; b < 4; ++b) {
                R[a][b] = r(i, a, b);
                O[a][b] = o(i, a, b);
            }
        }
        mat_mul(R, O, tmp);
        const int par = p(i);
        if (par < 0) {
            for (int a = 0; a < 4; ++a) {
                for (int b = 0; b < 4; ++b) {
                    g(i, a, b) = tmp[a][b];
                }
            }
        } else {
            double P[4][4];
            for (int a = 0; a < 4; ++a) {
                for (int b = 0; b < 4; ++b) {
                    P[a][b] = g(static_cast<py::ssize_t>(par), a, b);
                }
            }
            mat_mul(P, tmp, gm);
            for (int a = 0; a < 4; ++a) {
                for (int b = 0; b < 4; ++b) {
                    g(i, a, b) = gm[a][b];
                }
            }
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
    if (verts.ndim() != 2 || ghosts.ndim() != 4 || weights.ndim() != 2 ||
        bone_idx.ndim() != 2 || rest_inv.ndim() != 3) {
        throw std::invalid_argument(
            "skin_deform: expected verts[V,3], ghosts[G,B,4,4], weights[V,K], bone_idx[V,K], rest_inv[B,4,4]");
    }

    auto v_acc = verts.unchecked<2>();
    auto g_acc = ghosts.unchecked<4>();
    auto w_acc = weights.unchecked<2>();
    auto bi_acc = bone_idx.unchecked<2>();
    auto ri_acc = rest_inv.unchecked<3>();

    const py::ssize_t nV = v_acc.shape(0);
    const py::ssize_t nG = g_acc.shape(0);
    const py::ssize_t nB = g_acc.shape(1);
    const py::ssize_t K = w_acc.shape(1);

    py::array_t<double> out({nG, nV, 3});
    auto o_acc = out.mutable_unchecked<3>();

    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) {
        hw = 1;
    }
    unsigned nthreads = hw;
    if (static_cast<py::ssize_t>(nthreads) > nG) {
        nthreads = static_cast<unsigned>(nG > 0 ? nG : 1);
    }

    auto work = [&](py::ssize_t g0, py::ssize_t g1) {
        double M[4][4];
        double R[4][4];
        double GM[4][4];
        for (py::ssize_t g = g0; g < g1; ++g) {
            for (py::ssize_t v = 0; v < nV; ++v) {
                const double px = v_acc(v, 0);
                const double py_ = v_acc(v, 1);
                const double pz = v_acc(v, 2);
                double ax = 0.0;
                double ay = 0.0;
                double az = 0.0;
                for (py::ssize_t k = 0; k < K; ++k) {
                    const double w = w_acc(v, k);
                    if (w == 0.0) {
                        continue;
                    }
                    const int b = bi_acc(v, k);
                    if (b < 0 || static_cast<py::ssize_t>(b) >= nB) {
                        continue;
                    }
                    const auto bb = static_cast<py::ssize_t>(b);
                    for (int a = 0; a < 4; ++a) {
                        for (int c = 0; c < 4; ++c) {
                            GM[a][c] = g_acc(g, bb, a, c);
                            R[a][c] = ri_acc(bb, a, c);
                        }
                    }
                    mat_mul(GM, R, M);
                    ax += w * (M[0][0] * px + M[0][1] * py_ + M[0][2] * pz + M[0][3]);
                    ay += w * (M[1][0] * px + M[1][1] * py_ + M[1][2] * pz + M[1][3]);
                    az += w * (M[2][0] * px + M[2][1] * py_ + M[2][2] * pz + M[2][3]);
                }
                o_acc(g, v, 0) = ax;
                o_acc(g, v, 1) = ay;
                o_acc(g, v, 2) = az;
            }
        }
    };

    {
        // Worker threads only touch disjoint [g0,g1) rows of `out` and
        // read-only views of the inputs, so release the GIL for throughput.
        py::gil_scoped_release release;
        const py::ssize_t per =
            (nG + static_cast<py::ssize_t>(nthreads) - 1) / static_cast<py::ssize_t>(nthreads);
        std::vector<std::thread> pool;
        pool.reserve(nthreads);
        for (unsigned t = 0; t < nthreads; ++t) {
            const py::ssize_t g0 = static_cast<py::ssize_t>(t) * per;
            const py::ssize_t g1 = std::min(nG, g0 + per);
            if (g0 >= g1) {
                break;
            }
            pool.emplace_back(work, g0, g1);
        }
        for (auto &th : pool) {
            th.join();
        }
    }
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
