# Faiss in exterior

This source tree is built by ../faiss_bundle.cmake. The production CPU target is
linked privately by exterior_core and installed with its devel component.

Local build adaptation: CPU-only builds accept CMake 3.22 (verified with 3.22.1).
The parent disables GPU, Metal, cuVS, ROCm, Python and tests only for this build.
GPU and GPU Metal algorithm source files have not been edited or removed.
No binary search kernel was patched.

The upstream `demos/` directory and its CMake entry are removed. It contains
standalone examples, including RocksDB IVF and diversity-filter demonstrations;
the production CPU library, mapping tools and regression tests do not use them.
The unused `c_api/` C wrapper and `misc/test_blas.cpp` standalone diagnostic are
also removed. The backend calls the C++ API directly. C API/extras build options
and references to absent demos, benchmarks and tutorials are removed, including
the C API conversion step in the ROCm helper script. Python bindings/contrib and
the C++ `faiss/cppcontrib` kernels are retained; the latter includes ARM NEON code.

Removed during integration: .github workflows/templates, conda packaging
(including Windows recipes), faiss/cppcontrib/docker_dev, .dockerignore,
CODE_OF_CONDUCT.md, CONTRIBUTING.md, Doxyfile, and the three pyproject packaging
files. The license and notices remain. Existing caller-removed folders were not
restored or counted as integration cleanup. Platform branches in the core source
remain for portability; they are not separate Windows tools.

GPU-source SHA256 verification and exact cleanup manifest are retained in
outputs/faiss_integration at the project root. CPU integration does not certify a
GPU build; upstream GPU toolchain requirements still apply.
