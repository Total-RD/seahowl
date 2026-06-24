# Empty overlay metaport: the actual LAPACK implementation is provided by the
# `openblas` dependency declared in vcpkg.json. OpenBLAS bundles the reference
# LAPACK Fortran sources compiled with its own toolchain, so consumers of
# `find_package(LAPACK)` get a working LAPACK without seahowl needing to build
# `lapack-reference` (which would require a working Fortran compiler on
# Windows).
set(VCPKG_POLICY_EMPTY_PACKAGE enabled)
