# Empty overlay metaport: the actual BLAS implementation is provided by the
# `openblas` dependency declared in vcpkg.json. This overlay overrides vcpkg's
# default `blas` metaport so it does not pull in `lapack-reference` (which
# would require a working Fortran compiler on Windows).
set(VCPKG_POLICY_EMPTY_PACKAGE enabled)
