# Overlay triplet for x64-windows used by this project's CI to ensure that the
# Fortran compiler (FC) and PATH chosen on the runner are propagated into every
# vcpkg port's inner build (vcpkg normally sanitizes the environment between
# ports, which is what causes the VS-bundled LLVMFlang to be picked up for
# openfast even after we have prepended MSYS2 mingw gfortran on the outer
# PATH).
#
# Activated by setting VCPKG_OVERLAY_TRIPLETS=external/vcpkg/triplets in the
# workflow.

set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)

# Forward FC and PATH from the workflow environment into port builds so
# openfast's vcpkg_find_fortran / enable_language(Fortran) picks the mingw
# gfortran we installed instead of the VS-bundled LLVMFlang.
set(VCPKG_ENV_PASSTHROUGH FC PATH)
