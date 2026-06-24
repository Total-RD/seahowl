# Overlay triplet for x64-windows used by this project's CI.
#
# Identical to the upstream `x64-windows` triplet, except it forwards the
# Fortran compiler (FC) and PATH from the runner's environment into every
# vcpkg port's inner build. Without this passthrough, vcpkg sanitizes the
# environment between ports, which is what causes the VS-bundled LLVMFlang
# to be picked up for openfast even after we have configured a different
# Fortran compiler on the outer PATH.
#
# Activated by setting VCPKG_OVERLAY_TRIPLETS to this folder in the workflow.

set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)

set(VCPKG_ENV_PASSTHROUGH FC PATH)
