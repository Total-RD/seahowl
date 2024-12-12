vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/NREL/HydroChrono.git
    REF 9c3aa8e9932e619bd489e73261ddfdf90d0c833f
)

set(VCPKG_BUILD_TYPE release)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCMAKE_BUILD_TYPE=Release
        -DHYDROCHRONO_ENABLE_IRRLICHT=OFF
        -DHYDROCHRONO_ENABLE_DEMOS=OFF
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME HydroChrono
                         NO_PREFIX_CORRECTION )
