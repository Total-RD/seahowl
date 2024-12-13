vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/NREL/HydroChrono.git
    REF 89136b18a75fe567276dea1ae418baa3d5aeb5c3
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DHYDROCHRONO_ENABLE_IRRLICHT=OFF
        -DHYDROCHRONO_ENABLE_DEMOS=OFF
        -DHYDROCHRONO_ENABLE_TESTS=OFF
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME HydroChrono
                         NO_PREFIX_CORRECTION )
