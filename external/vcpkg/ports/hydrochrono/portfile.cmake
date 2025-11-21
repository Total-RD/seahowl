vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/Project-SEA-Stack//HydroChrono.git
    REF 71eff2bad24463ea42667f67dd4c39cb22246780
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
