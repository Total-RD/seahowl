vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL git@github.com:Total-RD/openfast4seahowl.git
    REF db2ceb82e2592dc2d013cb1217a37a4a3b23cf52
)
 
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCMAKE_BUILD_TYPE=Release
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake
                         PACKAGE_NAME OpenFAST
                         NO_PREFIX_CORRECTION)

