include(vcpkg_find_fortran)

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/Total-RD/openfast4seahowl.git
    REF db2ceb82e2592dc2d013cb1217a37a4a3b23cf52
)

vcpkg_find_fortran(FORTRAN_CMAKE)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FORTRAN_CMAKE}
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake
                         PACKAGE_NAME OpenFAST
                         NO_PREFIX_CORRECTION)
