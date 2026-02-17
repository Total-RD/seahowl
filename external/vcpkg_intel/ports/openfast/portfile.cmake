include(vcpkg_find_fortran)

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/openfast/openfast.git
    REF 3a9d3f29f03b52b536d391fbd360683f847be712
    PATCHES
        "openfast_custom_command.patch"
        "openfast_custom_command2.patch"
)

vcpkg_find_fortran(FORTRAN_CMAKE)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FORTRAN_CMAKE}
        -DCMAKE_CXX_FLAGS=-O3\ -fp-model=precise
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake
                         PACKAGE_NAME OpenFAST
                         NO_PREFIX_CORRECTION)
