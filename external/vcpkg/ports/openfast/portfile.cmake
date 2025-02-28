include(vcpkg_find_fortran)

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/openfast/openfast.git
    REF fc1110183bcc87b16d93129edabdce6d30e3a497
    PATCHES
        "openfast_custom_command.patch"
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
