include(vcpkg_find_fortran)

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/NREL/ROSCO.git
    REF fcc43940467c6b6ea24f99dd19eef6b8f8731276
)

vcpkg_find_fortran(FORTRAN_CMAKE)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}/rosco/controller"
    OPTIONS
        ${FORTRAN_CMAKE}
)
vcpkg_cmake_install()
