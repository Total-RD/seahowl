vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO HDFGroup/hdf5
    REF hdf5-1_10_8
    SHA512 87545f45ccd2c002569ffe2ffd07b615d53706bfbff07daaa71ff94704ebd2886458c4d7b512a8798a405d35d8a341968f25ad280e7a70e71a1c067e25c98743
    HEAD_REF master
)
 
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DHDF5_INSTALL_CMAKE_DIR=share/hdf5
        -DBUILD_SHARED_LIBS=OFF
        -DHDF5_BUILD_CPP_LIB=ON 
        -DHDF5_ENABLE_CXX=ON
        -DVCPKG_BUILD_TYPE=release
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup()
vcpkg_copy_tools(TOOL_NAMES mirror_server mirror_server_stop h5copy h5diff h5dump h5ls h5stat gif2h5 h52gif h5clear h5debug
h5format_convert h5jam h5unjam h5mkgrp h5repack h5repart h5watch h5import h5perf_serial
AUTO_CLEAN)

