vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/projectchrono/chrono.git
    REF 2617649bf687456328a122b63dc0bb64df91394a
    PATCHES
      "chrono_custom_command.patch"
)

if(VCPKG_BUILD_TYPE STREQUAL "Debug")
    set(IRRLICHT_INSTALL_DIR "${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/debug")
else()
    set(IRRLICHT_INSTALL_DIR "${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}")
endif()

vcpkg_check_linkage(ONLY_STATIC_LIBRARY)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_TESTING=ON
        -DBUILD_DEMOS=OFF
        -DBUILD_BENCHMARKING=OFF
        -DENABLE_HDF5=OFF
        -DENABLE_MODULE_POSTPROCESS=OFF
        -DENABLE_MODULE_PYTHON=OFF
        -DENABLE_MODULE_IRRLICHT=OFF
        -DENABLE_MODULE_VEHICLE=OFF
        -DENABLE_MODULE_MULTICORE=OFF
        -DENABLE_MODULE_OPENGL=OFF
        -DENABLE_MODULE_SYNCHRONO=OFF
        -DENABLE_MODULE_CSHARP=OFF
        -DENABLE_MODULE_COSIMULATION=OFF
        -DCMAKE_CXX_FLAGS=-O1\ -fp-model=precise

        # hack needed explicitly because of the way IRRLICHT_ROOT is set in Chrono 8.0.0
        -DIRRLICHT_INSTALL_DIR="${IRRLICHT_INSTALL_DIR}"
)


vcpkg_cmake_install()

# Force create debug/share/chrono
file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/debug/share/chrono")

vcpkg_cmake_config_fixup()

if(VCPKG_TARGET_IS_WINDOWS)
    file(APPEND "${CURRENT_PACKAGES_DIR}/share/${PORT}/usage"
    "Link with system library: Ws2_32.lib\n")
endif()
