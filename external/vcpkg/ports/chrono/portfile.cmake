vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/projectchrono/chrono.git
    REF 30cd3f2702cb58182e46d5b2724d2d2850a50e21
    PATCHES
      "chrono_custom_command.patch"
)

if(VCPKG_BUILD_TYPE STREQUAL "Debug")
    set(IRRLICHT_ROOT "${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/debug")
else()
    set(IRRLICHT_ROOT "${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}")
endif()


vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_TESTING=ON
        -DBUILD_DEMOS=OFF
        -DBUILD_BENCHMARKING=OFF
        -DENABLE_HDF5=OFF
        -DENABLE_MODULE_POSTPROCESS=OFF
        -DENABLE_MODULE_PYTHON=OFF
        -DENABLE_MODULE_IRRLICHT=ON
        -DENABLE_MODULE_VEHICLE=OFF
        -DENABLE_MODULE_MULTICORE=OFF
        -DENABLE_MODULE_OPENGL=OFF
        -DENABLE_MODULE_SYNCHRONO=OFF
        -DENABLE_MODULE_CSHARP=OFF
        -DENABLE_MODULE_COSIMULATION=OFF

        # hack needed explicitly because of the way IRRLICHT_ROOT is set in Chrono 8.0.0
        -DIRRLICHT_ROOT="${IRRLICHT_ROOT}"
)


vcpkg_cmake_install()

# Force create debug/share/chrono
file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/debug/share/chrono")

vcpkg_cmake_config_fixup()
