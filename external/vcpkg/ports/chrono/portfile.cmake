vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/projectchrono/chrono.git
    REF 30cd3f2702cb58182e46d5b2724d2d2850a50e21
)

set(VCPKG_BUILD_TYPE release)

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
        -DCMAKE_BUILD_TYPE=Release

        # hack needed explicitly because of the way IRRLICHT_ROOT is set in Chrono 8.0.0
        -DIRRLICHT_ROOT="${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}"
)

# hack needed for Windows as Irrlicht DLLs are copied automatically in chrono_irrlicht CMakeLists
# this needs to be fixed within Chrono source to remove the hack
if(VCPKG_TARGET_IS_WINDOWS)
    file(MAKE_DIRECTORY ${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/bin/Win32-VisualStudio)
    file(MAKE_DIRECTORY ${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/bin/Win64-VisualStudio)
    file(COPY ${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/bin/Irrlicht.dll DESTINATION ${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/bin/Win32-VisualStudio)
    file(COPY ${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/bin/Irrlicht.dll DESTINATION ${_VCPKG_INSTALLED_DIR}/${TARGET_TRIPLET}/bin/Win64-VisualStudio)
endif()

vcpkg_cmake_install()
vcpkg_cmake_config_fixup()
