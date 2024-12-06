vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/NREL/HydroChrono.git
    REF 43439c3b6dae2d195a7eb8e1dde440d406ca0e07
)
 
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCMAKE_BUILD_TYPE=Release
        -DHYDROCHRONO_ENABLE_IRRLICHT=OFF
        
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME HydroChrono
                         NO_PREFIX_CORRECTION )


