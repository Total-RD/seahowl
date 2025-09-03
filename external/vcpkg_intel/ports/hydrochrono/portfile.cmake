vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/NREL/HydroChrono.git
    REF 7ea31390117d9f1a0c38ec449e19ba17239699a9
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DHYDROCHRONO_ENABLE_IRRLICHT=OFF
        -DHYDROCHRONO_ENABLE_DEMOS=OFF
        -DHYDROCHRONO_ENABLE_TESTS=OFF
        -DCMAKE_CXX_FLAGS=-O3\ -fp-model=precise
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME HydroChrono
                         NO_PREFIX_CORRECTION )
