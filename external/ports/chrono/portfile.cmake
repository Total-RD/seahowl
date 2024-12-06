vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/projectchrono/chrono.git
    REF 30cd3f2702cb58182e46d5b2724d2d2850a50e21
)
 
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_TESTING=ON 
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
        -DCMAKE_BUILD_TYPE=Release
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup()


