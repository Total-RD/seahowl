vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL ssh://git@forge-02.cesgenslab.cloud:2222/total-seahowl-test/chrono-v8-0-0.git
    REF 8458cf49c625df378182bdd5f8b5ce71b85d942f
)
 
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_TESTING=ON 
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
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup()


