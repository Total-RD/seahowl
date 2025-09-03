vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO partic2/libffi
    REF main
    SHA512 4baaed1421b4ad06f2ab616236def5fd6d24e4ae34cca497d30b27b5110a8b13c0746bfe092286296a56049ad3e8b8b9e53d4f30cbf4de1610e717840321e940
    FILENAME "libffi-master.tar.gz"
)

# Définir le compilateur Intel
set(ENV{CC} "icx")
set(ENV{CXX} "icpx")

# Configuration CMake
vcpkg_configure_cmake(
    SOURCE_PATH ${SOURCE_PATH}
    PREFER_NINJA
    OPTIONS
        -DBUILD_SHARED_LIBS=ON
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DCMAKE_INSTALL_PREFIX=${CURRENT_PACKAGES_DIR}
)

# Compilation et installation
vcpkg_install_cmake()

# Copier pdb/fichiers additionnels
vcpkg_copy_pdbs()
vcpkg_fixup_pkgconfig()

# Ajustements pour build statique si nécessaire
if (VCPKG_LIBRARY_LINKAGE STREQUAL "static")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/ffi.h" "defined(FFI_STATIC_BUILD)" "1")
endif()

# Installation des fichiers supplémentaires
file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/unofficial-libffi-config.cmake" DESTINATION "${CURRENT_PACKAGES_DIR}/share/unofficial-libffi")
file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/libffiConfig.cmake" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")

# Nettoyage
file(REMOVE_RECURSE
    "${CURRENT_PACKAGES_DIR}/debug/share"
    "${CURRENT_PACKAGES_DIR}/share/man3"
)

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
