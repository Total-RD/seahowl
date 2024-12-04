#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "HydroChrono::HydroChrono" for configuration "Release"
set_property(TARGET HydroChrono::HydroChrono APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(HydroChrono::HydroChrono PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libHydroChrono.a"
  )

list(APPEND _cmake_import_check_targets HydroChrono::HydroChrono )
list(APPEND _cmake_import_check_files_for_HydroChrono::HydroChrono "${_IMPORT_PREFIX}/lib/libHydroChrono.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
