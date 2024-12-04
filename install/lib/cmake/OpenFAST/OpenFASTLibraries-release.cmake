#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "nwtclibs" for configuration "Release"
set_property(TARGET nwtclibs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nwtclibs PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "Fortran"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libnwtclibs.a"
  )

list(APPEND _cmake_import_check_targets nwtclibs )
list(APPEND _cmake_import_check_files_for_nwtclibs "${_IMPORT_PREFIX}/lib/libnwtclibs.a" )

# Import target "versioninfolib" for configuration "Release"
set_property(TARGET versioninfolib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(versioninfolib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "Fortran"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libversioninfolib.a"
  )

list(APPEND _cmake_import_check_targets versioninfolib )
list(APPEND _cmake_import_check_files_for_versioninfolib "${_IMPORT_PREFIX}/lib/libversioninfolib.a" )

# Import target "ifwlib" for configuration "Release"
set_property(TARGET ifwlib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ifwlib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "Fortran"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libifwlib.a"
  )

list(APPEND _cmake_import_check_targets ifwlib )
list(APPEND _cmake_import_check_files_for_ifwlib "${_IMPORT_PREFIX}/lib/libifwlib.a" )

# Import target "inflowwind_driver" for configuration "Release"
set_property(TARGET inflowwind_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(inflowwind_driver PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/inflowwind_driver"
  )

list(APPEND _cmake_import_check_targets inflowwind_driver )
list(APPEND _cmake_import_check_files_for_inflowwind_driver "${_IMPORT_PREFIX}/bin/inflowwind_driver" )

# Import target "ifw_c_binding" for configuration "Release"
set_property(TARGET ifw_c_binding APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ifw_c_binding PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libifw_c_binding.so"
  IMPORTED_SONAME_RELEASE "libifw_c_binding.so"
  )

list(APPEND _cmake_import_check_targets ifw_c_binding )
list(APPEND _cmake_import_check_files_for_ifw_c_binding "${_IMPORT_PREFIX}/lib/libifw_c_binding.so" )

# Import target "aerodynlib" for configuration "Release"
set_property(TARGET aerodynlib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(aerodynlib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "Fortran"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libaerodynlib.a"
  )

list(APPEND _cmake_import_check_targets aerodynlib )
list(APPEND _cmake_import_check_files_for_aerodynlib "${_IMPORT_PREFIX}/lib/libaerodynlib.a" )

# Import target "aerodyn_driver_subs" for configuration "Release"
set_property(TARGET aerodyn_driver_subs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(aerodyn_driver_subs PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "Fortran"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libaerodyn_driver_subs.a"
  )

list(APPEND _cmake_import_check_targets aerodyn_driver_subs )
list(APPEND _cmake_import_check_files_for_aerodyn_driver_subs "${_IMPORT_PREFIX}/lib/libaerodyn_driver_subs.a" )

# Import target "aerodyn_driver" for configuration "Release"
set_property(TARGET aerodyn_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(aerodyn_driver PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/aerodyn_driver"
  )

list(APPEND _cmake_import_check_targets aerodyn_driver )
list(APPEND _cmake_import_check_files_for_aerodyn_driver "${_IMPORT_PREFIX}/bin/aerodyn_driver" )

# Import target "unsteadyaero_driver" for configuration "Release"
set_property(TARGET unsteadyaero_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(unsteadyaero_driver PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/unsteadyaero_driver"
  )

list(APPEND _cmake_import_check_targets unsteadyaero_driver )
list(APPEND _cmake_import_check_files_for_unsteadyaero_driver "${_IMPORT_PREFIX}/bin/unsteadyaero_driver" )

# Import target "aerodyn_inflow_c_binding" for configuration "Release"
set_property(TARGET aerodyn_inflow_c_binding APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(aerodyn_inflow_c_binding PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libaerodyn_inflow_c_binding.so"
  IMPORTED_SONAME_RELEASE "libaerodyn_inflow_c_binding.so"
  )

list(APPEND _cmake_import_check_targets aerodyn_inflow_c_binding )
list(APPEND _cmake_import_check_files_for_aerodyn_inflow_c_binding "${_IMPORT_PREFIX}/lib/libaerodyn_inflow_c_binding.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
