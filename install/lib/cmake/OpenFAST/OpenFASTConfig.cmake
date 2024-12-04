#
# Copyright 2016 National Renewable Energy Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was OpenFASTConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Compilers use by OpenFAST build
set(OpenFAST_CXX_COMPILER "/usr/bin/c++")
set(OpenFAST_C_COMPILER "/usr/bin/cc")
set(OpenFAST_Fortran_COMPILER "/usr/bin/gfortran")

# Compiler flags used by OpenFAST build
set(OpenFAST_CXX_COMPILER_FLAGS " -fpic")
set(OpenFAST_C_COMPILER_FLAGS " -fpic")
set(OpenFAST_Fortran_COMPILER_FLAGS " -fpic  -ffree-line-length-none -cpp -fstack-reuse=none -fdefault-real-8 -fdefault-double-8")

set_and_check(OpenFAST_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/")
set_and_check(OpenFAST_LIBRARY_DIRS "${PACKAGE_PREFIX_DIR}/lib/")
set_and_check(OpenFAST_FTNMOD_DIR "${PACKAGE_PREFIX_DIR}/include/openfast/")

set(OpenFAST_LIBRARIES "openfastlib")
set(OpenFAST_CPP_LIBRARIES "openfastcpplib")

include("${CMAKE_CURRENT_LIST_DIR}/OpenFASTLibraries.cmake")

set(OpenFAST_HAS_CXX_API FALSE)
set(OpenFAST_FOUND TRUE)

if (NOT OpenFAST_FIND_COMPONENTS)
  if (OpenFAST_HAS_CXX_API)
    set(OpenFAST_FIND_COMPONENTS "openfastcpplib;openfastlib")
  else()
    set(OpenFAST_FIND_COMPONENTS "openfastlib")
  endif()
endif ()

foreach (comp IN LISTS OpenFAST_FIND_COMPONENTS)
  if (${comp} STREQUAL "openfastcpplib")
    if (NOT OpenFAST_HAS_CXX_API)
      set(OpenFAST_${comp}_FOUND FALSE)
      message(WARNING "OpenFAST ${comp} NOT found")
      if(OpenFAST_FIND_REQUIRED_${comp})
        set(OpenFAST_FOUND FALSE)
      endif()
    else()
      set(OpenFAST_${comp}_FOUND TRUE)
    endif()
  else()
    set(OpenFAST_${comp}_FOUND TRUE)
  endif()
endforeach ()

check_required_components(OpenFAST)
