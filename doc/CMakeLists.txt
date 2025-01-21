# ===================
# SPHINX
# ===================

if(SEAHOWL_ENABLE_PYTHON)
  # Specify the path to the SPHINX installation
  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(SPHINX_EXECUTABLE "${CMAKE_BINARY_DIR}/__env__/bin/sphinx-build")
  else()
    set(SPHINX_EXECUTABLE "${CMAKE_BINARY_DIR}/__env__/Scripts/sphinx-build")
  endif()

  find_package(Sphinx)

  if(NOT Sphinx_FOUND)
      message(WARNING "Can't build user's docs. Sphinx not found!")
      return()
  endif(NOT Sphinx_FOUND)

  set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/source)
  set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/user)
  set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
          set(ACTIVATE_PY "${CMAKE_BINARY_DIR}/__env__/bin/activate")
          file(WRITE ${CMAKE_BINARY_DIR}/run_sphinx.sh
          "#!/bin/bash\n"
          "export PYTHONPATH=${CMAKE_BINARY_DIR}:$PYTHONPATH\n"
          "source ${ACTIVATE_PY}\n"
          "${SPHINX_EXECUTABLE} -a -b html ${SPHINX_SOURCE} ${SPHINX_BUILD}\n"
          )
          set(FILE_SPHINX ${CMAKE_BINARY_DIR}/run_sphinx.sh)
          set(LAUNCH_SPHINX bash)

  else()
          set(ACTIVATE_PY "${CMAKE_BINARY_DIR}/__env__/Scripts/activate")
          file(WRITE ${CMAKE_BINARY_DIR}/run_sphinx.bat
          "set PYTHONPATH=${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE};%PYTHONPATH%\n"
          "call ${ACTIVATE_PY}\n"
          "call ${SPHINX_EXECUTABLE} -a -b html ${SPHINX_SOURCE} ${SPHINX_BUILD}\n"
          )
          set(FILE_SPHINX ${CMAKE_BINARY_DIR}/run_sphinx.bat)
          set(LAUNCH_SPHINX cmd.exe /c)
  endif()

  # Make the script executable
  file(COPY ${FILE_SPHINX} DESTINATION ${CMAKE_BINARY_DIR})
  file(CHMOD ${FILE_SPHINX} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

  add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}
      COMMAND ${LAUNCH_SPHINX} ${FILE_SPHINX}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      DEPENDS ${SPHINX_SOURCE} # Every source files
      MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
      COMMENT "Generating documentation with Sphinx"
      VERBATIM
  )

  add_custom_target(Sphinx DEPENDS pyseahowl ${SPHINX_SOURCE} ${SPHINX_INDEX_FILE})
endif(SEAHOWL_ENABLE_PYTHON)
# ===================
# DOXYGEN
# ===================
find_package(Doxygen
    REQUIRED dot
    OPTIONAL_COMPONENTS mscgen dia)

    if (DOXYGEN_FOUND)

    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    add_custom_target( doxygen
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating documentation (Doxygen)"
      VERBATIM
    )

  else (DOXYGEN_FOUND)
    message("Doxygen needs to be install. You can use `sudo apt install doxygen` on ubuntu")
  endif (DOXYGEN_FOUND)

if(SEAHOWL_ENABLE_PYTHON)
  set(TARGET_DOC Sphinx doxygen)
else(SEAHOWL_ENABLE_PYTHON)
  set(TARGET_DOC doxygen)
endif(SEAHOWL_ENABLE_PYTHON)

if(SEAHOWL_ENABLE_DOC)
    add_custom_target(docs ALL DEPENDS ${TARGET_DOC})
else(SEAHOWL_ENABLE_DOC)
    add_custom_target(docs DEPENDS ${TARGET_DOC})
endif(SEAHOWL_ENABLE_DOC)

if(SEAHOWL_ENABLE_PYTHON)
  add_dependencies(docs pyseahowl)
endif(SEAHOWL_ENABLE_PYTHON)

# For MSVC IDE
set_target_properties(
    docs ${TARGET_DOC}
    PROPERTIES
    FOLDER DOC
  )
