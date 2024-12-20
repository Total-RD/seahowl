# ===================
# SPHINX
# ===================

find_package(Sphinx)

if(NOT Sphinx_FOUND)
    message(WARNING "Can't build user's docs. Sphinx not found!")
    return()
endif(NOT Sphinx_FOUND)

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/source)
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/user)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}

    COMMAND

    ${SPHINX_EXECUTABLE} -a -b html # Force write all files
    ${SPHINX_SOURCE} ${SPHINX_BUILD}

    WORKING_DIRECTORY

    ${CMAKE_CURRENT_BINARY_DIR}

    DEPENDS

    ${SPHINX_SOURCE} # Every source files

    MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py

    COMMENT "Generating documentation with Sphinx")

add_custom_target(Sphinx DEPENDS ${SPHINX_SOURCE} ${SPHINX_INDEX_FILE})



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
  
    install(
      DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/doc/html
      DESTINATION doxygen
    )
  else (DOXYGEN_FOUND)
    message("Doxygen needs to be install. You can use `sudo apt install doxygen` on ubuntu")
  endif (DOXYGEN_FOUND)


add_custom_target(documentation ALL DEPENDS Sphinx doxygen)
add_dependencies(documentation pyseahowl)

# For MSVC IDE
set_target_properties(
        Sphinx documentation doxygen

		PROPERTIES

		FOLDER DOC
)
