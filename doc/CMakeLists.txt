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

set(DOXYGEN_PROJECT_BRIEF "Servo-Elasto-Aero-Hydro Offshore Wind Lab")
set(DOXYGEN_QUIET YES)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_HIDE_UNDOC_MEMBERS NO)
set(DOXYGEN_HIDE_UNDOC_CLASSES NO)
set(DOXYGEN_GENERATE_TODOLIST YES)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_GENERATE_MAN NO)
set(DOXYGEN_IMAGE_PATH ${SPHINX_SOURCE}/_static/images)

doxygen_add_docs(
    doxygen
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
    COMMENT "Generate HTML pages"
)


add_custom_target(documentation ALL DEPENDS Sphinx doxygen)
add_dependencies(documentation pyseahowl)

# For MSVC IDE
set_target_properties(
        Sphinx documentation doxygen

		PROPERTIES

		FOLDER DOC
)
