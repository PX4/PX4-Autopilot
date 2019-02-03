
# Use Python 3.5
set(Python_ADDITIONAL_VERSIONS 3.5)
find_package(PythonLibs 3 REQUIRED)
find_package(PythonInterp 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

 # Find numpy include
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())" OUTPUT_VARIABLE NUMPY_INCLUDE_DIRS ERROR_QUIET)
include_directories(${NUMPY_INCLUDE_DIRS})

set(CMAKE_SWIG_FLAGS "")
set_source_files_properties(../ecl_EKF.i PROPERTIES CPLUSPLUS ON)
include_directories(../..)

# Add swig module
swig_add_library(ecl_EKF
	LANGUAGE python
	SOURCES ../ecl_EKF.i
)
swig_link_libraries(ecl_EKF ecl_EKF ${PYTHON_LIBRARIES})

# Files to install with Python
set(PYTHON_INSTALL_FILES
	${CMAKE_CURRENT_BINARY_DIR}/ecl_EKF.py
	${CMAKE_CURRENT_BINARY_DIR}/_ecl_EKF.so
	)

# Configure setup.py and copy to output directory
set(SETUP_PY_IN ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in)
set(SETUP_PY_OUT ${CMAKE_CURRENT_BINARY_DIR}/setup.py)
configure_file(${SETUP_PY_IN} ${SETUP_PY_OUT})

# Declare install target for python
#install(TARGETS swig_example
#        COMMAND "${PYTHON_EXECUTABLE} setup.py"
#        COMPONENT swig-python)

# Install target to call setup.py
add_custom_target(install-python
	DEPENDS _ecl_EKF
	COMMAND python ${SETUP_PY_OUT} install
	)
