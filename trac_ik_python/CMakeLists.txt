cmake_minimum_required(VERSION 3.12)
project(trac_ik_python)

cmake_policy(SET CMP0078 NEW)
cmake_policy(SET CMP0086 NEW)

set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(SWIG REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

include(${SWIG_USE_FILE})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
include_directories(
  ${trac_ik_lib_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

set(CMAKE_SWIG_FLAGS -c++)
set_source_files_properties(swig/trac_ik_wrap.i PROPERTIES CPLUSPLUS ON)
set_property(SOURCE swig/trac_ik_wrap.i PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)

set(PYTHON_PACKAGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/python/trac_ik_python)
file(MAKE_DIRECTORY ${PYTHON_PACKAGE_DIR})
set(CMAKE_SWIG_OUTDIR ${PYTHON_PACKAGE_DIR})

swig_add_library(trac_ik_wrap
  TYPE MODULE
  LANGUAGE python
  SOURCES swig/trac_ik_wrap.i
)

set(TRAC_IK_PYTHON_TARGET trac_ik_wrap)

target_include_directories(${TRAC_IK_PYTHON_TARGET} PRIVATE
  ${Python3_INCLUDE_DIRS}
)

target_link_libraries(${TRAC_IK_PYTHON_TARGET}
  ${Python3_LIBRARIES}
)

ament_target_dependencies(${TRAC_IK_PYTHON_TARGET}
  trac_ik_lib
  kdl_parser
  urdf
)

set_target_properties(${TRAC_IK_PYTHON_TARGET} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PYTHON_PACKAGE_DIR}
)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/trac_ik_python)

install(FILES ${PYTHON_PACKAGE_DIR}/trac_ik_wrap.py
  DESTINATION ${PYTHON_INSTALL_DIR}/trac_ik_python
)

install(TARGETS ${TRAC_IK_PYTHON_TARGET}
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/trac_ik_python
)

install(PROGRAMS
  scripts/test_pkg.py
  scripts/test_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
