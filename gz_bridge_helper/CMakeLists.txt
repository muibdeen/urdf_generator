cmake_minimum_required(VERSION 3.16)
project(gz_bridge_helper VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(GZBH_BUILD_GUI "Build the Qt6 GUI" ON)
option(GZBH_BUILD_TESTS "Build and register the tests" ON)

# ROS 2 is optional at build time. Inside a colcon workspace ament_cmake is found and this is a
# normal ROS 2 package; elsewhere it is a plain CMake project.
find_package(ament_cmake QUIET)

# ------------------------------------------------------------------ third-party libraries
find_package(tinyxml2 CONFIG QUIET)
if(NOT TARGET tinyxml2::tinyxml2)
  find_path(TINYXML2_INCLUDE_DIR tinyxml2.h)
  find_library(TINYXML2_LIBRARY tinyxml2)
  if(NOT TINYXML2_INCLUDE_DIR OR NOT TINYXML2_LIBRARY)
    message(FATAL_ERROR "tinyxml2 not found (sudo apt install libtinyxml2-dev)")
  endif()
  add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
  set_target_properties(tinyxml2::tinyxml2 PROPERTIES
    IMPORTED_LOCATION "${TINYXML2_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}")
endif()

find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(GZBH_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(GZBH_YAML_TARGET yaml-cpp)  # older yaml-cpp releases
endif()

# ---------------------------------------------------------------------- core library
add_library(gzbh_core STATIC
  src/common.cpp
  src/types.cpp
  src/types_table.cpp
  src/urdf_model.cpp
  src/bridge.cpp
  src/yaml_input.cpp
  src/detect.cpp
  src/plan.cpp
  src/emit.cpp
  src/project.cpp
  src/verify.cpp)
target_include_directories(gzbh_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(gzbh_core PUBLIC tinyxml2::tinyxml2 ${GZBH_YAML_TARGET})
set_target_properties(gzbh_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(NOT MSVC)
  target_compile_options(gzbh_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------ CLI
add_executable(gz_bridge_helper_cli tools/cli.cpp)
target_link_libraries(gz_bridge_helper_cli PRIVATE gzbh_core)
set(_gzbh_install_targets gz_bridge_helper_cli)

# ------------------------------------------------------------------------------ GUI (Qt6)
if(GZBH_BUILD_GUI)
  find_package(Qt6 COMPONENTS Widgets QUIET)
  if(Qt6_FOUND)
    set(CMAKE_AUTOMOC ON)
    add_library(gzbh_gui STATIC
      gui/main_window.cpp
      gui/main_window.hpp
      gui/run_panel.cpp
      gui/run_panel.hpp
      gui/process_job.cpp
      gui/process_job.hpp
      gui/highlighter.cpp
      gui/highlighter.hpp)
    target_include_directories(gzbh_gui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/gui)
    target_link_libraries(gzbh_gui PUBLIC gzbh_core Qt6::Widgets)
    if(NOT MSVC)
      target_compile_options(gzbh_gui PRIVATE -Wall -Wextra)
    endif()
    add_executable(gz_bridge_helper_gui gui/main.cpp)
    target_link_libraries(gz_bridge_helper_gui PRIVATE gzbh_gui)
    list(APPEND _gzbh_install_targets gz_bridge_helper_gui)
  else()
    message(WARNING "Qt6 Widgets not found: building without the GUI (sudo apt install qt6-base-dev)")
  endif()
endif()

# ---------------------------------------------------------------------------------- tests
if(GZBH_BUILD_TESTS)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE gzbh_core)
  target_compile_definitions(test_core PRIVATE GZBH_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  add_test(NAME core COMMAND test_core)

  # Drives the real window without a display; the ROS tools are replaced by scripts.
  if(TARGET gzbh_gui)
    add_executable(test_gui test/test_gui.cpp)
    target_link_libraries(test_gui PRIVATE gzbh_gui)
    target_compile_definitions(test_gui PRIVATE
      GZBH_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
      GZBH_MOCK_BIN="${CMAKE_CURRENT_SOURCE_DIR}/test/mock_bin")
    add_test(NAME gui COMMAND test_gui)
    set_tests_properties(gui PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 180)
  endif()
  # Independent checks of what the helper writes, with tools that share no code with it. Each is
  # registered only when its Python modules are installed (a sourced ROS 2 environment has them).
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_FOUND)
    file(GLOB _gzbh_urdfs ${CMAKE_CURRENT_SOURCE_DIR}/test/data/corpus/*.urdf ${CMAKE_CURRENT_SOURCE_DIR}/test/data/*.urdf)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import yaml" RESULT_VARIABLE _gzbh_yaml_rc OUTPUT_QUIET ERROR_QUIET)
    if(_gzbh_yaml_rc EQUAL 0)
      add_test(NAME generated_packages
               COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/test/check_generated.py
                       $<TARGET_FILE:gz_bridge_helper_cli> ${CMAKE_CURRENT_BINARY_DIR}/check_generated ${_gzbh_urdfs})
    endif()
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import yourdfpy, numpy" RESULT_VARIABLE _gzbh_urdfpy_rc OUTPUT_QUIET ERROR_QUIET)
    if(_gzbh_urdfpy_rc EQUAL 0)
      add_test(NAME drive_numbers_cross_check
               COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/test/cross_check_drive.py
                       $<TARGET_FILE:gz_bridge_helper_cli> ${CMAKE_CURRENT_BINARY_DIR}/cross_check_drive ${_gzbh_urdfs})
    endif()
  endif()
endif()

# -------------------------------------------------------------------------------- install
# Executables go to lib/<package> so that `ros2 run gz_bridge_helper gz_bridge_helper_gui` works.
install(TARGETS ${_gzbh_install_targets} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY test/data/ DESTINATION share/${PROJECT_NAME}/examples
        PATTERN "corpus" EXCLUDE)

if(ament_cmake_FOUND)
  ament_package()
endif()
