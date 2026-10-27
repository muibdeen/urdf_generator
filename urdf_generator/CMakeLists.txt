cmake_minimum_required(VERSION 3.16)
project(urdf_generator VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(URDFGEN_BUILD_GUI "Build the Qt6 GUI" ON)
option(URDFGEN_BUILD_TESTS "Build and register the tests" ON)

# ROS 2 is optional at build time. Inside a colcon workspace ament_cmake is
# found and this is a normal ROS 2 package; elsewhere it is a plain CMake project.
find_package(ament_cmake QUIET)

# ---------------------------------------------------------------- core library
add_library(urdfgen_core STATIC
  src/model.cpp
  src/emitter.cpp
  src/builders.cpp
  src/sensors.cpp
  src/scene.cpp
  src/render.cpp)
target_include_directories(urdfgen_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(urdfgen_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(NOT MSVC)
  target_compile_options(urdfgen_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------ CLI
add_executable(urdf_generator_cli tools/cli.cpp)
target_link_libraries(urdf_generator_cli PRIVATE urdfgen_core)
set(_urdfgen_install_targets urdf_generator_cli)

# ------------------------------------------------------------------------ GUI
if(URDFGEN_BUILD_GUI)
  find_package(Qt6 REQUIRED COMPONENTS Widgets)
  set(CMAKE_AUTOMOC ON)
  # The windows and pages live in a library so the tests can drive them without a display.
  add_library(urdfgen_gui STATIC
    gui/main_window.cpp
    gui/main_window.hpp
    gui/config_pages.cpp
    gui/config_pages.hpp
    gui/joint_panel.cpp
    gui/joint_panel.hpp
    gui/sensor_panel.cpp
    gui/sensor_panel.hpp
    gui/robot_view.cpp
    gui/robot_view.hpp)
  target_include_directories(urdfgen_gui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/gui)
  target_link_libraries(urdfgen_gui PUBLIC urdfgen_core Qt6::Widgets)
  if(NOT MSVC)
    target_compile_options(urdfgen_gui PRIVATE -Wall -Wextra)
  endif()
  add_executable(urdf_generator_gui gui/main.cpp)
  target_link_libraries(urdf_generator_gui PRIVATE urdfgen_gui)
  list(APPEND _urdfgen_install_targets urdf_generator_gui)
endif()

# -------------------------------------------------------------------- install
# Executables go to lib/<package> so that `ros2 run urdf_generator urdf_generator_gui` works.
install(TARGETS ${_urdfgen_install_targets} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------- tests
if(URDFGEN_BUILD_TESTS)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE urdfgen_core)
  add_test(NAME core COMMAND test_core)

  # Drives the real window without a display.
  if(TARGET urdfgen_gui)
    add_executable(test_gui test/test_gui.cpp)
    target_link_libraries(test_gui PRIVATE urdfgen_gui)
    add_test(NAME gui COMMAND test_gui)
    set_tests_properties(gui PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  endif()

  # Expand every generated .xacro with the real xacro tool and compare it with
  # the flat .urdf. Only registered when the Python module `xacro` is available
  # (it is in any sourced ROS 2 environment).
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_FOUND)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import xacro"
                    RESULT_VARIABLE _urdfgen_xacro_rc OUTPUT_QUIET ERROR_QUIET)
    if(_urdfgen_xacro_rc EQUAL 0)
      add_test(NAME generate_all
               COMMAND urdf_generator_cli all ${CMAKE_CURRENT_BINARY_DIR}/generated)
      add_test(NAME xacro_matches_urdf
               COMMAND ${Python3_EXECUTABLE}
                       ${CMAKE_CURRENT_SOURCE_DIR}/test/compare_xacro_urdf.py
                       ${CMAKE_CURRENT_BINARY_DIR}/generated)
      set_tests_properties(xacro_matches_urdf PROPERTIES DEPENDS generate_all)

      # The same check for unusual settings (polar offsets, prismatic joints, ...).
      add_test(NAME generate_variants
               COMMAND test_core --dump ${CMAKE_CURRENT_BINARY_DIR}/generated_variants)
      add_test(NAME xacro_matches_urdf_variants
               COMMAND ${Python3_EXECUTABLE}
                       ${CMAKE_CURRENT_SOURCE_DIR}/test/compare_xacro_urdf.py
                       ${CMAKE_CURRENT_BINARY_DIR}/generated_variants)
      set_tests_properties(xacro_matches_urdf_variants PROPERTIES DEPENDS generate_variants)

      # The 3D scene (link poses and bounds) against an independent URDF implementation.
      execute_process(COMMAND ${Python3_EXECUTABLE} -c "import yourdfpy, numpy"
                      RESULT_VARIABLE _urdfgen_yourdfpy_rc OUTPUT_QUIET ERROR_QUIET)
      if(_urdfgen_yourdfpy_rc EQUAL 0)
        add_test(NAME scene_matches_urdf_fk
                 COMMAND ${Python3_EXECUTABLE}
                         ${CMAKE_CURRENT_SOURCE_DIR}/test/compare_scene_fk.py
                         ${CMAKE_CURRENT_BINARY_DIR}/generated_variants)
        set_tests_properties(scene_matches_urdf_fk PROPERTIES DEPENDS generate_variants)
      else()
        message(STATUS "Python modules 'yourdfpy'/'numpy' not found: skipping the 3D scene consistency test")
      endif()

      # If urdfdom's check_urdf is installed (the parser ROS 2 itself uses), run it on everything.
      find_program(URDFGEN_CHECK_URDF check_urdf)
      if(URDFGEN_CHECK_URDF)
        add_test(NAME check_urdf
                 COMMAND ${Python3_EXECUTABLE}
                         ${CMAKE_CURRENT_SOURCE_DIR}/test/run_check_urdf.py ${URDFGEN_CHECK_URDF}
                         ${CMAKE_CURRENT_BINARY_DIR}/generated
                         ${CMAKE_CURRENT_BINARY_DIR}/generated_variants)
        set_tests_properties(check_urdf PROPERTIES DEPENDS "generate_all;generate_variants")
      endif()
    else()
      message(STATUS "Python module 'xacro' not found: skipping the xacro/URDF consistency test")
    endif()
  endif()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
