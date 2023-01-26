cmake_minimum_required(VERSION 3.14.0)
project(slts_control VERSION 1.0.0)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)
set(Eigen3_DEPENDENCY "find_dependency (Eigen3 ${Eigen3_VERSION})")

set(COMPILE_OPTS
    "$<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>$<$<CXX_COMPILER_ID:Clang>:-fcolor-diagnostics>"
    "$<$<CONFIG:Debug>:-Og>"
    "$<$<CONFIG:Release>:-O3>")

add_library(slts_controller src/${PROJECT_NAME}/slts_controller.cpp)
target_include_directories(
  slts_controller
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
  PRIVATE include)
target_compile_options(slts_controller PUBLIC ${COMPILE_OPTS})
target_compile_features(slts_controller PUBLIC cxx_std_17)
target_link_libraries(slts_controller PUBLIC Eigen3::Eigen)

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install_rules.cmake)
endif()

if(NOT DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(AUTHOR_WARNING "Developer mode is intended for developers of shared")
endif()

include(cmake/dev_mode.cmake)
