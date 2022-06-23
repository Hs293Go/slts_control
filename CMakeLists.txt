cmake_minimum_required(VERSION 3.14.0)
project(slts_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(Eigen3 REQUIRED)

set(COMPILE_OPTS
    -Werror
    -Wall
    -Wextra
    -Wpedantic
    "$<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>$<$<CXX_COMPILER_ID:Clang>:-fcolor-diagnostics>"
    "$<$<CONFIG:Debug>:-g3>"
    "$<$<CONFIG:Release>:-O3>")

add_library(robust_tracker src/${PROJECT_NAME}/robust_tracker.cpp)
target_include_directories(robust_tracker PRIVATE include)
target_compile_options(robust_tracker PUBLIC ${COMPILE_OPTS})
target_link_libraries(robust_tracker PUBLIC Eigen3::Eigen)

add_library(system_model src/${PROJECT_NAME}/system_model.cpp)
target_include_directories(system_model PRIVATE include)
target_compile_options(system_model PUBLIC ${COMPILE_OPTS})
target_link_libraries(system_model Eigen3::Eigen)

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()
