cmake_minimum_required(VERSION 3.14.0)
project(slts_control VERSION 1.0.0)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)

set(COMPILE_OPTS
    "$<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>$<$<CXX_COMPILER_ID:Clang>:-fcolor-diagnostics>"
    "$<$<CONFIG:Debug>:-g3>"
    "$<$<CONFIG:Release>:-O3>")

set(WARN_OPTS -Werror -Wall -Wextra -Wpedantic)

add_library(robust_tracker src/${PROJECT_NAME}/robust_tracker.cpp)
target_include_directories(
  robust_tracker
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
  PRIVATE include)
target_compile_options(
  robust_tracker
  PUBLIC ${COMPILE_OPTS}
  PRIVATE ${WARN_OPTS})
target_link_libraries(robust_tracker PUBLIC Eigen3::Eigen)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.h")

install(
  TARGETS robust_tracker
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

include(CMakePackageConfigHelpers)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

include(CTest)
add_subdirectory(tests EXCLUDE_FROM_ALL)
