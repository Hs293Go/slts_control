find_package(RapidJSON REQUIRED)

find_package(GTest REQUIRED)

add_executable(test_controller test_controller.cpp)
target_compile_definitions(
  test_controller
  PRIVATE
    TEST_DATAFILE="${CMAKE_CURRENT_SOURCE_DIR}/slts_control_matlab/test_controller.json"
    TEST_PARAMFILE="${CMAKE_CURRENT_SOURCE_DIR}/slts_control_matlab/params.json"
)
target_include_directories(test_controller PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(test_controller PRIVATE Eigen3::Eigen GTest::gtest
                                              GTest::gtest_main robust_tracker)

include(GoogleTest)
gtest_discover_tests(test_controller)
