set(ABSL_PROPAGATE_CXX_STD ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG main)

FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz)

FetchContent_MakeAvailable(googletest json)

add_executable(test_controller test_controller.cpp)
target_compile_definitions(
  test_controller
  PRIVATE TEST_DATAFILE="${CMAKE_CURRENT_SOURCE_DIR}/test_controller.json")
target_include_directories(test_controller PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(
  test_controller
  PRIVATE Eigen3::Eigen
          GTest::gmock
          GTest::gtest
          GTest::gmock_main
          GTest::gtest_main
          nlohmann_json::nlohmann_json
          robust_tracker)

include(GoogleTest)
gtest_discover_tests(test_controller)
