set(ABSL_PROPAGATE_CXX_STD ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG main)

FetchContent_MakeAvailable(googletest)
find_package(jsoncpp REQUIRED)

add_executable(test_system_model test_system_model.cpp)
target_include_directories(test_system_model
                           PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(
  test_system_model PRIVATE Eigen3::Eigen GTest::gmock GTest::gtest
                            GTest::gmock_main GTest::gtest_main system_model)

add_executable(test_controller test_controller.cpp)
target_include_directories(test_controller PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(
  test_controller
  PRIVATE Eigen3::Eigen
          GTest::gmock
          GTest::gtest
          GTest::gmock_main
          GTest::gtest_main
          jsoncpp
          tbb)
