cmake_minimum_required(VERSION 3.16)
project(VantagePath VERSION 0.1.0 LANGUAGES CXX)

add_library(vantagepath
  src/controller.cpp
  src/follower.cpp
  src/odometry.cpp
  src/sensor_fusion.cpp
  src/trajectory.cpp)
add_library(VantagePath::VantagePath ALIAS vantagepath)

target_include_directories(vantagepath PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vantagepath PUBLIC cxx_std_17)
target_compile_options(vantagepath PRIVATE -Wall -Wextra -Wpedantic)

option(VANTAGEPATH_BUILD_TESTS "Build host-side tests" ON)
if(VANTAGEPATH_BUILD_TESTS)
  enable_testing()
  add_executable(vantagepath_tests tests/test_vantage.cpp)
  target_link_libraries(vantagepath_tests PRIVATE VantagePath::VantagePath)
  add_test(NAME vantagepath_tests COMMAND vantagepath_tests)
endif()

include(GNUInstallDirs)
install(TARGETS vantagepath EXPORT VantagePathTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT VantagePathTargets NAMESPACE VantagePath::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/VantagePath)
