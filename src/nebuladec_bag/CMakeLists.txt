cmake_minimum_required(VERSION 3.14)
project(nebuladec_bag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(SQLite3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/bag_io.cpp
  src/packet_source.cpp
  src/point_cloud2.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# inspect() on a bare .db3 file bypasses rosbag2_cpp::Reader to skip its
# 17GB/941k-row metadata reconstruction and talks to libsqlite3 directly.
target_link_libraries(${PROJECT_NAME} SQLite::SQLite3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Formatting is enforced by clang-format via pre-commit; skip the redundant
  # ament_uncrustify lint test whose style conflicts with clang-format.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detect_input test/test_detect_input.cpp)
  target_link_libraries(test_detect_input ${PROJECT_NAME})

  ament_add_gtest(test_point_cloud2 test/test_point_cloud2.cpp)
  target_link_libraries(test_point_cloud2 ${PROJECT_NAME})

  # Regression test for the Hesai QT128 cloud-ownership bug. Uses the
  # QT128 ground-truth bag shipped by upstream nebula_hesai.
  set(_qt128_bag
    "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/nebula/src/nebula_hesai/nebula_hesai/test_resources/decoder_ground_truth/qt128/1730273789074637152/1730273789074637152_0.db3")
  if(EXISTS "${_qt128_bag}")
    ament_add_gtest(test_convert_hesai_qt128 test/test_convert_hesai_qt128.cpp)
    target_link_libraries(test_convert_hesai_qt128 ${PROJECT_NAME})
    target_compile_definitions(test_convert_hesai_qt128 PRIVATE
      "NEBULADEC_BAG_TEST_QT128_BAG=\"${_qt128_bag}\"")
  else()
    message(STATUS "Skipping test_convert_hesai_qt128: QT128 ground-truth bag not found at ${_qt128_bag}")
  endif()

  # Regression test for the Velodyne trailing-scan bug. Uses the VLP16
  # ground-truth bag shipped by upstream nebula_velodyne.
  set(_vlp16_bag
    "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/nebula/src/nebula_velodyne/nebula_velodyne/test_resources/decoder_ground_truth/vlp16/1673400471837873222/1673400471837873222_0.db3")
  if(EXISTS "${_vlp16_bag}")
    ament_add_gtest(test_convert_velodyne_vlp16 test/test_convert_velodyne_vlp16.cpp)
    target_link_libraries(test_convert_velodyne_vlp16 ${PROJECT_NAME})
    target_compile_definitions(test_convert_velodyne_vlp16 PRIVATE
      "NEBULADEC_BAG_TEST_VLP16_BAG=\"${_vlp16_bag}\"")
  else()
    message(STATUS "Skipping test_convert_velodyne_vlp16: VLP16 ground-truth bag not found at ${_vlp16_bag}")
  endif()
endif()

ament_auto_package()
