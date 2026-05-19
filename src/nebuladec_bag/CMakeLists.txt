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
# mcap_vendor exports the header-only libmcap C++ library. We use it
# directly on the write side because rosbag2_storage_mcap's only way to
# obtain a Schema record is to look the type up in the local ament
# index -- when the type is not installed, the schema is written empty.
# Forwarding embedded schemas requires bypassing that path entirely.
find_package(mcap_vendor REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ament_message_loader.cpp
  src/bag_io.cpp
  src/convert_parallel.cpp
  src/mcap_definition_source.cpp
  src/mcap_definition_writer.cpp
  src/message_definition.cpp
  src/output_writer.cpp
  src/packet_source.cpp
  src/point_cloud2.cpp
  src/sqlite_definition_source.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# inspect() on a bare .db3 file bypasses rosbag2_cpp::Reader to skip its
# 17GB/941k-row metadata reconstruction and talks to libsqlite3 directly.
target_link_libraries(${PROJECT_NAME} SQLite::SQLite3 mcap_vendor::mcap)

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

  # Covers MessageDefinitionSource for both mcap (Schema records) and
  # sqlite3 (metadata.yaml's Iron+ `message_definition` block, plus the
  # Humble soft-fail-to-empty case). The test builds its MCAP fixtures
  # in-process via mcap::McapWriter so no ground-truth bag is needed.
  ament_add_gtest(test_message_definition_source test/test_message_definition_source.cpp)
  target_link_libraries(test_message_definition_source ${PROJECT_NAME} mcap_vendor::mcap)

  # End-to-end regression for the embedded-definition forwarding fix:
  # synthesizes an MCAP with one topic whose ROS 2 type package is not
  # installed in the build environment, runs `bag::convert()`, and
  # verifies the output's Schema record carries the same .msg text.
  # Mirrors the user's reported `oxts_msgs/msg/Ncom` scenario.
  ament_add_gtest(test_convert_embedded_definition test/test_convert_embedded_definition.cpp)
  target_link_libraries(test_convert_embedded_definition ${PROJECT_NAME} mcap_vendor::mcap)

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

    # End-to-end equivalence test for the parallel pipeline: runs the
    # VLP16 fixture through both `options.sequential = true` (legacy
    # single-threaded path) and the default 3-stage pipeline, and
    # asserts that the output bags hold the same multiset of
    # (topic, log_time, payload) tuples plus matching ConvertResult
    # counts.
    ament_add_gtest(test_convert_parallel_equivalence
      test/test_convert_parallel_equivalence.cpp)
    target_link_libraries(test_convert_parallel_equivalence ${PROJECT_NAME})
    target_compile_definitions(test_convert_parallel_equivalence PRIVATE
      "NEBULADEC_BAG_TEST_VLP16_BAG=\"${_vlp16_bag}\"")

    # ConvertOptions::on_progress contract test. Reuses the VLP16
    # fixture across the sequential path and the default 3-stage
    # pipeline, plus a throwing-callback case that exercises the
    # ProgressReporter's exception swallowing.
    ament_add_gtest(test_convert_progress test/test_convert_progress.cpp)
    target_link_libraries(test_convert_progress ${PROJECT_NAME})
    target_compile_definitions(test_convert_progress PRIVATE
      "NEBULADEC_BAG_TEST_VLP16_BAG=\"${_vlp16_bag}\"")

    # Regression test for the `--workers < K` deadlock in the parallel
    # pipeline. Synthesises a two-topic bag from the VLP16 fixture and
    # runs convert() with workers=1 inside a future + wait_for(60s);
    # the pre-fix code never returned for this configuration.
    ament_add_gtest(test_convert_workers_one test/test_convert_workers_one.cpp)
    target_link_libraries(test_convert_workers_one ${PROJECT_NAME})
    target_compile_definitions(test_convert_workers_one PRIVATE
      "NEBULADEC_BAG_TEST_VLP16_BAG=\"${_vlp16_bag}\"")

    # Contract test for `ConvertOptions::mcap`: on sqlite3 input the
    # MCAP writer options must be ignored (warn-and-ignore policy), and
    # an empty McapWriteOptions{} must be a strict no-op.
    ament_add_gtest(test_convert_mcap_options test/test_convert_mcap_options.cpp)
    target_link_libraries(test_convert_mcap_options ${PROJECT_NAME})
    target_compile_definitions(test_convert_mcap_options PRIVATE
      "NEBULADEC_BAG_TEST_VLP16_BAG=\"${_vlp16_bag}\"")
  else()
    message(STATUS "Skipping test_convert_velodyne_vlp16: VLP16 ground-truth bag not found at ${_vlp16_bag}")
    message(STATUS "Skipping test_convert_parallel_equivalence: VLP16 ground-truth bag not found at ${_vlp16_bag}")
    message(STATUS "Skipping test_convert_progress: VLP16 ground-truth bag not found at ${_vlp16_bag}")
    message(STATUS "Skipping test_convert_workers_one: VLP16 ground-truth bag not found at ${_vlp16_bag}")
    message(STATUS "Skipping test_convert_mcap_options: VLP16 ground-truth bag not found at ${_vlp16_bag}")
  endif()
endif()

ament_auto_package()
