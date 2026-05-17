cmake_minimum_required(VERSION 3.14)
project(nebuladec_adapters)

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

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/decoder.cpp
  src/accelerated_hesai_driver.cpp
  src/accelerated_seyond_decoder.cpp
  src/accelerated_velodyne_decoder.cpp
  src/accelerated_velodyne_driver.cpp
  src/hesai_adapter.cpp
  src/seyond_adapter.cpp
  src/velodyne_adapter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Mirror nebuladec_core's opt-in profiler switch. ament does not propagate
# target_compile_definitions across packages, so adapters must independently
# define NEBULADEC_PROFILE=1 for the profiling macro to expand to active
# instrumentation in the adapter translation units.
option(NEBULADEC_PROFILE "Enable opt-in nebuladec micro-profiler" OFF)
if(NEBULADEC_PROFILE)
  target_compile_definitions(${PROJECT_NAME} PUBLIC NEBULADEC_PROFILE=1)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Formatting is enforced by clang-format via pre-commit; skip the redundant
  # ament_uncrustify lint test whose style conflicts with clang-format.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_decoder test/test_decoder.cpp)
  target_link_libraries(test_decoder ${PROJECT_NAME})

  ament_add_gtest(test_hesai_adapter test/test_hesai_adapter.cpp)
  target_link_libraries(test_hesai_adapter ${PROJECT_NAME})

  ament_add_gtest(test_velodyne_adapter test/test_velodyne_adapter.cpp)
  target_link_libraries(test_velodyne_adapter ${PROJECT_NAME})

  ament_add_gtest(test_decoder_integration test/test_decoder_integration.cpp)
  target_link_libraries(test_decoder_integration ${PROJECT_NAME})

  ament_add_gtest(test_decoder_concurrency test/test_decoder_concurrency.cpp)
  target_link_libraries(test_decoder_concurrency ${PROJECT_NAME})
endif()

ament_auto_package()
