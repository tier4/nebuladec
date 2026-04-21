cmake_minimum_required(VERSION 3.14)
project(nebuladec_core)

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
  src/packet_sniffer.cpp
  src/topic_mapping.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Formatting is enforced by clang-format via pre-commit; skip the redundant
  # ament_uncrustify lint test whose style conflicts with clang-format.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_packet_sniffer test/test_packet_sniffer.cpp)
  target_link_libraries(test_packet_sniffer ${PROJECT_NAME})

  ament_add_gtest(test_topic_mapping test/test_topic_mapping.cpp)
  target_link_libraries(test_topic_mapping ${PROJECT_NAME})
endif()

ament_auto_package()
