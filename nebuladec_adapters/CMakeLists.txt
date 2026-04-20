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
  src/hesai_adapter.cpp
  src/robosense_adapter.cpp
  src/seyond_adapter.cpp
  src/velodyne_adapter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_decoder test/test_decoder.cpp)
  target_link_libraries(test_decoder ${PROJECT_NAME})

  ament_add_gtest(test_hesai_adapter test/test_hesai_adapter.cpp)
  target_link_libraries(test_hesai_adapter ${PROJECT_NAME})

  ament_add_gtest(test_velodyne_adapter test/test_velodyne_adapter.cpp)
  target_link_libraries(test_velodyne_adapter ${PROJECT_NAME})

  ament_add_gtest(test_robosense_adapter test/test_robosense_adapter.cpp)
  target_link_libraries(test_robosense_adapter ${PROJECT_NAME})

  ament_add_gtest(test_decoder_integration test/test_decoder_integration.cpp)
  target_link_libraries(test_decoder_integration ${PROJECT_NAME})
endif()

ament_auto_package()
