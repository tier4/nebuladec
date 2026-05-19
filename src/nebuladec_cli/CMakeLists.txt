cmake_minimum_required(VERSION 3.14)
project(nebuladec_cli)

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

add_executable(nebuladec src/main.cpp)
if(${PROJECT_NAME}_FOUND_BUILD_DEPENDS)
  ament_target_dependencies(nebuladec ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endif()
# tabulate (p-ranav/tabulate) and indicators (p-ranav/indicators) are
# vendored as single-header amalgamations at third_party/tabulate.hpp
# and third_party/indicators.hpp (both MIT) -- keeps IntelliSense
# working before the first build and avoids extra rosdep dependencies.
target_include_directories(nebuladec SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(TARGETS nebuladec
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Formatting is enforced by clang-format via pre-commit; skip the redundant
  # ament_uncrustify lint test whose style conflicts with clang-format.
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Source-scanning linters (copyright headers, cpplint, cppcheck) would
  # trip on the vendored single-header tabulate (include/third_party/
  # tabulate.hpp). Setting *_FOUND before ament_lint_auto_find_test_
  # dependencies() prevents their ament-package extension hooks from
  # registering, so they do not run. The package's own sources are
  # covered by clang-format + cpplint via pre-commit (see
  # .pre-commit-config.yaml, which excludes the same third_party tree).
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
