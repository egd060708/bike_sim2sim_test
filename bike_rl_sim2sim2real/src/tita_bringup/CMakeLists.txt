cmake_minimum_required(VERSION 3.8)
project(tita_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
cmake_policy(SET CMP0074 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common filters io)
include_directories(include)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    src/tita_utils.cpp
    src/tita_pcl.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

