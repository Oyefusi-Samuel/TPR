cmake_minimum_required(VERSION 3.8)
project(jackal_ar4_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_common REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/ground_truth_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
