cmake_minimum_required(VERSION 3.8)
project(a_star_smooth_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

#---------- CPP -------------------------------------
add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit ${dependencies})

add_executable(a_star_smoother src/a_star_smoother.cpp)
ament_target_dependencies(a_star_smoother ${dependencies})

install(TARGETS
  pure_pursuit
  a_star_smoother
  DESTINATION lib/${PROJECT_NAME}
)
#-----------------------------------------------------


#------PYTHON--------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/a_star_planner.py
  ${PROJECT_NAME}/a_star_smoother.py
  DESTINATION lib/${PROJECT_NAME}
)
#---------------------------------------------


#------INSTALL FOLDERS AND FILES--------------
install(DIRECTORY
    config
    launch
    maps
    DESTINATION share/${PROJECT_NAME}/
)
#---------------------------------------------


ament_package()
