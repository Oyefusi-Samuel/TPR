cmake_minimum_required(VERSION 3.8)
project(jackal_ar4_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY .
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.sdf" PATTERN "*.world"
)

ament_package()