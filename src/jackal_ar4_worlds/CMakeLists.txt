cmake_minimum_required(VERSION 3.8)
project(jackal_ar4_worlds)

find_package(ament_cmake REQUIRED)

# Install worlds/ and models/ into share so that:
#   get_package_share_directory('jackal_ar4_worlds')/worlds/  → .sdf world files
#   get_package_share_directory('jackal_ar4_worlds')/models/  → Gazebo model folders
#
# The launch file sets GZ_SIM_RESOURCE_PATH to the models/ directory so that
# <include><uri>room_with_walls_star</uri> in world SDFs resolves correctly.
install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()