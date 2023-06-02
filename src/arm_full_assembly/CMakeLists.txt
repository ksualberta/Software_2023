cmake_minimum_required(VERSION 3.5)

project(arm_full_assembly)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# This macro ensures modules and global scripts declared herein get installed
ament_package()