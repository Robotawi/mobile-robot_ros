#1 Update cmake version
cmake_minimum_required(VERSION 3.8)
project(my_robot)

#2 udate compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#3 update dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)

#4 install pkg directories
install(DIRECTORY
  launch
  meshes
  urdf
  worlds
  models
  params

  DESTINATION share/${PROJECT_NAME}/
)

#5 register package and make discoverable
ament_package()