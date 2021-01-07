cmake_minimum_required(VERSION 2.8.3)
project(my_dynamixel_workbench)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  geometry_msgs
  message_generation
  roscpp
  std_msgs
)

# add Eigen3
find_package(Eigen3 REQUIRED)

# add yaml_cpp 
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
    #dxl_state.msg
    #desired_trajectory.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
    #DynamixelCommand.srv


# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_dynamixel_workbench
  CATKIN_DEPENDS dynamixel_workbench_toolbox roscpp std_msgs dynamixel_workbench_msgs message_runtime
  DEPENDS EIGEN3
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)



add_executable(my_dynamixel_master src/my_dxl_master.cpp src/trajectory_generator.cpp)
target_link_libraries(my_dynamixel_master ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(my_dynamixel_operator src/my_operator.cpp)
target_link_libraries(my_dynamixel_operator ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(multi_motors_demo src/multi_motors_demo.cpp src/my_dxl_controller.cpp)
target_link_libraries(multi_motors_demo ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})



