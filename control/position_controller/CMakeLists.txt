cmake_minimum_required(VERSION 3.0.2)
project(position_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/position_controller.cpp
)

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/position_controller_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

cs_install()
cs_export()
