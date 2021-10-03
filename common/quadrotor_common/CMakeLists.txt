cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_common)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

###########
## Build ##
###########

## Declare a C++ library
cs_add_library(${PROJECT_NAME}
  src/quadrotor_common/quadrotor_state_estimate.cpp
#   src/${PROJECT_NAME}/quadrotor_common.cpp
)

#############
## Install ##
#############

cs_install()
cs_export()
