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
cs_add_library(${PROJECT_NAME}
  src/position_controller/position_controller.cpp
  src/reference_inputs/reference_inputs.cpp
  src/reference_inputs/nominal_reference_inputs.cpp
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_position_controller.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
