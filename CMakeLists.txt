cmake_minimum_required(VERSION 2.8.3)
project(um6)

find_package(catkin REQUIRED COMPONENTS roscpp serial sensor_msgs)

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(um6_driver src/main.cpp src/registers.cpp src/comms.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(um6_driver
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_um6.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
