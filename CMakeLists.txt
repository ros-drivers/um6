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

##########
## Docs ##
##########

add_custom_target (${PROJECT_NAME}_doc 
  rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/../src/${PROJECT_NAME}
)

FILE(GLOB LINT_SRCS src/*.cpp include/um6/*.h)
FILE(GLOB NOLINT_SRCS include/um6/firmware_registers.h)
LIST(REMOVE_ITEM LINT_SRCS ${NOLINT_SRCS})
add_custom_target(${PROJECT_NAME}_cpplint
  COMMAND cpplint --filter=-whitespace/line_length ${LINT_SRCS})

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}_test_registers test/test_registers.cpp src/registers.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_comms test/test_comms.cpp src/comms.cpp src/registers.cpp)
if(TARGET ${PROJECT_NAME}_test_comms)
  target_link_libraries(${PROJECT_NAME}_test_comms util ${catkin_LIBRARIES})
endif()
