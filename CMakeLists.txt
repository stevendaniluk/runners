cmake_minimum_required(VERSION 2.8.3)
project(runners)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
)

# Declare the catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES runners
#  CATKIN_DEPENDS navigation runners_navigation stage_ros
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ executables
add_executable(main src/main.cpp)

# Specify link libraries
target_link_libraries(main ${catkin_LIBRARIES} runner)

# Declare a C++ library
add_library(runner src/runner/runner.cpp)
# Specify link libraries
target_link_libraries(runner ${catkin_LIBRARIES})
