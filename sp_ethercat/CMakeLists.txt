cmake_minimum_required(VERSION 2.8.3)
project(sp_ethercat)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES sp_ethercat 
#  CATKIN_DEPENDS hardware_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(sp_ethercat
  src/sp_ethercat.c
)

## Specify libraries to link a library or executable target against
target_link_libraries(sp_ethercat ethercat ${catkin_LIBRARIES})

