cmake_minimum_required(VERSION 2.8.3)
project(crane_dh_inverse)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
INCLUDE_DIRS include
LIBRARIES crane_dh_lib
DEPENDS EIGEN3
)

###########
## Build ##
###########
find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR/cmake})
#find_package(Eigen REQUIRED)
# include_directories(
#     /home/rel/Desktop/Eigen
# )

add_library(crane_dh_lib src/kinematics.cpp)
# projectの名前とかぶるとexecutableが追加されない
add_executable(crane_dh_inverse src/dh_inverse.cpp)

target_link_libraries(crane_dh_lib   ${catkin_LIBRARIES})

target_link_libraries(crane_dh_inverse
    ${catkin_LIBRARIES}
    crane_dh_lib
)
