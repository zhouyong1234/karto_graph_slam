cmake_minimum_required(VERSION 3.0.2)
project(karto_graph_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# lib karto 

add_subdirectory(lib/open_karto)

# lib sparse_bundle_adjustment 
add_subdirectory(lib/sparse_bundle_adjustment)



find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    visualization_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")



# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
geometry_msgs
nav_msgs
roscpp
sensor_msgs
tf
open_karto
)

list(APPEND CMAKE_MODULE_PATH /home/zy/ekf_ws/src/slambook2/3rdparty/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3 REQUIRED)

# Ceres
find_package(Ceres REQUIRED)

# Gtsam
find_package(GTSAM REQUIRED)

find_package(CSparse REQUIRED)

# g2o
find_package(G2O REQUIRED)

find_package(Cholmod REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES karto_graph_slam
CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  lib/sparse_bundle_adjustment/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}

)

link_directories(${G2O_LIBRARY_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/karto_graph_slam.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

## Karto SLAM
add_executable(${PROJECT_NAME}_node
  src/karto_slam.cpp
  src/spa_solver.cpp
  src/ceres_solver.cpp
  src/gtsam_solver.cpp
  src/g2o_solver.cpp
)

add_dependencies(${PROJECT_NAME}_node
${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  sba
  ${CERES_LIBRARIES}
  gtsam
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARIES}
)
