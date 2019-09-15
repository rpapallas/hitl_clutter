cmake_minimum_required (VERSION 2.8.3)
project(nonprehensile_planning)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ompl REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Threads)

# ROS Stuff
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES lib
        CATKIN_DEPENDS roscpp
)

include(compilerSettings)
include(boost)
include(mujoco)
include(openmp)

include_directories(${Boost_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS} 
                    ${Mujoco_INCLUDE_DIRS}
                    "include")

link_directories(${Boost_LIBRARY_DIRS} 
                 ${OMPL_LIBRARY_DIRS}
                 ${CMAKE_CURRENT_SOURCE_DIR}/lib/mjpro200/)

include(executables)
