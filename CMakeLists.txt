cmake_minimum_required(VERSION 2.8.3)
project(bag2video)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pybind11_catkin
  roscpp
  rospy
  sensor_msgs
  std_msgs
  imaging_sonar_msgs
)

#find_package(NUMPY REQUIRED)
#find_package(pybind11 REQUIRED)

catkin_python_setup()

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    cmake_modules
    pybind11_catkin
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    #${PYBIND11_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/bag2video scripts/bag2images
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#message("pybind11_include_dir " ${PYBIND11_INCLUDE_DIRS})

#pybind_add_module(libpyrosmsg
#    src_wrap/module.cpp
#)
