cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  # cmake warns if loaded from a min-3.0-required parent dir, so silence the warning:
  cmake_policy(SET CMP0048 NEW)
endif()
project(uav_agent)



## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
 add_compile_options(-Wno-psabi)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  gcs
)


find_package(DJIOSDK REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
  CATKIN_DEPENDS
        serial
        geometry_msgs
        nav_msgs
        std_msgs
        gcs

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node
                src/controllers/pid.cpp
                src/base/base.cpp
                src/base/control.cpp
                src/base/planner.cpp

)

target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
                      ${EIGEN_LIBRARIES}
                      ${DJIOSDK_LIBRARIES}
                      )

add_dependencies(${PROJECT_NAME}_node
                  ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)                 
