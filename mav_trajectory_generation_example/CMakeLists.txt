cmake_minimum_required(VERSION 3.15)
project(mav_trajectory_generation_example)

find_package(ament_cmake REQUIRED)

find_package(nlopt_vendor REQUIRED)
find_package(eigen_vendor REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_trajectory_generation_ros REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  # ROS > Indigo.
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Zc:__cplusplus")

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

############
# BINARIES #
############
add_executable(trajectory_generation_example
        src/example_planner.cc
        src/example_planner_node.cc
)
if(TARGET trajectory_generation_example)
  ament_target_dependencies(trajectory_generation_example
    mav_trajectory_generation
    rclcpp
    mav_msgs
    nav_msgs
    mav_planning_msgs
    visualization_msgs
    mav_trajectory_generation_ros
  )
endif()

add_executable(trajectory_generation_6dof_example
        src/example_planner_6dof.cc
        src/example_planner_6dof_node.cc
)

if(TARGET trajectory_generation_6dof_example)
  ament_target_dependencies(trajectory_generation_6dof_example
    mav_trajectory_generation
    rclcpp
    mav_msgs
    nav_msgs
    mav_planning_msgs
    visualization_msgs
    mav_trajectory_generation_ros
  )
endif()

##########
# EXPORT #
##########
ament_package()
