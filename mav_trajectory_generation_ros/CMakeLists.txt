cmake_minimum_required(VERSION 3.15)
project(mav_trajectory_generation_ros)

find_package(ament_cmake REQUIRED)


# Link against system vendor yaml-cpp if installed.

find_package(yaml_cpp_vendor QUIET)
find_package(mav_trajectory_generation REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mav_visualization REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(eigen_vendor QUIET)
find_package(gflags_vendor REQUIRED)
find_package(glog_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(nlopt_vendor REQUIRED)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 14)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Zc:__cplusplus")

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

# if(yaml_cpp_vendor_FOUND)
#     message(STATUS "Found yaml_cpp_vendor, using instead of system library.")
#     set(YamlCpp_LIBRARIES ${yaml_cpp_vendor_LIBRARIES})
#     set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_vendor_INCLUDE_DIRS})
# else()
#     find_package(PkgConfig)
#     message(STATUS "No yaml_cpp_vendor, using yaml-cpp system library instead.")
#     pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
# endif()

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/feasibility_analytic.cpp
  src/feasibility_base.cpp
  src/feasibility_recursive.cpp
  src/feasibility_sampling.cpp
  src/input_constraints.cpp
  src/ros_conversions.cpp
  src/ros_visualization.cpp
)

# target_link_libraries(${PROJECT_NAME} ${yaml_cpp_vendor_LIBRARIES} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES} )

ament_target_dependencies(
  ${PROJECT_NAME}
  mav_trajectory_generation
  mav_msgs
  std_srvs
  yaml_cpp_vendor
  mav_visualization
  mav_planning_msgs
  rclcpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${YamlCpp_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(mav_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(yaml_cpp_vendor)
ament_export_dependencies(mav_trajectory_generation)
ament_export_dependencies(mav_visualization)
ament_export_dependencies(mav_planning_msgs)
ament_export_dependencies(rclcpp)

find_package(ament_cmake_gtest REQUIRED)

############
# BINARIES #
############
add_executable(trajectory_sampler_node
  src/trajectory_sampler_node.cpp
)
target_link_libraries(trajectory_sampler_node ${PROJECT_NAME})

add_executable(time_evaluation_node
  src/time_evaluation_node.cpp
)
target_link_libraries(time_evaluation_node ${PROJECT_NAME})

#########
# TESTS #
#########
ament_add_gtest(test_feasibility
  test/test_feasibility.cpp
)
target_link_libraries(test_feasibility ${PROJECT_NAME})

# ament_export_dependencies(${PROJECT_NAME})
# ament_export_dependencies(mav_trajectory_generation)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

##########
# EXPORT #
##########
ament_package()