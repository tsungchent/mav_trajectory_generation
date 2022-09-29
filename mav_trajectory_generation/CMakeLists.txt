cmake_minimum_required(VERSION 3.15)
# cmake_policy(SET CMP0074 NEW)

project(mav_trajectory_generation)

find_package(ament_cmake REQUIRED)

find_package(gflags_vendor REQUIRED)
find_package(glog_vendor REQUIRED)
find_package(eigen_vendor REQUIRED)
find_package(yaml_cpp_vendor QUIET)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(nlopt_vendor REQUIRED)
find_package(Boost REQUIRED)

# if(${yaml_cpp_vendor_FOUND})
#   message(STATUS "Found yaml_cpp_vendor, using instead of system library.")
#   set(YamlCpp_LIBRARIES ${yaml_cpp_vendor_LIBRARIES})
#   set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_vendor_INCLUDE_DIRS})
# else()
#   find_package(PkgConfig)
#   message(STATUS "No yaml_cpp_vendor, using yaml-cpp system library instead.")
#   pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
# endif()

set(CMAKE_C_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Zc:__cplusplus")
# message(CMAKE_CXX_FLAGS:${CMAKE_CXX_FLAGS})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "mav_msgs"
  "geometry_msgs"
  "std_msgs"
  "yaml_cpp_vendor"
)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES} ${NLOPT_LIBRARIES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>"
  "$<BUILD_INTERFACE:${GFLAGS_INCLUDE_DIR}>"
  "$<BUILD_INTERFACE:${GLOG_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
# if(MSVC)
#  target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
# endif()

install(TARGETS
  ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})


ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(yaml_cpp_vendor)
ament_export_dependencies(gflags_vendor)
ament_export_dependencies(glog_vendor)
ament_export_dependencies(mav_msgs)
ament_export_dependencies(mav_planning_msgs)


############
# BINARIES #
############
add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME})

find_package(ament_cmake_gtest REQUIRED)
#########
# TESTS #
#########
ament_add_gtest(test_polynomial
  test/test_polynomial.cpp
)
target_link_libraries(test_polynomial ${PROJECT_NAME})

ament_add_gtest(test_polynomial_optimization
  test/test_polynomial_optimization.cpp
)
target_link_libraries(test_polynomial_optimization ${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

##########
# EXPORT #
##########
ament_package()
