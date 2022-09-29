cmake_minimum_required(VERSION 3.15)
project(mav_visualization)

find_package(ament_cmake REQUIRED)
find_package(eigen_vendor REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Zc:__cplusplus")

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/marker_group.cpp
  src/leica_marker.cpp
  src/hexacopter_marker.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} visualization_msgs rclcpp)

install(TARGETS
  ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(visualization_msgs)
ament_export_dependencies(rclcpp)

############
# BINARIES #
############
add_executable(hexacopter_publisher
  src/hexacopter_publisher.cpp
)
target_link_libraries(hexacopter_publisher ${PROJECT_NAME})

add_executable(leica_publisher
  src/leica_publisher.cpp
)
target_link_libraries(leica_publisher ${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
##########
# EXPORT #
##########
ament_package()
