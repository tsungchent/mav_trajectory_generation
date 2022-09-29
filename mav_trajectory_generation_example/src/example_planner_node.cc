/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include "rclcpp/rclcpp.hpp"
#include <mav_trajectory_generation_example/example_planner.h>

#include <iostream>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("simple_planner");
  ExamplePlanner planner(n);
  RCLCPP_INFO(rclcpp::get_logger(""), "SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  rclcpp::Rate(200.0).sleep();
  RCLCPP_INFO(rclcpp::get_logger(""), "WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");
  // define set point
  Eigen::Vector3d position, velocity;
  position << 0.0, 1.0, 2.0;
  velocity << 0.0, 0.0, 0.0;
  
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(n);

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  RCLCPP_WARN(rclcpp::get_logger(""), "PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();
  for (int i = 0; i < 10; i++) {
      executor->spin_once();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  mav_trajectory_generation::Trajectory trajectory;
  planner.planTrajectory(position, velocity, &trajectory);
  planner.publishTrajectory(trajectory);
  RCLCPP_WARN(rclcpp::get_logger(""), "DONE. GOODBYE.");

  return 0;
}