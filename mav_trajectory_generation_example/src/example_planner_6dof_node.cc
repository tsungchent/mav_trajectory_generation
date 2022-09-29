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

// #include  "ros/ros.h"
#include <rclcpp/rclcpp.hpp>
#include <mav_trajectory_generation_example/example_planner.h>

#include <iostream>

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  // ros::NodeHandle n;
  rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("simple_planner");
  ExamplePlanner planner(n);
  RCLCPP_INFO(rclcpp::get_logger(""), "SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  rclcpp::Rate(200.0).sleep();
  RCLCPP_INFO(rclcpp::get_logger(""), "WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // define set point
  Eigen::VectorXd pose, twist;
  pose.resize(6);
  twist.resize(6);
  Eigen::Vector3d position, rotation_vec;
  Eigen::Matrix3d rotation_mat;
  rotation_mat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) 
              * Eigen::AngleAxisd(M_PI / 2.0,  Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  mav_msgs::vectorFromRotationMatrix(rotation_mat, &rotation_vec);
  position << 0.0, 1.0, 2.0;
  pose << position, rotation_vec;
  twist << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  RCLCPP_WARN(rclcpp::get_logger(""), "PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(n);
  for (int i = 0; i < 10; i++) {
    // ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
      executor->spin_once();
  }

  mav_trajectory_generation::Trajectory trajectory;
  planner.planTrajectory(pose, twist, &trajectory);
  planner.publishTrajectory(trajectory);
  RCLCPP_WARN(rclcpp::get_logger(""), "DONE. GOODBYE.");

  return 0;
}