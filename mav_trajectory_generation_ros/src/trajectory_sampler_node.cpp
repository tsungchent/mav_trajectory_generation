/*
 * Copyright (c) 2017, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <rclcpp/rclcpp.hpp>
#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>

TrajectorySamplerNode::TrajectorySamplerNode(const rclcpp::Node::SharedPtr& nh /*,
                                             const rclcpp::Node& nh_private */)
    : nh_(nh),
      publish_whole_trajectory_(false),
      dt_(0.01),
      current_sample_time_(0.0) {
  nh_->set_parameter(rclcpp::Parameter("publish_whole_trajectory", publish_whole_trajectory_));
  nh_->set_parameter(rclcpp::Parameter("dt", dt_));


  command_pub_ = nh_->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  trajectory_sub_ = nh_->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory>(
      "path_segments", 10, std::bind( & TrajectorySamplerNode::pathSegmentsCallback, this, std::placeholders::_1));
  trajectory4D_sub_ = nh_->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>(
      "path_segments_4D", 10,  std::bind( & TrajectorySamplerNode::pathSegments4DCallback, this, std::placeholders::_1));
  stop_srv_ = nh_->create_service<std_srvs::srv::Empty>(
       "stop_sampling", std::bind( & TrajectorySamplerNode::stopSamplingCallback, this, std::placeholders::_1, std::placeholders::_2));
  // stop_srv_ = nh_->create_service<std_srvs::srv::Empty>(
   //    "stop_sampling", &TrajectorySamplerNode::stopSamplingCallback);

  position_hold_client_ =
      nh_->create_client<std_srvs::srv::Empty>("back_to_position_hold");

  const bool oneshot = false;
  const bool autostart = false;
  
  publish_timer_ = nh_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_)),
                                   std::bind( & TrajectorySamplerNode::commandTimerCallback, this));
}

TrajectorySamplerNode::~TrajectorySamplerNode() { 
    publish_timer_->cancel(); 
    publish_timer_->reset();
}

void TrajectorySamplerNode::pathSegmentsCallback(
    const mav_planning_msgs::PolynomialTrajectory& segments_message) {
  if (segments_message.segments.empty()) {
    RCLCPP_WARN(rclcpp::get_logger(""), "Trajectory sampler: received empty waypoint message");
    return;
  } else {
    RCLCPP_INFO(rclcpp::get_logger(""), "Trajectory sampler: received %lu waypoints",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::pathSegments4DCallback(
    const mav_planning_msgs::PolynomialTrajectory4D& segments_message) {
  if (segments_message.segments.empty()) {
    RCLCPP_WARN(rclcpp::get_logger(""), "Trajectory sampler: received empty waypoint message");
    return;
  } else {
    RCLCPP_INFO(rclcpp::get_logger(""), "Trajectory sampler: received %lu waypoints",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::processTrajectory() {
  // Call the service call to takeover publishing commands.
  // exists
  if (position_hold_client_->service_is_ready()) {
    // std_srvs::srv::Empty empty_call;
      std_srvs::srv::Empty::Request::SharedPtr empty_call_request = std::make_shared< std_srvs::srv::Empty::Request>();
    // position_hold_client_->call(empty_call);
    position_hold_client_->async_send_request(empty_call_request);
  }

  if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
                                                     &trajectory_points);
    trajectory_msgs::msg::MultiDOFJointTrajectory msg_pub;
    msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
    command_pub_->publish(msg_pub);
  } else {
    // publish_timer_->start();
    current_sample_time_ = 0.0;
    start_time_ = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  }
}

bool TrajectorySamplerNode::stopSamplingCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) {
  // publish_timer_.stop();
  
    return true;
}

void TrajectorySamplerNode::commandTimerCallback(/*const ros::TimerEvent&*/) {
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    trajectory_msgs::msg::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
    if (!success) {
      // publish_timer_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    msg.points[0].time_from_start = rclcpp::Duration(static_cast<int64_t>(current_sample_time_ * 1000000000));
    command_pub_->publish(msg);
    current_sample_time_ += dt_;
  } else {
    // publish_timer_.stop();
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv/*, "trajectory_sampler_node"*/);
  // ros::NodeHandle nh("");
  // ros::NodeHandle nh_private("~");
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("trajectory_sampler_node");
  TrajectorySamplerNode trajectory_sampler_node(nh/*, nh_private*/);
  RCLCPP_INFO(rclcpp::get_logger(""), "Initialized trajectory sampler.");
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(nh);
  // ros::spin();
  executor->spin();
}
