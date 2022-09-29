#include <mav_trajectory_generation_example/example_planner.h>
#include <rclcpp/rclcpp.hpp>

ExamplePlanner::ExamplePlanner(rclcpp::Node::SharedPtr nh) :
	nh_(nh),
	max_v_(2.0),
	max_a_(2.0),
	current_velocity_(Eigen::Vector3d::Zero()),
	current_pose_(Eigen::Affine3d::Identity()) {

	// Load params
	rclcpp::Parameter param;
	if (nh_->get_parameter(std::string(nh_->get_name()) + "/max_v", param)) {
		RCLCPP_WARN(rclcpp::get_logger(""), "[example_planner] param max_v not found");
	}
	else {
		max_v_ = param.as_double();
	}

	if (nh_->get_parameter(std::string(nh_->get_name()) + "/max_a", param)) {
		RCLCPP_WARN(rclcpp::get_logger(""), "[example_planner] param max_v not found");
	}
	else {
		max_a_ = param.as_double();
	}
	//if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
	//  RCLCPP_WARN(rclcpp::get_logger(""), "[example_planner] param max_v not found");
	//}
	//if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
	//  RCLCPP_WARN(rclcpp::get_logger(""), "[example_planner] param max_a not found");
	//}
	// create publisher for RVIZ markers
	pub_markers_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 1);
	// nh.advertise<visualization_msgs::msg::MarkerArray>("trajectory_markers", 0);
	pub_trajectory_ = nh_->create_publisher<mav_planning_msgs::PolynomialTrajectory>("trajectory", 1);
	// pub_trajectory_ =
	//  nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory",
	// 0);
	// subscriber for Odometry
	// sub_odom_ =
	//	nh.subscribe("uav_pose", 1, &ExamplePlanner::uavOdomCallback, this);
	sub_odom_ = nh_->create_subscription<nav_msgs::msg::Odometry>("uav_pose", 1, std::bind( & ExamplePlanner::uavOdomCallback, this, std::placeholders::_1));
}

// Callback to get current Pose of UAV
void ExamplePlanner::uavOdomCallback(const nav_msgs::msg::Odometry& odom) {

	// store current position in our planner
	tf2::fromMsg(odom.pose.pose, current_pose_);
	// store current vleocity
	tf2::fromMsg(odom.twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
	max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool ExamplePlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
	const Eigen::VectorXd& goal_vel,
	mav_trajectory_generation::Trajectory* trajectory) {


	// 3 Dimensional trajectory => through carteisan space, no orientation
	const int dimension = 3;

	// Array for all waypoints and their constrains
	mav_trajectory_generation::Vertex::Vector vertices;

	// Optimze up to 4th order derivative (SNAP)
	const int derivative_to_optimize =
		mav_trajectory_generation::derivative_order::SNAP;

	// we have 2 vertices:
	// Start = current position
	// end = desired position and velocity
	mav_trajectory_generation::Vertex start(dimension), end(dimension);


	/******* Configure start point *******/
	// set start point constraints to current position and set all derivatives to zero
	start.makeStartOrEnd(current_pose_.translation(),
		derivative_to_optimize);

	// set start point's velocity to be constrained to current velocity
	start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
		current_velocity_);

	// add waypoint to list
	vertices.push_back(start);


	/******* Configure end point *******/
	// set end point constraints to desired position and set all derivatives to zero
	end.makeStartOrEnd(goal_pos,
		derivative_to_optimize);

	// set start point's velocity to be constrained to current velocity
	end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
		goal_vel);

	// add waypoint to list
	vertices.push_back(end);

	// setimate initial segment times
	std::vector<double> segment_times;
	segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

	// Set up polynomial solver with default params
	mav_trajectory_generation::NonlinearOptimizationParameters parameters;

	// set up optimization problem
	const int N = 10;
	mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
	opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

	// constrain velocity and acceleration
	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

	// solve trajectory
	opt.optimize();

	// get trajectory as polynomial parameters
	opt.getTrajectory(&(*trajectory));

	return true;
}

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory) {
	// send trajectory as markers to display them in RVIZ
	visualization_msgs::msg::MarkerArray markers;
	double distance =
		0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
	std::string frame_id = "world";

	mav_trajectory_generation::drawMavTrajectory(trajectory,
		distance,
		frame_id,
		&markers);
	pub_markers_->publish(markers);

	// send trajectory to be executed on UAV
	mav_planning_msgs::PolynomialTrajectory msg;
	mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
		&msg);
	msg.header.frame_id = "world";
	pub_trajectory_->publish(msg);

	return true;
}

