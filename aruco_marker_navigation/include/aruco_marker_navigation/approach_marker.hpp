// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERNAVIGATION__APPROACHMARKER_HPP_
#define ARUCOMARKERNAVIGATION__APPROACHMARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <aruco_marker_detector_msgs/action/approach_marker.hpp>
#include <aruco_marker_detector_msgs/msg/marker_infos.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <chrono>

using ApproachMarkerMsg = aruco_marker_detector_msgs::action::ApproachMarker;
using GoalHandleApproachMarker = rclcpp_action::ServerGoalHandle<ApproachMarkerMsg>;

namespace ArucoMarkerNavigation{
class ApproachMarker : public rclcpp::Node
{
	public:
		ApproachMarker();
		~ApproachMarker();
		void initPubSub();
		void initParam();
		void initAction();
		void initTf();
		void odomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg);
		void markerInfosCb(aruco_marker_detector_msgs::msg::MarkerInfos::ConstSharedPtr msg);
		void pubCmdVel(double linear_vel, double ang_vel);

		// For action server
		rclcpp_action::GoalResponse handle_goal(
				[[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
				[[maybe_unused]] std::shared_ptr<const ApproachMarkerMsg::Goal> goal
    	);
		rclcpp_action::CancelResponse handle_cancel(
    	    const std::shared_ptr<GoalHandleApproachMarker> goal_handle
    	);
		void handle_accepted(
    	    const std::shared_ptr<GoalHandleApproachMarker> goal_handle
    	);

    	void execute(
    	    const std::shared_ptr<GoalHandleApproachMarker> goal_handle
    	);
	
	private:
		rclcpp::Subscription<aruco_marker_detector_msgs::msg::MarkerInfos>::SharedPtr marker_infos_sub_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
		rclcpp_action::Server<ApproachMarkerMsg>::SharedPtr approach_marker_srv_;

		double max_linear_vel_;
		double max_angular_vel_;
		int loop_rate_;
		std::vector<int> ids_;
		std::vector<float> xs_, ys_, ts_;
		nav_msgs::msg::Odometry odom_;
		double torelance_length_error_;
		double torelance_angle_error_;
		double kp_x_, kp_y_, kp_t_;
		int torelance_lost_time_;
		int torelance_lost_count_;

		std::shared_ptr<tf2_ros::Buffer> tf_;
    	std::shared_ptr<tf2_ros::TransformListener> tfl_;
};
}

#endif // ARUCOMARKERNAVIGATION__APPROACHMARKER_HPP_
