// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERNAVIGATION__ACTIONMANAGER_HPP_
#define ARUCOMARKERNAVIGATION__ACTIONMANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aruco_marker_detector_msgs/action/rotate_action.hpp>
#include <aruco_marker_detector_msgs/action/adjust_direction.hpp>
#include <aruco_marker_detector_msgs/action/adjust_position.hpp>
#include <aruco_marker_detector_msgs/action/approach_marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

// For Action Server
using ActionManagerMsg = aruco_marker_detector_msgs::action::ApproachMarker;
using GoalHandleActionManager = rclcpp_action::ServerGoalHandle<ActionManagerMsg>;

// For Action Client
// RotateAction 
using RotateActionMsg = aruco_marker_detector_msgs::action::RotateAction;
using GoalHandleRotateAction = rclcpp_action::ClientGoalHandle<RotateActionMsg>;

// AdjustDirection 
using AdjustDirectionMsg = aruco_marker_detector_msgs::action::AdjustDirection;
using GoalHandleAdjustDirection = rclcpp_action::ClientGoalHandle<AdjustDirectionMsg>;

// AdjustPosition
using AdjustPositionMsg = aruco_marker_detector_msgs::action::AdjustPosition;
using GoalHandleAdjustPosition= rclcpp_action::ClientGoalHandle<AdjustPositionMsg>;

// ApproachMarker
using ApproachMarkerMsg = ActionManagerMsg;
using GoalHandleApproachMarker = rclcpp_action::ClientGoalHandle<ApproachMarkerMsg>;

namespace ArucoMarkerNavigation{
class ActionManager : public rclcpp::Node
{
	public:
		ActionManager();
		~ActionManager();
		void initPubSub();
		void initParam();
		void initActionServer();
		void initActionClient();
		void waitResult();
		void waitReload();

		// For action server
		rclcpp_action::GoalResponse handle_goal(
				[[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
				[[maybe_unused]] std::shared_ptr<const ActionManagerMsg::Goal> goal
	    	);
		rclcpp_action::CancelResponse handle_cancel(
	    	    const std::shared_ptr<GoalHandleActionManager> goal_handle
	    	);
		void handle_accepted(
	    	    const std::shared_ptr<GoalHandleActionManager> goal_handle
	    	);
	    	void execute(
	    	    const std::shared_ptr<GoalHandleActionManager> goal_handle
	    	);

		void odomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg);
	
	private:
		rclcpp_action::Server<ActionManagerMsg>::SharedPtr rotate_action_srv_;

		// RotateAction Client
		rclcpp_action::Client<RotateActionMsg>::SendGoalOptions rotate_action_goal_options_;
		rclcpp_action::Client<RotateActionMsg>::SharedPtr rotate_action_client_;

		// AdjustDirection Client
		rclcpp_action::Client<AdjustDirectionMsg>::SendGoalOptions adjust_direction_goal_options_;
		rclcpp_action::Client<AdjustDirectionMsg>::SharedPtr adjust_direction_client_;

		// AdjustPosition Client
		rclcpp_action::Client<AdjustPositionMsg>::SendGoalOptions adjust_position_goal_options_;
		rclcpp_action::Client<AdjustPositionMsg>::SharedPtr adjust_position_client_;

		// ApproachMarker Client
		rclcpp_action::Client<ApproachMarkerMsg>::SendGoalOptions approach_marker_goal_options_;
		rclcpp_action::Client<ApproachMarkerMsg>::SharedPtr approach_marker_client_;

		int loop_rate_;
		bool wait_result_;
		double marker_x_, marker_y_, marker_t_;
		bool succeed_;
		int load_time_;		
		double odom_x_, odom_y_, odom_t_;

		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}

#endif // ARUCOMARKERNAVIGATION__ACTIONMANAGER_HPP_
