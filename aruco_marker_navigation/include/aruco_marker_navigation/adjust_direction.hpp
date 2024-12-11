// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERNAVIGATION__ADJUSTDIRECTION_HPP_
#define ARUCOMARKERNAVIGATION__ADJUSTDIRECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <aruco_marker_detector_msgs/action/adjust_direction.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>

using AdjustDirectionMsg = aruco_marker_detector_msgs::action::AdjustDirection;
using GoalHandleAdjustDirection = rclcpp_action::ServerGoalHandle<AdjustDirectionMsg>;

namespace ArucoMarkerNavigation{
class AdjustDirection : public rclcpp::Node
{
	public:
		AdjustDirection();
		~AdjustDirection();
		void initPubSub();
		void initParam();
		void initAction();
		void initTf();
		void odomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg);

		// For action server
		rclcpp_action::GoalResponse handle_goal(
				[[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
				[[maybe_unused]] std::shared_ptr<const AdjustDirectionMsg::Goal> goal
    	);
		rclcpp_action::CancelResponse handle_cancel(
    	    const std::shared_ptr<GoalHandleAdjustDirection> goal_handle
    	);
		void handle_accepted(
    	    const std::shared_ptr<GoalHandleAdjustDirection> goal_handle
    	);

    	void execute(
    	    const std::shared_ptr<GoalHandleAdjustDirection> goal_handle
    	);
	
	private:
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
		rclcpp_action::Server<AdjustDirectionMsg>::SharedPtr rotate_action_srv_;

		double max_angular_vel_;
		bool init_tf_;
		int loop_rate_;
		std::vector<int> ids_;
		nav_msgs::msg::Odometry odom_;
		double torelance_error_;

		std::shared_ptr<tf2_ros::Buffer> tf_;
    	std::shared_ptr<tf2_ros::TransformListener> tfl_;
};
}

#endif // ARUCOMARKERNAVIGATION__ADJUSTDIRECTION_HPP_
