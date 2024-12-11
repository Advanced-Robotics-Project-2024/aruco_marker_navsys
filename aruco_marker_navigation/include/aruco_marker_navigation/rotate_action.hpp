// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERNAVIGATION__ROTATEACTION_HPP_
#define ARUCOMARKERNAVIGATION__ROTATEACTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aruco_marker_detector_msgs/msg/marker_infos.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <aruco_marker_detector_msgs/action/rotate_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>

using RotateActionMsg = aruco_marker_detector_msgs::action::RotateAction;
using GoalHandleRotateAction = rclcpp_action::ServerGoalHandle<RotateActionMsg>;

namespace ArucoMarkerNavigation{
class RotateAction : public rclcpp::Node
{
	public:
		RotateAction();
		~RotateAction();
		void initPubSub();
		void initParam();
		void initAction();
		void initTf();
		void markerInfosCb(aruco_marker_detector_msgs::msg::MarkerInfos::ConstSharedPtr msg);

		// For action server
		rclcpp_action::GoalResponse handle_goal(
				[[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
				[[maybe_unused]] std::shared_ptr<const RotateActionMsg::Goal> goal
    	);
		rclcpp_action::CancelResponse handle_cancel(
    	    const std::shared_ptr<GoalHandleRotateAction> goal_handle
    	);
		void handle_accepted(
    	    const std::shared_ptr<GoalHandleRotateAction> goal_handle
    	);

    	void execute(
    	    const std::shared_ptr<GoalHandleRotateAction> goal_handle
    	);
	
	private:
		rclcpp::Subscription<aruco_marker_detector_msgs::msg::MarkerInfos>::SharedPtr marker_infos_sub_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
		rclcpp_action::Server<RotateActionMsg>::SharedPtr rotate_action_srv_;

		double max_angular_vel_;
		bool init_tf_;
		int loop_rate_;
		std::vector<int> ids_;
		std::vector<float> xs_;
		std::vector<float> ys_;
		std::vector<float> ts_;

		std::shared_ptr<tf2_ros::Buffer> tf_;
    	std::shared_ptr<tf2_ros::TransformListener> tfl_;
};
}

#endif // ARUCOMARKERNAVIGATION__ROTATEACTION_HPP_
