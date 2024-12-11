// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERNAVIGATION__TOWARDMARKER_HPP_
#define ARUCOMARKERNAVIGATION__TOWARDMARKER_HPP_

#include<rclcpp/rclcpp.hpp>
#include<rclcpp_action/rclcpp_action.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include<vector>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_ros/buffer.h>
#include<tf2_ros/create_timer_ros.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<tf2/convert.h>
#include<tf2/utils.h>
#include<geometry_msgs/msg/twist.hpp>

namespace ArucoMarkerNavigation{
class TowardMarker : public rclcpp::Node{
	public:
		TowardMarker();
		~TowardMarker();
		void initPubSub();
		void goalposeCb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
		void estimatedPoseCb(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
		void rotateTmpGoalTheta(double theta);
		void initParam();
		void initTimer();
		void controllerTimerCb();
	
	private:
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_pose_sub_;
		double goal_x_, goal_y_, goal_t_;
		double est_pose_x_, est_pose_y_, est_pose_t_;
		double torelance_goal_theta_;
		double loop_rate_;
		bool on_controller_timer_;
		int control_state_;
		rclcpp::TimerBase::SharedPtr controller_timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};
}

#endif // ARUCOMARKERNAVIGATION__TOWARDMARKER_HPP_
