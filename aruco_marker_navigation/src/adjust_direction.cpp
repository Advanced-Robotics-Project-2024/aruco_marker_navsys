// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/qos.hpp>
#include <aruco_marker_navigation/adjust_direction.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include<tf2_ros/create_timer_ros.h>
#include<tf2/convert.h>
#include<tf2/utils.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

namespace ArucoMarkerNavigation{
	AdjustDirection::AdjustDirection() : Node("adjust_direction")
	{
		initParam();
		initPubSub();
		initAction();	
	}
	
	AdjustDirection::~AdjustDirection(){}

	void AdjustDirection::initParam()
	{
		this->declare_parameter("max_angular_vel", 0.3);
		this->get_parameter("max_angular_vel", max_angular_vel_);
		this->declare_parameter("loop_rate", 20);
		this->get_parameter("loop_rate", loop_rate_);
		this->declare_parameter("torelance_error", 0.1);
		this->get_parameter("torelance_error", torelance_error_);
	}

	void AdjustDirection::initAction()
	{
		rotate_action_srv_ = rclcpp_action::create_server<AdjustDirectionMsg>(
        	this, "adjust_direction",
        	std::bind(&AdjustDirection::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        	std::bind(&AdjustDirection::handle_cancel, this, std::placeholders::_1),
        	std::bind(&AdjustDirection::handle_accepted, this, std::placeholders::_1));
	}

	void AdjustDirection::initTf()
	{
		tf_.reset();
		tfl_.reset();

		tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
          get_node_base_interface(), get_node_timers_interface(),
          create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
        tf_->setCreateTimerInterface(timer_interface);
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
	}

	rclcpp_action::GoalResponse AdjustDirection::handle_goal(
	    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
	    [[maybe_unused]] std::shared_ptr<const AdjustDirectionMsg::Goal> goal)
	{
	        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse AdjustDirection::handle_cancel(
	    const std::shared_ptr<GoalHandleAdjustDirection> goal_handle)
	{
	    RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request to cancel goal");
	    (void)goal_handle;
	    return rclcpp_action::CancelResponse::ACCEPT;
	}  

	void AdjustDirection::handle_accepted(
	    const std::shared_ptr<GoalHandleAdjustDirection> goal_handle)
	{
	    std::thread{
	    std::bind(&AdjustDirection::execute, this, std::placeholders::_1),
	        goal_handle
	    }.detach();
	}

	void AdjustDirection::execute(
	    const std::shared_ptr<GoalHandleAdjustDirection> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request");
		rclcpp::Rate loop_rate(loop_rate_);
		if(!init_tf_){
			initTf();
			init_tf_ = true;
		}
		double goal_rotate_direction = goal_handle->get_goal()->goal_rotate_direction;
		geometry_msgs::msg::Twist msg;
		nav_msgs::msg::Odometry last_odom;
		double goal_robot_direction = tf2::getYaw(odom_.pose.pose.orientation) + goal_rotate_direction;
		while(goal_robot_direction > M_PI) goal_robot_direction -= 2*M_PI;
		while(goal_robot_direction < -M_PI) goal_robot_direction += 2*M_PI;
		RCLCPP_INFO(this->get_logger(), "Goal Robot Direction: %lf", goal_robot_direction);
		while(abs(goal_robot_direction - tf2::getYaw(odom_.pose.pose.orientation)) > torelance_error_){
			if(goal_robot_direction - tf2::getYaw(odom_.pose.pose.orientation) > 0){
				msg.angular.z = max_angular_vel_;
			}else{
				msg.angular.z = -max_angular_vel_;
			}
			cmd_vel_pub_->publish(msg);
			loop_rate.sleep();
		}
		msg.angular.z = 0.;
		cmd_vel_pub_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Completed Rotate For Adjust(%lf)", goal_rotate_direction);
	}

	void AdjustDirection::initPubSub()
	{
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10), 
				std::bind(&AdjustDirection::odomCb, this, std::placeholders::_1));
	}

	void AdjustDirection::odomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg)
	{
		//RCLCPP_INFO(this->get_logger(), "Received Odometry");
		odom_ = *msg;
	}
}

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoMarkerNavigation::AdjustDirection>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
