// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/qos.hpp>
#include <aruco_marker_navigation/adjust_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include<tf2_ros/create_timer_ros.h>
#include<tf2/convert.h>
#include<tf2/utils.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

namespace ArucoMarkerNavigation{
	AdjustPosition::AdjustPosition() : Node("adjust_position")
	{
		initParam();
		initPubSub();
		initAction();	
	}
	
	AdjustPosition::~AdjustPosition(){}

	void AdjustPosition::initParam()
	{
		this->declare_parameter("max_linear_vel", 0.22);
		this->get_parameter("max_linear_vel", max_linear_vel_);
		this->declare_parameter("loop_rate", 20);
		this->get_parameter("loop_rate", loop_rate_);
		this->declare_parameter("torelance_error", 0.05);
		this->get_parameter("torelance_error", torelance_error_);
	}

	void AdjustPosition::initAction()
	{
		adjust_position_srv_ = rclcpp_action::create_server<AdjustPositionMsg>(
        	this, "adjust_position",
        	std::bind(&AdjustPosition::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        	std::bind(&AdjustPosition::handle_cancel, this, std::placeholders::_1),
        	std::bind(&AdjustPosition::handle_accepted, this, std::placeholders::_1));
	}

	void AdjustPosition::initTf()
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

	rclcpp_action::GoalResponse AdjustPosition::handle_goal(
	    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
	    [[maybe_unused]] std::shared_ptr<const AdjustPositionMsg::Goal> goal)
	{
	        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse AdjustPosition::handle_cancel(
	    const std::shared_ptr<GoalHandleAdjustPosition> goal_handle)
	{
	    RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request to cancel goal");
	    (void)goal_handle;
	    return rclcpp_action::CancelResponse::ACCEPT;
	}  

	void AdjustPosition::handle_accepted(
	    const std::shared_ptr<GoalHandleAdjustPosition> goal_handle)
	{
	    std::thread{
	    std::bind(&AdjustPosition::execute, this, std::placeholders::_1),
	        goal_handle
	    }.detach();
	}

	void AdjustPosition::execute(
	    const std::shared_ptr<GoalHandleAdjustPosition> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request");
		rclcpp::Rate loop_rate(loop_rate_);
		if(!init_tf_){
			initTf();
			init_tf_ = true;
			RCLCPP_INFO(this->get_logger(), "Initialized TF");
		}
		double goal_movement_length = goal_handle->get_goal()->movement_length;
		geometry_msgs::msg::Twist msg;
		nav_msgs::msg::Odometry last_odom;
		double movement_length = 0.;
		// goal_robot_position = tf2::getYaw(odom_.pose.pose.orientation) + goal_rotate_direction;
		bool init_odom = false;
		//while(goal_robot_direction > M_PI) goal_robot_direction -= 2*M_PI;
		//while(goal_robot_direction < -M_PI) goal_robot_direction += 2*M_PI;
		while(abs(goal_movement_length - movement_length) > torelance_error_){
			if(!init_odom){
				last_odom = odom_;
				init_odom = true;
			}else{
				double dx = odom_.pose.pose.position.x - last_odom.pose.pose.position.x;
				double dy = odom_.pose.pose.position.y - last_odom.pose.pose.position.y;
				RCLCPP_INFO(this->get_logger(), "Movement Length: %lf", movement_length);
				if(goal_movement_length - movement_length > 0.){
					msg.linear.x = -max_linear_vel_;
					RCLCPP_INFO(this->get_logger(), "Foward");
					movement_length += hypot(dy, dx);
				}else{
					msg.linear.x = max_linear_vel_;
					RCLCPP_INFO(this->get_logger(), "Back");
					movement_length -= hypot(dy, dx);
				}
				cmd_vel_pub_->publish(msg);
				last_odom = odom_;
				RCLCPP_INFO(this->get_logger(), "Goal Movement Length: %lf", goal_movement_length);
			}
			loop_rate.sleep();
		}
		msg.linear.x = 0.;
		cmd_vel_pub_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Completed Movement For Adjust(%lf)", goal_movement_length);
	}

	void AdjustPosition::initPubSub()
	{
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10), 
				std::bind(&AdjustPosition::odomCb, this, std::placeholders::_1));
	}

	void AdjustPosition::odomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg)
	{
		//RCLCPP_INFO(this->get_logger(), "Received Odometry");
		odom_ = *msg;
	}
}

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoMarkerNavigation::AdjustPosition>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
