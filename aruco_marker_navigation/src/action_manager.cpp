// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/qos.hpp>
#include <aruco_marker_navigation/action_manager.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include<tf2_ros/create_timer_ros.h>
#include<tf2/convert.h>
#include<tf2/utils.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

namespace ArucoMarkerNavigation{
	ActionManager::ActionManager() : Node("approach_marker")
	{
		initParam();
		initPubSub();
		initActionServer();	
		initActionClient();	
	}
	
	ActionManager::~ActionManager(){}

	void ActionManager::initParam()
	{
		this->declare_parameter("loop_rate", 20);
		this->get_parameter("loop_rate", loop_rate_);
	}

	void ActionManager::initActionServer()
	{
		rotate_action_srv_ = rclcpp_action::create_server<ActionManagerMsg>(
        	this, "action_manager",
        	std::bind(&ActionManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        	std::bind(&ActionManager::handle_cancel, this, std::placeholders::_1),
        	std::bind(&ActionManager::handle_accepted, this, std::placeholders::_1));
	}

	void ActionManager::initActionClient()
	{
		RCLCPP_INFO(this->get_logger(), "Initialize Action Client");

		// RotateAction
		rotate_action_client_ = rclcpp_action::create_client<RotateActionMsg>(this, "rotate_action");
		rotate_action_goal_options_ = rclcpp_action::Client<RotateActionMsg>::SendGoalOptions();
		rotate_action_goal_options_.goal_response_callback = 
			[this](const std::shared_ptr<GoalHandleRotateAction> & goal_handle){
				if(!goal_handle){
					RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
				}else{
					RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
				}
			};
		rotate_action_goal_options_.result_callback = 
			[this](const GoalHandleRotateAction::WrappedResult & result){
				marker_x_ = result.result->x;
				marker_y_ = result.result->y;
				marker_t_ = result.result->t;
				wait_result_ = false;
			};
		RCLCPP_INFO(this->get_logger(), "Initialize Action Client");

		// AdjustDirection 
		adjust_direction_client_ = rclcpp_action::create_client<AdjustDirectionMsg>(this, "adjust_direction");
		adjust_direction_goal_options_ = rclcpp_action::Client<AdjustDirectionMsg>::SendGoalOptions();
		adjust_direction_goal_options_.goal_response_callback = 
			[this](const std::shared_ptr<GoalHandleAdjustDirection> & goal_handle){
				if(!goal_handle){
					RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
				}else{
					RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
				}
			};
		adjust_direction_goal_options_.result_callback = 
			[this]([[maybe_unused]] const GoalHandleAdjustDirection::WrappedResult & result){
				wait_result_ = false;
			};
		
		// AdjustPosition
		adjust_position_client_ = rclcpp_action::create_client<AdjustPositionMsg>(this, "adjust_position");
		adjust_position_goal_options_ = rclcpp_action::Client<AdjustPositionMsg>::SendGoalOptions();
		adjust_position_goal_options_.goal_response_callback = 
			[this](const std::shared_ptr<GoalHandleAdjustPosition> & goal_handle){
				if(!goal_handle){
					RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
				}else{
					RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
				}
			};
		adjust_position_goal_options_.result_callback = 
			[this]([[maybe_unused]] const GoalHandleAdjustPosition::WrappedResult & result){
				wait_result_ = false;
			};
	}

	rclcpp_action::GoalResponse ActionManager::handle_goal(
	    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
			[[maybe_unused]] std::shared_ptr<const ActionManagerMsg::Goal> goal)
	{
	        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse ActionManager::handle_cancel(
	    const std::shared_ptr<GoalHandleActionManager> goal_handle)
	{
	    RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request to cancel goal");
	    (void)goal_handle;
	    return rclcpp_action::CancelResponse::ACCEPT;
	}  

	void ActionManager::handle_accepted(
	    const std::shared_ptr<GoalHandleActionManager> goal_handle)
	{
	    std::thread{
	    std::bind(&ActionManager::execute, this, std::placeholders::_1),
	        goal_handle
	    }.detach();
	}

	void ActionManager::execute(
	    const std::shared_ptr<GoalHandleActionManager> goal_handle)
	{
		int goal_id = goal_handle->get_goal()->goal_id;
		double goal_length = goal_handle->get_goal()->goal_length;
		RCLCPP_INFO(this->get_logger(), "Recieved Navigation Request to Marker(%d)", goal_id);

		// Send Goal to RoateAction
		RotateActionMsg::Goal rotate_action_goal_msg;
		rotate_action_goal_msg.goal_id = goal_id;
		while(!this->rotate_action_client_->wait_for_action_server()){
			RCLCPP_INFO(get_logger(), "Waiting for action server...");
		}
		wait_result_ = true;
		rotate_action_client_->async_send_goal(rotate_action_goal_msg, rotate_action_goal_options_);
		waitResult();
		AdjustDirectionMsg::Goal goal_msg;

		//Send Goal to AdjustDirection
		AdjustDirectionMsg::Goal adjust_direction_goal_msg;
		adjust_direction_goal_msg.goal_rotate_direction = M_PI / 2 - marker_t_;
		while(!this->adjust_direction_client_->wait_for_action_server()){
			RCLCPP_INFO(get_logger(), "Waiting for action server...");
		}
		wait_result_ = true;
		adjust_direction_client_->async_send_goal(adjust_direction_goal_msg, adjust_direction_goal_options_);
		waitResult();

		//Send Goal to AdjustPosition
		AdjustPositionMsg::Goal adjust_position_goal_msg;
		adjust_position_goal_msg.movement_length = marker_y_;
		while(!this->adjust_position_client_->wait_for_action_server()){
			RCLCPP_INFO(get_logger(), "Waiting for action server...");
		}
		wait_result_ = true;
		adjust_position_client_->async_send_goal(adjust_position_goal_msg, adjust_position_goal_options_);
		waitResult();

		//Send Goal to AdjustDirection
		adjust_direction_goal_msg.goal_rotate_direction = -M_PI / 2;
		while(!this->adjust_direction_client_->wait_for_action_server()){
			RCLCPP_INFO(get_logger(), "Waiting for action server...");
		}
		wait_result_ = true;
		adjust_direction_client_->async_send_goal(adjust_direction_goal_msg, adjust_direction_goal_options_);
		waitResult();


		RCLCPP_INFO(this->get_logger(), "Completed Navigation to Marker(%lf)", goal_length);
	}

	void ActionManager::initPubSub()
	{
	}

	void ActionManager::waitResult()
	{
		rclcpp::Rate loop_rate(loop_rate_);
		while(wait_result_){
			loop_rate.sleep();
		}
	}
}

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoMarkerNavigation::ActionManager>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
