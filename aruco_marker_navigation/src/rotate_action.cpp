// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/qos.hpp>
#include <aruco_marker_navigation/rotate_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include<tf2_ros/create_timer_ros.h>

#include <algorithm>

namespace ArucoMarkerNavigation{
	RotateAction::RotateAction() : Node("rotate_action")
	{
		initParam();
		initPubSub();
		initAction();	
	}
	
	RotateAction::~RotateAction(){}

	void RotateAction::initParam()
	{
		this->declare_parameter("max_angular_vel", 0.3);
		this->get_parameter("max_angular_vel", max_angular_vel_);
		this->declare_parameter("loop_rate", 20);
		this->get_parameter("loop_rate", loop_rate_);
	}

	void RotateAction::initAction()
	{
		rotate_action_srv_ = rclcpp_action::create_server<RotateActionMsg>(
        	this, "rotate_action",
        	std::bind(&RotateAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        	std::bind(&RotateAction::handle_cancel, this, std::placeholders::_1),
        	std::bind(&RotateAction::handle_accepted, this, std::placeholders::_1));
	}

	void RotateAction::initTf()
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

	rclcpp_action::GoalResponse RotateAction::handle_goal(
	    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
	    [[maybe_unused]] std::shared_ptr<const RotateActionMsg::Goal> goal)
	{
	        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse RotateAction::handle_cancel(
	    const std::shared_ptr<GoalHandleRotateAction> goal_handle)
	{
	    RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request to cancel goal");
	    (void)goal_handle;
	    return rclcpp_action::CancelResponse::ACCEPT;
	}  

	void RotateAction::handle_accepted(
	    const std::shared_ptr<GoalHandleRotateAction> goal_handle)
	{
	    std::thread{
	    std::bind(&RotateAction::execute, this, std::placeholders::_1),
	        goal_handle
	    }.detach();
	}

	void RotateAction::execute(
	    const std::shared_ptr<GoalHandleRotateAction> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request");
		rclcpp::Rate loop_rate(loop_rate_);
		if(!init_tf_){
			initTf();
			init_tf_ = true;
		}
		int goal_id = goal_handle->get_goal()->goal_id;
		geometry_msgs::msg::Twist msg;
		int goal_id_index;
		auto feedback = std::make_shared<RotateActionMsg::Feedback>();
		auto result = std::make_shared<RotateActionMsg::Result>();
		while(std::find(ids_.begin(), ids_.end(), goal_id) == ids_.end()){
			msg.angular.z = max_angular_vel_;
			cmd_vel_pub_->publish(msg);
			loop_rate.sleep();
			feedback->status = "Search";
		}
		auto it = std::find(ids_.begin(), ids_.end(), goal_id);
		size_t index = distance(ids_.begin(), it);
		result->success = true;
		result->id = goal_id;
		result->x = xs_[index];
		result->y = ys_[index];
		result->t = ts_[index];
		
		msg.angular.z = 0.;
		cmd_vel_pub_->publish(msg);
		goal_handle->succeed(result);
		RCLCPP_INFO(this->get_logger(), "Find ID(%d)", goal_id);
	}

	void RotateAction::initPubSub()
	{
		marker_infos_sub_ = this->create_subscription<aruco_marker_detector_msgs::msg::MarkerInfos>(
				"marker_infos", rclcpp::QoS(10), std::bind(&RotateAction::markerInfosCb, this, std::placeholders::_1));
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
	}

	void RotateAction::markerInfosCb(aruco_marker_detector_msgs::msg::MarkerInfos::ConstSharedPtr msg)
	{
		int id_size = msg->id.size();
		if(id_size != 0){
			ids_.resize(id_size);
			xs_.resize(id_size);
			ys_.resize(id_size);
			ts_.resize(id_size);
			for(int i=0; i<id_size; ++i){
				ids_[i] = msg->id[i];
				xs_[i] = msg->x[i];
				ys_[i] = msg->y[i];
				ts_[i] = msg->t[i];
				// RCLCPP_INFO(this->get_logger(), "id[%d]: %d", i, ids_[i]);
			}
		}else{
			ids_.clear();
			xs_.clear();
			ys_.clear();
			ts_.clear();
		}
	}
}

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoMarkerNavigation::RotateAction>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
