// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/qos.hpp>
#include <aruco_marker_navigation/approach_marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include<tf2_ros/create_timer_ros.h>
#include<tf2/convert.h>
#include<tf2/utils.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

namespace ArucoMarkerNavigation{
	ApproachMarker::ApproachMarker() : Node("approach_marker")
	{
		initParam();
		initPubSub();
		initAction();	
	}
	
	ApproachMarker::~ApproachMarker(){}

	void ApproachMarker::initParam()
	{
		this->declare_parameter("max_linear_vel", 0.2);
		this->get_parameter("max_linear_vel", max_linear_vel_);
		this->declare_parameter("max_angular_vel", 0.3);
		this->get_parameter("max_angular_vel", max_angular_vel_);
		this->declare_parameter("loop_rate", 20);
		this->get_parameter("loop_rate", loop_rate_);
		this->declare_parameter("torelance_angle_error", 0.1);
		this->get_parameter("torelance_angle_error", torelance_angle_error_);
		this->declare_parameter("torelance_length_error", 0.1);
		this->get_parameter("torelance_length_error", torelance_length_error_);
		this->declare_parameter("kp_x", 5.);
		this->get_parameter("kp_x", kp_x_);
		this->declare_parameter("kp_y", 2.);
		this->get_parameter("kp_y", kp_y_);
		this->declare_parameter("kp_t", 5.);
		this->get_parameter("kp_t", kp_t_);
		this->declare_parameter("torelance_lost_time", 5);
		this->get_parameter("torelance_lost_time", torelance_lost_time_);
		this->declare_parameter("torelance_lost_count", 5);
		this->get_parameter("torelance_lost_count", torelance_lost_count_);
	}

	void ApproachMarker::initAction()
	{
		approach_marker_srv_ = rclcpp_action::create_server<ApproachMarkerMsg>(
        	this, "approach_marker",
        	std::bind(&ApproachMarker::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        	std::bind(&ApproachMarker::handle_cancel, this, std::placeholders::_1),
        	std::bind(&ApproachMarker::handle_accepted, this, std::placeholders::_1));
	}

	rclcpp_action::GoalResponse ApproachMarker::handle_goal(
	    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
	    [[maybe_unused]] std::shared_ptr<const ApproachMarkerMsg::Goal> goal)
	{
	        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse ApproachMarker::handle_cancel(
	    const std::shared_ptr<GoalHandleApproachMarker> goal_handle)
	{
	    RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request to cancel goal");
	    (void)goal_handle;
	    return rclcpp_action::CancelResponse::ACCEPT;
	}  

	void ApproachMarker::handle_accepted(
	    const std::shared_ptr<GoalHandleApproachMarker> goal_handle)
	{
	    std::thread{
	    std::bind(&ApproachMarker::execute, this, std::placeholders::_1),
	        goal_handle
	    }.detach();
	}

	void ApproachMarker::execute(
	    const std::shared_ptr<GoalHandleApproachMarker> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Rotate Action: Received request");
		rclcpp::Rate loop_rate(loop_rate_);
		double goal_movement_length = goal_handle->get_goal()->goal_length;
		int goal_id = goal_handle->get_goal()->goal_id;
		geometry_msgs::msg::Twist msg;
		double error_x = torelance_length_error_ * 10.;
		double error_y = error_x;
		double error_t = torelance_angle_error_ * 10.;
		auto result = std::make_shared<ApproachMarkerMsg::Result>();
		bool lost_marker = false;
		int16_t lost_times = 0;
		std::chrono::system_clock::time_point lost_marker_time;
		do{
			auto it = std::find(ids_.begin(), ids_.end(), goal_id);
			if(it == ids_.end()){
				if(!lost_marker){
					lost_times ++;
					lost_marker = true;
					lost_marker_time = std::chrono::system_clock::now();
				}
				int64_t time_diff = std::chrono::duration_cast<std::chrono::seconds >(
						std::chrono::system_clock::now() - lost_marker_time).count();
				if(time_diff > torelance_lost_time_ || lost_times > torelance_lost_count_){
					result->success = false;
					goal_handle->abort(result);
					pubCmdVel(0., 0.);
					return;
				}
				RCLCPP_INFO(this->get_logger(), "NotFound: %d", goal_id);
				pubCmdVel(-max_linear_vel_/2, 0.);
			}else{
				if(lost_marker) lost_marker = false;
				size_t index = distance(ids_.begin(), it);
				error_x = xs_[index] - goal_movement_length;
				error_y = ys_[index];
				error_t = 0. - ts_[index];
				double ang_vel_y = kp_y_*error_y, ang_vel_t = kp_t_*error_t;
				//if(abs(ang_vel_y) > abs(ang_vel_t)) pubCmdVel(kp_x_*error_x, ang_vel_y);
				//else pubCmdVel(kp_x_*error_x, ang_vel_t);
				pubCmdVel(kp_x_*error_x, ang_vel_t);
				result->ex = xs_[index] - goal_movement_length;
				result->ey = ys_[index];
				result->et = ts_[index];
			}
			loop_rate.sleep();
		//}while(abs(error_x) > torelance_length_error_ || abs(error_t) > torelance_angle_error_); 
		}while(abs(error_x) > torelance_length_error_); 
		pubCmdVel(0., 0.);
		result->id = goal_id;
		if(abs(result->ex) <= torelance_length_error_){ 
			result->success = true;
		}else{
			result->success = false;
		}
		goal_handle->succeed(result);
		RCLCPP_INFO(this->get_logger(), "Completed Approach Marker(%d)", goal_id);
	}

	void ApproachMarker::pubCmdVel(double linear_vel, double ang_vel)
	{
		geometry_msgs::msg::Twist msg;
		if(linear_vel > max_linear_vel_){
			msg.linear.x = max_linear_vel_;
		}else if(linear_vel < -max_linear_vel_){
			msg.linear.x = -max_linear_vel_;
		}else{
			msg.linear.x = linear_vel;
		}
		if(ang_vel > max_angular_vel_){
			msg.angular.z = max_angular_vel_;
		}else if(ang_vel < -max_angular_vel_){
			msg.angular.z = -max_angular_vel_;
		}else{
			msg.angular.z = ang_vel;
		}
		cmd_vel_pub_->publish(msg);
	}

	void ApproachMarker::initPubSub()
	{
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10), 
				std::bind(&ApproachMarker::odomCb, this, std::placeholders::_1));
		marker_infos_sub_ = this->create_subscription<aruco_marker_detector_msgs::msg::MarkerInfos>("marker_infos",
				rclcpp::QoS(10), std::bind(&ApproachMarker::markerInfosCb, this, std::placeholders::_1));
	}

	void ApproachMarker::odomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg)
	{
		//RCLCPP_INFO(this->get_logger(), "Received Odometry");
		odom_ = *msg;
	}

	void ApproachMarker::markerInfosCb(aruco_marker_detector_msgs::msg::MarkerInfos::ConstSharedPtr msg)
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
	auto node = std::make_shared<ArucoMarkerNavigation::ApproachMarker>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
