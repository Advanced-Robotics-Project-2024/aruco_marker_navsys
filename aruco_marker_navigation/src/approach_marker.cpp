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
		this->declare_parameter("max_linear_vel", 0.22);
		this->get_parameter("max_linear_vel", max_linear_vel_);
		this->declare_parameter("max_angular_vel", 0.3);
		this->get_parameter("max_angular_vel", max_angular_vel_);
		this->declare_parameter("loop_rate", 20);
		this->get_parameter("loop_rate", loop_rate_);
		this->declare_parameter("torelance_angle_error", 0.1);
		this->get_parameter("torelance_angle_error", torelance_angle_error_);
		this->declare_parameter("torelance_length_error", 0.1);
		this->get_parameter("torelance_length_error", torelance_length_error_);
	}

	void ApproachMarker::initAction()
	{
		rotate_action_srv_ = rclcpp_action::create_server<ApproachMarkerMsg>(
        	this, "approach_marker",
        	std::bind(&ApproachMarker::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        	std::bind(&ApproachMarker::handle_cancel, this, std::placeholders::_1),
        	std::bind(&ApproachMarker::handle_accepted, this, std::placeholders::_1));
	}

	void ApproachMarker::initTf()
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
		if(!init_tf_){
			initTf();
			init_tf_ = true;
			RCLCPP_INFO(this->get_logger(), "Initialized TF");
		}
		double goal_movement_length = goal_handle->get_goal()->goal_length;
		double goal_id = goal_handle->get_goal()->goal_id;
		geometry_msgs::msg::Twist msg;
		nav_msgs::msg::Odometry last_odom;
		bool init_odom = false;
		double current_distance;
		double current_diff_direction;
		//while(goal_robot_direction > M_PI) goal_robot_direction -= 2*M_PI;
		//while(goal_robot_direction < -M_PI) goal_robot_direction += 2*M_PI;
		do{
			auto it = std::find(ids_.begin(), ids_.end(), goal_id);
			size_t index = distance(ids_.begin(), it);
			current_distance = xs_[index];
			if(current_distance - goal_movement_length > 0.){
				msg.linear.x = max_linear_vel_;
			}else{
				msg.linear.x = -max_linear_vel_;
			}
			current_diff_direction = ts_[index];
			if(current_diff_direction > 0.){
				msg.angular.z = -max_angular_vel_;
			}else{
				msg.angular.x = max_angular_vel_;
			}
			cmd_vel_pub_->publish(msg);
			loop_rate.sleep();
		}while(current_distance - goal_movement_length > torelance_length_error_ || current_diff_direction > torelance_angle_error_); 
	//		if(!init_odom){
	//			last_odom = odom_;
	//			init_odom = true;
	//		}else{
	//			double dx = odom_.pose.pose.position.x - last_odom.pose.pose.position.x;
	//			double dy = odom_.pose.pose.position.y - last_odom.pose.pose.position.y;
	//			RCLCPP_INFO(this->get_logger(), "Movement Length: %lf", movement_length);
	//			if(goal_movement_length - movement_length > 0.){
	//				msg.linear.x = max_linear_vel_;
	//				RCLCPP_INFO(this->get_logger(), "Foward");
	//				movement_length += hypot(dy, dx);
	//			}else{
	//				msg.linear.x = -max_linear_vel_;
	//				RCLCPP_INFO(this->get_logger(), "Back");
	//				movement_length -= hypot(dy, dx);
	//			}
	//			last_odom = odom_;
	//			RCLCPP_INFO(this->get_logger(), "Goal Movement Length: %lf", goal_movement_length);
	//		}
	//		loop_rate.sleep();
	//	}
		msg.linear.x = 0.;
		msg.angular.z = 0.;
		cmd_vel_pub_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Completed Approach Marker(%lf)", goal_id);
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
