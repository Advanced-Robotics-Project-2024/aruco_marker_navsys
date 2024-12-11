// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include<aruco_marker_navigation/toward_marker.hpp>
#include<cmath>
#include<thread>
#include<chrono>

using namespace std::chrono_literals;

namespace ArucoMarkerNavigation{
	TowardMarker::TowardMarker() : Node("toward_marker")
	{
		initPubSub();
		initParam();
		initTimer();
	}
	
	TowardMarker::~TowardMarker(){}

	void TowardMarker::initPubSub()
	{
		goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"goal_pose", rclcpp::QoS(10), std::bind(&TowardMarker::goalposeCb, this, std::placeholders::_1));
		estimated_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
				"estimated_pose", rclcpp::QoS(10), std::bind(&TowardMarker::estimatedPoseCb, this, std::placeholders::_1));
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
				"cmd_vel", 10);
	}

	void TowardMarker::initParam()
	{
		this->declare_parameter("torelance_goal_theta", 0.1);
		this->get_parameter("torelance_goal_theta", torelance_goal_theta_);
		this->declare_parameter("loop_rate", 20.);
		this->get_parameter("loop_rate", loop_rate_);
	}

	void TowardMarker::initTimer()
	{
		controller_timer_ = this->create_wall_timer(
				50ms, std::bind(&TowardMarker::controllerTimerCb, this));
	}

	void TowardMarker::estimatedPoseCb(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
	{
		est_pose_x_ = msg->pose.pose.position.x;
		est_pose_y_ = msg->pose.pose.position.y;
		est_pose_t_ = tf2::getYaw(msg->pose.pose.orientation);
		while(est_pose_t_ > M_PI) est_pose_t_ -= 2*M_PI;
		while(est_pose_t_ < -M_PI) est_pose_t_ += 2*M_PI;
		// RCLCPP_INFO(this->get_logger(), "x, y, t=%lf, %lf, %lf", est_pose_x_, est_pose_y_, est_pose_t_);
	}

	void TowardMarker::goalposeCb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
	{
		goal_x_ = msg->pose.position.x;
		goal_y_ = msg->pose.position.y;
		goal_t_ = tf2::getYaw(msg->pose.orientation);
		on_controller_timer_ = true;
		control_state_ = 1;
		
		double tmp_goal_theta = M_PI - atan2(goal_y_ - est_pose_y_, goal_x_ - est_pose_x_);

		//if(tmp_goal_theta - est_pose_t_ > )
	}

	void TowardMarker::controllerTimerCb()
	{
		if(on_controller_timer_){
			geometry_msgs::msg::Twist msg;
			double tmp_goal_theta = M_PI - atan2(goal_x_ - est_pose_x_, goal_y_ - est_pose_y_);
			switch (control_state_){
				case 1:
					RCLCPP_INFO(this->get_logger(), "Diff: %lf", abs(tmp_goal_theta - est_pose_t_));
					if(abs(tmp_goal_theta - est_pose_t_) > torelance_goal_theta_){
						msg.angular.z = 0.2;
						cmd_vel_pub_->publish(msg);	
					}else{
						msg.angular.z = 0.0;
						cmd_vel_pub_->publish(msg);
						control_state_ = 2;
					}
					break;
				case 2:
					RCLCPP_INFO(this->get_logger(), "linear");
					control_state_ = 3;
					break;
				case 3:
					RCLCPP_INFO(this->get_logger(), "last rotate");
					control_state_ = 0;
					break;
				default:
					break;
			}
		}else{
			RCLCPP_INFO(this->get_logger(), "waiting");
		}
	}

	void TowardMarker::rotateTmpGoalTheta(double theta)
	{
		rclcpp::Rate loop_rate(loop_rate_);
		while(abs(est_pose_t_ - theta) > torelance_goal_theta_){
			RCLCPP_INFO(this->get_logger(), "ROTATE: goal theta: %lf, est theta: %lf", theta, est_pose_t_);
			loop_rate.sleep();
		}
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoMarkerNavigation::TowardMarker>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
