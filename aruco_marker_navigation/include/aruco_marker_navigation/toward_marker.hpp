// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERNAVIGATION__TOWARDMARKER_HPP_
#define ARUCOMARKERNAVIGATION__TOWARDMARKER_HPP_

#include<rclcpp/rclcpp.hpp>
#include<rclcpp_action/rclcpp_action.hpp>
#include<aruco_marker_detector_msgs/msg/marker_infos.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<vector>
//#include<

namespace ArucoMarkerNavigation{
class TowardMarker : public rclcpp::Node{
	public:
		TowardMarker();
		~TowardMarker();
		void initPubSub();
		void 
	
	private:
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};
}

#endif // ARUCOMARKERNAVIGATION__TOWARDMARKER_HPP_
