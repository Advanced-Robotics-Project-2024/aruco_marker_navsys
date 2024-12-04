// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include<aruco_marker_navigation/toward_marker.hpp>

namespace ArucoMarkerNavigation{
	TowardMarker::TowardMarker() : Node("toward_marker")
	{
		initPubSub();
	}
	
	TowardMarker::~TowardMarker(){}

	void TowardMarker::initPubSub()
	{
		marker_infos_sub_ = this->create_subscription<aruco_marker_detector_msgs::msg::MarkerInfos>(
				"marker_infos", rclcpp::QoS(10), std::bind(&TowardMarker::markerInfoArrayCb, this, std::placeholders::_1));
	}

	void TowardMarker::markerInfoArrayCb(aruco_marker_detector_msgs::msg::MarkerInfos::ConstSharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "Received");
		// marker_infos_.resize(msg.size());
		marker_infos_  = *msg;
		// for(int i = 0; i < marker_infos_){
		// 	int id = marker_info.id;
		// 	int x = marker_info.x;
		// 	int y = marker_info.y;
		// 	RCLCPP_INFO(this->get_logger(), "ID; %d, (x, y) = (%d, %d)", id, x, y);
		// }
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
