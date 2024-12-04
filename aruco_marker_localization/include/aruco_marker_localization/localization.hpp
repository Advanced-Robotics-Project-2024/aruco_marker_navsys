// SPDX-FileCopyrightText: 2024 MakotoYoshigoe 
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERLOCALIZATION__LOCALIZATION_HPP_
#define ARUCOMARKERLOCALIZATION__LOCALIZATION_HPP_

#include<rclcpp/rclcpp.hpp>
#include<aruco_marker_detector_msgs/msg/marker_infos.hpp>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_ros/static_transform_broadcaster.h>
#include<map>
#include<vector>
#include<rclcpp/time.hpp>
#include<geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <aruco_marker_localization/pose.hpp>
#include<string>

namespace ArucoMarkerLocalization{
class Localization : public rclcpp::Node{
	// private:
	// struct Pose{
	// 	double x;
	// 	double y;
	// 	double t;
	// 	Pose operator=(const Pose & p)
	// 	{
	// 		if(this != &p){
	// 			x = p.x;
	// 			y = p.y;
	// 			t = p.t;
	// 		}
	// 		return *this;
	// 	}
	// 	Pose operator-(const Pose & p)
	// 	{
	// 		Pose ans{x-p.x, y-p.y, t-p.t};
	// 		ans.angleNomalization();
	// 		return ans;
	// 	}
	// 	void angleNomalization()
	// 	{
	// 		while(t>M_PI) t -= 2*M_PI;
	// 		while(t<M_PI) t += 2*M_PI;
	// 	}
	// };

    public:
    Localization();
    ~Localization();
	void initPubSub();
	void markerInfosCb(aruco_marker_detector_msgs::msg::MarkerInfos::ConstSharedPtr msg);
	void initMap();
	void pubMapTfStatic();
	void mainLoop();
	void initParam();
	int getLoopRate();
	void initializeFilter();
	void pubOdomFrame();
	void pubEstimatedPose();
	bool updateOdomInfo();
	void initTf();
	bool motionUpdate();
	void initialposeCb(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
	bool getCameraPose(Pose & rtn_camera_pose);
	bool sensorUpdate();

	private:
	rclcpp::Subscription<aruco_marker_detector_msgs::msg::MarkerInfos>::SharedPtr marker_infos_sub_;
	int landmark_num_;
	std::map<int, std::vector<double>> map_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_br_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	int loop_rate_;
	bool init_filter_;
	bool receive_initialpose_;
	Pose ep_;
	rclcpp::Time camera_time_stamp_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr epose_pub_;
	bool init_ar_info_;
	Pose last_odom_, prev_odom_;
	bool init_odom_;
	std::shared_ptr<tf2_ros::Buffer> tf_;
	bool init_tf_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
	std::string odom_frame_id_;
	std::string base_footprint_id_;
	std::string camera_frame_id_;
	aruco_marker_detector_msgs::msg::MarkerInfos marker_infos_;
};  
}

#endif
