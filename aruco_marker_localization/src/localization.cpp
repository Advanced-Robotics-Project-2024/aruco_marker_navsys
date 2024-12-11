// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include<aruco_marker_localization/localization.hpp>
#include<aruco_marker_detector_msgs/msg/marker_infos.hpp>
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include<tf2_ros/static_transform_broadcaster.h>
#include<string>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_ros/buffer.h>
#include<tf2_ros/create_timer_ros.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<tf2/convert.h>
#include<tf2/utils.h>
#include<cmath>

namespace ArucoMarkerLocalization{
	Localization::Localization() : Node("aruco_marker_localization")
	{
		initPubSub();
		initMap();
		initParam();
	}

	Localization::~Localization(){}

	void Localization::initPubSub()
	{
		marker_infos_sub_ = this->create_subscription<aruco_marker_detector_msgs::msg::MarkerInfos>(
				"marker_infos", rclcpp::QoS(10), 
				std::bind(&Localization::markerInfosCb, this, std::placeholders::_1));
		epose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
				"estimated_pose", 10);
		initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
				"initialpose", rclcpp::QoS(10),
				std::bind(&Localization::initialposeCb, this, std::placeholders::_1));
	}

	void Localization::initParam()
	{
		this->declare_parameter("loop_rate", 20);
		this->get_parameter("loop_rate", loop_rate_);
		this->declare_parameter("odom_frame_id", "odom");
		this->get_parameter("odom_frame_id", odom_frame_id_);
		this->declare_parameter("base_footprint_id", "base_footprint");
		this->get_parameter("base_footprint_id", base_footprint_id_);
		this->declare_parameter("camera_frame_id", "camera_rgb_frame");
		this->get_parameter("camera_frame_id", camera_frame_id_);
	}

	void Localization::markerInfosCb(aruco_marker_detector_msgs::msg::MarkerInfos::ConstSharedPtr msg)
	{
		if(!init_ar_info_){
			init_ar_info_ = true;
		}
		marker_infos_ = *msg;
		//RCLCPP_INFO(this->get_logger(), "Received");
	}
	
	void Localization::initialposeCb(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
	{
		ep_.x_ = msg->pose.pose.position.x;
		ep_.y_ = msg->pose.pose.position.y;
		ep_.t_ = tf2::getYaw(msg->pose.pose.orientation);
	}

	void Localization::initMap()
	{
		this->declare_parameter("landmark_num", 0);
		this->get_parameter("landmark_num", landmark_num_);
		std::vector<int64_t> ids(landmark_num_, 0);
		std::vector<double> xs(landmark_num_, 0.), ys(landmark_num_, 0.);
		this->declare_parameter("landmarks.ids", ids);
		this->declare_parameter("landmarks.xs", xs); 
		this->declare_parameter("landmarks.ys", ys);
		this->get_parameter("landmarks.ids", ids);
		this->get_parameter("landmarks.xs", xs);
		this->get_parameter("landmarks.ys", ys);
		for(int i=0; i<landmark_num_; ++i){
			std::vector<double> p = {xs[i], ys[i]};
		 	map_.insert(std::make_pair(ids[i], p));
		}
		pubMapTfStatic();
	}

	void Localization::pubMapTfStatic()
	{
		tf_static_br_.reset();
		tf_static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
		geometry_msgs::msg::TransformStamped tf;
		tf.header.frame_id = "map";
		for(auto it=map_.begin(); it!=map_.end(); ++it){
			RCLCPP_DEBUG(this->get_logger(), "key: %d, (x, y): (%lf, %lf)", it->first, it->second[0], it->second[1]);
			tf.header.stamp = this->get_clock()->now();
			tf.child_frame_id = "marker_id"+std::to_string(it->first);
			tf.transform.translation.x = it->second[0];
			tf.transform.translation.y = it->second[1];
			tf_static_br_->sendTransform(tf);
		}
	}

	void Localization::initializeFilter()
	{
		this->declare_parameter("initialpose.x", 0.);
		this->get_parameter("initialpose.x", ep_.x_);
		this->declare_parameter("initialpose.y", 0.);
		this->get_parameter("initialpose.y", ep_.y_);
		this->declare_parameter("initialpose.t", 0.);
		this->get_parameter("initialpose.t", ep_.t_);
	}

	void Localization::initTf()
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

	bool Localization::updateOdomInfo()
	{
		geometry_msgs::msg::PoseStamped ident;
		ident.header.frame_id = base_footprint_id_;
		ident.header.stamp = rclcpp::Time(0);
		tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
		geometry_msgs::msg::PoseStamped odom_pose;
		try {
			this->tf_->transform(ident, odom_pose, "odom");
		} catch (tf2::TransformException & e) {
			RCLCPP_WARN(
			  get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
			return false;
		}
		
		if(!init_odom_){
			prev_odom_.x_ = odom_pose.pose.position.x;	
			prev_odom_.y_ = odom_pose.pose.position.y;	
			prev_odom_.t_ = tf2::getYaw(odom_pose.pose.orientation);
			init_odom_ = true;
			return false;
		}else{
			last_odom_.x_ = odom_pose.pose.position.x;	
			last_odom_.y_ = odom_pose.pose.position.y;	
			last_odom_.t_ = tf2::getYaw(odom_pose.pose.orientation);
			return true;
		}
	}

	bool Localization::motionUpdate()
	{
		Pose d = last_odom_ - prev_odom_;
		d.angleNomalization();
		double delta_trans = sqrt(d.x_*d.x_ + d.y_*d.y_);
		double delta_rot1 = atan2(d.y_, d.x_) - prev_odom_.t_;
		double delta_rot2 = d.t_ - delta_rot1;
		if(delta_trans > 0.001){
			// RCLCPP_INFO(this->get_logger(), "dt: %lf", delta_trans);
			//RCLCPP_INFO(this->get_logger(), "cos: %lf", cos(last_odom_.t + delta_rot1));
			ep_.x_ = ep_.x_ + delta_trans * cos(last_odom_.t_ + delta_rot1);
			ep_.y_ = ep_.y_ + delta_trans * sin(last_odom_.t_ + delta_rot1);
		}
		//RCLCPP_INFO(this->get_logger(), "dt: %lf", ep_.x);
		ep_.t_ += d.t_;
		ep_.angleNomalization();
		prev_odom_ = last_odom_;

		return true;
	}

	bool Localization::getCameraPose(Pose & rtn_camera_pose)
	{
		geometry_msgs::msg::PoseStamped ident;
        ident.header.frame_id = camera_frame_id_;
        ident.header.stamp = rclcpp::Time(0);
        tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
        geometry_msgs::msg::PoseStamped camera_pose;
        try {
            this->tf_->transform(ident, camera_pose, base_footprint_id_);
        } catch (tf2::TransformException & e) {
            RCLCPP_WARN(
              get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }
		rtn_camera_pose.x_ = camera_pose.pose.position.x;
		rtn_camera_pose.y_ = camera_pose.pose.position.y;
		rtn_camera_pose.t_ = tf2::getYaw(camera_pose.pose.orientation);
		return true;
	}

	bool Localization::sensorUpdate()
	{
		Pose camera_pose;
		getCameraPose(camera_pose);
		//RCLCPP_INFO(this->get_logger(), "(off.x, off.y, off.t) = (%lf, %lf, %lf)", camera_pose.x_, camera_pose.y_, camera_pose.t_);
		int marker_n = marker_infos_.id.size();
		if(marker_n < 0) return false;
		double sum_x = 0., sum_y = 0, sum_t = 0.;
		for(int i=0; i<marker_n; ++i){
			//RCLCPP_INFO(this->get_logger(), "RAW: id: %d, (x, y, t) = (%lf, %lf, %lf)", marker_infos_.id[i], marker_infos_.x[i], marker_infos_.y[i], marker_infos_.t[i]);
			int id = marker_infos_.id[i];
			Pose truth(map_.at(id)[0], map_.at(id)[1], 0.);
			//RCLCPP_INFO(this->get_logger(), "TRUTH: (x, y)=(%lf, %lf)", truth.x_, truth.y_);
			Pose p(marker_infos_.x[i], marker_infos_.y[i], marker_infos_.t[i]);
			Pose d_base_to_camera = p - camera_pose;
			//RCLCPP_INFO(this->get_logger(), "BASE TO CAMERA: (x, y)=(%lf, %lf)", d_base_to_camera.x_, d_base_to_camera.y_);
			// d_base_to_camera.angleNomalization();
			Pose diff = (truth-d_base_to_camera);
			sum_x += diff.x_;
			sum_y += diff.y_;
			sum_t += diff.t_;
			//RCLCPP_INFO(this->get_logger(), "id: %d, (x, y, t) = (%lf, %lf, %lf)", marker_infos_.id[i], diff.x_, diff.y_, diff.t_);
		}
		//Pose mean_pose(mean_pose.x_ / static_cast<double>(marker_n), mean_pose.y_ / static_cast<double>(marker_n), mean_pose.t_ / static_cast<double>(marker_n));
		Pose mean_pose(sum_x, sum_y, sum_t);
		mean_pose.angleNomalization();
		//RCLCPP_INFO(this->get_logger(), "MEAN: (x, y, t) = (%lf, %lf, %lf)", mean_pose.x_, mean_pose.y_, mean_pose.t_);
		ep_ = mean_pose;
		return true;
	}
		
	void Localization::mainLoop()
	{
		if(!init_filter_){
			initializeFilter();
			init_filter_ = true;
		}
		if(!init_tf_){
			initTf();
			init_tf_ = true;
		}
		if(init_filter_ && init_tf_){
			bool res_motion_update;
			if(updateOdomInfo()){
				res_motion_update = motionUpdate();
				// sensorUpdate();
			}
			pubEstimatedPose();
		}
	}

	void Localization::pubEstimatedPose()
	{
		geometry_msgs::msg::PoseWithCovarianceStamped msg;
		msg.header.stamp = now();
		msg.header.frame_id = "map";
		msg.pose.pose.position.x = ep_.x_;
		msg.pose.pose.position.y = ep_.y_;
		tf2::Quaternion q;
		q.setRPY(0, 0, ep_.t_);
		msg.pose.pose.orientation.x = q[0];
		msg.pose.pose.orientation.y = q[1];
		msg.pose.pose.orientation.z = q[2];
		msg.pose.pose.orientation.w = q[3];
		epose_pub_->publish(msg);
	}

	void Localization::pubOdomFrame()
	{
		//geometry_msgs::msg::PoseStamped odom_to_map;
		//try {
		//	tf2::Quaternion q;
		//	q.setRPY(0, 0, ep_.t);
		//	tf2::Transform tmp_tf(q, tf2::Vector3(ep_.x, ep_.y, 0.0));

		//	geometry_msgs::msg::PoseStamped tmp_tf_stamped;
		//	tmp_tf_stamped.header.frame_id = "base_footprint";
	}

	int Localization::getLoopRate()
	{
		return loop_rate_;
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoMarkerLocalization::Localization>();
	rclcpp::Rate loop_rate(node->getLoopRate());
	while(rclcpp::ok()){
		node->mainLoop();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}
