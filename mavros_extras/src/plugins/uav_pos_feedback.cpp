/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace mavplugin {
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class UavPosFeedbackPlugin : public MavRosPlugin
{
public:
	UavPosFeedbackPlugin() :
		mp_nh("~uav_posefeedback"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		// bool use_tf;
		// bool use_pose;

		uas = &uas_;

		// /** @note For VICON ROS package, subscribe to TransformStamped topic */
		// mp_nh.param("use_tf", use_tf, false);

		// /** @note For Optitrack ROS package, subscribe to PoseStamped topic */
		// mp_nh.param("use_pose", use_pose, true);
      pose_feedback_sub = mp_nh.subscribe("pose", 10, &UavPosFeedbackPlugin::pose_feedback_cb, this);
		// if (use_tf && !use_pose) {
		// 	mocap_tf_sub = mp_nh.subscribe("tf", 1, &MocapPoseEstimatePlugin::mocap_tf_cb, this);
		// }
		// else if (use_pose && !use_tf) {
		// 	mocap_pose_sub = mp_nh.subscribe("pose", 1, &MocapPoseEstimatePlugin::mocap_pose_cb, this);
		// }
		// else {
		// 	ROS_ERROR_NAMED("mocap", "Use one motion capture source.");
		// }
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;
	UAS *uas;

	// ros::Subscriber mocap_pose_sub;
	// ros::Subscriber mocap_tf_sub;
	ros::Subscriber pose_feedback_sub;

	/* -*- low-level send -*- */
	void uav_pose_send
		(float x, float y, float z)
	{
		mavlink_message_t msg;
		mavlink_msg_uav_position_feedback_pack_chan(UAS_PACK_CHAN(uas), &msg,
			x,
			y,
			z);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */
	void pose_feedback_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		// Eigen::Quaterniond q_enu;
		// float q[4];

		// tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);
		// UAS::quaternion_to_mavlink(
		// 		UAS::transform_frame_enu_ned(q_enu),
		// 		q);

		// auto position = UAS::transform_frame_enu_ned(
		// 		Eigen::Vector3d(
		// 			pose->pose.position.x,
		// 			pose->pose.position.y,
		// 			pose->pose.position.z));

		uav_pose_send(
				pose->pose.position.x,
				pose->pose.position.y,
				pose->pose.position.z);
	}

	
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::UavPosFeedbackPlugin, mavplugin::MavRosPlugin)
