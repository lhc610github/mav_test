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
#include <demo_test/demo_code.h>
#include "demo_test/pos_write_team.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace mavplugin {
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class UavPosSetPlugin : public MavRosPlugin
{
public:
	UavPosSetPlugin() :
		mp_nh("~uav_poseset"),
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
      pose_set_sub = mp_nh.subscribe("code", 10, &UavPosSetPlugin::pose_set_cb, this);
      pose_set_sub2 = mp_nh.subscribe("forma_write_topic", 10, &UavPosSetPlugin::team_set_cb, this);//
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
	demo_test::demo_code code_temp;
	demo_test::pos_write_team write_team_temp;
	// ros::Subscriber mocap_pose_sub;
	// ros::Subscriber mocap_tf_sub;
	ros::Subscriber pose_set_sub;
	ros::Subscriber pose_set_sub2;
	bool _formation_vaild = false;
	/* -*- low-level send -*- */
	void uav_pose_send
		(uint64_t usec, float x_d, float y_d, float z_d, float yaw_d, uint8_t type, uint8_t flag)
	{
		mavlink_message_t msg;
		// mavlink_msg_att_pos_mocap_pack_chan(UAS_PACK_CHAN(uas), &msg,
		// 		usec,
		// 		q,
		// 		x,
		// 		y,
		// 		z);
		ROS_INFO_ONCE("[ COMMAND ] SENDING !!");
		mavlink_msg_uav_position_setpoint_pack_chan(UAS_PACK_CHAN(uas), &msg,
			usec,
			x_d,
			y_d,
			z_d,
			yaw_d,
			type,
			flag);
		UAS_FCU(uas)->send_message(&msg);
	}
	void uav_team_send
		(float x,float y,float z)
	{
		mavlink_message_t msg;
		ROS_INFO_ONCE("[ FORMATION ] SENDING !!");
		mavlink_msg_uav_team_setpoint_pack_chan(UAS_PACK_CHAN(uas), &msg,
			x,
			y,
			z);
		UAS_FCU(uas)->send_message(&msg);
	}
	/* -*- mid-level helpers -*- */
	void pose_set_cb(const demo_test::demo_code::ConstPtr &code)
	{
		// Eigen::Quaterniond q_enu;
		// float q[4];
			//memcpy(&code_temp, &code, sizeof(code));
		// tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);
		// UAS::quaternion_to_mavlink(
		// 		UAS::transform_frame_enu_ned(q_enu),
		// 		q);

		// auto position = UAS::transform_frame_enu_ned(
		// 		Eigen::Vector3d(
		// 			pose->pose.position.x,
		// 			pose->pose.position.y,
		// 			pose->pose.position.z));
		if(code->type == 3)
		{
			_formation_vaild = true;
		}
		else
		{
			_formation_vaild = false;
		}

		uav_pose_send(
			    code->header.stamp.toNSec() / 1000,
				code->x_d,
				code->y_d,
				code->z_d,
				code->yaw_d,
				code->type,
				code->flag);
	}
	void team_set_cb(const demo_test::pos_write_team::ConstPtr &team_code)
	{
		if(ros::Time::now()-team_code->header.stamp <= ros::Duration(0.5) && _formation_vaild)
		{
			uav_team_send(
				team_code->pos[0],
				team_code->pos[1],
				team_code->pos[2]);
		}
	}
	
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::UavPosSetPlugin, mavplugin::MavRosPlugin)
