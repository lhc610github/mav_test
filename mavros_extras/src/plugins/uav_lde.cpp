
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include "demo_test/ekf_leader.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace mavplugin {
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class UavLdePlugin : public MavRosPlugin
{
public:
	UavLdePlugin() :
		mp_nh("~leader_topic"),
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
      uav_leader_sub = mp_nh.subscribe("leader_topic", 10, &UavLdePlugin::leader_cb, this);
      // pose_set_sub2 = mp_nh.subscribe("forma_write_topic", 10, &UavPosSetPlugin::team_set_cb, this);//
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
	demo_test::ekf_leader leader_temp;
	// ros::Subscriber mocap_pose_sub;
	// ros::Subscriber mocap_tf_sub;
	ros::Subscriber uav_leader_sub;
	// bool _formation_vaild = false;
	/* -*- low-level send -*- */
	void uav_leader_send
		(uint64_t usec, float x_d, float y_d, float z_d)
	{
		mavlink_message_t msg;
		// mavlink_msg_att_pos_mocap_pack_chan(UAS_PACK_CHAN(uas), &msg,
		// 		usec,
		// 		q,
		// 		x,
		// 		y,
		// 		z);
		ROS_INFO_ONCE("[ LDE ] SENDING !!");
		mavlink_msg_uav_leader_setpoint_pack_chan(UAS_PACK_CHAN(uas), &msg,
			usec,
			x_d,
			y_d,
			z_d);
		UAS_FCU(uas)->send_message(&msg);
	}
	
	/* -*- mid-level helpers -*- */
	void leader_cb(const demo_test::ekf_leader::ConstPtr &code)
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

		uav_leader_send(
			    code->header.stamp.toNSec() / 1000,
				code->pos[0],
				code->pos[1],
				code->pos[2]);
	}
	
	
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::UavLdePlugin, mavplugin::MavRosPlugin)
