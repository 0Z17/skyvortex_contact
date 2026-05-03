#pragma once

#include <array>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <uav_contact_msgs/ContactCommand.h>
#include <uav_contact_msgs/SafetyState.h>
#include <uav_contact_msgs/TaskPhase.h>
#include <uav_contact_msgs/TrajectoryPoint.h>

namespace uav_contact_core {

class UavMotionControllerNode {
 public:
  UavMotionControllerNode();
  void Spin();

 private:
  void TrajectoryReferenceCallback(const uav_contact_msgs::TrajectoryPoint::ConstPtr& msg);
  void ContactCommandCallback(const uav_contact_msgs::ContactCommand::ConstPtr& msg);
  void TaskPhaseCallback(const uav_contact_msgs::TaskPhase::ConstPtr& msg);
  void SafetyStateCallback(const uav_contact_msgs::SafetyState::ConstPtr& msg);
  void MavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
  void LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void PublishSetpoint();

  std::array<double, 3> FuseVelocityCommand() const;
  std::array<double, 3> NormalizedNormal() const;
  std::array<double, 3> TangentialComponent(const std::array<double, 3>& v) const;
  std::array<double, 3> ClampNorm(const std::array<double, 3>& v, double limit) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber trajectory_ref_sub_;
  ros::Subscriber contact_cmd_sub_;
  ros::Subscriber task_phase_sub_;
  ros::Subscriber safety_state_sub_;
  ros::Subscriber mavros_state_sub_;
  ros::Subscriber local_pose_sub_;
  ros::Publisher setpoint_pub_;
  ros::Timer publish_timer_;

  std::array<double, 3> v_ref_;
  std::array<double, 3> p_ref_;
  std::array<double, 3> p_meas_;
  std::array<double, 3> n_;
  double v_normal_cmd_;
  uint8_t task_phase_;
  bool safety_unsafe_;
  bool emergency_retreat_required_;
  bool zero_when_not_offboard_ready_;
  bool mavros_connected_;
  bool mavros_armed_;
  bool mavros_offboard_;

  double max_velocity_;
  double max_normal_velocity_;
  double max_tangent_velocity_;
  double tangent_position_kp_;
  double publish_rate_hz_;
  double input_timeout_sec_;

  bool has_velocity_ref_;
  bool has_pose_ref_;
  bool has_pose_meas_;
  bool has_velocity_normal_cmd_;
  bool has_task_phase_;
  bool has_safety_state_;
  ros::Time last_velocity_ref_time_;
  ros::Time last_velocity_normal_cmd_time_;
  ros::Time last_task_phase_time_;
  ros::Time last_safety_state_time_;
};

}  // namespace uav_contact_core
