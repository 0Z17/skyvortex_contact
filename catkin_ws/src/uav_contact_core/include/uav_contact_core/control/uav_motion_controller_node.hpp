#pragma once

#include <array>

#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace uav_contact_core {

class UavMotionControllerNode {
 public:
  UavMotionControllerNode();
  void Spin();

 private:
  void VelocityRefCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  void VelocityNormalCallback(const std_msgs::Float64::ConstPtr& msg);
  void SurfaceNormalCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  void SafetyUnsafeCallback(const std_msgs::Bool::ConstPtr& msg);
  void PublishSetpoint();

  std::array<double, 3> FuseVelocityCommand() const;
  std::array<double, 3> ClampNorm(const std::array<double, 3>& v) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber vel_ref_sub_;
  ros::Subscriber vel_normal_sub_;
  ros::Subscriber normal_sub_;
  ros::Subscriber safety_unsafe_sub_;
  ros::Publisher setpoint_pub_;
  ros::Timer publish_timer_;

  std::array<double, 3> v_ref_;
  std::array<double, 3> n_;
  double v_normal_cmd_;
  double max_velocity_;
  double publish_rate_hz_;
  double input_timeout_sec_;
  bool safety_unsafe_;
  bool has_velocity_ref_;
  bool has_velocity_normal_cmd_;
  ros::Time last_velocity_ref_time_;
  ros::Time last_velocity_normal_cmd_time_;
};

}  // namespace uav_contact_core
