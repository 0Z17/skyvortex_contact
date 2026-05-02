#include "uav_contact_core/control/uav_motion_controller_node.hpp"

#include <algorithm>
#include <cmath>

namespace uav_contact_core {

namespace {
constexpr double kDefaultMaxVelocity = 0.25;
constexpr double kDefaultPublishRateHz = 30.0;
constexpr uint16_t kVelocityOnlyTypeMask =
    mavros_msgs::PositionTarget::IGNORE_PX |
    mavros_msgs::PositionTarget::IGNORE_PY |
    mavros_msgs::PositionTarget::IGNORE_PZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
}  // namespace

UavMotionControllerNode::UavMotionControllerNode()
    : nh_(),
      pnh_("~"),
      v_ref_{0.0, 0.0, 0.0},
      n_{1.0, 0.0, 0.0},
      v_normal_cmd_(0.0),
      max_velocity_(kDefaultMaxVelocity),
      publish_rate_hz_(kDefaultPublishRateHz) {
  pnh_.param("max_velocity", max_velocity_, max_velocity_);
  pnh_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);

  vel_ref_sub_ = nh_.subscribe("velocity_ref", 10,
                               &UavMotionControllerNode::VelocityRefCallback,
                               this);
  vel_normal_sub_ = nh_.subscribe(
      "velocity_normal_cmd", 10,
      &UavMotionControllerNode::VelocityNormalCallback, this);
  normal_sub_ = nh_.subscribe("surface_normal", 10,
                              &UavMotionControllerNode::SurfaceNormalCallback,
                              this);

  setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);

  const double timer_period = 1.0 / std::max(1.0, publish_rate_hz_);
  publish_timer_ = nh_.createTimer(
      ros::Duration(timer_period),
      [this](const ros::TimerEvent&) { PublishSetpoint(); });
}

void UavMotionControllerNode::Spin() { ros::spin(); }

void UavMotionControllerNode::VelocityRefCallback(
    const geometry_msgs::Vector3::ConstPtr& msg) {
  v_ref_[0] = msg->x;
  v_ref_[1] = msg->y;
  v_ref_[2] = msg->z;
}

void UavMotionControllerNode::VelocityNormalCallback(
    const std_msgs::Float64::ConstPtr& msg) {
  v_normal_cmd_ = msg->data;
}

void UavMotionControllerNode::SurfaceNormalCallback(
    const geometry_msgs::Vector3::ConstPtr& msg) {
  n_[0] = msg->x;
  n_[1] = msg->y;
  n_[2] = msg->z;
}

std::array<double, 3> UavMotionControllerNode::FuseVelocityCommand() const {
  return {v_ref_[0] + v_normal_cmd_ * n_[0],
          v_ref_[1] + v_normal_cmd_ * n_[1],
          v_ref_[2] + v_normal_cmd_ * n_[2]};
}

std::array<double, 3> UavMotionControllerNode::ClampNorm(
    const std::array<double, 3>& v) const {
  const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (norm <= max_velocity_ || norm <= 0.0) {
    return v;
  }

  const double scale = max_velocity_ / norm;
  return {v[0] * scale, v[1] * scale, v[2] * scale};
}

void UavMotionControllerNode::PublishSetpoint() {
  const std::array<double, 3> fused = FuseVelocityCommand();
  const std::array<double, 3> clamped = ClampNorm(fused);

  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = kVelocityOnlyTypeMask;
  msg.velocity.x = clamped[0];
  msg.velocity.y = clamped[1];
  msg.velocity.z = clamped[2];

  setpoint_pub_.publish(msg);
}

}  // namespace uav_contact_core

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_motion_controller_node");
  uav_contact_core::UavMotionControllerNode node;
  node.Spin();
  return 0;
}
