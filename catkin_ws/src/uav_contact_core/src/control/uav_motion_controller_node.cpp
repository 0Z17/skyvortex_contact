#include "uav_contact_core/control/uav_motion_controller_node.hpp"

#include <algorithm>
#include <cmath>

namespace uav_contact_core {

namespace {
constexpr double kDefaultMaxVelocity = 0.25;
constexpr double kDefaultMaxNormalVelocity = 0.08;
constexpr double kDefaultMaxTangentVelocity = 0.25;
constexpr double kDefaultPublishRateHz = 50.0;
constexpr uint16_t kVelocityOnlyTypeMask =
    mavros_msgs::PositionTarget::IGNORE_PX |
    mavros_msgs::PositionTarget::IGNORE_PY |
    mavros_msgs::PositionTarget::IGNORE_PZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
constexpr uint16_t kPositionOnlyTypeMask =
    mavros_msgs::PositionTarget::IGNORE_VX |
    mavros_msgs::PositionTarget::IGNORE_VY |
    mavros_msgs::PositionTarget::IGNORE_VZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
}  // namespace

UavMotionControllerNode::UavMotionControllerNode()
    : nh_(),
      pnh_("~"),
      v_ref_{0.0, 0.0, 0.0},
      p_ref_{0.0, 0.0, 0.0},
      p_meas_{0.0, 0.0, 0.0},
      n_{1.0, 0.0, 0.0},
      retreat_position_target_{0.0, 0.0, 0.0},
      psi_ref_(0.0),
      vpsi_ref_(0.0),
      retreat_yaw_ref_(0.0),
      v_normal_cmd_(0.0),
      task_phase_(uav_contact_msgs::TaskPhase::IDLE),
      safety_unsafe_(false),
      emergency_retreat_required_(false),
      zero_when_not_offboard_ready_(true),
      mavros_connected_(false),
      mavros_armed_(false),
      mavros_offboard_(false),
      max_velocity_(kDefaultMaxVelocity),
      max_normal_velocity_(kDefaultMaxNormalVelocity),
      max_tangent_velocity_(kDefaultMaxTangentVelocity),
      tangent_position_kp_(1.0),
      retreat_distance_m_(0.3),
      retreat_start_max_deviation_m_(0.3),
      retreat_max_position_deviation_m_(0.3),
      publish_rate_hz_(kDefaultPublishRateHz),
      input_timeout_sec_(0.5),
      approach_use_position_mode_(true),
      approach_max_position_deviation_(0.3),
      has_velocity_ref_(false),
      has_pose_ref_(false),
      has_pose_meas_(false),
      has_retreat_position_target_(false),
      has_velocity_normal_cmd_(false),
      has_task_phase_(false),
      has_safety_state_(false),
      last_velocity_ref_time_(0.0),
      last_velocity_normal_cmd_time_(0.0),
      last_task_phase_time_(0.0),
      last_safety_state_time_(0.0) {
  pnh_.param("max_velocity", max_velocity_, max_velocity_);
  pnh_.param("max_normal_velocity", max_normal_velocity_, max_normal_velocity_);
  pnh_.param("max_tangent_velocity", max_tangent_velocity_, max_tangent_velocity_);
  pnh_.param("tangent_position_kp", tangent_position_kp_, tangent_position_kp_);
  nh_.param("/trajectory_server/retreat_distance_m", retreat_distance_m_,
            retreat_distance_m_);
  pnh_.param("retreat_start_max_deviation_m",
             retreat_start_max_deviation_m_,
             retreat_start_max_deviation_m_);
  pnh_.param("retreat_max_position_deviation_m",
             retreat_max_position_deviation_m_,
             retreat_max_position_deviation_m_);
  pnh_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
  pnh_.param("input_timeout_sec", input_timeout_sec_, input_timeout_sec_);
  pnh_.param("zero_when_not_offboard_ready", zero_when_not_offboard_ready_,
             zero_when_not_offboard_ready_);
  pnh_.param("approach_use_position_mode", approach_use_position_mode_,
             approach_use_position_mode_);
  pnh_.param("approach_max_position_deviation",
             approach_max_position_deviation_,
             approach_max_position_deviation_);

  trajectory_ref_sub_ = nh_.subscribe(
      "/uav_contact/trajectory/reference", 10,
      &UavMotionControllerNode::TrajectoryReferenceCallback, this);
  contact_cmd_sub_ = nh_.subscribe(
      "/uav_contact/contact/normal_velocity_cmd", 10,
      &UavMotionControllerNode::ContactCommandCallback, this);
  task_phase_sub_ = nh_.subscribe(
      "/uav_contact/task/phase", 10,
      &UavMotionControllerNode::TaskPhaseCallback, this);
  safety_state_sub_ = nh_.subscribe(
      "/uav_contact/safety/state", 10,
      &UavMotionControllerNode::SafetyStateCallback, this);
  mavros_state_sub_ = nh_.subscribe(
      "/mavros/state", 10, &UavMotionControllerNode::MavrosStateCallback, this);
  local_pose_sub_ = nh_.subscribe(
      "/mavros/local_position/pose", 10,
      &UavMotionControllerNode::LocalPoseCallback, this);

  setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);

  const double timer_period = 1.0 / std::max(1.0, publish_rate_hz_);
  publish_timer_ = nh_.createTimer(
      ros::Duration(timer_period),
      [this](const ros::TimerEvent&) { PublishSetpoint(); });
}

void UavMotionControllerNode::Spin() { ros::spin(); }

void UavMotionControllerNode::TrajectoryReferenceCallback(
    const uav_contact_msgs::TrajectoryPoint::ConstPtr& msg) {
  v_ref_[0] = msg->vx;
  v_ref_[1] = msg->vy;
  v_ref_[2] = msg->vz;
  n_[0] = msg->nx;
  n_[1] = msg->ny;
  n_[2] = msg->nz;
  p_ref_[0] = msg->x;
  p_ref_[1] = msg->y;
  p_ref_[2] = msg->z;
  psi_ref_ = msg->psi;
  vpsi_ref_ = msg->vpsi;
  has_velocity_ref_ = true;
  has_pose_ref_ = true;
  last_velocity_ref_time_ = ros::Time::now();
}

void UavMotionControllerNode::ContactCommandCallback(
    const uav_contact_msgs::ContactCommand::ConstPtr& msg) {
  v_normal_cmd_ = msg->normal_velocity;
  has_velocity_normal_cmd_ = true;
  last_velocity_normal_cmd_time_ = ros::Time::now();
}

void UavMotionControllerNode::TaskPhaseCallback(
    const uav_contact_msgs::TaskPhase::ConstPtr& msg) {
  if (msg->phase == uav_contact_msgs::TaskPhase::RETREAT &&
      task_phase_ != uav_contact_msgs::TaskPhase::RETREAT) {
    CaptureRetreatPositionTarget();
  } else if (msg->phase != uav_contact_msgs::TaskPhase::RETREAT) {
    has_retreat_position_target_ = false;
  }

  task_phase_ = msg->phase;
  has_task_phase_ = true;
  last_task_phase_time_ = ros::Time::now();
}

void UavMotionControllerNode::SafetyStateCallback(
    const uav_contact_msgs::SafetyState::ConstPtr& msg) {
  safety_unsafe_ = !msg->safe;
  emergency_retreat_required_ = msg->require_emergency_retreat;
  has_safety_state_ = true;
  last_safety_state_time_ = ros::Time::now();
}

void UavMotionControllerNode::MavrosStateCallback(
    const mavros_msgs::State::ConstPtr& msg) {
  mavros_connected_ = msg->connected;
  mavros_armed_ = msg->armed;
  mavros_offboard_ = (msg->mode == "OFFBOARD");
}

void UavMotionControllerNode::LocalPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  p_meas_[0] = msg->pose.position.x;
  p_meas_[1] = msg->pose.position.y;
  p_meas_[2] = msg->pose.position.z;
  has_pose_meas_ = true;
}

std::array<double, 3> UavMotionControllerNode::NormalizedNormal() const {
  const double norm = std::sqrt(n_[0] * n_[0] + n_[1] * n_[1] + n_[2] * n_[2]);
  if (norm <= 1e-9) {
    return {1.0, 0.0, 0.0};
  }
  return {n_[0] / norm, n_[1] / norm, n_[2] / norm};
}

std::array<double, 3> UavMotionControllerNode::TangentialComponent(
    const std::array<double, 3>& v) const {
  const std::array<double, 3> normal = NormalizedNormal();
  const double dot = v[0] * normal[0] + v[1] * normal[1] + v[2] * normal[2];
  return {
      v[0] - dot * normal[0],
      v[1] - dot * normal[1],
      v[2] - dot * normal[2],
  };
}

std::array<double, 3> UavMotionControllerNode::TangentialTrackingVelocityCommand() const {
  std::array<double, 3> tangent_ff = TangentialComponent(v_ref_);

  if (has_pose_ref_ && has_pose_meas_) {
    const std::array<double, 3> p_error = {
        p_ref_[0] - p_meas_[0],
        p_ref_[1] - p_meas_[1],
        p_ref_[2] - p_meas_[2],
    };
    const std::array<double, 3> p_error_tangent = TangentialComponent(p_error);
    tangent_ff[0] += tangent_position_kp_ * p_error_tangent[0];
    tangent_ff[1] += tangent_position_kp_ * p_error_tangent[1];
    tangent_ff[2] += tangent_position_kp_ * p_error_tangent[2];
  }

  return ClampNorm(tangent_ff, max_tangent_velocity_);
}

std::array<double, 3> UavMotionControllerNode::FuseVelocityCommand() const {
  const std::array<double, 3> normal = NormalizedNormal();
  const std::array<double, 3> tangential = TangentialTrackingVelocityCommand();
  
  const double normal_velocity = std::max(
      -max_normal_velocity_, std::min(v_normal_cmd_, max_normal_velocity_));
  return {
      tangential[0] + normal_velocity * normal[0],
      tangential[1] + normal_velocity * normal[1],
      tangential[2] + normal_velocity * normal[2],
  };
}

std::array<double, 3> UavMotionControllerNode::ClampNorm(
    const std::array<double, 3>& v, double limit) const {
  const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (norm <= limit || norm <= 0.0) {
    return v;
  }

  const double scale = limit / norm;
  return {v[0] * scale, v[1] * scale, v[2] * scale};
}

std::array<double, 3> UavMotionControllerNode::LimitPositionTarget(
    const std::array<double, 3>& target, double max_deviation) const {
  const double limit = std::max(0.0, max_deviation);
  if (!has_pose_meas_ || limit <= 0.0) {
    return target;
  }

  const std::array<double, 3> delta = {
      target[0] - p_meas_[0],
      target[1] - p_meas_[1],
      target[2] - p_meas_[2],
  };
  const double distance =
      std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
  if (distance <= limit || distance <= 0.0) {
    return target;
  }

  const double scale = limit / distance;
  return {
      p_meas_[0] + delta[0] * scale,
      p_meas_[1] + delta[1] * scale,
      p_meas_[2] + delta[2] * scale,
  };
}

void UavMotionControllerNode::PublishApproachPositionSetpoint(
    const ros::Time& stamp) {
  const std::array<double, 3> target =
      LimitPositionTarget(p_ref_, approach_max_position_deviation_);

  mavros_msgs::PositionTarget msg;
  msg.header.stamp = stamp;
  msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = kPositionOnlyTypeMask;
  msg.position.x = target[0];
  msg.position.y = target[1];
  msg.position.z = target[2];
  msg.velocity.x = 0.0;
  msg.velocity.y = 0.0;
  msg.velocity.z = 0.0;
  msg.yaw = psi_ref_;
  msg.yaw_rate = 0.0;

  setpoint_pub_.publish(msg);
}

void UavMotionControllerNode::CaptureRetreatPositionTarget() {
  if (!has_pose_meas_) {
    has_retreat_position_target_ = false;
    ROS_WARN_THROTTLE(1.0, "RETREAT requested before measured pose is available");
    return;
  }

  std::array<double, 3> start = p_meas_;
  if (has_pose_ref_) {
    const std::array<double, 3> delta = {
        p_ref_[0] - p_meas_[0],
        p_ref_[1] - p_meas_[1],
        p_ref_[2] - p_meas_[2],
    };
    const double deviation =
        std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    const double max_deviation = std::max(0.0, retreat_start_max_deviation_m_);
    if (deviation <= max_deviation) {
      start = p_ref_;
    } else {
      ROS_WARN_STREAM(
          "RETREAT reference start rejected: measured/reference deviation "
          << deviation << " m exceeds " << max_deviation
          << " m; falling back to measured pose");
    }
  } else {
    ROS_WARN("RETREAT requested before trajectory reference is available; "
             "falling back to measured pose");
  }

  const std::array<double, 3> normal = NormalizedNormal();
  const double retreat_distance = std::max(0.0, retreat_distance_m_);
  retreat_position_target_ = {
      start[0] - retreat_distance * normal[0],
      start[1] - retreat_distance * normal[1],
      start[2] - retreat_distance * normal[2],
  };
  retreat_yaw_ref_ = psi_ref_;
  has_retreat_position_target_ = true;
}

void UavMotionControllerNode::PublishRetreatPositionSetpoint(
    const ros::Time& stamp) {
  const std::array<double, 3> target =
      LimitPositionTarget(retreat_position_target_,
                          retreat_max_position_deviation_m_);

  mavros_msgs::PositionTarget msg;
  msg.header.stamp = stamp;
  msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = kPositionOnlyTypeMask;
  msg.position.x = target[0];
  msg.position.y = target[1];
  msg.position.z = target[2];
  msg.velocity.x = 0.0;
  msg.velocity.y = 0.0;
  msg.velocity.z = 0.0;
  msg.yaw = retreat_yaw_ref_;
  msg.yaw_rate = 0.0;

  setpoint_pub_.publish(msg);
}

void UavMotionControllerNode::PublishSetpoint() {
  const ros::Time now = ros::Time::now();

  if (!has_task_phase_) {
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = now;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = kVelocityOnlyTypeMask;
    msg.velocity.x = 0.0;
    msg.velocity.y = 0.0;
    msg.velocity.z = 0.0;
    msg.yaw = psi_ref_;
    msg.yaw_rate = 0.0;
    setpoint_pub_.publish(msg);
    return;
  }

  bool should_override = false;
  std::array<double, 3> clamped{0.0, 0.0, 0.0};

  const bool offboard_ready =
      mavros_connected_ && mavros_armed_ && mavros_offboard_;
  if (zero_when_not_offboard_ready_ && !offboard_ready) {
    should_override = true;
  }

  if (safety_unsafe_ || emergency_retreat_required_) {
    should_override = true;
  }

  switch (task_phase_) {
    case uav_contact_msgs::TaskPhase::IDLE:
    case uav_contact_msgs::TaskPhase::FINISHED:
    case uav_contact_msgs::TaskPhase::ERROR:
      should_override = true;
      break;
    case uav_contact_msgs::TaskPhase::STABILIZE:
      should_override = true;
      break;
    case uav_contact_msgs::TaskPhase::APPROACH: {
      const bool velocity_ref_fresh =
          has_velocity_ref_ &&
          ((now - last_velocity_ref_time_).toSec() <= input_timeout_sec_);
      if (!should_override && approach_use_position_mode_ &&
          velocity_ref_fresh && has_pose_ref_ && has_pose_meas_) {
        PublishApproachPositionSetpoint(now);
        return;
      }
      if (velocity_ref_fresh) {
        clamped = ClampNorm(v_ref_, max_tangent_velocity_);
      }
      break;
    }
    case uav_contact_msgs::TaskPhase::INITIAL_CONTACT:
    case uav_contact_msgs::TaskPhase::SLIDING_CONTACT: {
      const bool velocity_ref_fresh =
          has_velocity_ref_ &&
          ((now - last_velocity_ref_time_).toSec() <= input_timeout_sec_);
      const bool velocity_normal_fresh =
          has_velocity_normal_cmd_ &&
          ((now - last_velocity_normal_cmd_time_).toSec() <= input_timeout_sec_);
      if (velocity_ref_fresh && velocity_normal_fresh) {
        const std::array<double, 3> fused = FuseVelocityCommand();
        clamped = ClampNorm(fused, max_velocity_);
      } else if (velocity_ref_fresh) {
        clamped = ClampNorm(TangentialComponent(v_ref_), max_tangent_velocity_);
      }
      break;
    }
    case uav_contact_msgs::TaskPhase::RETREAT: {
      if (!should_override && !has_retreat_position_target_) {
        CaptureRetreatPositionTarget();
      }
      if (!should_override && has_retreat_position_target_) {
        PublishRetreatPositionSetpoint(now);
        return;
      }
      break;
    }
    case uav_contact_msgs::TaskPhase::EMERGENCY_RETREAT:
      should_override = true;
      break;
    default:
      should_override = true;
      break;
  }

  if (should_override) {
    clamped[0] = 0.0;
    clamped[1] = 0.0;
    clamped[2] = 0.0;
  }

  mavros_msgs::PositionTarget msg;
  msg.header.stamp = now;
  msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = kVelocityOnlyTypeMask;
  msg.velocity.x = clamped[0];
  msg.velocity.y = clamped[1];
  msg.velocity.z = clamped[2];
  msg.yaw = psi_ref_;
  msg.yaw_rate = 0.0;

  setpoint_pub_.publish(msg);
}

}  // namespace uav_contact_core

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_motion_controller");
  uav_contact_core::UavMotionControllerNode node;
  node.Spin();
  return 0;
}
