#include <geometry_controller/controller.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void GEOMETRY_CONTROL::init_param(ros::NodeHandle& nh_)
{
    nh_.param("controller_hz", controller_hz, 200.0);
    nh_.param("max_lin_vel", max_lin_vel_, 2.0);
    nh_.param("max_lin_acc", max_lin_acc_, 10.0);
    nh_.param("max_ang_vel", max_ang_vel_, 1.57);
    nh_.param("max_pos_deviation", max_pos_deviation_, 0.3);
    nh_.param("kp_force2vel", kp_force2vel_, 0.4);
    nh_.param("kp_vel", kp_vel_, 0.1);
    nh_.param("kp_torque2angvel", kp_torque2angvel_, 0.15);
    nh_.param("kp_omega", kp_omega_, 0.1);
    
    imu_sub_ = nh_.subscribe("/mavros/imu/data", 10, &GEOMETRY_CONTROL::imuCallback, this);
    position_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &GEOMETRY_CONTROL::mavposeCallback, this);
    velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &GEOMETRY_CONTROL::mavtwistCallback, this);
    mavstate_sub_ = nh_.subscribe("/mavros/state", 1, &GEOMETRY_CONTROL::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
    trajectory_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/target_trajectory", 10, &GEOMETRY_CONTROL::trajectoryCallback, this);
    vrpn_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/zzx/pose", 10, &GEOMETRY_CONTROL::vrpnCallback, this);
    mask_sub = nh_.subscribe<std_msgs::UInt16>("/traj_mask", 10, &GEOMETRY_CONTROL::maskCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    actuatorControl_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control",10);
    systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 10);
    velocity_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // velocity_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_position/local", 10);
    reference_pos_sub_ = nh_.subscribe("/reference_pos", 10, &GEOMETRY_CONTROL::referencePosCallback, this);
    contact_force_sub_ = nh_.subscribe("/ft_sensor_topic", 10, &GEOMETRY_CONTROL::contactForceCallback, this);
    contact_force_raw_sub_ = nh_.subscribe("/ft_sensor_raw_topic", 10, &GEOMETRY_CONTROL::contactForceRawCallback, this);

    // end_pose_sub_ = nh_.subscribe("/end_effector_pose", 10, &GEOMETRY_CONTROL::endPoseCallback, this);
    end_pose_sub_ = nh_.subscribe("/vrpn_client_node/end_effector/pose", 10, &GEOMETRY_CONTROL::endPoseCallback, this);
    end_pose_ref_sub_ = nh_.subscribe("/end_effector_pose_ref", 10, &GEOMETRY_CONTROL::endPoseRefCallback, this);
    end_vel_sub = nh_.subscribe("/end_effector_velocity", 10, &GEOMETRY_CONTROL::endVelCallback, this);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    wrench_sub_ = nh_.subscribe("/ft_sensor_topic", 10, &GEOMETRY_CONTROL::forceCallback, this);
    mydata_pub_ = nh_.advertise<geometry_controller::my_data>("/my_data",10);

    // std::string base_path = "/home/nvidia/zzx_ws/rosbag/data_";
    std::string base_path = "/home/nvidia/zzx_ws/rosbag/EXP/data_";
    std::string file_path = base_path + get_time_suffix() + ".bag";
    bag_.open(file_path, rosbag::bagmode::Write);

    ct = 0.0;
    dt = 1.0 / controller_hz;
    str_switch = 0.0;
    mass = 7.1;
    gravity_ = 9.81;
    J_ << 0.1560383,0,0, 0,0.1567601,0, 0,0,0.290817;
    e3_ << 0,0,1;
    force_imp << 0, 0, mass * gravity_;
    torque_input = Eigen::Vector3d::Zero();

    // mav states
    drone_p = Eigen::Vector3d::Zero();
    drone_v = Eigen::Vector3d::Zero();
    drone_a = Eigen::Vector3d::Zero();
    drone_att = Eigen::Vector3d::Zero();
    drone_Omega = Eigen::Vector3d::Zero();
    drone_q = Eigen::Vector4d::Zero();
    drone_R = Eigen::Matrix3d::Identity();
    // drone_R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // wrench estimation
    force_est = Eigen::Vector3d::Zero();
    torque_est = Eigen::Vector3d::Zero();
    xi_p_est = Eigen::Vector3d::Zero();
    xi_a_est = Eigen::Vector3d::Zero();
    // Ke_p <<1.0,0,0, 0,1.0,0, 0,0,1.0;
    // Ke_a << 0.2,0,0, 0,0.2,0, 0,0,0.2;
    // Ke_p <<1.2,0,0, 0,1.2,0, 0,0,1.2;
    Ke_p <<0.6,0,0, 0,0.6,0, 0,0,0.6;
    Ke_a << 0.25,0,0, 0,0.25,0, 0,0,0.25;

    // impedance control
    // ref_p << -0.272, 0.939, 1.6;
    // ref_p << 0.746377 - 0.1, 1.84411, 1.19919;
    // double ref_p_x = nh_.param("ref_p_x", -0.20);
    // double ref_p_y = nh_.param("ref_p_y", 1.47);
    // double ref_p_z = nh_.param("ref_p_z", 1.5);

    // double ref_p_x = 1.85634;
    // double ref_p_y = 1.61638;
    // double ref_p_z = 1.2573;

    // double ref_p_x = -0.268090383;
    // double ref_p_y = 0.73545;
    // double ref_p_z = 1.5;

    double ref_p_x = -0.3;
    double ref_p_y = 0.7;
    double ref_p_z = 1.5;

    ref_p << ref_p_x, ref_p_y, ref_p_z;
    ref_v << 0, 0, 0;
    ref_a << 0, 0, 0;
    e_p = Eigen::Vector3d::Zero();
    e_v = Eigen::Vector3d::Zero();
    e_R = Eigen::Vector3d::Zero();
    e_Omega = Eigen::Vector3d::Zero();
    // Kd_p << 12,0,0, 0,12,0, 0,0,15;
    // Dd_p << 6,0,0, 0,6,0, 0,0,5;
    // Kd_a << 0.3,0,0, 0,0.3,0, 0,0,0.3;
    // Dd_a << 0.18,0,0, 0,0.18,0, 0,0,0.18;

    // Kd_p << 20,0,0, 0,20,0, 0,0,20;
    // Dd_p << 12,0,0, 0,12,0, 0,0,12;

    // Kd_p << 20,0,0, 0,20,0, 0,0,21.5;
    // Kd_p << 20,0,0, 0,20,0, 0,0,21.5;
    // Kd_p << 30,0,0, 0,30,0, 0,0,30;
    Kd_p << 4,0,0, 0,4,0, 0,0,4;
    // Dd_p << 12,0,0, 0,12,0, 0,0,12;
    Dd_p << 20,0,0, 0,20,0, 0,0,20;
    Kd_a << 0.3,0,0, 0,0.3,0, 0,0,0.3;
    Dd_a << 0.2,0,0, 0,0.2,0, 0,0,0.2;
    
    // ref_R = Eigen::Matrix3d::Identity();
    // ref_R_last = Eigen::Matrix3d::Identity();

    double psi = M_PI/3;
    ref_R <<    cos(psi), -sin(psi),   0,
                sin(psi),  cos(psi),   0,
                       0,         0,   1;
    
    ref_R_last <<    cos(psi), -sin(psi),   0,
                    sin(psi),  cos(psi),   0,
                    0,         0,   1;
    

    // ref_R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // ref_R_last = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // impedance adaptation
    s_var = Eigen::Vector3d::Zero();
    Gamma_ << 5.0,0,0, 0,5.0,0, 0,0,5.0;
    L_K_ << 5.0,0,0, 0,5.0,0, 0,0,5.0;
    L_D_ << 1.0,0,0, 0,1.0,0, 0,0,1.0;
    a0_ = 0.2;
    a1_ = 5.0;
    epsilon_ = 0.0;
    Kd_max = 20;
    Kd_min = 10;
    Dd_max = 10;
    Dd_min = 5;

    // force control
    ref_f << -5.0, 0.0, 0.0;
    e_f_last = Eigen::Vector3d::Zero();
    K_p << 1.5,0,0, 0,1.5,0, 0,0,1.5;
    K_i << 0.3,0,0, 0,0.3,0, 0,0,0.3;
    K_v << 0.1,0,0, 0,0.1,0, 0,0,0.1;
    force_pid = Eigen::Vector3d::Zero();

    // force adaptation
    force_ff = Eigen::Vector3d::Zero();
    L_f_ << 0.05,0,0, 0,0.05,0, 0,0,0.05;

    // unified controller
    force_input << 0, 0, mass * gravity_;

    // wrench measurement
    force_ = 0;
    force_filter_ = 0;
    force_filter_last_ = 0;
    filter_a_ = 0.1571;

    // energy tank
    gamma1_ = 1.0;
    gamma2_ = 1.0;
    Energy_t_ = 20.0;
    Storage_t_ = 0.5 * Energy_t_ * Energy_t_;
    str_P1_ = 0.0;
}

// get mav state
void GEOMETRY_CONTROL::mavstateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_ = *msg;
}

// update the imu data to calculate the rotation matrix
void GEOMETRY_CONTROL::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu_data_ = *msg;
    Eigen::Vector4d drone_q;
    drone_q(0) = imu_data_.orientation.w;
    drone_q(1) = imu_data_.orientation.x;
    drone_q(2) = imu_data_.orientation.y;
    drone_q(3) = imu_data_.orientation.z;

    drone_R = quat2RotMatrix(drone_q);

    drone_Omega(0) = imu_data_.angular_velocity.x;
    drone_Omega(1) = imu_data_.angular_velocity.y;
    drone_Omega(2) = imu_data_.angular_velocity.z;
    
    drone_a(0) = imu_data_.linear_acceleration.x;
    drone_a(1) = imu_data_.linear_acceleration.y;
    drone_a(2) = imu_data_.linear_acceleration.z;
}
// get local position.
void GEOMETRY_CONTROL::mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pos_data = *msg;
    drone_p << pos_data.pose.position.x, pos_data.pose.position.y, pos_data.pose.position.z;
    drone_quat << pos_data.pose.orientation.w, pos_data.pose.orientation.x, pos_data.pose.orientation.y, pos_data.pose.orientation.z;
}

// get Velocity in the base_link frame.
void GEOMETRY_CONTROL::mavtwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    geometry_msgs::TwistStamped odom = *msg;
        
    drone_v(0) = odom.twist.linear.x;
    drone_v(1) = odom.twist.linear.y;
    drone_v(2) = odom.twist.linear.z;
}

// get force
void GEOMETRY_CONTROL::forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    geometry_msgs::WrenchStamped wrench = *msg;
    force_ = wrench.wrench.force.z;

    // low pass filter
    force_filter_ = filter_a_ * force_ + (1 - filter_a_) * force_filter_last_;
    force_filter_last_ = force_filter_;

    std::cout << "f:" << force_ << std::endl;
    // std::cout << "f_filter:" << force_filter_ << std::endl;
}

// void GEOMETRY_CONTROL::trajectoryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//     const geometry_msgs::PoseStamped& target_pose = *msg;
//     ref_p << target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z;
//     Vector4d quat;
//     quat << target_pose.pose.orientation.w,
//             target_pose.pose.orientation.x,
//             target_pose.pose.orientation.y,
//             target_pose.pose.orientation.z;
//     ref_R = quat2RotMatrix(quat);
// }

void GEOMETRY_CONTROL::trajectoryCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    const std_msgs::Float32MultiArray& target_pose = *msg;
    ref_p << target_pose.data[0], target_pose.data[1], target_pose.data[2];
    psi_ = target_pose.data[3];
    // ref_v <<  target_pose.data[4], target_pose.data[5], target_pose.data[6];
    // psi_rate = target_pose.data[7];
    ref_v << 0, 0, 0;
    psi_rate = 0;
    ref_a << target_pose.data[8], target_pose.data[9], target_pose.data[10];
    psi_acc = target_pose.data[11];
    ref_R <<    cos(psi_), -sin(psi_),   0,
                sin(psi_),  cos(psi_),   0,
                       0,         0,   1;
    // std::cout << "ref_R:" << ref_R << std::endl;
}

void GEOMETRY_CONTROL::referencePosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    reference_pos_ = msg->data;
}

void GEOMETRY_CONTROL::contactForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    contact_force_ = msg->wrench.force.z;
}

void GEOMETRY_CONTROL::contactForceRawCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    contact_force_raw_ = msg->wrench.force.z;
}


void GEOMETRY_CONTROL::process()
{
    Eigen::Vector3d drone_p_d, drone_v_d, drone_a_d;
    drone_p_d << -0.245, 1.385, 1.907;
    drone_v_d << 0.0, 0.0, 0.0;
    drone_a_d << 0.0, 0.0, 0.0;

    // ref_p << -0.268090383, 0.93545, 1.5;
    // psi_ = 0;
    // ref_p << -0.245, 1.385, 1.907;
    // ref_p << 0, 0, 1.0;
    // ref_v << 0.0, 0.0, 0.0;
    // ref_a << 0.0, 0.0, 0.0;
    
    reference_trajectory();
    wrench_estimation();
    if (current_state_.mode == "OFFBOARD")
    {
        wrench_estimation();
    }
    impedance_control();
    attitude_control();
    unified_controller();
    publish_cmd();
    publish_mydata();
}


void GEOMETRY_CONTROL::reference_trajectory()
{
    if (current_state_.mode == "OFFBOARD")
    {
        ct += dt;
        str_switch = 1.0;
    }
    else
    {
        ct = 0.0;
        str_switch = 0.0;
    }
    
    std::cout << "ct: " << ct << std::endl;
    // std::cout << "f_filter:" << force_filter_ << std::endl;
}

void GEOMETRY_CONTROL::wrench_estimation()
{
    xi_p = mass * drone_R.transpose() * drone_v;
    xi_a = J_ * drone_Omega;

    Eigen::Vector3d xi_p_est_dot = Eigen::Vector3d::Zero();
    // xi_p_est_dot = force_input - mass * matrix_vex(drone_Omega) * (drone_R.transpose() * drone_v)
    //             - mass * gravity_ * drone_R.transpose() * e3_ + force_est;
    // Eigen::Matrix3d Gamma2_;
    // Gamma2_ << gamma2_,0,0, 0,1,0, 0,0,1;
    // xi_p_est_dot = Gamma2_ * (force_input - mass * matrix_vex(drone_Omega) * (drone_R.transpose() * drone_v)
    //             - mass * gravity_ * drone_R.transpose() * e3_ + Ke_p * xi_p) - Ke_p * xi_p_est;
    // xi_p_est += xi_p_est_dot * dt;
    Eigen::Matrix3d Gamma2_;
    // xi_p_est_dot = Gamma2_ * (force_input - mass * matrix_vex(drone_Omega) * (drone_R.transpose() * drone_v)
    //             - mass * gravity_ * drone_R.transpose() * e3_ + Ke_p * xi_p) - Ke_p * xi_p_est;
    xi_p_est_dot = force_input - mass * matrix_vex(drone_Omega) * (drone_R.transpose() * drone_v) - mass * gravity_ * drone_R.transpose() * e3_ + force_est;
    xi_p_est += xi_p_est_dot * dt;


    Eigen::Vector3d xi_a_est_dot = Eigen::Vector3d::Zero();
    xi_a_est_dot = torque_input - matrix_vex(drone_Omega) * (J_ * drone_Omega) + torque_est;
    xi_a_est += xi_a_est_dot * dt;
    
    // safe constraints
    if (xi_p_est.norm() > 9)
	{
        xi_p_est = (9 / xi_p_est.norm()) * xi_p_est;
    }
    if (xi_a_est.norm() > 1)
	{
        xi_a_est = (1 / xi_a_est.norm()) * xi_a_est;
    }

    force_est = Ke_p * (xi_p - xi_p_est);
    torque_est = Ke_a * (xi_a - xi_a_est);

    std::cout << "force_est: " << force_est(0) << ", " << force_est(1) << ", " << force_est(2) << std::endl;
    std::cout << "torque_est: " << torque_est(0) << ", " << torque_est(1) << ", " << torque_est(2) << std::endl;
}

void GEOMETRY_CONTROL::impedance_control()
{
    e_p = drone_p - ref_p;
    e_v = drone_v - ref_v;

    // force_imp = drone_R.transpose() * (-Kd_p * e_p - Dd_p * e_v) + mass * matrix_vex(drone_Omega) * (drone_R.transpose() * drone_v)
    //     + mass * gravity_ * drone_R.transpose() * e3_ - force_est;

    // force_imp = drone_R.transpose() * (-Kd_p * e_p - Dd_p * e_v) + mass * matrix_vex(drone_Omega) * (drone_R.transpose() * drone_v)
    // + mass * gravity_ * drone_R.transpose() * e3_ - force_est;

    force_imp = drone_R.transpose() * (mass * ref_a -Kd_p * e_p - Dd_p * e_v) + mass * matrix_vex(drone_Omega) * (drone_R.transpose() * drone_v)
    + mass * gravity_ * drone_R.transpose() * e3_ - force_est;
}



void GEOMETRY_CONTROL::attitude_control()
{
    // Eigen::Matrix3d ref_R = Eigen::Matrix3d::Identity();
    // Eigen::Matrix3d ref_R;
    // ref_R << 0,1,0, -1,0,0, 0,0,1;
    Eigen::Vector3d ref_Omega = Eigen::Vector3d::Zero();

    // std::cout << "ref_R:" << ref_R << std::endl;
    
    ref_Omega = vex_matrix((ref_R * (ref_R_last.inverse()) - Eigen::Matrix3d::Identity()) / dt);
    e_R = 0.5 * vex_matrix(ref_R.transpose() * drone_R - drone_R.transpose() * ref_R);
    e_Omega = drone_Omega - drone_R.transpose() * ref_R * ref_Omega;

    torque_input = -Kd_a * e_R - Dd_a * e_Omega + matrix_vex(drone_Omega) * J_ * drone_Omega - torque_est;

    std::cout << "torque_input: " << torque_input(0) << ", " << torque_input(1) << ", " << torque_input(2) << std::endl;

    //update 
    ref_R_last = ref_R;
}


void GEOMETRY_CONTROL::endPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    end_p(0) = msg->pose.position.x;
    end_p(1) = msg->pose.position.y;
    end_p(2) = msg->pose.position.z;

    Eigen::Vector4d end_q;
    end_q(0) = msg->pose.orientation.w;
    end_q(1) = msg->pose.orientation.x;
    end_q(2) = msg->pose.orientation.y;
    end_q(3) = msg->pose.orientation.z;

    end_R = quat2RotMatrix(end_q);
}

void GEOMETRY_CONTROL::endPoseRefCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    end_ref_p(0) = msg->pose.position.x;
    end_ref_p(1) = msg->pose.position.y;
    end_ref_p(2) = msg->pose.position.z;

    Eigen::Vector4d end_ref_q;
    end_ref_q(0) = msg->pose.orientation.w;
    end_ref_q(1) = msg->pose.orientation.x;
    end_ref_q(2) = msg->pose.orientation.y;
    end_ref_q(3) = msg->pose.orientation.z;

    end_ref_R = quat2RotMatrix(end_ref_q);
}

void GEOMETRY_CONTROL::endVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    end_vel_x = msg->linear.x;
    end_vel_y = msg->linear.y;
}

void GEOMETRY_CONTROL::force_control()
{
    Eigen::Vector3d e_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_f_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_f_int = Eigen::Vector3d::Zero();
    e_f = force_est - ref_f;
    e_f_dot = (e_f - e_f_last) / dt;
    e_f_int = (e_f - e_f_last) * dt;

    Eigen::Vector3d str_force_pid = Eigen::Vector3d::Zero();
    // str_force_pid = K_p * e_f + K_i * e_f_int + K_v * e_f_dot;
    // str_force_pid = K_p * e_f + K_i * e_f_int;
    str_force_pid = -ref_f + K_p * e_f + K_i * e_f_int;
    e_f_last = e_f;
    // if (str_force_pid(0) > 3)
    // {
    //     str_force_pid(0) = 3;
    // }

    force_pid << str_force_pid(0), 0, 0;

    std::cout << "force_pid: " << force_pid << std::endl;
}

void GEOMETRY_CONTROL::unified_controller()
{
    // if (ct <= 50)
    // {
    //     force_input = force_imp;
    // }
    // else
    // {
    //     force_input << gamma1_ * force_pid(0), force_imp(1), force_imp(2);
    //     energy_tank();
    //     // force_input = force_imp + force_pid;
    // }

    force_input = force_imp;

    std::cout << "force_input: " << force_input(0) << ", " << force_input(1) << ", " << force_input(2) << std::endl;
}


// void GEOMETRY_CONTROL::publish_cmd()
// {
//     mavros_msgs::PositionTarget cmd_raw;
//     cmd_raw.header.stamp = ros::Time::now();
    
//     // 设置坐标系类型
//     cmd_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    
//     // 设置控制类型（速度控制）
//     // cmd_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
//     //                    mavros_msgs::PositionTarget::IGNORE_PY |
//     //                    mavros_msgs::PositionTarget::IGNORE_PZ |
//     //                    mavros_msgs::PositionTarget::IGNORE_AFX |
//     //                    mavros_msgs::PositionTarget::IGNORE_AFY |
//     //                    mavros_msgs::PositionTarget::IGNORE_AFZ |
//     //                    mavros_msgs::PositionTarget::IGNORE_YAW;

//     // cmd_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
//     //                 mavros_msgs::PositionTarget::IGNORE_PY |
//     //                 mavros_msgs::PositionTarget::IGNORE_PZ |
//     //                 mavros_msgs::PositionTarget::IGNORE_VX |
//     //                 mavros_msgs::PositionTarget::IGNORE_VY |
//     //                 mavros_msgs::PositionTarget::IGNORE_VZ |
//     //                 mavros_msgs::PositionTarget::FORCE |
//     //                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

//         cmd_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
//                     mavros_msgs::PositionTarget::IGNORE_PY  |
//                     mavros_msgs::PositionTarget::IGNORE_PZ  |
//                     mavros_msgs::PositionTarget::IGNORE_VX  |
//                     mavros_msgs::PositionTarget::IGNORE_VY  |
//                     mavros_msgs::PositionTarget::IGNORE_VZ  |
//                     mavros_msgs::PositionTarget::IGNORE_YAW |
//                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;


//     // 将期望力转换为线速度（机体坐标系）
//     force_input(2) = force_input(2) - mass * gravity_;
//     // std::cout << "force_input: " << force_input(0) << ", " << force_input(1) << ", " << force_input(2) << std::endl;
//     // Eigen::Vector3d body_velocity = kp_force2vel_ * force_input / mass + kp_vel_ * drone_v;
//     // body_acc = force_input / mass;
//     body_acc = drone_R.transpose() * (ref_a -Kd_p * e_p - Dd_p * e_v - force_est/mass) ;
//     std::cout << "body_acc: " << body_acc(0) << ", " << force_input(1) << ", " << force_input(2) << std::endl;
    
//     //限制加速度范围
//     if (body_acc.norm() > max_lin_acc_) {
//         body_acc = body_acc.normalized() * max_lin_acc_;
//     }
//     // // 限制线速度范围
//     // if (body_velocity.norm() > max_lin_vel_) {
//     //     body_velocity = body_velocity.normalized() * max_lin_vel_;
//     // }

//     // 将期望力矩转换为角速度（机体坐标系）
//     Eigen::Vector3d angular_velocity = kp_torque2angvel_ * J_.inverse() * torque_input + kp_omega_ * drone_Omega;
    
//     // 限制角速度范围
//     angular_velocity = angular_velocity.cwiseMax(-max_ang_vel_).cwiseMin(max_ang_vel_);

//     // // 设置速度指令
//     // cmd_raw.velocity.x = body_velocity.x();
//     // cmd_raw.velocity.y = body_velocity.y();
//     // cmd_raw.velocity.z = body_velocity.z();
//     // 设置速度指令
//     cmd_raw.acceleration_or_force.x = body_acc.x();
//     cmd_raw.acceleration_or_force.y = body_acc.y();
//     cmd_raw.acceleration_or_force.z = body_acc.z();

//     // std::cout << M_PI/6 << std::endl;
//     cmd_raw.yaw = psi_ - M_PI/2;
//     // cmd_raw.velocity.x = 0.0;
//     // cmd_raw.velocity.y = 0.0;
//     // cmd_raw.velocity.z = 0.0;


//     // 设置角速度指令
//     // cmd_raw.yaw_rate = angular_velocity.z();
//     // cmd_raw.yaw_rate = 1.0;

//     // 发布速度指令
//     velocity_pub_.publish(cmd_raw);
    
//     // 调试输出
//     std::cout << "CmdRaw: vel[" << std::fixed << std::setprecision(2) 
//               << cmd_raw.acceleration_or_force.x << ", "
//               << cmd_raw.acceleration_or_force.y << ", "
//               << cmd_raw.acceleration_or_force.z << "] yaw_rate[" << std::endl;
//             //   << cmd_raw.yaw_rate << "]" << std::endl;
// }

void GEOMETRY_CONTROL::publish_cmd()
{
    mavros_msgs::PositionTarget cmd_raw;
    cmd_raw.header.stamp = ros::Time::now();
    
    // 设置坐标系类型
    cmd_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    // 设置控制类型（位置控制）
    // cmd_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
    //                    mavros_msgs::PositionTarget::IGNORE_VY |
    //                    mavros_msgs::PositionTarget::IGNORE_VZ |
    //                    mavros_msgs::PositionTarget::IGNORE_AFX |
    //                    mavros_msgs::PositionTarget::IGNORE_AFY |
    //                    mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    cmd_raw.type_mask = traj_mask;

    // cmd_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
    //                 mavros_msgs::PositionTarget::IGNORE_AFY |
    //                 mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;


    // 将期望力转换为线速度（机体坐标系）
    force_input(2) = force_input(2) - mass * gravity_ + 11;
    std::cout << "force_input: " << force_input(0) << ", " << force_input(1) << ", " << force_input(2) << std::endl;
    // Eigen::Vector3d body_velocity = kp_force2vel_ * force_input / mass + kp_vel_ * drone_v;
    body_acc = force_input / mass;
    
    //限制加速度范围
    if (body_acc.norm() > max_lin_acc_) {
        body_acc = body_acc.normalized() * max_lin_acc_;
    }
    // // 限制线速度范围
    // if (body_velocity.norm() > max_lin_vel_) {
    //     body_velocity = body_velocity.normalized() * max_lin_vel_;
    // }

    // 将期望力矩转换为角速度（机体坐标系）
    Eigen::Vector3d angular_velocity = kp_torque2angvel_ * J_.inverse() * torque_input + kp_omega_ * drone_Omega;
    
    // 限制角速度范围
    angular_velocity = angular_velocity.cwiseMax(-max_ang_vel_).cwiseMin(max_ang_vel_);

    // // 设置速度指令
    // cmd_raw.velocity.x = body_velocity.x();
    // cmd_raw.velocity.y = body_velocity.y();
    // cmd_raw.velocity.z = body_velocity.z();
    // 设置速度指令
    Eigen::Vector3d safe_ref_p = ref_p;
    Eigen::Vector3d position_delta = ref_p - drone_p;
    double position_delta_norm = position_delta.norm();
    if (position_delta_norm > max_pos_deviation_ && position_delta_norm > 1e-6) {
        safe_ref_p = drone_p + position_delta / position_delta_norm * max_pos_deviation_;
    }

    cmd_raw.position.x = safe_ref_p(0);
    cmd_raw.position.y = safe_ref_p(1);
    cmd_raw.position.z = safe_ref_p(2);
    cmd_raw.velocity.x = ref_v(0);
    cmd_raw.velocity.y = ref_v(1);
    cmd_raw.velocity.z = ref_v(2);

    // std::cout << M_PI/6 << std::endl;
    cmd_raw.yaw = psi_;
    // cmd_raw.velocity.x = 0.0;
    // cmd_raw.velocity.y = 0.0;
    // cmd_raw.velocity.z = 0.0;


    // 设置角速度指令
    // cmd_raw.yaw_rate = angular_velocity.z();
    // cmd_raw.yaw_rate = 1.0;

    // 发布速度指令
    velocity_pub_.publish(cmd_raw);
    
    // 调试输出
    std::cout << "CmdRaw: vel[" << std::fixed << std::setprecision(2) 
              << cmd_raw.position.x << ", "
              << cmd_raw.position.y << ", "
              << cmd_raw.position.z << "] yaw_rate[" << std::endl;
            //   << cmd_raw.yaw_rate << "]" << std::endl;
}

// void GEOMETRY_CONTROL::publish_cmd()
// {
//     mavros_msgs::ActuatorControl cmd;

//     cmd.group_mix = 0;
//     cmd.controls[0] = std::max(-0.3, std::min(0.3, torque_input(0)));
//     cmd.controls[1] = -std::max(-0.3, std::min(0.3, torque_input(1)));
//     cmd.controls[2] = -std::max(-0.3, std::min(0.3, torque_input(2)));
//     cmd.controls[3] = std::max(0.0, std::min(0.95, 0.6*(force_input.norm() / (mass * gravity_))));
//     // cmd.controls[3] = std::max(0.0, std::min(0.95, 0.05*(thrust_ - mass * gravity_) + 0.567));
//     cmd.controls[4] = 0.0;
//     cmd.controls[5] = 0.0;
//     cmd.controls[6] = 0.0;
//     cmd.controls[7] = 0.0;
//     cmd.controls[8] = std::max(-0.5, std::min(0.5, 0.05*force_input(0)));
//     cmd.controls[9] = -std::max(-0.5, std::min(0.5, 0.05*force_input(1)));
//     cmd.controls[10] = -std::max(0.0, std::min(0.95, 0.05*(force_input(2) - mass * gravity_) + 0.567));
//     // cmd.controls[8] = force_input(0);
//     // cmd.controls[9] = -force_input(1);
//     // cmd.controls[10] = -force_input(2);
    
//     std::cout << "controls[8]: " << cmd.controls[8] << std::endl;
//     std::cout << "controls[9]: " << cmd.controls[9] << std::endl;
//     std::cout << "controls[10]: " << cmd.controls[10] << std::endl;


//     actuatorControl_pub_.publish(cmd);
// }

std::string GEOMETRY_CONTROL::get_time_suffix()
{
    std::time_t now = std::time(nullptr);
    std::tm *tm_struct = std::localtime(&now);
    
    std::ostringstream oss;
    oss << std::put_time(tm_struct, "%Y%m%d_%H%M%S"); // 格式示例：20231015_153045
    return oss.str();
}

void GEOMETRY_CONTROL::publish_mydata()
{
    // rosbag::Bag bag;
    // bag.open("/home/wsl/rosbag/data.bag", rosbag::bagmode::Write);
    
    std::cout << "///////////////////Publishing mydata/////////////////////" << std::endl;

    geometry_controller::my_data data;
    
    data.str_switch = str_switch;
    data.pos_x = drone_p(0);
    data.pos_y = drone_p(1);
    data.pos_z = drone_p(2);
    data.pos_xd = ref_p(0);
    data.pos_yd = ref_p(1);  
    data.pos_zd = ref_p(2); 
    data.pos_xd_n = ref_p(0) - 0.125;
    data.pos_yd_n = ref_p(1) + 0.28;
    data.pos_zd_n = ref_p(2) + 0.37;
    data.vel_x = drone_v(0);
    data.vel_y = drone_v(1);
    data.vel_z = drone_v(2);
    data.vel_x_d = ref_v(0);
    data.vel_y_d = ref_v(1);
    data.vel_z_d = ref_v(2);
    data.e_vel_x = data.vel_x_d - data.vel_x;
    data.e_vel_y = data.vel_y_d - data.vel_y;
    data.e_vel_z = data.vel_z_d - data.vel_z;
    data.e_pos_x = drone_p(0) - ref_p(0);
    data.e_pos_y = drone_p(1) - ref_p(1);
    data.e_pos_z = drone_p(2) - ref_p(2);

    data.pos_psid = psi_;
    
    // Eigen::Vector3d euler_angles = drone_R.eulerAngles(2, 1, 0);
    // data.pos_psi = euler_angles[2];
    // drone_quat.normalize();
    // tf2::Quaternion tf_quat(
    //     drone_quat[1],  // x
    //     drone_quat[2],  // y
    //     drone_quat[3],  // z
    //     drone_quat[0]   // w
    //     );
    tf2::Quaternion tf_quat(
        drone_quat[1],  // x
        drone_quat[2],  // y
        drone_quat[3],  // z
        drone_quat[0]   // w
    );
    tf2::Matrix3x3 tf_matrix(tf_quat);
    double roll, pitch, yaw;
    tf_matrix.getRPY(roll, pitch, yaw);
    data.pos_psi = yaw;
    data.pos_roll = roll;
    data.pos_pitch = pitch;

    data.acc_x_ref = ref_a(0);
    data.acc_y_ref = ref_a(1);
    data.acc_z_ref = ref_a(2);

    data.acc_x = drone_a(0);
    data.acc_y = drone_a(1);
    data.acc_z = drone_a(2);
    
    data.e_roll = e_R(0);
    data.e_pitch = e_R(1);
    data.e_yaw = e_R(2);
    
    data.force_est_x = force_est(0);
    data.force_est_y = force_est(1);
    data.force_est_z = force_est(2);

    data.force_d = ref_f(0);
    data.force_input_x = -force_input(0);
    
    data.gamma_1 = gamma1_;
    data.gamma_2 = gamma2_;
    data.storage_energy = Storage_t_;
    data.p1 = str_P1_;

    data.reference_pos = reference_pos_;
    data.contact_force = contact_force_;
    data.contact_force_raw = contact_force_raw_;

    e_end_R = 0.5 * vex_matrix(end_ref_R.transpose() * end_R - end_R.transpose() * end_ref_R);


    data.end_x = end_p(0);
    data.end_y = end_p(1);
    data.end_z = end_p(2);
    data.end_qw = end_q(0);
    data.end_qx = end_q(1);
    data.end_qy = end_q(2);
    data.end_qz = end_q(3);


    // cout << "end_ref_x" << end_ref_p(0) << "end_x" <<  end_p(0) << endl;
    data.e_end_x = end_ref_p(0) - end_p(0);
    data.e_end_y = end_ref_p(1) - end_p(1);
    data.e_end_z = end_ref_p(2) - end_p(2);
    data.e_end_roll = e_end_R(0);
    data.e_end_pitch = e_end_R(1);
    data.e_end_yaw = e_end_R(2);

    data.vrpn_x = vrpn_p(0);
    data.vrpn_y = vrpn_p(1);
    data.vrpn_z = vrpn_p(2);
    
    geometry_msgs::Twist end_vel;
    end_vel.linear.x = end_vel_x;
    end_vel.linear.y = end_vel_y;

    mydata_pub_.publish(data);
    bag_.write("my_data", ros::Time::now(), data);
    bag_.write("/end_effector_velocity", ros::Time::now(), end_vel);
}

void GEOMETRY_CONTROL::pubSystemStatus()
{
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void GEOMETRY_CONTROL::set_px4_mode_func(string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        //ROS_INFO("set mode success!");
    }
    pubSystemStatus();
}

void GEOMETRY_CONTROL::arm_disarm_func(bool on_or_off)
{
    mavros_msgs::CommandBool arm_cmd;

    if (current_state_.armed)
    {
        if (!on_or_off)
        {
            arm_cmd.request.value = on_or_off;
            arming_client_.call(arm_cmd);
            if (arm_cmd.response.success)
            {
                //ROS_INFO("vehicle disarming, success!");
            }
            else
            {
                //ROS_INFO("vehicle disarming, fail!");
            }
        }
        else
        {
            //ROS_INFO("vehicle already armed");
        }
    }
    else if (on_or_off)
    {
        arm_cmd.request.value = on_or_off;
        arming_client_.call(arm_cmd);
        if (arm_cmd.response.success)
        {
            //ROS_INFO("vehicle arming, success!");
        }
        else
        {
            //ROS_INFO("vehicle arming, fail!");
        }
    }
    else
    {
        //ROS_INFO("vehicle already disarmed");
    }
}

void GEOMETRY_CONTROL::maskCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    traj_mask = msg->data;
}

void GEOMETRY_CONTROL::vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vrpn_p(0) = msg->pose.position.x;
    vrpn_p(1) = msg->pose.position.y;
    vrpn_p(2) = msg->pose.position.z;
}

Eigen::Vector4d GEOMETRY_CONTROL::rot2Quaternion(const Eigen::Matrix3d &Rot) {
    Eigen::Vector4d uav_quat;
    double tr = Rot.trace();
    if (tr > 0.0) {
        double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
        uav_quat(0) = 0.25 * S;
        uav_quat(1) = (Rot(2, 1) - Rot(1, 2)) / S;
        uav_quat(2) = (Rot(0, 2) - Rot(2, 0)) / S;
        uav_quat(3) = (Rot(1, 0) - Rot(0, 1)) / S;
    } else if ((Rot(0, 0) > Rot(1, 1)) & (Rot(0, 0) > Rot(2, 2))) {
        double S = sqrt(1.0 + Rot(0, 0) - Rot(1, 1) - Rot(2, 2)) * 2.0;  // S=4*qx
        uav_quat(0) = (Rot(2, 1) - Rot(1, 2)) / S;
        uav_quat(1) = 0.25 * S;
        uav_quat(2) = (Rot(0, 1) + Rot(1, 0)) / S;
        uav_quat(3) = (Rot(0, 2) + Rot(2, 0)) / S;
    } else if (Rot(1, 1) > Rot(2, 2)) {
        double S = sqrt(1.0 + Rot(1, 1) - Rot(0, 0) - Rot(2, 2)) * 2.0;  // S=4*qy
        uav_quat(0) = (Rot(0, 2) - Rot(2, 0)) / S;
        uav_quat(1) = (Rot(0, 1) + Rot(1, 0)) / S;
        uav_quat(2) = 0.25 * S;
        uav_quat(3) = (Rot(1, 2) + Rot(2, 1)) / S;
    } else {
        double S = sqrt(1.0 + Rot(2, 2) - Rot(0, 0) - Rot(1, 1)) * 2.0;  // S=4*qz
        uav_quat(0) = (Rot(1, 0) - Rot(0, 1)) / S;
        uav_quat(1) = (Rot(0, 2) + Rot(2, 0)) / S;
        uav_quat(2) = (Rot(1, 2) + Rot(2, 1)) / S;
        uav_quat(3) = 0.25 * S;
    }
return uav_quat;
}

Eigen::Matrix3d GEOMETRY_CONTROL::quat2RotMatrix(const Eigen::Vector4d &q) {
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    
    return rotmat;
}

Eigen::Vector4d GEOMETRY_CONTROL::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
    p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);

    return quat;
}

Eigen::Vector3d GEOMETRY_CONTROL::vex_matrix(Eigen::Matrix3d S)
{
    Eigen::Vector3d v;

    v << 0.5 * (S(2,1) - S(1,2)), 0.5 * (S(0,2) - S(2,0)), 0.5 * (S(1,0) - S(0,1));

    return v;
    
}

Eigen::Matrix3d GEOMETRY_CONTROL::matrix_vex(Eigen::Vector3d v)
{
    Eigen::Matrix3d mat;
    mat << 0, -v(2), v(1), 
            v(2), 0, -v(0), 
            -v(1), v(0), 0;
    return mat;
}




