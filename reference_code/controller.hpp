#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <std_msgs/Empty.h>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/WrenchStamped.h>

#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <rosbag/bag.h>
#include <geometry_controller/my_data.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

using namespace std;
using namespace Eigen;

#define PI 3.1415926

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class GEOMETRY_CONTROL
{
        private:
        enum FlightState { WAITING_FOR_HOME_POSE, TAKE_OFF, MISSION_EXECUTION, LANDING, LANDED } node_state;
        MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;
        template <class T>
        void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
            ros::Rate pause(hz);
            ROS_INFO_STREAM(msg);
            while (ros::ok() && !(*pred)) {
                ros::spinOnce();
                pause.sleep();
            }  
        };
        geometry_msgs::Pose home_pose_;
        bool received_home_pose;
        public:
        ros::Subscriber imu_sub_,velocity_sub_,position_sub_, mavstate_sub_, wrench_sub_, trajectory_sub_, reference_pos_sub_, contact_force_sub_,
                        end_pose_sub_, end_pose_ref_sub_, end_vel_sub, contact_force_raw_sub_, imu_acc_sub, mask_sub, vrpn_sub;
        ros::Publisher  pose_pub_, actuatorControl_pub_, systemstatusPub_, mydata_pub_, velocity_pub_;
        ros::ServiceClient arming_client_, set_mode_client_;

        rosbag::Bag bag_;
        
        //control frequency
        double controller_hz;
        double dt, ct;
        double str_switch;

        // parameter
        double mass, gravity_;
        Eigen::Matrix3d J_;
        Eigen::Vector3d e3_;
        mavros_msgs::State current_state_;
        mavros_msgs::CommandBool arm_cmd_;
        Eigen::Vector3d force_imp, torque_input;
        
        // mav states
        Eigen::Vector3d drone_p, drone_v, drone_a, drone_att, drone_Omega;
        Eigen::Vector4d drone_q;
        Eigen::Matrix3d drone_R;
        Eigen::Vector4d drone_quat;
        Eigen::Vector3d vrpn_p;
        
        // reference states
        Eigen::Vector3d ref_p, ref_v, ref_a, ref_att, ref_Omega;
        double psi_{0}, psi_rate{0}, psi_acc{0};
        Eigen::Matrix3d ref_R;

        // wrench estimation
        Eigen::Vector3d force_est, torque_est;
        Eigen::Vector3d xi_p, xi_p_est;
        Eigen::Vector3d xi_a, xi_a_est;
        Eigen::Matrix3d Ke_p, Ke_a;

        // impedance control
        Eigen::Vector3d e_p, e_v;
        Eigen::Vector3d e_R, e_Omega;
        Eigen::Matrix3d Kd_p, Dd_p; //impedance control gain
        Eigen::Matrix3d Kd_a, Dd_a;
        Eigen::Matrix3d ref_R_last;

        double reference_pos_;
        double contact_force_;
        double contact_force_raw_;

        // end_pose
        Eigen::Vector3d end_p, end_ref_p;
        Eigen::Vector3d e_end_R;
        Eigen::Vector4d end_q, end_ref_q;
        Eigen::Matrix3d end_R, end_ref_R;
        double end_vel_x, end_vel_y;

        // impedance adaptation
        Eigen::Vector3d s_var;
        Eigen::Matrix3d Gamma_, L_K_, L_D_;
        double epsilon_, a0_, a1_;
        double Kd_max, Kd_min, Dd_max, Dd_min;

        // force adaptation
        Eigen::Vector3d force_ff;
        Eigen::Matrix3d L_f_;

        // force control (PID)
        Eigen::Vector3d ref_f;
        Eigen::Vector3d e_f_last;
        Eigen::Matrix3d K_p, K_i, K_v;
        Eigen::Vector3d force_pid;

        // unified controller
        Eigen::Vector3d force_input;

        double force_, force_filter_, force_filter_last_, filter_a_;

        // energy tank
        double gamma1_, gamma2_;
        double Storage_t_, Energy_t_, str_P1_;

        //velocity params
        double max_lin_vel_;
        double max_lin_acc_;
        double max_ang_vel_;
        double max_pos_deviation_;
        double kp_force2vel_;
        double kp_vel_;
        double kp_torque2angvel_;
        double kp_omega_;

        uint16_t traj_mask{mavros_msgs::PositionTarget::IGNORE_PX |
                       mavros_msgs::PositionTarget::IGNORE_PY |
                       mavros_msgs::PositionTarget::IGNORE_PZ |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE};

        Eigen::Vector3d body_acc;

        public:
        GEOMETRY_CONTROL(){}
        ~GEOMETRY_CONTROL(){}

        void init_param(ros::NodeHandle& nh_);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void mavtwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
        void statusloopCallback(const ros::TimerEvent &event);
        // void trajectoryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void trajectoryCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
        void publish_mydata();

        std::string get_time_suffix();

        void referencePosCallback(const std_msgs::Float32::ConstPtr& msg);
        void contactForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
        void contactForceRawCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
      
        void endPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void endPoseRefCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void endVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void maskCallback(const std_msgs::UInt16::ConstPtr& msg);
        void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void pubSystemStatus();
        void arm_disarm_func(bool on_or_off);
        void set_px4_mode_func(string mode);
        void process();
        void reference_trajectory();
        void wrench_estimation();
        void impedance_control();
        void impedance_adaptation();
        void attitude_control();
        void force_adaptation();
        void force_control();
        void unified_controller();
        void energy_tank();
        void publish_cmd();
        
        Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &Rot);
        Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);
        Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
        Eigen::Vector3d vex_matrix(Eigen::Matrix3d S);
        Eigen::Matrix3d matrix_vex(Eigen::Vector3d v);
};
