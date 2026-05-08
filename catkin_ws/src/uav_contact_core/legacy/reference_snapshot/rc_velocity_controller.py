#!/usr/bin/env python3

import rospy
import math
import time
from std_msgs.msg import Float32MultiArray, UInt16, Float32
from mavros_msgs.msg import RCIn, State, PositionTarget
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from enum import Enum
import numpy as np

class RCVelocityController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rc_velocity_controller', anonymous=True)
        
        # 创建发布者，发布到/target_trajectory话题
        self.trajectory_pub = rospy.Publisher('/target_trajectory', Float32MultiArray, queue_size=10)

        # 创建发布者，发布到/traj_mask话题, 用于设定控制器的mask，即忽略哪些控制变量
        self.mask_pub = rospy.Publisher('/traj_mask', UInt16, queue_size=10)

        # 创建发布者，发布速度给末端执行器
        self.end_effector_pub = rospy.Publisher('/end_effector_velocity', Twist, queue_size=10)
        
        # 创建订阅者，订阅遥控器输入
        self.rc_sub = rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)

        # 创建订阅者，订阅x轴速度控制器输出
        self.rc_sub = rospy.Subscriber('/cmd_vel_x', Float32, self.vx_callback)
        
        # 创建订阅者，订阅当前位置
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # 创建订阅者，订阅mavros状态
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)

        # 创建订阅者，订阅joint角度
        self.joint_sub = rospy.Subscriber('/joint_pos', Float32, self.joint_cb)
        
        # 设置发布频率
        self.hz = 50
        self.rate = rospy.Rate(self.hz)  # 50Hz\


        # z轴控制器参数
        self.kpv = 2
        self.kdv = 0.2
        self.ez_prev = None
        self.vz_min = -0.3
        self.vz_max = 0.3

        # 控制器记录速度
        self.rc_vy = 0.0
        self.rc_vz = 0.0
        
        # 遥控器参数
        self.rc_min = 1044   # 遥控器最小值
        self.rc_max = 1944  # 遥控器最大值
        self.rc_mid = (self.rc_min + self.rc_max) / 2  # 中位值 
        self.deadzone = 50  # 死区范围
        self.max_velocity = 0.2  # 最大速度 m/s
        self.vx = 0         # x轴速度
        
        # 当前状态
        self.current_rc = RCIn()
        self.current_pose = PoseStamped()
        self.current_state = State()
        self.target_position = [0.0, 0.0, 1.5]  # 初始目标位置 [x, y, z]
        self.target_velocity = [0.0, 0.0, 0.0]  # 目标速度 [vx, vy, vz]
        self.current_position = [0.0, 0.0, 1.5]

        
        # OFFBOARD模式管理
        self.is_offboard = False
        self.was_offboard = False
        self.position_initialized = False
        
        rospy.loginfo("遥控器速度控制节点已启动")
        rospy.loginfo("订阅话题: /mavros/rc/in, /mavros/state")
        rospy.loginfo("发布话题: /target_trajectory, /end_effector_velocity")
        rospy.loginfo(f"遥控器范围: {self.rc_min}-{self.rc_max}, 死区: ±{self.deadzone}")
        rospy.loginfo(f"最大速度: {self.max_velocity} m/s")
        rospy.loginfo("通道0(左右) -> Y轴速度, 通道1(上下) -> Z轴速度")
        rospy.loginfo("注意: 只有在OFFBOARD模式下才会响应遥控器输入")

        
        # trajectory mask
        self.vyz_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )

        self.vyz_yaw_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        self.pz_vy_yaw_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |          
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        self.pyz_yaw_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        #映射控制模式
        POS_MODE = 0    #位置模式
        VEL_MODE = 1    #速度模式
        HIGH_MODE = 2   #z轴高度，Y轴速度模式
        VEL_WY_MODE = 3
        
        self.mode_dc = {POS_MODE:self.pyz_yaw_mask,
                        VEL_MODE:self.vyz_yaw_mask,
                        HIGH_MODE:self.pz_vy_yaw_mask,
                        VEL_WY_MODE:self.vyz_mask}
        self.mode = VEL_WY_MODE

        self.mask = self.mode_dc[self.mode]
        self.change_mask = False
        self.change_mask_time = time.time()

        # 朝向向量
        self.approach_vec = np.array([1,0,0])
        self.lift_vec = np.array([0,0,1])
        self.convert_vx = 0
        self.convert_vz = 0

        self.vel_up_bound = 0.3
        self.vel_bottom_bound = -0.3
    
    def state_callback(self, msg):
        """mavros状态回调函数"""
        self.current_state = msg
        
        # 检测OFFBOARD模式
        current_offboard = (msg.mode == "OFFBOARD")
        
        # 检测模式切换
        if current_offboard and not self.was_offboard:
            # 进入OFFBOARD模式
            rospy.loginfo("进入OFFBOARD模式，重置目标位置为当前位置")
            self.reset_target_position()
            self.change_mask = True
            self.change_mask_time = time.time()
            
        elif not current_offboard and self.was_offboard:
            # 切出OFFBOARD模式
            rospy.loginfo("切出OFFBOARD模式，停止遥控器控制")
            # 清零速度
            self.target_velocity = [0.0, 0.0, 0.0]
            self.mask = self.mode_dc[self.mode]
            self.change_mask = False
        
        # 更新状态
        self.is_offboard = current_offboard
        self.was_offboard = current_offboard
    
    def pose_callback(self, msg):
        """位置回调函数"""
        self.current_pose = msg
        
        # 初始化目标位置为当前位置
        if not self.position_initialized:
            self.target_position[0] = msg.pose.position.x
            self.target_position[1] = msg.pose.position.y
            self.target_position[2] = msg.pose.position.z
            self.position_initialized = True
            rospy.loginfo(f"初始位置设定: x={self.target_position[0]:.2f}, y={self.target_position[1]:.2f}, z={self.target_position[2]:.2f}")
        self.current_position[0] = msg.pose.position.x
        self.current_position[1] = msg.pose.position.y
        self.current_position[2] = msg.pose.position.z
    
    def reset_target_position(self):
        """重置目标位置为当前位置"""
        if self.position_initialized:
            self.target_position[0] = self.current_pose.pose.position.x
            self.target_position[1] = self.current_pose.pose.position.y
            self.target_position[2] = self.current_pose.pose.position.z
            rospy.loginfo(f"目标位置已重置: x={self.target_position[0]:.2f}, y={self.target_position[1]:.2f}, z={self.target_position[2]:.2f}")
    
    def rc_callback(self, msg):
        """遥控器输入回调函数"""
        self.current_rc = msg
        
        # 只有在OFFBOARD模式下才处理遥控器输入
        if not self.is_offboard:
            return
        
        # 确保有足够的通道数据
        if len(msg.channels) < 2:
            rospy.logwarn("遥控器通道数据不足")
            return
        
        # 获取通道0和1的值
        ch0 = msg.channels[0]  # 左右方向 -> Y轴
        ch1 = msg.channels[1]  # 上下方向 -> Z轴
        
        # 映射通道值到速度
        self.rc_vy = self.map_channel_to_velocity(ch0)  # 通道0 -> Y轴速度
        self.rc_vz = self.map_channel_to_velocity(ch1)  # 通道1 -> Z轴速度
        
        # 更新目标速度
        # self.target_velocity[0] = self.vx 
        # self.target_velocity[1] = vy   # Y轴速度
        # self.target_velocity[2] = vz   # Z轴速度
        
        # 根据速度更新目标位置（积分）
        # dt = 1.0 / 50.0  # 假设50Hz更新频率
        # if self.position_initialized:
        #     self.target_position[1] += vy * dt  # Y位置积分
            # self.target_position[2] += vz * dt  # Z位置积分
            # self.target_position[1] = self.current_position[1]  
            # self.target_position[2] = self.current_position[2]
    
    def vx_callback(self, msg):
        self.vx = msg.data
    
    def map_channel_to_velocity(self, channel_value):
        """
        将遥控器通道值映射到速度
        参数:
            channel_value: 遥控器通道值 (999-1999)
        返回:
            速度值 (-max_velocity 到 +max_velocity)
        """
        # 限制输入范围
        channel_value = max(self.rc_min, min(self.rc_max, channel_value))
        
        # 计算相对于中位值的偏移
        offset = channel_value - self.rc_mid
        
        # 应用死区
        if abs(offset) < self.deadzone:
            return 0.0
        
        # 移除死区后重新计算偏移
        if offset > 0:
            offset = offset - self.deadzone
            max_offset = (self.rc_max - self.rc_mid) - self.deadzone
        else:
            offset = offset + self.deadzone
            max_offset = (self.rc_mid - self.rc_min) - self.deadzone
        
        # 映射到速度范围
        velocity = - (offset / max_offset) * self.max_velocity
        
        # 限制速度范围
        velocity = max(-self.max_velocity, min(self.max_velocity, velocity))
        
        return velocity
    
    def create_trajectory_message(self):
        """
        创建轨迹消息
        返回包含12个元素的列表: [px, py, pz, yaw, vx, vy, vz, yaw_rate, ax, ay, az, ayaw]
        """
        # 位置 (0-3): [px, py, pz, yaw]
        px = self.target_position[0]
        py = self.target_position[1] 
        pz = self.target_position[2]
        yaw = 0.0  # 固定偏航角
        
        # 速度 (4-7): [vx, vy, vz, yaw_rate]
        # 只有在OFFBOARD模式下才使用遥控器速度，否则速度为0
        if self.is_offboard:
            vx = max(self.vel_bottom_bound,min(self.vel_up_bound,self.target_velocity[0]))
            vy = max(self.vel_bottom_bound,min(self.vel_up_bound,self.target_velocity[1]))
            vz = max(self.vel_bottom_bound,min(self.vel_up_bound,self.target_velocity[2]))
        else:
            vx = 0.0
            vy = 0.0
            vz = 0.0
        
        yaw_rate = 0.0  # 固定偏航角速度
        
        # 加速度 (8-11): [ax, ay, az, ayaw]
        ax = 0.0
        ay = 0.0
        az = 0.0
        ayaw = 0.0
        
        return [px, py, pz, yaw, vx, vy, vz, yaw_rate, ax, ay, az, ayaw]
    
    def joint_cb(self, msg: Float32):
        joint_ang = msg.data
        self.approach_vec = np.array([np.cos(joint_ang), 0, np.sin(-joint_ang)])
        self.lift_vec = np.array([np.sin(joint_ang), 0, np.cos(joint_ang)])

        approach_vel = self.vx * self.approach_vec
        lift_vel = self.rc_vz * self.lift_vec

        vel_xz = approach_vel + lift_vel
        self.convert_vx = vel_xz[0]
        self.convert_vz = vel_xz[2]
        # print(self.convert_vx)
        
    
    def run(self):
        """主循环"""
        rospy.loginfo("开始遥控器速度控制循环")
        
        while not rospy.is_shutdown():

            # set target position and velocity
            self.target_velocity[0] = self.convert_vx
            self.target_velocity[1] = self.rc_vy
            self.target_velocity[2] = self.convert_vz

            self.target_position[0] += self.convert_vx * (1/self.hz)
            self.target_position[1] += self.rc_vy * (1/self.hz)
            self.target_position[2] += self.convert_vz * (1/self.hz)

            # self.target_position[0] += min(self.vx * (1/self.hz),0.6)
            # self.target_position[1] += self.rc_vy * (1/self.hz)


            # ez = self.target_position[2] - self.current_position[2]
            # p_term = self.kpv * ez
            # if self.ez_prev is None:
            #     d_term = 0.0
            # else:
            #     d_term = self.kdv * (self.ez_prev - ez)/self.hz
            # self.target_velocity[2] = np.clip(p_term + d_term,self.vz_min,self.vz_max)
            
            # 创建并发布轨迹消息
            trajectory_data = self.create_trajectory_message()
            
            msg = Float32MultiArray()
            msg.data = trajectory_data
            
            self.trajectory_pub.publish(msg)
            

            # print(self.change_mask, time.time() - self.change_mask_time)
            
            if self.change_mask and ((time.time() - self.change_mask_time) > 1.5):
                rospy.loginfo("重置目标位置并切换到z轴定点")
                self.reset_target_position()
                self.mask = self.mode_dc[self.mode]
                self.change_mask = False

            mask_msg = UInt16()
            mask_msg.data = self.mask
            self.mask_pub.publish(mask_msg)

            end_msg = Twist()
            end_msg.linear.x = -self.rc_vy
            end_msg.linear.y = self.rc_vz

            self.end_effector_pub.publish(end_msg)
            
            # 打印调试信息（降低频率）
            if rospy.get_time() % 1.0 < 0.02:  # 大约每2秒打印一次
                mode_status = "OFFBOARD" if self.is_offboard else self.current_state.mode
                if len(self.current_rc.channels) >= 2 and self.is_offboard:
                    ch0 = self.current_rc.channels[0]
                    ch1 = self.current_rc.channels[1]
                    rospy.loginfo(f"模式: {mode_status} | RC: ch0={ch0}, ch1={ch1} | 速度: vx={self.target_velocity[0]:.3f}, vy={self.target_velocity[1]:.3f}, vz={self.target_velocity[2]:.3f} | 位置: y={trajectory_data[1]:.3f}, z={trajectory_data[2]:.3f}")
                else:
                    rospy.loginfo(f"模式: {mode_status} | 位置: x={trajectory_data[0]:.3f}, y={trajectory_data[1]:.3f}, z={trajectory_data[2]:.3f}")
            
            self.rate.sleep()

def main():
    try:
        # 创建遥控器速度控制器实例
        controller = RCVelocityController()
        
        # 运行控制循环
        controller.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("遥控器速度控制节点已停止")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")

if __name__ == '__main__':
    main()