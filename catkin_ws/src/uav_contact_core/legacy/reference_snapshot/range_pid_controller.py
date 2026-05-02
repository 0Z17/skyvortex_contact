#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation as R
from mavros_msgs.msg import RCIn, State
from lan_telemetry import TelemetryServer


class RangePIDController(object):
    def __init__(self):
        # 参数配置
        self.laser_topic = rospy.get_param("~laser_topic", "/laser")
        # self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel_x")  # 发布 Float32 的速度话题
        self.cmd_topic = rospy.get_param("~cmd_topic", "/reference_pos")  # 发布 Float32 的速度话题
        self.range_d = rospy.get_param("~range_d", 0.30)              # 期望弹簧长度
        self.kp = rospy.get_param("~kp", 1.2)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.0)
        self.v_max = rospy.get_param("~v_max", 0.06)                 # 速度饱和限幅
        self.i_max = rospy.get_param("~i_max", 10.0)                 # 积分项限幅（反积分饱和）
        self.min_dt = rospy.get_param("~min_dt", 1e-3)               # 避免 dt 过小导致微分爆炸（不用于固定步长）
        self.enable_clip_invalid = rospy.get_param("~clip_invalid", True)  # 无效量测处理策略
        self.dt = rospy.get_param("~dt", 0.02)                       # 固定控制周期（秒）
        self.withdraw_vel = rospy.get_param("~withdraw_vel", 0.15)    # 撤退离开接触面速度

        # 低通滤波配置
        self.cutoff_hz = rospy.get_param("~cutoff_hz", 0.0)          # 截止频率（Hz），<=0 表示不滤波
        self.tau = (1.0 / (2.0 * math.pi * self.cutoff_hz)) if self.cutoff_hz > 0.0 else None

        self.integral = 0.0
        self.prev_error = None
        self.raw_range = None
        self.filtered_range = None

        self.up_bound = 0.05
        self.bottom_bound = 0.05
        self.max_range = rospy.get_param("~max_range", self.range_d + self.up_bound)          # 最大量测范围（用于无效量测处理）
        self.min_range = rospy.get_param("~min_range", self.range_d - self.bottom_bound)           # 最小量测范围（用于无效量测处理）

        # RC 使能控制（读取 /mav/rc/in 的 channels[6]）
        self.rc_topic = rospy.get_param("~rc_topic", "/mavros/rc/in")
        self.rc_channel = rospy.get_param("~rc_channel", 6)
        self.rc_threshold = rospy.get_param("~rc_threshold", 1494)
        # 新增：RC 有效范围与中位边界
        self.rc_min = rospy.get_param("~rc_min", 1044)
        self.rc_max = rospy.get_param("~rc_max", 1944)
        self.rc_bound = rospy.get_param("~rc_bound", 50)
        # -1: 反向；0: 中位/无效；+1: 正向
        self.rc_mode = 0
        self.rc_allow = False

        # Offboard 模式检测
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.require_offboard = rospy.get_param("~require_offboard", True)
        self.offboard_enabled = False

        # 紧急避离参数与状态
        self.emergency_v = rospy.get_param("~emergency_v", 0.05)  # 紧急时固定反向速度大小
        self.range_jump_rate_threshold = rospy.get_param("~range_jump_rate_threshold", 1.7)  # m/s，raw_range 的突nspeed阈值
        self.emergency_range_high = rospy.get_param("~emergency_range_high", self.range_d + 0.03)  # 跳变后认为“较大”的下限
        self.range_jump_persist_n = rospy.get_param("~range_jump_persist_n", 1)  # 连续多少周期保持较大值才触发
        self.prev_raw_range = None
        self.high_range_counter = 0
        self.pitch = None
        self.pitch_threshold_deg = rospy.get_param("~pitch_threshold_deg", 11.0)
        # 新增：紧急状态锁存（需 RC 中位复位）
        self.emergency_active = False
        self.emergency_reason = ""

        self.pub_cmd = rospy.Publisher(self.cmd_topic, Float32, queue_size=10)
        self.sub_laser = rospy.Subscriber(self.laser_topic, Range, self.laser_cb, queue_size=10)
        self.sub_rc = rospy.Subscriber(self.rc_topic, RCIn, self.rc_cb, queue_size=10)
        self.sub_state = rospy.Subscriber(self.state_topic, State, self.state_cb, queue_size=10)
        # 新增：订阅局部位置以获取姿态（pitch）
        from geometry_msgs.msg import PoseStamped
        self.local_position_topic = rospy.get_param("~local_position_topic", "/mavros/local_position/pose")
        self.sub_local_pos = rospy.Subscriber(self.local_position_topic, PoseStamped, self.local_position_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update_cb)

        rospy.loginfo(
            "RangePIDController started. laser_topic=%s, cmd_topic=%s, range_d=%.3f, kp=%.3f ki=%.3f kd=%.3f v_max=%.3f dt=%.3f cutoff_hz=%.3f rc_topic=%s rc_channel=%d rc_threshold=%d state_topic=%s require_offboard=%s",
            self.laser_topic, self.cmd_topic, self.range_d, self.kp, self.ki, self.kd, self.v_max, self.dt, self.cutoff_hz,
            self.rc_topic, self.rc_channel, self.rc_threshold, self.state_topic, str(self.require_offboard)
        )

        self.server = TelemetryServer(
            host="0.0.0.0",
            port=8000,
            viewer_token="",      # 置空：浏览器无需 token
            sender_token="send123",
            source_name="demo_random",
        )
        self.server.start()
        print("Open dashboard:", self.server.url())

    def laser_cb(self, msg):
        r = msg.range

        # 验证量测有效性
        if not math.isfinite(r):
            rospy.logwarn_throttle(1.0, "Range not finite, skip this reading.")
            self.raw_range = None
            return
        if self.enable_clip_invalid:
            # 如果量测超出传感器标注范围，则忽略
            if r < self.min_range or r > self.max_range:
                rospy.logwarn_throttle(1.0, "Range out of sensor bounds [%.3f, %.3f]: %.3f, skip.",
                                       self.min_range, self.max_range, r)
                self.raw_range = None
                return

        # 保存最新有效量测
        self.raw_range = r

    def rc_cb(self, msg):
        ch_idx = self.rc_channel
        if msg.channels and len(msg.channels) > ch_idx:
            val = msg.channels[ch_idx]
    
            # 映射到三模态：-1(反向)、0(中位/无效)、+1(正向)
            if val < self.rc_min or val > self.rc_max:
                self.rc_mode = 0
            elif abs(val - self.rc_threshold) <= self.rc_bound:
                self.rc_mode = 0
            elif val < self.rc_threshold - self.rc_bound:
                self.rc_mode = +1
            elif val > self.rc_threshold + self.rc_bound:
                self.rc_mode = -1
            else:
                self.rc_mode = 0
        else:
            rospy.logwarn_throttle(1.0, "RC channel %d not available.", ch_idx)
            self.rc_mode = 0
            self.rc_allow = False

    def state_cb(self, msg):
        # 仅在 OFFBOARD 模式下才允许发布
        self.offboard_enabled = (msg.mode == "OFFBOARD")

    # 新增方法：避险检测
    def _check_emergency(self):
        # 条件 1：raw_range 突增并维持较大值（接触表面突然脱开）
        emergency = False
        reason = ""

        if self.rc_mode == 1:
            if self.raw_range is not None and self.prev_raw_range is not None:
                dr_dt = (self.raw_range - self.prev_raw_range) / self.dt
                if dr_dt >= self.range_jump_rate_threshold and self.raw_range >= self.emergency_range_high:
                    self.high_range_counter += 1
                else:
                    self.high_range_counter = 0

                if self.high_range_counter >= self.range_jump_persist_n:
                    emergency = True
                    reason = "脱离"

        # 条件 2：pitch 偏差超过阈值
        if self.pitch is not None and abs(self.pitch) > self.pitch_threshold_deg:
            emergency = True
            reason = "倾转" if not reason else (reason + "+pitch_exceed")

        return emergency, reason

    def send_msg(self, v_cmd = None):
        self.server.send({
            "range_f": round(-1.0 if self.filtered_range is None else self.filtered_range, 3),
            "v_cmd": round(-1.0 if v_cmd is None else v_cmd, 3),
            "pitch_deg": round(0.0 if self.pitch is None else self.pitch, 3),
            "emergency": "无" if not self.emergency_active else self.emergency_reason,
        })

    def update_cb(self, event):

        # OFFBOARD 未使能时，不发布任何控制输出（可通过 require_offboard 关闭此门控）
        if self.require_offboard and not self.offboard_enabled:
            self._publish_velocity(0.0)
            return



        # 紧急状态锁存处理（RC 中位复位）
        if self.emergency_active:
            if self.rc_mode == 0:
                # RC 回到中位，解除紧急锁存并复位为 0 速度
                self.emergency_active = False
                self.emergency_reason = ""
                self._publish_velocity(0.0)
                self.prev_raw_range = self.raw_range
                rospy.logwarn_throttle(1.0, "EMERGENCY cleared: RC centered")
                self.send_msg(0.0)
                return
            else:
                # 紧急状态锁存中，持续固定反向速度
                v_cmd = -abs(self.emergency_v)
                self._publish_velocity(v_cmd)
                self.prev_raw_range = self.raw_range
                self.send_msg(v_cmd)
                rospy.logwarn_throttle(1.0, "EMERGENCY latched (%s): v=%.3f",
                                       self.emergency_reason, v_cmd)
                return
        
        # 避险检测（首次触发进入锁存）
        emergency, reason = self._check_emergency()
        if emergency:
            self.emergency_active = True
            self.emergency_reason = reason
            v_cmd = -abs(self.emergency_v)
            self._publish_velocity(v_cmd)
            self.prev_raw_range = self.raw_range
            rospy.logwarn_throttle(1.0, "EMERGENCY triggered (%s): v=%.3f",
                                       reason, v_cmd)
            self.send_msg(v_cmd)
            return
        
        # RC 中位或无效时，发布 0 速度
        if self.rc_mode == 0:
            self._publish_velocity(0.0)
            self.prev_raw_range = self.raw_range
            self.send_msg(0.0)
            return
        
        # 若无有效量测则输出 0（也可选择直接 return）
        if self.raw_range is None:
            self._publish_velocity(0.0)
            self.prev_raw_range = self.raw_range
            self.send_msg(0.0)
            return
        
        # 一阶低通滤波
        if self.cutoff_hz <= 0.0 or self.tau is None:
            self.filtered_range = self.raw_range
        else:
            alpha = self.dt / (self.tau + self.dt)
            if self.filtered_range is None:
                self.filtered_range = self.raw_range
            else:
                self.filtered_range = self.filtered_range + alpha * (self.raw_range - self.filtered_range)
        
        # 误差使用滤波后的长度
        e = self.filtered_range - self.range_d
        
        # PID 计算（得到“大小”）
        p_term = self.kp * e
        self.integral += e * self.dt
        if self.i_max is not None and self.i_max > 0.0:
            self.integral = max(min(self.integral, self.i_max), -self.i_max)
        i_term = self.ki * self.integral
        if self.prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (e - self.prev_error) / self.dt
        
        v_cmd_raw = p_term + i_term + d_term

        if self.rc_mode == 1:
            v_cmd = min(v_cmd_raw,self.v_max)
        elif self.rc_mode == -1:
            v_cmd = -self.withdraw_vel
        else:
            v_cmd = 0.0

        # 发布速度
        self._publish_velocity(v_cmd)
        
        # 更新状态
        self.prev_error = e
        self.prev_raw_range = self.raw_range
        
        rospy.logdebug("range_f=%.3f e=%.3f v=%.3f (P=%.3f, I=%.3f, D=%.3f) rc_mode=%d pitch=%f",
                           self.filtered_range, e, v_cmd, p_term, i_term, d_term, self.rc_mode, 0.0 if self.pitch is None else self.pitch)
        rospy.loginfo("range_f=%.3f e=%.3f v=%.3f (P=%.3f, I=%.3f, D=%.3f) rc_mode=%d pitch=%f",
                      self.filtered_range, e, v_cmd, p_term, i_term, d_term, self.rc_mode, 0.0 if self.pitch is None else self.pitch)
        self.send_msg(v_cmd)



    def _publish_velocity(self, v):
        msg = Float32()
        msg.data = float(v)
        self.pub_cmd.publish(msg)

    def local_position_cb(self, msg):
        quaternion = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        rotation = R.from_quat(quaternion)
        angle = rotation.as_euler('xyz', degrees=True)
        self.pitch = angle[1]




def main():
    rospy.init_node("range_pid_controller")
    controller = RangePIDController()
    rospy.spin()


if __name__ == "__main__":
    main()
