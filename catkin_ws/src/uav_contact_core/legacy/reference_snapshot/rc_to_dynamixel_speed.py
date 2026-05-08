#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import Float32
import numpy as np
from dynamixel_control import DynamixelController
import time

class RcToSpeedNode:
    def __init__(self):
        # 参数设置
        self.rc_topic = rospy.get_param("~rc_topic", "/mavros/rc/in")
        self.neutral_pos = int(rospy.get_param("~neutral_pos", 1494))
        self.speed_value = int(rospy.get_param("~speed_value", 20))
        self.threshold = int(rospy.get_param("~threshold", 15))  # 中立带宽阈值（整数）
        self.channel_mag = rospy.get_param("~channel_mag", 13)   # 用于决定正/负/零
        self.dc = DynamixelController(dxl_id=1, devicename='/dev/uav/joint_servo')
        # 初始化舵机（函数式 API
        self.dc.initialize()
        self.dc.set_operating_mode(1)
        # 状态记录/日志
        self.speed_cmd = 0.0

        # 订阅 RC 输入
        rospy.Subscriber(self.rc_topic, RCIn, self.rc_cb)
        self.joint_pos_pub = rospy.Publisher('/joint_pos', Float32, queue_size=10)

        rospy.loginfo("rc_to_dynamixel_speed node started.")
        rospy.loginfo("Listening RC: %s", self.rc_topic)
        rospy.loginfo("Params: neutral_pos=%d, speed_value=%.3f, threshold=%.3f, channel_mag=%d",
                      self.neutral_pos, self.speed_value, self.threshold, self.channel_mag)

    def rc_cb(self, msg: RCIn):
        channels = msg.channels

        # 仅检查速度通道索引有效性
        if self.channel_mag < 0 or self.channel_mag >= len(channels):
            rospy.logwarn_throttle(2.0, "RC channels size=%d, missing CH%d",
                                    len(channels), self.channel_mag)
            return

        # 读取整数 RC 值
        ch_mag = channels[self.channel_mag]

        # 阈值控制逻辑（整数比较）：
        # ch_mag > neutral_pos + threshold → 正向速度 +speed_value
        # ch_mag < neutral_pos - threshold → 反向速度 -speed_value
        # 处于 [neutral_pos - threshold, neutral_pos + threshold] → 速度 0
        upper = self.neutral_pos + self.threshold
        lower = self.neutral_pos - self.threshold

        if ch_mag > upper:
            self.speed_cmd = -self.speed_value
            self.dc.set_vel(self.speed_cmd)
        elif ch_mag < lower:
            self.speed_cmd = +self.speed_value
            self.dc.set_vel(self.speed_cmd)
        else:
            self.speed_cmd = 0.0
            self.dc.set_vel(0.0)

        present_rad = self.dc.get_present_rad()
        msg_out = Float32()
        msg_out.data = present_rad
        self.joint_pos_pub.publish(msg_out)

        rospy.logdebug("CH%d=%d (mag) -> speed=%.3f",
                        self.channel_mag, ch_mag, self.speed_cmd)
        rospy.loginfo(f"joiint_pos = {np.rad2deg(present_rad)}")
    
    def on_shutdown(self):
        # 优雅退出：下发 0 速度，避免意外转动
        try:
            self.dc.set_vel(0.0)
            self.last_speed_cmd = 0.0
            rospy.loginfo("Shutdown: set velocity to 0 to stop Dynamixel.")
            time.sleep(0.1)  
        except Exception as e:
            rospy.logwarn("Shutdown stop failed: %s", e)

def main():
    rospy.init_node("rc_to_dynamixel_speed")
    node = RcToSpeedNode()
    rospy.on_shutdown(node.on_shutdown)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        node.dc.set_vel(0.0)




if __name__ == "__main__":
    main()