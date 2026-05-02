#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import numpy as np
from dynamixel_control import DynamixelController


joint_pos = 0
# joint_offset = np.pi/6
joint_offset = 0
def joint_mapping_cb(msg):
    global joint_pos
    joint_pos = (msg.data+joint_offset)/np.pi*180

if __name__ == "__main__":
    rospy.init_node("joint_mapping")

    cmd_sb = rospy.Subscriber('/skyvortex/operator_1_joint/pos_cmd', Float32, callback = joint_mapping_cb)
    joint_pos_pub = rospy.Publisher('/joint_pos', Float32)

    rate = rospy.Rate(30)

    print("joint_mapping node started")

    ctrl = DynamixelController()
    ctrl.initialize()
    ctrl.set_operating_mode(4)
    ctrl.set_profile_vel(30)


    while(not rospy.is_shutdown()):
        ctrl.set_pos(joint_pos)
        pos_msg = Float32()
        pos_msg.data = ctrl.get_present_rad()
        joint_pos_pub.publish(pos_msg)
        print(f"current joint rad is {pos_msg.data}")
        # print(f"present current is {ctrl.get_torque()}")
        # print(f"joint_pos is set to {joint_pos}")
        rate.sleep()
