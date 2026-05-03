#!/usr/bin/env python3

from dynamixel_sdk import *
import os
import numpy as np
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_OPERATING_MODE     = 11                # 操作模式地址
ADDR_GOAL_VELOCITY      = 104               # 目标速度
ADDR_PRESENT_VELOCITY   = 128               # 当前速度
ADDR_PRESENT_CURRENT    = 126               # 当前电流

ADDR_PROFILE_ACCELERATION = 108             # 加速度
ADDR_PROFILE_VELOCITY     = 112             # 速度

POS_UNIT = 0.0878907                        # 每个信号值对应的角度增量(deg)
VEL_UNIT = 0.229746                         # 每个信号值对应的角速度增量(rpm)
CURRENT_UNIT = 2.69                         # 每个信号值对应的电流(mA)
TORQUE_UNIT = 1.97 * 0.001                  # 每单位电流对应的输出力矩（N/mA）
TRANS_RATE = 4.99023                        # 传动比

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DEFAULT_DXL_ID              = 1                 # Dynamixel ID : 1
DEFAULT_BAUDRATE            = 57600             # Dynamixel default baudrate : 57600
DEFAULT_DEVICENAME          = '/dev/ttyUSB1'    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


class DynamixelController:
    def __init__(self,
                 dxl_id: int = DEFAULT_DXL_ID,
                 baudrate: int = DEFAULT_BAUDRATE,
                 devicename: str = DEFAULT_DEVICENAME,
                 protocol_version: float = PROTOCOL_VERSION):
        self.dxl_id = dxl_id
        self.baudrate = baudrate
        self.devicename = devicename
        self.protocol_version = protocol_version

        self.portHandler = PortHandler(self.devicename)
        self.packetHandler = PacketHandler(self.protocol_version)

        self.initial_pos = None

    def feedback_handle(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()

    def initialize(self):
        # Open port
        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        try:
            self.portHandler.setBaudRate(self.baudrate)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        profile_acc = 100
        profile_vel = 5

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.feedback_handle(dxl_comm_result, dxl_error)
        print("DYNAMIXEL has been successfully connected")

        # Set Profile Acceleration and Velocity
        self.set_profile_acc(profile_acc)
        self.set_profile_vel(profile_vel)

        print("Ready to get & set Position.")

        # Get the current position
        self.initial_pos = self.get_present_pos()

    def set_operating_mode(self, mode):
        """
        0   力矩控制
        1   速度控制
        3   位置控制
        4   拓展位置控制
        5   基于力矩的位置控制模式
        16  PWM 模式
        https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/#operating-mode
        """
        # 禁用扭矩
        self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        # 设置模式
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != 0 or dxl_error != 0:
            print("Failed to set Operating Mode")
            exit()
        print("Operating Mode set to %d" % mode)
        # 启用扭矩
        self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_profile_vel(self, profile_vel):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, ADDR_PROFILE_VELOCITY, profile_vel)
        self.feedback_handle(dxl_comm_result, dxl_error)
        print(f"Profile Velocity set to {profile_vel}")

    def set_profile_acc(self, profile_acc):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, ADDR_PROFILE_ACCELERATION, profile_acc)
        self.feedback_handle(dxl_comm_result, dxl_error)
        print(f"Profile Acceleration set to {profile_acc}")

    def get_present_pos(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.dxl_id, ADDR_PRESENT_POSITION)
        if dxl_present_position > 2147483647:
            dxl_present_position -= 4294967296
        self.feedback_handle(dxl_comm_result, dxl_error)
        return dxl_present_position
    
    def get_present_pos_raw(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.dxl_id, ADDR_PRESENT_POSITION)
        self.feedback_handle(dxl_comm_result, dxl_error)
        return dxl_present_position

    def get_present_deg(self):
        if self.initial_pos is None:
            print(" Initial pos is not set!")
            return 0
        pos_deg = (self.get_present_pos() - self.initial_pos) / TRANS_RATE * POS_UNIT
        return pos_deg

    def get_present_rad(self):
        return np.deg2rad(self.get_present_deg())

    def set_pos(self, pos):
        if self.initial_pos is None:
            self.initial_pos = self.get_present_pos()
        target_pos = self.initial_pos + (-pos * TRANS_RATE / POS_UNIT)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, ADDR_GOAL_POSITION, int(target_pos))
        self.feedback_handle(dxl_comm_result, dxl_error)

    def set_pos_rad(self,pos):
        self.set_pos(np.rad2deg(pos))
    
    def set_pos_raw(self,pos_raw):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, ADDR_GOAL_POSITION, int(pos_raw))
        self.feedback_handle(dxl_comm_result, dxl_error)

    def set_vel(self, vel):
        try:
            target_vel = vel / 1.374
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, ADDR_GOAL_VELOCITY, int(target_vel))
            self.feedback_handle(dxl_comm_result, dxl_error)
        except Exception as e:
            print(f"Error in set_vel: {e}")

    def get_torque(self):
        dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.dxl_id, ADDR_PRESENT_CURRENT)
        self.feedback_handle(dxl_comm_result, dxl_error)
        return dxl_present_current * CURRENT_UNIT


if __name__ == '__main__':
    ctrl = DynamixelController()
    ctrl.initialize()
    ctrl.set_operating_mode(1)
    # 示例：设置速度
    # ctrl.set_vel(20)