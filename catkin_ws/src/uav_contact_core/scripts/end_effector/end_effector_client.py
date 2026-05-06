#!/usr/bin/env python3

import struct
import rospy
import serial
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, WrenchStamped
from mavros_msgs.msg import State
import numpy as np

hex_data = bytes([0x7B, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6A, 0x7D])
# offboard_triggered = False
offboard_triggered = True

K = 4.186
C = -5.205
is_C_initialized = False
gravity_offset = 0
# joint_offset = np.deg2rad(30)


# offboard_triggered = True
# hex_data = bytes([0x7B, 0x00, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B, 0x7D])

def find_frame(buffer):
    """get a frame from buffer"""
    while len(buffer) >= 2:  # the buffer should have at least two bytes
        # get the start byte
        if buffer[0] == 0x7B:
            # get the end byte
            try:
                end_index = buffer.index(0x7D, 1)   # find the end byte from the second byte
                frame = buffer[:end_index + 1]      # extract the frame from the buffer
                buffer = buffer[end_index + 1:]     # leave the remaining part in the buffer
                return frame, buffer
            except ValueError:
                # if no end byte found, wait for more data
                break
        else:
            # discard invalid bytes
            buffer.pop(0)
    return None, buffer

def xor_checksum(hex_list):
    """
    Calculate the XOR checksum for a list of hexadecimal numbers
    :param hex_list: Input list containing integers or hexadecimal strings (e.g. 0xA1 or 'A1')
    :return: Checksum as hexadecimal integer (e.g. 0x12), returns None for invalid input
    """
    checksum = 0x00  # XOR initialization value
    
    for item in hex_list:
        try:
            # Convert to integer
            num = int(item)
            
            # Validate single-byte range
            if not (0x00 <= num <= 0xFF):
                raise ValueError(f"Value {hex(num)} exceeds single-byte range")
            
            checksum ^= num  # Perform XOR operation
        
        except (ValueError, TypeError):
            print(f"Error: Invalid hex value - {item}")
            return None
    
    return checksum  # Returns checksum as hexadecimal integer

def state_callback(msg):
    global offboard_triggered
    if msg.mode == "OFFBOARD":
        # rospy.loginfo("Detected OFFBOARD mode! Triggering program start.")
        offboard_triggered = True
    pass


def end_effector_velocity_callback(data):
    global hex_data
    """callback function for end_effector_velocity topic"""
    if offboard_triggered:
        x_vel = data.linear.x*1000.0
        y_vel = data.linear.y*1000.0
        z_vel = data.linear.z*1000.0
        # print(x_vel,y_vel,z_vel)
    else:
        print("testing!!!!!!!!")
        x_vel = 0.0
        y_vel = 0.0
        z_vel = 0.0
    print(f"x{x_vel/1000.0},y{y_vel/1000.0},z{z_vel/1000.0}")
    # convert the velocity to hex data
    bytes_list = [0x7B, 0x00, 0x00, 0x11]
    bytes_list.extend(float_to_hex(x_vel))
    bytes_list.extend(float_to_hex(y_vel))
    bytes_list.extend(float_to_hex(z_vel))
    bytes_list.append(xor_checksum(bytes_list))
    bytes_list.append(0x7D)
    # print([hex(byte) for byte in bytes_list])
    hex_data = bytes(bytes_list)
    # print(hex_data)

def gravity_offset_callback(msg):
    global gravity_offset
    joint_rad = msg.data
    gravity_offset = - (K * np.sin(np.rad2deg(joint_rad)) + C)

def float_to_hex(value):
    """convert a float to hex data"""
    high_val = (int(value) >> 8) & 0xFF
    low_val = int(value) & 0xFF
    return [high_val, low_val]

if __name__ == '__main__':
    rospy.init_node('serial_client')
    print("Serial client started")
    # get the serial port from the parameter server
    # port = rospy.get_param('~port', '/dev/ttyUSB_end_effector')
    port = rospy.get_param('~port', '/dev/uav/end_effector')
    baud_rate = rospy.get_param('~baud_rate', 115200)
    # open the serial port
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"Serial port {port} opened with baud rate {baud_rate}")
    # force_pub = rospy.Publisher('/ft_sensor_topic', WrenchStamped, queue_size=10)
    force_raw_pub = rospy.Publisher('/ft_sensor_raw_topic', WrenchStamped, queue_size=10)
    rospy.Subscriber('/end_effector_velocity', Twist, end_effector_velocity_callback)
    rospy.Subscriber('/mavros/state',State, state_callback)
    rospy.Subscriber('/joint_pos', Float32, gravity_offset_callback)

    rosrate = rospy.Rate(100)

    ser.write(hex_data)
    buffer = bytearray()
    
    while not rospy.is_shutdown():
        # rospy.spin()

        if ser.in_waiting:                # print("//////////////////////////////////")
            data = ser.read(ser.in_waiting)
            buffer.extend(data)
        
        frame, buffer = find_frame(buffer)
        # print(frame)
        if frame and len(frame) == 12:
            # print(frame)
            if frame[3] == 0x40:
                # print("//////////////////////////////////")
                ser.write(hex_data)
            force_value = struct.unpack('<f', frame[4:8])[0]
            # print(f"Force value: {force_value}")
            if force_value == 0:
                continue
            if not is_C_initialized:
                C = force_value
                is_C_initialized = True
            force_msg = WrenchStamped()
            force_msg.header.stamp = rospy.Time.now()
            # print(f"gravity_offset:{gravity_offset}N")
            # force_msg.wrench.force.z = force_value + gravity_offset
            force_msg.wrench.force.z = force_value
            # force_pub.publish(force_msg)
            
            force_msg = WrenchStamped()
            force_msg.header.stamp = rospy.Time.now()
            force_msg.wrench.force.z = force_value 
            force_raw_pub.publish(force_msg)

        rosrate.sleep()