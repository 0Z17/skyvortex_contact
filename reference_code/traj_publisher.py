#!/usr/bin/env python3

import rospy
import pandas as pd
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import State, PositionTarget
import numpy as np
import csv
from datetime import datetime
import os

class TrajectoryPublisher:
    def __init__(self):
        self.ref_pos_sub = rospy.Subscriber('/reference_pos', Float32, self.ref_pos_callback)
        self.target_pos_pub = rospy.Publisher('/target_trajectory', Float32MultiArray, queue_size=10)
        self.joint_pos_pub = rospy.Publisher('/skyvortex/operator_1_joint/pos_cmd', Float32, queue_size=10)
        self.target_end_pose_pub = rospy.Publisher('/end_effector_pose_ref', PoseStamped, queue_size=10)
        self.target_end_vel_pub = rospy.Publisher('/end_effector_velocity',Twist,queue_size=10)
        self.traj_mask_pub = rospy.Publisher('/traj_mask', UInt16, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state',State, self.state_callback)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.refine_path = []
        # self.target_config_offset = -0.087
        self.target_config_offset = 0.0
        # self.x_offset = 0.04
        # self.x_offset = 0.055
        # self.x_offset = 0.0045
        # self.x_offset = -0.015
        # self.x_offset = 0.13
        # self.x_offset = 0.00
        self.x_offset = 0.00

        # self.z_offset = 0.07
        # self.z_offset = 0.03
        self.z_offset = 0.00
        # self.z_offset = 0.00
        self.psi_offset = np.deg2rad(-10)
        # self.x_offset = -0.4
        self.weight = np.array([1, 1, 1, 1, 3])
        # self.cost_res = 0.001
        # self.cost_res = 0.0005
        # self.cost_res = 0.0005
        self.cost_res = 0.0005
        self.pub_count = 0
        # self.joint_pos_offset = -np.pi/6
        self.joint_pos_offset = 0
        self.pos_offset = np.array([0.0, 0.0, 0.0])
        # self.pos_offset[0] = rospy.get_param('/control_node/ref_p_x', default=0.0)
        # self.pos_offset[1] = rospy.get_param('/control_node/ref_p_y', default=0.0)
        # self.pos_offset[2] = rospy.get_param('/control_node/ref_p_z', default=0.0)
        self.length = 0.7835
        self.previous_config = None
        self.rate = 30
        # self.vel_factor = 3.5
        self.vel_factor = 2.0
        # self.vel_factor = 4.0
        # self.offboard_triggered = True
        self.offboard_triggered = False
        self.was_offboard = False
        self.path_initialized = False
        self.current_pose = PoseStamped()
        self.pose_received = False
        self.path_csv_file = "/home/nvidia/zzx_ws/test/path/exp_path.csv"
        self.impd_count = 1e5
        # self.stable_pos = [-0.268090383, 0.93545, 1.5, 0, 0]
        self.stable_pos = [0.0 - self.x_offset, 0.0, 1.5, 0, 0] # for outdoor experiment
        # self.stable_pos = [1.04 - self.x_offset, 1.61, 1.5, 0, 0]
        timestamp = datetime.now().strftime("%m%d_%H%M%S")
        self.filename = f"velocity_trajectory_{timestamp}.csv"
        self.file_initialized = False

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
        self.pos_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        self.zero_vel_time = 2.0
        self.zero_vel_pub_num = int(self.zero_vel_time * self.rate)
        self.zero_vel_count = 0

        ## Load the path csv file

    def ref_pos_callback(self, msg):
        if self.pub_count < self.impd_count:
            return
        self.target_config_offset = msg.data
        pass

    def state_callback(self, msg):
        is_offboard = (msg.mode == "OFFBOARD")

        if is_offboard and not self.was_offboard:
            rospy.loginfo("Detected OFFBOARD mode! Triggering program start.")
            self.offboard_triggered = True
            self.pub_count = 0
            self.previous_config = None
            self.refine_path = []
            self.zero_vel_count = self.zero_vel_pub_num

            if not self.pose_received:
                rospy.logwarn("Current pose not received yet, cannot initialize trajectory.")
                self.offboard_triggered = False
            else:
                self.stable_pos = [
                    self.current_pose.pose.position.x - self.x_offset,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z - self.z_offset,
                    0,
                    0,
                ]
                self.load_path_csv(self.path_csv_file)
                self.path_interpolation()
                self.path_initialized = True
                rospy.loginfo(f"Stable position initialized at OFFBOARD switch: {self.stable_pos}")
        elif not is_offboard:
            self.offboard_triggered = False
            self.pub_count = 0
            self.zero_vel_count = 0

        self.was_offboard = is_offboard

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_received = True

    def convert_end_pose(self, target_pos_msg):
        end_pose_msg = PoseStamped()
        x, y, z, psi, theta = target_pos_msg.data
        norm_vec = self.normal_vec(target_pos_msg.data)
        end_pos = np.array([x, y, z]) + self.pos_offset + np.array(norm_vec) * self.length 
        end_pose_msg.pose.position.x = end_pos[0]
        end_pose_msg.pose.position.y = end_pos[1]
        end_pose_msg.pose.position.z = end_pos[2]
        q = quaternion_from_euler(0, theta, psi)
        end_pose_msg.pose.orientation.x = q[0]
        end_pose_msg.pose.orientation.y = q[1]
        end_pose_msg.pose.orientation.z = q[2]
        end_pose_msg.pose.orientation.w = q[3]
        return end_pose_msg

    def convert_end_vel(self, previous_config, current_config):
        end_vel_msg = Twist()
        if (self.pub_count < self.impd_count) or (self.pub_count > self.leave_count):
            end_vel_msg.linear.x = 0
            end_vel_msg.linear.y = 0
            return end_vel_msg
        print("/////////////////////////////////////////////////////////////////////////")
        dx = current_config.data[0] - previous_config.data[0]
        dy = current_config.data[1] - previous_config.data[1]
        dz = current_config.data[2] - previous_config.data[2]
        dpsi = current_config.data[3] - previous_config.data[3]
        dtheta = current_config.data[4] - previous_config.data[4]
        vel_vec = np.array([dx,dy,dz,dpsi,dtheta])
        jac = self.jacobian(current_config,self.length)
        vel_end = jac @ vel_vec
        

        # calculate the unit vec of the frame of the end effector
        norm_vec = np.array(self.normal_vec(current_config.data))
        ex_row = np.cross(np.array([0,0,1]),norm_vec)
        ex = ex_row / np.linalg.norm(ex_row)
        ey_row = np.cross(norm_vec,ex)
        ey = ey_row / np.linalg.norm(ey_row)

        # calculate the projection of the velocity to the end effector
        # vx = min(0.4, np.dot(vel_end[:3],ex)* self.rate * self.vel_factor)
        # vy = min(0.4, np.dot(vel_end[:3],ey)* self.rate * self.vel_factor)
        vx = min(0.4, np.dot(vel_end[:3],ex)* self.rate * self.vel_factor)
        vy = min(0.4, np.dot(vel_end[:3],ey)* self.rate * self.vel_factor)
        end_vel_msg.linear.x = - vx
        end_vel_msg.linear.y = vy
        # print(end_vel_msg)

        # with open(self.filename,"a",newline="") as f:
        #     writer = csv.writer(f)
        #     if not self.file_initialized:
        #         writer.writerow(["vx", "vy"])
        #         self.file_initialized = True
        #     writer.writerow([vx, vy])

        return end_vel_msg
    
    def jacobian(self,config,l):
        psi = config.data[3]
        theta = config.data[4]

        jac = np.array([[1, 0, 0, -l*np.cos(theta)*np.sin(psi), -l*np.cos(psi)*np.sin(theta)],
                        [0, 1, 0, l*np.cos(theta)*np.cos(psi),  -l*np.sin(theta)*np.sin(psi)],
                        [0, 0, 1, 0,                            -l*np.cos(theta)],
                        [0, 0, 0, 1,                            0],
                        [0, 0, 0, 0,                            1]])
        
        return jac   
        
    def publish_target_trajectory(self):
        global start_wait_time
        if not self.offboard_triggered:
            if (rospy.Time.now() - start_wait_time).to_sec() > 2:
                print("waiting for offboard signal")
                start_wait_time = rospy.Time.now()
            return

        if self.zero_vel_count > 0:
            traj_mask_msg = UInt16()
            traj_mask_msg.data = self.vyz_mask
            self.traj_mask_pub.publish(traj_mask_msg)

            self.zero_vel_count -= 1
            print(f"publishing zero velocity command before position mode, remaining count: {self.zero_vel_count}")
            return

        traj_mask_msg = UInt16()
        traj_mask_msg.data = self.pos_mask
        self.traj_mask_pub.publish(traj_mask_msg)

        target_pos_msg = Float32MultiArray()
        if self.pub_count >= len(self.refine_path):
            terminate_msg = Twist()
            self.target_end_vel_pub.publish(terminate_msg)
            return
        normal_vec = self.normal_vec(self.refine_path[self.pub_count])
        target_pos_msg.data = np.array(self.refine_path[self.pub_count]) + \
                                (self.target_config_offset * np.array(normal_vec + [0, 0]))
        target_pos_msg.data[0] += self.x_offset
        target_pos_msg.data[2] += self.z_offset
        target_pos_msg.data[3] += self.psi_offset
        target_pos_msg.data[4] += self.joint_pos_offset 
        self.pub_count += 1
        end_pose_msg = self.convert_end_pose(target_pos_msg)
        if self.previous_config is not None :
            end_vel_msg = self.convert_end_vel(self.previous_config,target_pos_msg.data)
            self.target_end_vel_pub.publish(end_vel_msg)
            print(end_vel_msg)
        self.previous_config = target_pos_msg.data
        self.target_end_pose_pub.publish(end_pose_msg)
        # print(f"publish target_end_pos_msg:{end_pose_msg}")
        self.target_pos_pub.publish(target_pos_msg)
        joint_pos_msg = Float32()
        joint_pos_msg.data =  target_pos_msg.data[4]
        self.joint_pos_pub.publish(joint_pos_msg)
        print(f"pub_count:{self.pub_count}")
        print(f"publishing target_pos_msg: {target_pos_msg.data}")

    # def publish_target_start(self):
    #     if not self.offboard_triggered:
    #         return
    #     target_pos_msg = Float32MultiArray()
    #     normal_vec = self.normal_vec(self.refine_path[0])
    #     target_pos_msg.data = np.array(self.refine_path[0]) + \
    #                             (self.target_config_offset * np.array(normal_vec + [0, 0]))
    #     target_pos_msg.data[4] += self.joint_pos_offset
    #     self.target_pos_pub.publish(target_pos_msg)
    #     end_pose_msg = self.convert_end_pose(target_pos_msg)
    #     self.target_end_pose_pub.publish(end_pose_msg)
    #     # print(f"publish target_end_pos_msg:{end_pose_msg}")
    #     joint_pos_msg = Float32()
    #     joint_pos_msg.data =  target_pos_msg.data[4]
    #     self.joint_pos_pub.publish(joint_pos_msg)
    #     print(f"publishing start_pos_msg: {target_pos_msg.data}")
        
        
    def load_path_csv(self, path_csv_file):
        self.path_df = pd.read_csv(path_csv_file, header=0)
        self.path = np.array(self.path_df.values.tolist())
        # self.path = self.path_df.iloc[::-1].to_numpy()

        # process the x data
        # self.path[:, 0] = self.path[:, 0] - 1.0
        
        # self.path[:, 3] = self.path[:, 3] - np.pi

    def normal_vec(self, config): 
        psi = config[3]
        theta = config[4]
        normal_vec = [np.cos(psi)*np.cos(theta), np.sin(psi)*np.cos(theta), np.sin(theta)]
        return normal_vec

    def path_interpolation(self):
        self.refine_path = []
        # new_row = self.path[0].copy()
        # new_row[3] = 0.0
        # self.path = np.insert(self.path,0,new_row, axis=0)
        # cost = self.calculate_cost([self.path[0], self.path[1]])
        # interpolate = np.linspace(self.path[0], self.path[1], num=int(np.ceil(cost/self.cost_res)))
        # self.refine_path.extend(interpolate[:-1].tolist())

        # insert a path for stable
        stable_time = 5
        stable_num = stable_time*self.rate
        for _ in range(stable_num):
            self.refine_path.append(self.stable_pos)
        
        stable_appro_time = 30
        stable_appro_num = stable_appro_time*self.rate

        init_pos = self.path[0].copy()
        
        init_pos[0] = init_pos[0] - 0.3
        init_pos[4] = 0.0

        interpolate = np.linspace(self.stable_pos, init_pos, num=stable_appro_num)
        self.refine_path.extend(interpolate[:-1].tolist())


        # insert a path to approach the initial pos
        init_time = 5
        init_num = init_time*self.rate
        init_pos = self.path[0].copy()
        
        init_pos[0] = init_pos[0] - 0.3
        init_pos[4] = 0.0

        appro_pos = self.path[0].copy()
        appro_pos[0] = appro_pos[0] - 0.3

        interpolate = np.linspace(init_pos, appro_pos, num=init_num)
        self.refine_path.extend(interpolate[:-1].tolist())


        # insert a path to approach the start 
        appro_time = 5
        insert_num = appro_time*self.rate
        interpolate = np.linspace(appro_pos, self.path[0], num=insert_num)
        self.refine_path.extend(interpolate[:-1].tolist())

        # wait for 10 second to be stable
        wait_time = 10
        wait_num = wait_time*self.rate
        for _ in range(wait_num):
            self.refine_path.append(self.path[0].tolist()) 

        self.impd_count = stable_num + stable_appro_num + insert_num + wait_num

        for i in range(1,len(self.path)-1):
            cost = self.calculate_cost([self.path[i], self.path[i+1]])  # calculate the cost of the segment
            interpolate = np.linspace(self.path[i], self.path[i+1], num=int(np.ceil(cost/self.cost_res)))  # subdivide the segment
            self.refine_path.extend(interpolate[:-1].tolist())  # add the subdivided segment to the refine_path list
        self.refine_path.append(self.path[-1])  # add the last point of the path to the refine_path list

        # insert a path to leave the surface 
        leave_time = 20
        leave_num = leave_time * self.rate
        leave_pos = self.path[-1].copy()
        leave_pos[0] -= 0.5
        leave_pos[4] = 0
        interpolate = np.linspace(self.path[-1], leave_pos, num=leave_num)
        self.refine_path.extend(interpolate[:-1].tolist())

        for row in self.refine_path:
            row[0] += self.pos_offset[0]
            row[1] += self.pos_offset[1]
            row[2] += self.pos_offset[2]

        self.leave_count = len(self.refine_path) - leave_num
        pass

    def calculate_cost(self, segment):
        return np.linalg.norm(self.weight * (segment[1] - segment[0]))  # assing weights to each dimension and calculate the cost
    
    def save_path_csv(self, path_csv_file):
        path_df = pd.DataFrame(self.refine_path, columns=['x', 'y', 'z', 'psi', 'theta'])
        path_df.to_csv(path_csv_file, index=False, header=False)


if __name__ == '__main__':
    rospy.init_node('trajectory_publisher')
    tp = TrajectoryPublisher()
    # tp.load_path_csv("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_PCSFMT.csv")
    # tp.load_path_csv("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_FMT.csv")
    # tp.load_path_csv("/home/nuc/zzx_ws/data/state_path_FMT_4.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/data/state_path_PCSFMT_exp1.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/data/state_path_FMT_exp1.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/data/state_path_FMT_S101.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/data/state_path_PCSFMT_S101.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/data/state_path_PCSFMT_S2.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/data/state_path_PCSFMT_S3_01.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/test/path/blade_indoor_FMT.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/test/path/state_path_FMT.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/test/path/state_path_PCSFMT.csv")
    # tp.load_path_csv("/home/nvidia/zzx_ws/data/state_path_FMT_S2.csv")
    # tp.load_path_csv("/home/nuc/zzx_ws/data/base_path.csv")
    # print(f"refine_path: {tp.refine_path}")
    # tp.save_path_csv("/home/nuc/zzx_ws/data/base_path_refine.csv")
    # tp.save_path_csv("/home/nvidia/zzx_ws/data/state_path_PCSFMT_exp1_refine.csv")
    # tp.save_path_csv("/home/nvidia/zzx_ws/data/state_path_FMT_exp1_refine.csv")
    # tp.save_path_csv("/home/nvidia/zzx_ws/data/state_path_FMT_S101_refine.csv")
    # tp.save_path_csv("/home/nvidia/zzx_ws/data/state_path_FMT_S2_refine.csv")
    # tp.save_path_csv("/home/nvidia/zzx_ws/data/state_path_PCSFMT_S2_refine.csv")
    tp.save_path_csv("/home/nvidia/zzx_ws/data/exp_path_output.csv")
    rate = rospy.Rate(tp.rate)

    # # publish the start position for 10 seconds
    # start_time = rospy.Time.now()
    # while ((rospy.Time.now() - start_time).to_sec() < 10 ) and (not rospy.is_shutdown()) :
    #     tp.publish_target_start()
    #     rate.sleep()
    start_wait_time = rospy.Time.now()
    while not rospy.is_shutdown():
        tp.publish_target_trajectory()
        rate.sleep()


        

# Calculates the distance between two points

# Subdivides accourding to the distance
