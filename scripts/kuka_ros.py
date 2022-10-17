"""
This script includes implementation for robot control 
functions for UR robots using the ur_rtde package
"""
import sys, os.path
kukavar_dir = (os.path.abspath(os.path.join(os.path.dirname(__file__))) + '/kukavarproxymsgformat/python/')
sys.path.append(kukavar_dir)
from math import degrees
from socket import setdefaulttimeout
from turtle import pos
from kukavarproxy import KUKA
import rospy
from robot_base import Robot
import numpy as np
from scipy.spatial.transform import Rotation as R
from multiprocessing import Process, Array

#NOTE: ABB quaternion format: [w x y z]
#Scipy quaternion format [x y z w]


class KukaRobot(Robot):
    def __init__(self, robot_ip='10.10.105.200'):
        Robot.__init__(self)
        self.robot_ip = robot_ip
        self.robot = KUKA(self.robot_ip)

        # self.get_pose_process = Process(target=self.get_cartesian_pose, name="getting_pose")

        self.last_position = [0., 0., 0.]
        self.last_rpy = [0., 0., 0.]
        self.last_linear_pos = 0.

        self.robot_y_reach = 1000 #mm
        self.robot_default_y_delta = 700 #mm

        self.in_action = False
        
    # def get_cartesian_pose(self, in_position, in_quat):
    #     [position, quat] = self.robot.get_cartesian()
    #     in_position[:] = position
    #     in_quat[:] = quat
        
    def move_TCP(self, pose_vec, vel, acc, slow=False):
        position = [pose_vec[i] * 1000 for i in range(3)] #convert from m to mm
        rot_vec = pose_vec[3:]

        #Rotate poses by 90 deg to be compatible with base frame of UR10
        kuka_R_ur = R.from_euler('z', 90, degrees=True).as_dcm()

        position = np.matmul(kuka_R_ur, np.array(position)).tolist()

        ur_R_tcp = R.from_rotvec(rot_vec).as_dcm()
        kuka_R_tcp = np.matmul(kuka_R_ur, ur_R_tcp)

        rpy = R.from_dcm(kuka_R_tcp).as_euler('zyx', degrees=True).tolist()

        #adjust linear rail position
        linear_rail = self.last_linear_pos
        if (np.abs(-position[1] - self.last_linear_pos)) > self.robot_y_reach:
            linear_rail = -position[1] - np.sign(-position[1] - self.last_linear_pos) * self.robot_default_y_delta
        
        pose_values = [position[0], position[1], position[2], rpy[0], rpy[1], rpy[2], linear_rail]
        pose_string = self.construct_pose_msg(pose_values)
        
        self.in_action = True
        self.robot.write("COM_E6POS", pose_string)
        self.robot.write("COM_ACTION", 3)
        rospy.sleep(0.15)
        self.wait_for_move()
        self.in_action = False
        

        return True
    
    def speed_command(self, twist_vec, acc):
        raise NotImplementedError

    def get_pose(self):

        #Get the robot pose
        try:#if not self.in_action:
            pose_measurement_string = self.robot.read("$POS_ACT_MES")

            pose_measurement = self.parse_msg(pose_measurement_string)

            position = pose_measurement[:3] 
            rot_rpy = pose_measurement[3:6]
            linear_rail = pose_measurement[6]

            self.last_position = position
            self.last_rpy = rot_rpy
            self.last_linear_pos = linear_rail
        except:#else:
            # rospy.logwarn("Error getting pose")
            position = self.last_position
            rot_rpy = self.last_rpy
            linear_rail = self.last_linear_pos

        rot_mat = R.from_euler('zyx', [rot_rpy[0], rot_rpy[1], rot_rpy[2]], degrees=True).as_dcm()
        #Rotate poses by -90 deg to be compatible with base frame of UR10
        ur_R_kuka = R.from_euler('z', -90, degrees=True).as_dcm()

        updated_rot_mat = np.matmul(ur_R_kuka, rot_mat)
        rotvec = R.from_dcm(updated_rot_mat).as_rotvec().tolist()


        position = [x / 1000 for x in position] #convert from mm to m
        position = np.matmul(ur_R_kuka, np.array(position)).tolist()
            
        pose = position + rotvec
      
        return pose

    def get_vel(self):
        # vel_s = self.robot.read("$VEL_ACT")
        # print("velocity: ", vel_s)
        return [0, 0, 0, 0, 0, 0] #TODO: implement rate computation
    #    vel, _ = self.compute_rate()
    #    return vel
                
    def get_wrench(self):
        return [0, 0, 0, 0, 0, 0] #TODO: implement rate computation
        # raise NotImplementedError

    def setio(self, pin, value):
        raise NotImplementedError

    def get_analog_input(self):
        raise NotImplementedError

    def compute_rate(self, new_pose):
        raise NotImplementedError
    
    def wait_for_move(self):
        while True:
            read_value = 0
            try:
                read_value = int(self.robot.read("COM_ACTION"))
            except:
                read_value = 1
            if read_value == 0:
                break
            rospy.sleep(0.01)

    def parse_msg(self, msg_string, variables=['X', 'Y', 'Z', 'A', 'B', 'C', 'E1'], msg_type='E6POS'):
        output_vec = []
        last_idx = len(msg_type)
        for variable in variables:
            start_idx = msg_string.find(variable, last_idx)
            last_idx = start_idx

            end_idx = msg_string.find(',', last_idx)
            last_idx = end_idx

            variable_s = msg_string[start_idx+len(variable):end_idx]
            variable_f = float(variable_s)
            
            output_vec.append(variable_f)
        return output_vec

    def construct_pose_msg(self, values, variables=['X', 'Y', 'Z', 'A', 'B', 'C', 'E1'], msg_type='E6POS'):
        output_vec = []
        last_idx = len(msg_type)
        pose_msg_string = '{' + msg_type + ': '
        N_variables = len(variables)
        for i in range(N_variables):
            pose_msg_string = pose_msg_string + variables[i] + ' '
            if i == (N_variables - 1):
                pose_msg_string = pose_msg_string + str(round(values[i],3))
            else:
                pose_msg_string = pose_msg_string + str(round(values[i],3)) + ', '

        pose_msg_string = pose_msg_string + '}'
        return pose_msg_string