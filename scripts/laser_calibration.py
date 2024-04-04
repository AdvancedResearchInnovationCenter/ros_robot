#!/usr/bin/env python
import numpy as np
import time
import rospy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2 
import ctypes 
import struct 
from std_msgs.msg import Header
from ros_robot import RosRobot
import _thread
from ur_rtde import UrRtde
import pickle 
from datetime import date
import math 
from pathlib import Path
import rospy
from alive_progress import alive_bar 
import rospkg
import random 
import os 
from datetime import datetime

base_to_ping_pong = np.array([[1, 0, 0, -0.042], [0,1, 0, -0.78], [0,0, -1, 0.09], [0,0,0,1]]) #The estimated pose for the bing bong ball 

class robot_laser_calibration:
    '''initiliaze '''

    def __init__(self,robot_ip):

        time.sleep(0.2)
        self.ur_robot = UrRtde("192.168.50.110")
        self.robot = RosRobot(self.ur_robot)
        # laser topic for the point cloud 
        # self.radius = [0.2, 0.25]
        self.radius = [0.25, 0.025]
        self.calibration_poses = [] #laser poses relative to bing bong
        self.N_cycle = 5#5
        self.max_angle = 0.3#0.4
        self.N_pose_per_cycle = 20 #20
        self.dump_data_list = []
        # self.dump_file_name = str(date.today()) + '.pickle'
        current_datetime = datetime.now()
        self.dump_file_name = current_datetime.strftime("%Y-%m-%d_%H-%M-%S") + '.pickle'
        
        # rospy.init_node('point_cloud_acquisition')
        self.point_cloud_topic = "/scancontrol_pointcloud"
        self.setup_calibration_poses() 

    def pointCloudCallBack(self):
        # return []
        laser_scan_msg = rospy.wait_for_message(self.point_cloud_topic,PointCloud2,timeout=5)
        # laser_scan_msg = rospy.wait_for_message('scancontrol_pointcloud',PointCloud2)
        xyz = np.array([[0,0,0]])
        gen = pc2.read_points(laser_scan_msg,skip_nans=True)
        # gen = pc2.read_points(self.cloud_sub,skip_nans=True)

        point_cloud_list = list(gen)
        # print(point_cloud_list)
        for points in point_cloud_list:
            xyz = np.append(xyz,[[points[0],points[1],points[2]]], axis = 0)
        return xyz 

    
    def getEEPose(self):
        robot_pose = self.robot.robot_controller.get_pose()
        tvec = [robot_pose[0], robot_pose[1], robot_pose[2]]
        return np.array(tvec), R.from_rotvec(robot_pose[3:6]).as_matrix()

    def savePickle(self):
        # self.rospack = rospkg.RosPack()
        # path = self.rospack.get_path('ros_robot_pkg')
        # rospy.loginfo(path)
        # # open(path + '/data/' + self.dump_file_name, 'w')
        # pickle.dump(self.dump_data_list,open(path + '/data/' + self.dump_file_name, 'wb'))
        # rospy.sleep(1)
        # rospy.loginfo("Pickel File Saved!")
        self.rospack = rospkg.RosPack()
        path = self.rospack.get_path('ros_robot_pkg')
        data_directory = os.path.join(path, 'data')  # Create path to data directory
        if not os.path.exists(data_directory):  # Check if directory exists
            os.makedirs(data_directory)  # If not, create it
        try:
            with open(os.path.join(data_directory, self.dump_file_name), 'wb') as file:
                pickle.dump(self.dump_data_list, file)
            rospy.sleep(1)
            rospy.loginfo("Pickle File Saved!")
        except Exception as e:
            rospy.logerr("Failed to save pickle file: {}".format(str(e)))


    def setup_calibration_poses(self):

        transformation_matrix = np.eye(4)
        transformation_matrix[2,3] = -self.radius[0]
        self.calibration_poses.append(transformation_matrix)

        for i in range(self.N_cycle):
            theta = (i+1) * self.max_angle / self.N_cycle
            for j in range(self.N_pose_per_cycle):
                phi = j * 2 * math.pi / self.N_pose_per_cycle

                rx = theta * math.cos(phi)
                ry = theta * math.sin(phi)
                # rz = 0.0
                # rz = 0.2 * (2 * random.random() - 1)
                rz = 0.12 * (2 * random.random() - 1)

                transformation_matrix = np.eye(4)
                transformation_matrix[:3,:3] = R.from_rotvec([rx, ry, rz]).as_matrix().transpose()
                transformation_matrix[:3, 3] = np.matmul(transformation_matrix[:3,:3], np.array([0.0, 0.0, -self.radius[0]])).reshape(3)
                self.calibration_poses.append(transformation_matrix)

    def perfomLaserCalibration(self):
        rospy.sleep(1)

        # self.robot.kinematics.set_transform('ur_base', 'ping_pong', base_to_ping_pong, mode='static')
        input("Check rviz, then proceed ...")
        # current_working_directory = Path.cwd()
        # print(current_working_directory)
        # print(self.calibration_poses)
        #start routine 
        # counter = 0
        with alive_bar((self.N_cycle * self.N_pose_per_cycle),force_tty=True) as bar:
            for target_pose in self.calibration_poses:
                # counter = counter +1 
                self.robot.kinematics.set_transform('ping_pong', 'scancontrol_intermediate_desired', target_pose, mode='static')
                rospy.sleep(0.2)
                _, base_to_target = self.robot.kinematics.receive_transform('ur_base', 'scancontrol_intermediate_desired')
                
                self.robot.set_TCP('scancontrol_intermediate')
                pose_msg = self.robot.kinematics.transformation_matrix_to_pose(base_to_target)
                self.robot.move_to_pose(pose_msg)
                rospy.sleep(0.5)

                current_EE_tvec, current_EE_rot = self.getEEPose()
                current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])
                # print(counter)
                current_point_cloud_xyz = self.pointCloudCallBack()     
                dump_data = {'ee_pose': current_ee_transformation,
                             'xyz_array': current_point_cloud_xyz}
                
                # dump_data = {'ee_pose': current_ee_transformation}
                             
                # dump_data = {'xyz_array': current_point_cloud_xyz}
                self.dump_data_list.append(dump_data)
                
                bar()
        # print(current_working_directory)
        rospy.loginfo("Calibration Procedure is Done!")
        rospy.loginfo("Waiting for The Pickle file to be saved")
        self.savePickle()
        # pickle.dump(self.dump_data_list, open(self.dump_file_name, 'wb'))


    def cleanup(self):
        self.robot.close()

   

if __name__ == '__main__':
    robot = robot_laser_calibration("192.168.50.110")   
    _thread.start_new_thread( robot.robot.run_node, () )
    _thread.start_new_thread(robot.perfomLaserCalibration, () )
    # robot_laser_calibration().perfomLaserCalibration()
    
    while not rospy.is_shutdown():
        pass
    robot.cleanup()

    exit()