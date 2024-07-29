#!/usr/bin/env python3
import numpy as np
import time
import rospy
from scipy.spatial.transform import Rotation as R
import sensor_msgs.point_cloud2 as pc2 
import ctypes 
import struct 
from std_msgs.msg import Header
from ros_robot import RosRobot
import _thread
from abb_ros import AbbRobot
import pickle 
from datetime import date
import math 
from pathlib import Path
import rospy
from alive_progress import alive_bar 
import rospkg
import random 
import abb
import cv2 
import os 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DataCollection: 
    def __init__(self,_robotIP):
        time.sleep(0.2)
        self.robotIp = _robotIP
        self.AbbRobot = AbbRobot(self.robotIp)
        self.robot = RosRobot(self.AbbRobot)
        # laser topic for the point cloud 
        self.length = 0.200
        self.width = 0.100
        self.max_z = 0.100
        self.step_z = 0.01

        self.N_pose_per_cycle = 16 #20
        self.dump_data_list = []
        self.no_of_poses_per_layer = 3 
        # self.dump_file_name = str(date.today()) + '.pickle'
        self.progress = self.no_of_poses_per_layer * self.no_of_poses_per_layer * (self.max_z /self.step_z)

        self.point_cloud_topic = "/camera/color/image_raw"
        self.setup_calibration_poses() 

        self.save_directory = 'saved_images'
        self.running = True
        self.current_image = None
        self.count = 0 

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
    
    def setup_calibration_poses(self):
        x_values = np.linspace(-100, 100, self.no_of_poses_per_layer)  # Change num to increase the number of poses in x-axis
        y_values = np.linspace(-50, 50, self.no_of_poses_per_layer)    # Change num to increase the number of poses in y-axis

        for z in np.arange(0, self.max_z, self.step_z):
            for x in x_values:
                for y in y_values:
                    transformation_matrix = np.eye(4)
                    transformation_matrix[:3, 3] = [x, y, min(z, self.max_z)]
                    self.calibration_poses.append(transformation_matrix)

        return self.calibration_poses
    
    def image_callback(self, data):
        if not self.running:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_image = cv_image
        except Exception as e:
            rospy.logerr("Error converting ROS Image to OpenCV image: %s", str(e))
            return

        cv2.imshow("Camera Stream", cv_image)
        key = cv2.waitKey(1)


        if key & 0xFF == ord('q'):
            self.stop()

    def save_image(self, cv_image):
        image_filename = os.path.join(self.save_directory, f'image_{self.count}.png')
        cv2.imwrite(image_filename, cv_image)
        rospy.loginfo("Saved image %s", image_filename)
        self.count += 1

    def stop(self):
        self.running = False
        rospy.signal_shutdown("User stopped the program")

    def ImageCollection(self):
        rospy.sleep(1)

        input("Check rviz, then proceed ...")

        with alive_bar(self.progress,force_tty=True) as bar:
            for target_pose in self.calibration_poses:
                # counter = counter +1 
                self.robot.kinematics.set_transform('center_box', 'davis', target_pose, mode='static')
                rospy.sleep(0.2)
                _, base_to_target = self.robot.kinematics.receive_transform('ur_base', 'davis')
                
                self.robot.set_TCP('davis')
                pose_msg = self.robot.kinematics.transformation_matrix_to_pose(base_to_target)
                self.robot.move_to_pose(pose_msg)
                rospy.sleep(0.5)

                # current_EE_tvec, current_EE_rot = self.getEEPose()
                # current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])
                self.save_image(self.current_image)

                
                bar()
        # print(current_working_directory)
        rospy.loginfo("Calibration Procedure is Done!")
        rospy.loginfo("Waiting for The Pickle file to be saved")



    def cleanup(self):
        self.robot.close()

if __name__ == '__main__':

    _robotIp = "192.168.125.1"
    robot = DataCollection(_robotIp)   
    _thread.start_new_thread(robot.robot.run_node,())
    _thread.start_new_thread(robot.ImageCollection,())
    
    while not rospy.is_shutdown():
        pass
    robot.cleanup()

    exit()