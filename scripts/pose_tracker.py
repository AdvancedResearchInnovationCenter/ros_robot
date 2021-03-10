#!/usr/bin/env python
import numpy as np
import urx
import cv2
from cv2 import aruco
import sys
import pickle
import os
import math
import time
import rospy
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import date
from ur_node import urx_ros
from multiprocessing import Process
import thread
import random
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform, Point

class ur_10_pose_track:

    def __init__(self, robot_ip):
        time.sleep(0.2)
        self.robot = urx_ros(robot_ip)

    def move_robot(self):
        rospy.sleep(2)

        #set velocity
        self.robot.vel = 3
        self.robot.acc = 10
        # self.robot.set_TCP('davis')

        # pose_msg = Pose()
        # pose_msg.orientation.x = 0
        # pose_msg.orientation.y = 1
        # pose_msg.orientation.z = 0
        # pose_msg.orientation.w = 0

        # pose_msg.position.x = 0.5
        # pose_msg.position.y = -0.7
        # pose_msg.position.z = 0.75
        # self.robot.move_to_pose(pose_msg)
        
        # rospy.loginfo("Started moving")
        # pose_msg.position.x = -0.5
        # pose_msg.position.y = -0.7
        # pose_msg.position.z = 0.75
        # self.robot.move_to_pose(pose_msg)
        # rospy.loginfo("End moving")



        current_joints = self.robot.robot.getj()
        current_joints[0] = current_joints[0] - math.pi/2
        self.robot.robot.movej(current_joints, acc=100, vel=100, wait=True)

        current_joints = self.robot.robot.getj()
        current_joints[0] = current_joints[0] + math.pi/2
        self.robot.robot.movej(current_joints, acc=100, vel=100, wait=True)



    def cleanup(self):
        self.robot.close()


if __name__ == '__main__':

    robot = ur_10_pose_track("192.168.0.110")
    thread.start_new_thread( robot.robot.run_node, () )
    thread.start_new_thread( robot.move_robot, () )
    
    while not rospy.is_shutdown():
        pass
    robot.cleanup()

    exit()
