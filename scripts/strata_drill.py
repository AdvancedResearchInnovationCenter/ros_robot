#!/usr/bin/env python3

import numpy as np
import urx
import math3d as m3d
import sys
import math
import time
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation as R
import copy

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf

pose_list = [[0.808829010749, -0.31777891384, 0.240050308193],
            [0.811032133363, -0.281023117164, 0.240690052391],
            [0.810983884566, -0.244815981092, 0.241312405147],
            [0.810880515087, -0.208848753405, 0.240630985119],
            [0.811878680369, -0.170118651405, 0.241864662876],
            [0.813316836998, -0.134455137717, 0.240900708981]]
quat_list = [[0.565227254872, -0.52360365957, 0.470480963826, -0.430122099772],
            [0.57261510052, -0.517246622904, 0.460731800864, -0.438513495143],
            [0.566202029723, -0.52522455534, 0.438897583948, -0.459263909763],
            [0.555303148712, -0.526050863651, 0.441674229283, -0.468863281852],
            [0.559338346742, -0.536530351203, 0.4668742081, -0.425798391157],
            [0.548463857436,  -0.518585727097, 0.45988972629, -0.467715383957]]

class urx_ros:
    def __init__(self, robot_ip):
        self.vel = 0.3
        self.acc = 0.1
        self.stop_acc = 0.3
        self.v_x = 0
        self.v_y = 0
        self.v_z = 0
        self.r_x = 0
        self.r_y = 0
        self.r_z = 0
        self.stop = False

        self.detection_mode = False
        self.detection_move_rad = 0.05

        self.item_height = 0.11

        self.cam_pose_correction = (-0.01, 0.03, 0)
        self.holder_to_camera = (0, 0, 0)#(0.00, -0.036 ,0)

        self.robot = urx.Robot(robot_ip, True)
        self.my_tcp = m3d.Transform()  # create a matrix for our tool tcp
        
        #self.my_tcp.pos.z = 0.055 #camera with holder
        #self.my_tcp.pos.z = 0.047 #camera without holder
        #self.my_tcp.pos.z = 0.21

        #Camera without holder
        # self.my_tcp.orient.rotate_z(math.pi) 
        # self.my_tcp.orient.rotate_x(-math.pi/2) 

        #STRATA drilling
        self.my_tcp.pos.x = 0.10418
        self.my_tcp.pos.y = 0.10444
        self.my_tcp.pos.z = 0.07405
        self.my_tcp.orient.rotate_z(2.35537) 
        self.my_tcp.orient.rotate_y(-0.00049) 
        self.my_tcp.orient.rotate_x(1.54924) 

        #Camera without holder
        #self.my_tcp.orient.rotate_z(math.pi) 
        #self.my_tcp.orient.rotate_x(-math.pi/2) 
        
        self.robot.set_tcp(self.my_tcp)
        self.robot.set_payload(4.4)
        time.sleep(0.2)

        self.ros_node = rospy.init_node('ur10_node', anonymous=True)
        self.pose_publisher = rospy.Publisher('ur10_pose', PoseStamped, queue_size=10)
        self.cmd_vel_subs = rospy.Subscriber("ur_cmd_vel", Twist, self.move_robot_callback)
        self.cmd_pose_subs = rospy.Subscriber("ur_cmd_pose", Pose, self.move_pose_callback)
        self.cmd_pose_subs = rospy.Subscriber("ur_cmd_adjust_pose", Pose, self.adjust_pose_callback)
        self.cmd_pose_subs = rospy.Subscriber("ur_rotate_ee", Float64, self.angle_callback)
        self.cmd_pose_subs = rospy.Subscriber("ur_rotate_ee_x", Float64, self.angle_callback_x)
        self.cmd_pose_subs = rospy.Subscriber("ur_detection_mode", Bool, self.detection_mode_callback)
        self.pickup_service = rospy.Service("ur_pickup", Empty, self.pick_item)
        
        self.rate = rospy.Rate(100)

        self.robot_pose = PoseStamped()
        self.seq = 1
        self.pose = []
        self.initial_pose = []
        self.center_pose = []

        self.run_node()

    def detection_mode_callback(self, detection_mode_bool):
        self.get_pose()
        self.initial_pose = copy.copy(self.pose)
        self.center_pose = copy.copy(self.pose)
        self.center_pose.pos[0] = self.center_pose.pos[0] + self.detection_move_rad

        self.detection_mode = detection_mode_bool.data
        print("detection mode received:", detection_mode_bool)
        

    def move_robot_callback(self, Twist_msg):
        self.v_x = Twist_msg.linear.x
        self.v_y = Twist_msg.linear.y
        self.v_z = Twist_msg.linear.z
        self.r_x = Twist_msg.angular.x
        self.r_y = Twist_msg.angular.y
        self.r_z = Twist_msg.angular.z

        print("move command received:", self.v_x, self.v_y, self.v_z, self.r_x, self.r_y, self.r_z)
        if (self.v_x==0 and self.v_y==0 and self.v_z==0 and not self.stop):
                self.stop = True

    def move_pose_callback(self, Pose_msg):

        print("Pose command received:", Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w)
        
        command_attitude = R.from_quat([Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w])
        attitude_rot_vec = command_attitude.as_rotvec()
        print(attitude_rot_vec)

        self.robot.movel((Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

    def adjust_pose_callback(self, Pose_msg):
        print("Pose command received:", Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w)
        command_attitude = R.from_quat([Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w])
        attitude_rot_vec = command_attitude.as_rotvec()
        
        self.robot.movel_tool((Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)


    def angle_callback(self, target_angle_msg):

        print("angle command received:", target_angle_msg)

        #self.robot.translate_tool(self.cam_pose_correction, self.acc, self.vel, wait=False) #TODO:place better

        trans = self.robot.get_pose()  # get current transformation matrix (tool to base)
        
        if abs(target_angle_msg.data) > 1:
            target_angle_msg.data = target_angle_msg.data / abs(target_angle_msg.data)

        trans.orient.rotate_z(target_angle_msg.data)

        #self.robot.set_pose(trans, wait=False, acc=0.5, vel=0.2)  # apply the new pose

    def angle_callback_x(self, target_angle_msg):

        print("angle command received:", target_angle_msg)

        #self.robot.translate_tool(self.cam_pose_correction, self.acc, self.vel, wait=False) #TODO:place better

        trans = self.robot.get_pose()  # get current transformation matrix (tool to base)
        
        if abs(target_angle_msg.data) > 1:
            target_angle_msg.data = target_angle_msg.data / abs(target_angle_msg.data)

        trans.orient.rotate_x(target_angle_msg.data)

        self.robot.set_pose(trans, wait=False, acc=0.5, vel=0.1)  # apply the new pose

    def robot_rotate_z_calback(self, angle):
        current_joints = self.robot.getj()
        

    def run_node(self):
        
        for i in range(len(pose_list)):

            hole_pose = pose_list[i]
            hole_quat = quat_list[0]

            hole_rotation = R.from_quat(hole_quat)
            hole_dcm = hole_rotation.as_matrix()
            attitude_rot_vec = hole_rotation.as_rotvec()

            preset_pose = hole_pose - np.matmul(hole_dcm, [-0.105, -0.085, 0.04])
            print(preset_pose)
            input("press Enter to proceed")
            self.robot.movel((preset_pose[0], preset_pose[1], preset_pose[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)


            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
            print(preset_pose)
            input("press Enter to proceed")
            self.robot.movel((preset_pose[0], preset_pose[1], preset_pose[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.0])
            print(preset_pose)
            input("press Enter to proceed")
            self.robot.movel((preset_pose[0], preset_pose[1], preset_pose[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, -0.015])
            print(preset_pose)
            input("press Enter to proceed")
            self.robot.movel((preset_pose[0], preset_pose[1], preset_pose[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
            print(preset_pose)
            input("press Enter to proceed")
            self.robot.movel((preset_pose[0], preset_pose[1], preset_pose[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

        
        self.cleanup()

    def get_pose(self):
        self.pose = self.robot.get_pose()

        self.robot_pose.pose.position.x = self.pose.pos[0]
        self.robot_pose.pose.position.y = self.pose.pos[1]
        self.robot_pose.pose.position.z = self.pose.pos[2]
        
        roation = R.from_matrix(self.pose.orient.list)
        quat = roation.as_quat()
        self.robot_pose.pose.orientation.x = quat[0]
        self.robot_pose.pose.orientation.y = quat[1]
        self.robot_pose.pose.orientation.z = quat[2]
        self.robot_pose.pose.orientation.w = quat[3]

        self.robot_pose.header.seq = self.seq
        self.seq = self.seq+1

        self.robot_pose.header.frame_id = "camera_frame"
        self.robot_pose.header.stamp = rospy.Time.now()


    def pick_item(self, req):

        self.robot.translate_tool(self.holder_to_camera, self.acc, self.vel, wait=False)

        rospy.sleep(1)

        last_pose = self.robot.get_pose()
        
        pickup_pose = copy.copy(last_pose)

        pickup_pose.pos.z = self.item_height
        pickup_pose.pos.x = pickup_pose.pos.x + self.cam_pose_correction[0]
        pickup_pose.pos.y = pickup_pose.pos.y + self.cam_pose_correction[1]

        command_attitude = R.from_matrix(pickup_pose.orient.list)
        attitude_rot_vec = command_attitude.as_rotvec()
        
        self.robot.movel((pickup_pose.pos.x, pickup_pose.pos.y, pickup_pose.pos.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)
        
        rospy.sleep(6)

        self.robot.set_digital_out(0, True)
        
        rospy.sleep(0.1)

        #self.robot.movel((last_pose.pos.x, last_pose.pos.y, last_pose.pos.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

        return []


    def cleanup(self):
        self.robot.close()
        
if __name__ == '__main__':
    robot = urx_ros("192.168.1.2")
    exit()
