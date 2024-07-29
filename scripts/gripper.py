#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform, Point
from std_msgs.msg import Float64, Bool, Header
from std_srvs.srv import Empty, SetBool
from std_srvs.srv import Trigger
from kinematics import RobotKinematics
import math
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ros_robot_pkg.srv import moveRobot, desiredTCP, pegHole, setValue, moveRobotRelative

class HandGripper: 
    """
    This class for hand Grasping 
    """
    def __init__(self):
        self.localize_blade_srv = rospy.Service('localize_blade', SetBool, self.localize_blade_cb)
        self.moveRobotRelativeCall = rospy.ServiceProxy('move_ur_relative', moveRobotRelative)
    def localize_blade_cb(self, req):
        self.localize_blade()
        return True, "done"
    
    def localize_blade(self):
        #move to initial pose relative to target object
        self.reset_scan()

        relative_pose = Pose()

        
        relative_pose.orientation.x = 0
        relative_pose.orientation.y = 0
        relative_pose.orientation.z = 1
        relative_pose.orientation.w = 0
        
        relative_pose.position.x = 0
        relative_pose.position.y = -0.86
        relative_pose.position.z = 0.2
                        
        self.moveRobotRelativeCall('TCP', 'ur_base', relative_pose)

hand_gripping = HandGripper()