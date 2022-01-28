"""
This script includes implementation for robot control 
functions for UR robots using the ur_rtde package
"""
import rtde_control, rtde_receive, rtde_io
import rospy


class UrRtde:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.robot_c = rtde_control.RTDEControlInterface(robot_ip)#urx.Robot(robot_ip, True)
        self.robot_r = rtde_receive.RTDEReceiveInterface(robot_ip)#urx.Robot(robot_ip, True)
        self.robot_io = rtde_io.RTDEIOInterface(robot_ip)#urx.Robot(robot_ip, True)
        
    def move_TCP(self, pose_vec, vel, acc, slow=False):
        if not self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel, acc): #TODO: Debug wait
            self.robot_c.reuploadScript()
            rospy.sleep(1)
            if slow:
                self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel/5, acc/5)
            else:
                self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel, acc)
    
    def speed_command(self, twist_vec, acc):
        self.robot_c.speedL(twist_vec, acc, 0)

    def get_pose(self):
        return self.robot_r.getActualTCPPose()

    def get_vel(self):
       return  self.robot_r.getActualTCPSpeed()
                
    def get_wrench(self):
        return self.robot_r.getActualTCPForce()

    def setio(self, pin, value):
        self.robot_io = rtde_io.RTDEIOInterface(self.robot_ip) #TODO: investigate this
        self.robot_io.setio(pin, value)

    def get_analog_input(self):
        return self.robot_r.getStandardAnalogInput1()