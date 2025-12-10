"""
This script includes implementation for robot control 
functions for UR robots using the ur_rtde package
"""
import rtde_control, rtde_receive, rtde_io
import rospy
from robot_base import Robot
import threading

class UrRtde(Robot):
    def __init__(self, robot_ip):
        Robot.__init__(self)
        self.robot_ip = robot_ip
        self.robot_c = rtde_control.RTDEControlInterface(robot_ip)#urx.Robot(robot_ip, True)
        self.robot_r = rtde_receive.RTDEReceiveInterface(robot_ip)#urx.Robot(robot_ip, True)
        self.robot_io = rtde_io.RTDEIOInterface(robot_ip)#urx.Robot(robot_ip, True)
        
    # def move_TCP(self, pose_vec, vel, acc, slow=False, asynchronous=False):
    #     print(f"[Driver] Attempting moveL: {pose_vec[:3]}") # <--- DEBUG
    #     # Try moving
    #     success = self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel, acc, asynchronous=asynchronous)
        
    #     if not success: #TODO: Debug wait
    #         print("[Driver] moveL FAILED! Retrying...") # <--- DEBUG
    #         self.robot_c.reuploadScript()
    #         rospy.sleep(0.05)
            
    #         # Retry with slower speed if 'slow' flag is set, or just retry
    #         if slow:
    #             self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel/5, acc/5)
    #         else:
    #             self.robot_c.moveL((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel, acc)
    #     else:
    #         print("[Driver] moveL returned Success.") # <--- DEBUG

    def move_TCP(self, pose_vec, vel, acc, slow=False, asynchronous=False):
        # Prepare arguments for the hardware call
        # (x, y, z, rx, ry, rz), speed, acceleration
        cmd_args = (
            (pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), 
            vel, 
            acc
        )
        
        if asynchronous:
            # WORKAROUND: Since 'moveL' blocks and doesn't accept async arg,
            # we run it in a separate thread to unblock the Teleop node instantly.
            t = threading.Thread(target=self.robot_c.moveL, args=cmd_args)
            t.start()
            
            # print(f"[Driver] Async moveL dispatched: {pose_vec[:3]}")
            return True # Assume success to return immediately
            
        else:
            # Standard Blocking Call
            # print(f"[Driver] Attempting moveL: {pose_vec[:3]}")
            success = self.robot_c.moveL(*cmd_args)
            
            # Retry Logic (Only for synchronous calls)
            if not success:
                # print("[Driver] moveL FAILED! Retrying...")
                self.robot_c.reuploadScript()
                rospy.sleep(0.05)
                
                # Retry
                if slow:
                    self.robot_c.moveL(cmd_args[0], vel/5, acc/5)
                else:
                    self.robot_c.moveL(*cmd_args)
            else:
                pass
                # print("[Driver] moveL Success.")
                
            return success    

    def move_TCP_compound(self, pose_vec_list, vel, acc, blend=0.1, slow=False):
        pose_list = []
        print(pose_vec_list)
        for pose_vec in pose_vec_list:
            pose_list.append([pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5], vel, acc, blend])
        print(pose_list)
        self.robot_c.moveL(pose_list, vel, acc) #TODO: Debug wait
    
    def speed_command(self, twist_vec, acc):
        # print(f"[Driver] speedL {twist_vec}") # <--- OPTIONAL (Might flood)
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
    
    def stop(self):
        # 1. Stop the physical motion
        # print("[Driver] Executing stopL(1.0)...") 
        self.robot_c.stopL(1.0)
        
        # 2. CRITICAL: Kill the RTDE script so the controller is free for moveL
        # print("[Driver] Executing stopScript()...")
        self.robot_c.stopScript()
        
        # NEW: Bring the interface back to life!
        # print("[Driver] Executing reuploadScript()...")
        # self.robot_c.reuploadScript()
        
        # rospy.sleep(0.2)
        # print("[Driver] Stop sequence finished.")
        
    def reupload_script(self):
        # 1. Safety: Ensure any running script is killed first
        # print("[Driver] Ensuring script is stopped...")
        try:
            self.robot_c.stopScript()
        except Exception:
            # Ignore if already stopped
            print("[Driver] Script was not running, continuing...")
            
        rospy.sleep(0.1) # Brief pause for controller state change

        # 2. Upload
        print("[Driver] Re-uploading RTDE script...")
        self.robot_c.reuploadScript()