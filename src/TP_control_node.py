#!/usr/bin/env python
import rospy 
from utils import task, mobile_manipulator, taskhandler

class TP_controller:
    def __init__(self, joint_state_topic, odom_topic, task_topic, cmd_vel_topic, cmd_dq_topic):
        rospy.init_node("TP_control_node")
        rospy.loginfo("Starting Task Priority Controller....")
        
        robot = mobile_manipulator()
        taskhandler = taskhandler(robot)
        
        self.joint_state_sub=rospy.Subscriber(joint_state_topic, Type, self.jointstateCB)
        self.odom_sub=rospy.Subscriber(odom_topic, type, self.odomCB)
        self.task_sub=rospy.Subscriber(task_topic, type, self.taskCB)
        
        # PUBLISHERS
        # Timer for TP controller (Velocity Commands)
        rospy.Timer(rospy.Duration(1.0 / 10), self.controller)
        
        # Error Publisher
        self.err_pub = rospy.Publisher("/task_error", type, queue_size=1)
        
        # Command Velocity Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, type, queue_size=1)
        self.dq_pub = rospy.Publisher(cmd_dq_topic, type, queue_size=1)
        

    
    def jointstateCB(self,state):
        pass
        
    def odomCB(self,odom):
        pass
    
    def taskCB(self,task_info):
        pass
    
    def controller(self,event):
        pass
        
    def __send_base_command__(self,v,w):
        pass
    
    def __send_manipulator_command__(self,dq):
        pass
        

if __name__ == "__main__":
    ros_node = TP_controller()
    rospy.spin()