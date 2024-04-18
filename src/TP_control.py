#!/usr/bin/env python
import rospy 

class TP_control:
    def __init__(self, joint_state_topic, odom_topic, task_topic):
        rospy.init_node("TP_control")
        rospy.loginfo("Starting Task Priority Controller....")
        
        self.joint_state_sub=rospy.Subscriber(joint_state_topic, Type, self.jointstateCB)
        self.odom_sub=rospy.Subscriber(odom_topic, type, self.odomCB)
        self.task_sub=rospy.Subscriber(task_topic, type, self.taskCB)
        
        
        # Timer for TP controller (Velocity Commands)
        rospy.Timer(rospy.Duration(1.0 / 10), self.controller)
    
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
    ros_node = TP_control()
    rospy.spin()