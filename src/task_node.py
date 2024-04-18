#!/usr/bin/env python
import rospy 
from utils import task



class task_node:
    def __init__(self):
        rospy.init_node("ros_node")
        rospy.loginfo("Starting task_node...")

        pass


if __name__ == "__main__":
    ros_node = task_node()
    rospy.spin()