#!/usr/bin/python
import tf
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose as PoseMsg
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from utils.EKF.EKF_3DOF_InputDisplacement_Heading import *

from utils.EKF.Odometry import *
from utils.EKF.Magnetometer import *

odom_freq   = 0.1
odom_window = 100000.0


class EKF:
    def __init__(self, odom_topic) -> None:
        self.current_pose           = None
        self.xk           = np.zeros((3, 1))        # Robot pose in the k frame
        self.Pk           = np.zeros((3, 3))        # Robot covariance in the k frame  
        self.yawOffset    = 0.0
        self.ekf_filter   = None
        self.x_map        = np.zeros((3, 1))        # Robot pose in the map frame
        self.x_frame_k    = np.zeros((3, 1))        # k frame pose in the map frame

        self.mode         = rospy.get_param('~mode')

        # PUBLISHERS
        # Publisher for visualizing the path to with rviz
        # self.point_marker_pub   = rospy.Publisher('~point_marker', Marker, queue_size=1)
        
        # Publisher for sending Odometry
        self.odom_pub           = rospy.Publisher('/odom', Odometry, queue_size=1)
        
        # SUBSCRIBERS
        self.odom_sub               = rospy.Subscriber(odom_topic, JointState, self.get_odom) 

        if self.mode == "SIL":
            self.ground_truth_sub       = rospy.Subscriber('/turtlebot/kobuki/odom_ground_truth', Odometry, self.get_ground_truth) 
        
        # Init using sensors
        self.odom   = OdomData()
        self.mag    = Magnetometer()

        if self.mode == "SIL":
            # Move
            while True:
                if self.current_pose is not None:
                    # self.xk           = self.current_pose.reshape(3,1)
                    # self.xk           = np.zeros((3, 1))
                    # self.Pk           = np.zeros((3, 3))
                    self.yawOffset    = self.current_pose[2]
                    break
        
        # SERVICES
    

        # TIMERS
        # Timer for displacement reset
        # rospy.Timer(rospy.Duration(odom_window), self.reset_filter)

        # Init EKF Filter
        self.ekf_filter = EKF_3DOF_InputDisplacement_Heading(self.xk, self.Pk, self.odom, self.mag)
    
    # Ground Truth callback: Gets current robot pose and stores it into self.current_pose. Besides, get heading as a measurement to update filter
    def get_ground_truth(self, odom):
        timestamp        = odom.header.stamp
        # Read orientation and position
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

        # Get heading as a measurement to update filter
        if self.mag.read_magnetometer(yaw-self.yawOffset, timestamp) and self.ekf_filter is not None:
            self.ekf_filter.gotNewHeadingData()

    # Odometry callback: Gets encoder reading to compute displacement of the robot as input of the EKF Filter.
    # Run EKF Filter with frequency of odometry reading
    def get_odom(self, odom):
        timestamp        = odom.header.stamp
        # Read encoder
        if (len(odom.name) == 2 and self.mode == "HIL") or self.mode == "SIL":
            if self.odom.read_encoder(odom, timestamp) and self.ekf_filter is not None:
                self.ekf_filter.gotNewEncoderData()

        if self.ekf_filter is not None:
            # Run EKF Filter
            self.xk, self.Pk = self.ekf_filter.Localize(self.xk, self.Pk)

            self.x_map       = Pose3D.oplus(self.x_frame_k, self.xk)

            # Publish rviz
            self.odom_path_pub(timestamp)

            self.publish_tf_map(timestamp)

    # Publish markers
    def publish_point(self, p, timestamp):
        if p is not None:
            m = Marker()
            m.header.frame_id = 'world_ned'
            m.header.stamp = timestamp
            m.ns = 'point'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = p[0]
            m.pose.position.y = p[1]
            m.pose.position.z = 0.0
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            self.point_marker_pub.publish(m)

    # Publish Filter results
    def odom_path_pub(self, timestamp):
        # Transform theta from euler to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, float((self.xk[2, 0])))  # Convert euler angles to quaternion

        # Publish predicted odom
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "turtlebot/kobuki/predicted_base_footprint"


        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]

        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.pose.covariance = list(np.array([[self.Pk[0, 0], self.Pk[0, 1], 0, 0, 0, self.Pk[0, 2]],
                                [self.Pk[1, 0], self.Pk[1,1], 0, 0, 0, self.Pk[1, 2]],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [self.Pk[2, 0], self.Pk[2, 1], 0, 0, 0, self.Pk[2, 2]]]).flatten())

        # odom.twist.twist.linear.x = self.v
        # odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        tf.TransformBroadcaster().sendTransform((float(self.xk[0, 0]), float(self.xk[1, 0]), 0.0), quaternion, timestamp, odom.child_frame_id, odom.header.frame_id)

    def publish_tf_map(self, timestamp):
        x_map = self.x_map.copy()

        # Define the translation and rotation for the inverse TF (base_footprint to world)
        translation = (x_map[0], x_map[1], 0) # Set the x, y, z coordinates

        quaternion = tf.transformations.quaternion_from_euler(0, 0, x_map[2])  # Convert euler angles to quaternion
        rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        
        # Publish the inverse TF from world to base_footprint
        tf.TransformBroadcaster().sendTransform(
            translation,
            rotation,
            timestamp,
            "turtlebot/kobuki/base_footprint",
            "map"
        )

    def spin(self):
        pass

if __name__ == '__main__':
    rospy.init_node('EKF_node')
    node = EKF('/turtlebot/joint_states')	
    
    rate = rospy.Rate(odom_freq)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()