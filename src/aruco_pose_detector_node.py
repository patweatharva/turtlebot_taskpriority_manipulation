#!/usr/bin/env python3

#find the position of the aruco wrt camera   
# odometry transformation, base to camera TF, camera to aruco. listen directly from the camera frame to the world 
# aruco box height is 15cm 
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import tf
import tf.transformations as tft

class ArucoDetection:
    def __init__(self):
        #SUBSCRIBERS    
        #subscribe to camera image for the aruco detection
        self.image_sub = rospy.Subscriber("/turtlebot/kobuki/realsense/color/image_color", Image, self.imageToCV) 
        
        #subscribe to camera info to get the camera matrix and distortion coefficients
        self.camera_info_sub = rospy.Subscriber("/turtlebot/kobuki/realsense/color/camera_info", CameraInfo, self.camerainfoCallback) 
        
        #subscribe to the odometry topic to use later for transformation from world NED to robot base_footprint
        self.odom_sub = rospy.Subscriber('/odom',Odometry, self.odomCallback) 
        
        #PUBLISHERS 
        #publish pose of aruco marker in the world frame 
        self.marker_pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=1) 
        
        #define aruco dictionary and parameters 
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL) 
        self.aruco_params = cv2.aruco.DetectorParameters() 
        
        #bridge object to convert the image from ros to cv2 format
        self.bridge = CvBridge() 
        
        #aruco detector object
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params) 
        
        #initialise the camera matrix and distortion coefficients for pose estimation + camera size 
        self.camera_matrix = None 
        self.dist_coeffs = None 
        self.marker_size = 0.05 
        
        #aruco box dimensions 
        self.box_width  = 0.07
        self.box_length = 0.07
        self.box_height = 0.15
        
        #listen to tf directly from the camera frame to the world frame
        self.tf_listener = tf.TransformListener()
    
    #odometry transformation  
    def odomCallback(self, odom): 
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w])
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]).reshape(-1,1)
        
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z

        # Extract the orientation quaternion
        quaternion = [ 
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ]

        # Convert the quaternion to a rotation matrix
        wTr = tft.quaternion_matrix(quaternion)  # This returns a 4x4 matrix

        # Set the translation
        wTr[0, 3] = x
        wTr[1, 3] = y
        wTr[2, 3] = z

        # Store the transformation matrix
        self.transformation_matrix = wTr
        
    #transform the obtained image to cv2 format to be used by the aruco detector
    def imageToCV(self, image): 
        """
        converts image format from ros to cv2

        Args:
            image: The image to be saved
        """
        rospy.loginfo("Image received")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough').copy()
            self.publishAruco(cv_image)
            # self.saveImage(cv_image, "aruco_image.png") #save aruco image to the file (for debugging)
        except CvBridgeError as e:
            print(e)
            
    
    def camerainfoCallback(self, data): 
        """
        Callback from camera to get the K matrix and distortion coefficients

        Args:
            data: The camera info message.
        
        """
        self.camera_matrix = np.array(data.K).reshape(3,3) #get the camera matrix from the camera info topic 
        self.dist_coeffs = np.array(data.D) 
        self.camera_info_sub.unregister() 
        
    def publishAruco(self, cv_image): 
        """
        detect the aruco marker in the image obtained from the rgb camera
        Args:
            cv_image: converted image
        """
        
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image,self.aruco_dict)    # detect the marker 
        
        if len(corners)>0: #if the aruco is detected
            print("Aruco detected", ids)
            
            #estimate the aruco pose (returns the rotation and translation vector) 
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], self.marker_size, self.camera_matrix, self.dist_coeffs)
            print("tvec", tvec)
            print("rvec", rvec, "-" * 40)
        
            #draw the axis of the aruco marker
            #self.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec[i], tvec[i], 0.01)
            
            # #publish the pose of the aruco marker in the world frame
            self.transformArucoPose(rvec, tvec)
            
        else :
            print("No Aruco detected")
            
    def transformArucoPose(self, rvec, tvec):
        
        ################### TRANSFORMATION FROM ARUCO TO CAMERA FRAME ##########################            
        
        R_ac, _ = cv2.Rodrigues(rvec)           #3x3 rotation matrix of aruco to camera (from rotation vector rvec)
        T_ac = tvec.reshape((3,))               #convert the translation vector of the aruco into a 3x1 vector
        print(R_ac, "Rotation matrix camera to aruco", "\n")            
        
        #to double check that the transformation is correct
        arucoPoint = PointStamped()
        arucoPoint.header.frame_id = 'turtlebot/kobuki/realsense_color'
        arucoPoint.point.x = T_ac[0]
        arucoPoint.point.y = T_ac[1]
        arucoPoint.point.z = T_ac[2]
        
        point = self.tf_listener.transformPoint('map', arucoPoint)
        
        print(f"camera_to_aruco: {point.point.x}, {point.point.y}, {point.point.z}")
        print(point.header.frame_id)

        #arrange translation and rotation matrices for aruco to camera transformation 
        cTa= np.eye(4)
        cTa[0:3, 0:3] = R_ac                #rotation part
        cTa[0:3, 3] = T_ac                  #translation part
        print(T_ac, "translation vector camera to aruco", "\n")
        
        # cTa[0, 3] += self.box_width/2         # Add delta_x to the x component of translation
        # cTa[1, 3] += -self.box_height/2        # Add delta_y to the y component of translation
        # cTa[2, 3] += self.box_width/2        # Add delta_z to the z component of translation
    
        ################### TRANSFORMATION FROM CAMERA TO BASE FRAME ##########################             

        #extract transformation vectors from camera to world frame tf
        (trans,rot) = self.tf_listener.lookupTransform('turtlebot/kobuki/base_footprint', 'turtlebot/kobuki/realsense_color', rospy.Time(0))
        print((trans,rot), "trans rot vectors from base to camera", "\n")
        
        #convert translation and rotation vectors to transformation matrix
        bTc = tf.transformations.quaternion_matrix(rot)
        bTc[0,3] = trans[0]; bTc[1,3] = trans[1]; bTc[2,3] = trans[2]
        print(bTc, "t matrix base to camera", "\n")
        
        ################### TRANSFORMATION FROM BASE TO ROBOT FRAME ##########################
        
        # r = self.eta
        # wTb = np.eye(4)
        # wTb = np.array(
        #         [[np.cos(r[2,0]), -np.sin(r[2,0]), 0, r[0,0]],
        #         [np.sin(r[2,0]), np.cos(r[2,0]), 0, r[1,0]],
        #         [0, 0, 1, 0],
        #         [0, 0, 0, 1]]).reshape(4,4)
        # print(wTb, "t matrix world to base", "\n")
        
        
        #to place the aruco frame at the top center of the box
        centering_pose = np.array([
                                [1, 0, 0, 0],
                                [0, 1, 0, self.box_height/2],
                                [0, 0, 1, self.box_length/2],
                                [0, 0, 0, 1]
                            ])              
        
        wTa_uncentered = self.transformation_matrix @ bTc @ cTa #world to base * base to camera * camera to aruco
        wTa = wTa_uncentered @ centering_pose #world to base * topcenter matrix  
        print(wTa, "wTa")
        
        # to get the pose.orientation of the aruco in world ned frame
        wTa_q = tft.quaternion_from_matrix(wTa)

        
        #################### POSE PUBLISHER OF ARUCO IN WORLD NED FRAME #############################
        aruco_position = PoseStamped()
        aruco_position.header.frame_id = "map"
        aruco_position.pose.position.x = wTa[0,3]
        aruco_position.pose.position.y = wTa[1,3]
        aruco_position.pose.position.z = wTa[2,3]
        aruco_position.pose.orientation.x = wTa_q[0]
        aruco_position.pose.orientation.y = wTa_q[1]
        aruco_position.pose.orientation.z = wTa_q[2]
        aruco_position.pose.orientation.w = wTa_q[3]
        self.marker_pub.publish(aruco_position)
        
        rospy.loginfo("Aruco pose published")

    #for debugging
    def saveImage(self, image, file_path):

        plt.imshow(image)
        plt.axis('off')  # Hide axes
        plt.savefig(file_path, bbox_inches='tight', pad_inches=0)  # Save image without extra whitespace
        plt.close()  # Close plot to prevent display

if __name__ == "__main__":

    print("ARUCO POSE DETECTOR NODE INITIALIZED")
    
    rospy.init_node('aruco_pose_detector_node')   
    
    node = ArucoDetection()

    rospy.spin()                              
                              
