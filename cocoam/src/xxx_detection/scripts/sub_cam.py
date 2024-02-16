#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray,UInt16MultiArray
# from housem8_aruco.msg import Custom
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
import apriltag
from apriltag import _draw_pose
# from utils import ARUCO_DICT, aruco_display, aruco_pose_estimation


# arucoDict = arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
# arucoParams = cv2.aruco.DetectorParameters_create()

cam_matrix = [381.36,0.0,320.5,
                0.0,381.36,240.5,
                0.0,0.0,1.0]
cam_dist = [0.0,0.0,0.0,0.0,0.0]
CameraTopic = '/xxx1/camera/image_raw'
ArucoTopic = '/housem8/aruco'

marker_length = 0.2 #20cm


x = np.array(cam_matrix,dtype=np.float32)
# x = np.array(cam_matrix)
shape = (3,3)
cam_matrix = x.reshape(shape)

x = np.array(cam_dist,dtype=np.float32)
# x = np.array(cam_dist)
shape = (1,5)
cam_dist = x.reshape(shape)


class CameraSubsciber(Node):
    def __init__(self):
        super().__init__('SubCam') #cameraShow_aum = node name
        self.sub = self.create_subscription(Image,CameraTopic,self.listener_callback,10) #create subsciber

        #########################################
        # self.pub = self.create_publisher(Custom, ArucoTopic, 10)
        timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        #########################################
        self.tag_id = 0
        self.bridge = CvBridge()

    def pixel_to_worldframe(self, pixel):
        return  pixel * 0.45 / 1024

    def draw_line_marker(self, overlay, camera_params, tag_size, pose, z_sign=1):

        opoints = np.array([
            -1, -1, 0,
            1, -1, 0,
            1,  1, 0,
            -1,  1, 0,
            -1, -1, -2*z_sign,
            1, -1, -2*z_sign,
            1,  1, -2*z_sign,
            -1,  1, -2*z_sign,
        ]).reshape(-1, 1, 3) * 0.5*tag_size

        edges = np.array([
            0, 1,
            1, 2,
            2, 3,
            3, 0,
            0, 4,
            1, 5,
            2, 6,
            3, 7,
            4, 5,
            5, 6,
            6, 7,
            7, 4
        ]).reshape(-1, 2)
            
        fx, fy, cx, cy = camera_params

        K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

        rvec, _ = cv2.Rodrigues(pose[:3,:3])
        tvec = pose[:3, 3]

        dcoeffs = np.zeros(5)

        ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

        ipoints = np.round(ipoints).astype(int)
        
        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

        for i, j in edges:
            # print(ipoints)
            cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1)
    
    def listener_callback(self,img_msg):
        self.frame = self.bridge.imgmsg_to_cv2(img_msg,"bgr8") #camera frame rgb

        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) #change to gray frame
        trans_mat = np.array([  [0.0,  0.0,  1.0, 0.0],
                                [-1.0,   0.0,  0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0,  0.0,  0.0, 1.0]])
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(self.gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))
        
        for r in results:
            if r.tag_id == self.tag_id:
                M, _, _ = detector.detection_pose(detection=r, 
                                                    camera_params=(953.4061672028388, 953.4061672028388, 800.5, 400.5),
                                                    tag_size=0.45, 
                                                    z_sign=0.1)

                try:
                    self.draw_line_marker(overlay = self.frame, 
                                            camera_params = (953.4061672028388, 953.4061672028388, 800.5, 400.5), 
                                            tag_size = 0.45, 
                                            pose = M, 
                                            z_sign = 0.1)
                except:
                    print("Marker not found!!!")
                # print(M.shape)
                ppp = np.matmul(trans_mat,M)
                print(np.around(M, decimals=2))
                print("")
                print(np.around(ppp, decimals=2))
            cv2.imshow("Image", self.frame)
            cv2.waitKey(1)
        # cv2.imshow("A",self.frame)
        # cv2.waitKey(1)   

def main():
    rclpy.init()
    camera_service = CameraSubsciber()
    rclpy.spin(camera_service)
    camera_service.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()