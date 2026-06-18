#!/usr/bin/env python
# \rewriter  Zeqiang Zhu(Tony) zzhu488@aucklanduni.ac.nz
# \date    2025-04
import sys
import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from .robot_move import UR5eRobot
from yolo_msgs.msg import yolo
import logging
"""
It is used to detect the tool and pick it. You should train your model by YOLOv8 and add the right path when you use it.
Don’t change them!!!

Usage:
from function_library.basis_function.tool_detect_pick_camera import AzureKinectCamera
# Initialize the camera with the YOLOv8 model path
model_path = "path/to/your/yolov8_model.pt" 
camera = AzureKinectCamera(model_path)
# Detect the tool with ID 0S
camera.robot_pick_top_camera(height, tool_id)

final_coordinates = camera.robot_pick_top_camera(height, tool_id)
rospy.loginfo(f"Final picking coordinates: {final_coordinates}")
"""

class AzureKinectTopCamera:
    def __init__(self, model_path):
        """
        Initialize the Azure Kinect Camera class.
        """
        self.model = YOLO(model_path, verbose=False)
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.robot = UR5eRobot() 
        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)  
        self.rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.rgb_callback)    

        self.latest_rgb = None
        self.mask = None
    
    def rgb_callback(self, msg):
        """
        Callback function to process incoming RGB images.
        """
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")

    def image_callback(self, data):
        """
        Process an image, resize it, adjust contrast, and run YOLOv8 inference.
        """
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return
        
        # Define the target resolution (1920x1080 for 1080p)
        width = 1920
        height = 1080
        dim = (width, height)

        # Resize the image to 1080p
        resized_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        # Step 2: Adjust the contrast
        # alpha > 1.0 increases contrast, beta can adjust brightness (keep it 0 to only adjust contrast)
        alpha = 1.5  # Contrast control (1.0-3.0)
        beta = 0     # Brightness control (0-100)
        # Apply the contrast and brightness adjustment
        adjusted_image = cv2.convertScaleAbs(resized_image, alpha=alpha, beta=beta)
        # Run YOLOv8 inference
        results = self.model(resized_image)
        # print(results)
        # Visualize the results
        annotated_image = results[0].plot()

        # Display the image with cv2.imshow
        cv2.imshow('YOLOv8 Detections', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms (adjust as needed)
    
    
    def pixel_to_3d(self, u, v):
        """
        Convert pixel coordinates to 3D coordinates.
        """
        depth = 1.223  
        pixels = np.array([[u, v]], dtype=np.float32)
        K = [913.650390625, 0.0, 955.0496215820312, 0.0, 913.8749389648438, 550.6069946289062, 0.0, 0.0, 1.0]
        D = [0.18453791737556458, -2.423478603363037, 0.00043306572479195893, -0.0002455342037137598, 1.5398979187011719]
        camera_matrix = np.array(K).reshape((3, 3))
        dist_coeffs = np.array(D)
        undistorted_points = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs)
        X = depth * undistorted_points[0][0][0] - 0.02
        Y = depth * undistorted_points[0][0][1]
        return X, Y
        
    def transform_coords(self, x, y):
        """
        Transform coordinates to the robot's reference frame.
        """
        tf_x = -y + (0.766 - 0.095) 
        tf_y = -x + (1.036 - 0.78)    
        return tf_x, tf_y
    
    def detect_tool_position(self, tool_class_id):
        """
        Detect the position of a tool based on its class ID.
        """
        if self.latest_rgb is None:
            return None, None
        results = self.model(self.latest_rgb)
        for box in results[0].boxes:
            if int(box.cls[0]) == tool_class_id:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                X, Y = self.pixel_to_3d(center_x, center_y)
                return self.transform_coords(X, Y)
        return None, None


    def robot_pick_top_camera(self, height, tool_id):
        """
        Use the top camera to detect and pick up a tool.
        """
        rospy.loginfo(f"Attempting to detect tool with class_id: {tool_id}")   
        x, y = self.detect_tool_position(tool_id)
        
        if x is not None and y is not None:
            if tool_id == 4:
                x += 0.015
            rospy.loginfo(f"Tool detected at ({x}, {y}). Moving to pick it up.")
            over_list =[]
            over_list.append([x, y, height + 0.1, 2.231, -2.216, 0.0])
            self.robot.execute_cartesian_trajectory(over_list)
            over_list =[]
            over_list.append([x, y, height, 2.231, -2.216, 0.0])
            self.robot.execute_cartesian_trajectory(over_list)
            # self.robot.move_to_position([x, y, height, 2.231, -2.216, 0.0])
            time.sleep(1)

            # self.robot.move_to_position([0.7, 0.4, 0.3335, 2.231, -2.216, 0.0])
            
            #self.move_robot_home()
            return True
        return False
