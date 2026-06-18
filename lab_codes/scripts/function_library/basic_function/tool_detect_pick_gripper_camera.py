#!/usr/bin/env python
# \rewriter  Zeqiang Zhu(Tony) zzhu488@aucklanduni.ac.nz
# \date    2025-04
import sys
import rospy
import time
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from yolo_msgs.msg import yolo
from .robot_move import UR5eRobot


class AzureKinectGripperCamera:
    def __init__(self):
        self.listener = rospy.Subscriber("/yolov5", yolo, self._update_object_stat, queue_size=10)
        self._detect = yolo()
        self.robot = UR5eRobot() 


    def _update_object_stat(self, stat):
        self._detect = stat
    
    def get_detect(self):
        return self._detect
 
    def robot_pick_gripper_camera(self, height, type):
        count = 0
        last_tx, last_ty, last_rot = 0, 0, 0       
            # only move when count > 6 (object stable for 3 seconds)
        while count < 7:
            detection = self.get_detect()
            if count == 0:
                if type == "unpacked":
                    last_tx = detection.unpacked_tx / 1000
                    last_ty = detection.unpacked_ty / 1000
                    last_rot = detection.unpacked_rot
                elif type == "packed":
                    last_tx = detection.packed_tx / 1000
                    last_ty = detection.packed_ty / 1000
                    last_rot = detection.packed_rot
                else:
                    raise ValueError("The object type is not supported")

            if type == "unpacked":
                dtx = detection.unpacked_tx / 1000
                dty = detection.unpacked_ty / 1000
                drot = detection.unpacked_rot
            elif type == "packed":
                dtx = detection.packed_tx / 1000
                dty = detection.packed_ty / 1000
                drot = detection.packed_rot
            else:
                raise ValueError("The object type is not supported")

            if drot != -100:
                if np.abs(dtx -last_tx) < 0.01 and np.abs(dty-last_ty) < 0.01 and np.abs(drot - last_rot) < 0.05:
                    last_tx = dtx
                    last_ty = dty
                    last_rot = drot
                    current_pose = self.robot.get_actual_pose()
                    diag = np.sqrt((current_pose[0] + dtx)**2 + (current_pose[1] + dty)**2)
                    # object is too far or may collide with table
                    if diag > 0.92:
                        count = 0
                        rospy.loginfo("Object is beyond robot's reach")
                    else:
                        # all check pass, plus count
                        count += 1
                        if count == 3:
                            rospy.loginfo("Object stable, confirming pose...")
                else:
                    count = 0
                    rospy.loginfo("Object is not stable")
            else:
                count = 0
                rospy.loginfo("Object is not stable")

            time.sleep(0.5)

        picking_list = [] # go to picking position
        cp = self.robot.get_actual_pose()
        cj = self.robot.get_joint_pose()
        cj_conv = self.robot.joint_to_cart([cj[0], cj[1], cj[2], cj[3], cj[4], cj[5] + last_rot])
        picking_list.append([cp[0]+ last_tx, cp[1] + last_ty, height, cj_conv[3], cj_conv[4], cj_conv[5]])
        self.robot.execute_cartesian_trajectory(picking_list)
