#!/usr/bin/env python
# \rewriter  Zeqiang Zhu(Tony) zzhu488@aucklanduni.ac.nz
# \date    2025-04
import sys
import rospy
import time
from robotiq_85_msgs.msg import GripperCmd, GripperStat

"""
It is used to control the gripper. You can close the gripper with specific force.
Don’t change them!!!

Example usage:
from function_library.basis_function.gripper_control import Robotiq85Gripper
gripper = Robotiq85Gripper()

# Open the gripper
gripper.open()
# Close the gripper with the default force 100.0 N
gripper.close()
# Close the gripper with a specific force 150.0 N，Max force is 235.0 N
gripper.close_with_force(150.0)
# Check if the gripper is open
gripper.gripper_is_open()
# Check if the gripper is closed
gripper.gripper_is_closed()
"""

class Robotiq85Gripper:

    def __init__(self):
        rospy.Subscriber("/gripper/stat", GripperStat, self._update_gripper_stat, queue_size=10)
        self._gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)

        self._gripper_stat = GripperStat()
        self._gripper_cmd = GripperCmd()
        self._r = rospy.Rate(1)
        self.open()

    def _update_gripper_stat(self, stat):
        self._gripper_stat = stat

    def gripper_is_open(self):
        """
        Check if the gripper is open.
        :return: True if the gripper is open, False otherwise.
        """
        return abs(self._gripper_stat.position - 0.09) < 0.01  # Tolerance for open position

    def gripper_is_closed(self):
        """
        Check if the gripper is closed.
        :return: True if the gripper is closed, False otherwise.
        """
        return abs(self._gripper_stat.position - 0.000) < 0.01  # Tolerance for closed position

    def close(self):
        return self.close_with_force(100.0) 

    def close_with_force(self, force):
        """
        Close the gripper with a specified force.
        :param force: Force to apply when closing the gripper.
        """
        for i in range(10):
            if self._gripper_stat.is_ready:
                for j in range(10):
                    if self._gripper_stat.is_moving:
                        rospy.loginfo('Gripper is moving, retrying %2d/10' % (j))
                        self._r.sleep()
                        time.sleep(0.5)
                    else:
                        self._gripper_cmd.position = 0.0
                        self._gripper_cmd.speed = 0.02
                        self._gripper_cmd.force = force 
                        self._gripper_pub.publish(self._gripper_cmd)
                        rospy.loginfo(f"The gripper was successfully closed with force {force} N")
                        return True
                rospy.loginfo(f"The gripper failed to close with force {force} N")
                return False
            else:
                rospy.loginfo('Gripper is not ready, retrying %2d/10' % (i))
                self._r.sleep()
                time.sleep(0.5)
        rospy.loginfo(f"The gripper failed to close with force {force} N")
        return False

    def open(self):
        for i in range(10):
            if self._gripper_stat.is_ready:
                for j in range(10):
                    if self._gripper_stat.is_moving:
                        rospy.loginfo('Gripper is moving, retrying %2d/10' % (j))
                        self._r.sleep()
                        time.sleep(0.5)
                    else:
                        self._gripper_cmd.position = 0.085
                        self._gripper_cmd.speed = 0.02
                        self._gripper_cmd.force = 100.0
                        self._gripper_pub.publish(self._gripper_cmd)
                        rospy.loginfo("The gripper was successfully opened")
                        return True
                rospy.loginfo("The gripper failed to open")
                return False
            else:
                rospy.loginfo('Gripper is not ready, retrying %2d/10' % (i))
                self._r.sleep()
                time.sleep(0.5)
        rospy.loginfo("The gripper failed to open")
        return False

    def get_stat(self):
        return self._gripper_stat
