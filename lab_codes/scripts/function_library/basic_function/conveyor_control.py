#!/usr/bin/env python
# \rewriter  Zeqiang Zhu(Tony) zzhu488@aucklanduni.ac.nz
# \date    2025-04
import sys
import rospy
import time
from vention_conveyor_msgs.msg import ConveyorCmd, ConveyorStat
"""
It is used to control the conveyor belt.
Don’t change them!!!

Example usage:
from function_library.basis_function.conveyor_control import ConveyorBelt
# Initialize the conveyor belt
conveyor = ConveyorBelt()
# Move the conveyor to a specific position with speed and acceleration
conveyor.conveyor_move(speed=100.0, acceleration=100.0, position=270.0)
conveyor.go_home()
"""


class ConveyorBelt:
    
    def __init__(self):
        self._cmd = ConveyorCmd()
        self._cmd.acceleration = 100.0
        self._cmd.speed = 100.0
        self._cmd.desired_position = 0.0
        self._stat = ConveyorStat()
        self._r = rospy.Rate(1)

        rospy.Subscriber("/conveyor/stat", ConveyorStat, self._update_conveyor_stat, queue_size=10)
        self._conveyor_pub = rospy.Publisher('/conveyor/cmd', ConveyorCmd, queue_size=10)

    def _update_conveyor_stat(self, stat):
        self._stat = stat

    def get_coveyor_stat(self):
        return self._stat
    
def conveyor_move(self, speed=None, acceleration=None, position=None):
    """
    Configure the conveyor belt by setting speed, acceleration, and position.
    :param speed: Speed of the conveyor in mm/s.
    :param acceleration: Acceleration of the conveyor in mm/s^2.
    :param position: Desired position of the conveyor in mm.
    """
    if speed is not None:
        self._cmd.speed = speed
        rospy.loginfo("Conveyor speed set to " + str(self._cmd.speed) + "mm/s.")
    
    if acceleration is not None:
        self._cmd.acceleration = acceleration
        rospy.loginfo("Conveyor acceleration set to " + str(self._cmd.acceleration) + "mm/s^2.")
    
    if position is not None:
        self._cmd.desired_position = position
        rospy.loginfo("Conveyor position set to " + str(self._cmd.desired_position) + "mm.")
    
    self._conveyor_pub.publish(self._cmd)
    self._r.sleep()
    
    def go_home(self):
        self.configure_conveyor(100.0,100.0,0.0)
