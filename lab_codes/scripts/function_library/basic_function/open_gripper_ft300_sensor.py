#!/usr/bin/env python
# \author  Zeqiang Zhu(Tony) zzhu488@aucklanduni.ac.nz
# \date    2025-04
import rospy
import time
from threading import Lock
from .gripper_control import Robotiq85Gripper
"""
It is used to read the data from the ft300 sensor and will open the gripper when enough force is detected.
Don’t change them!!!

Example usage:
from function_library.basis_function.open_gripper_ft300_sensor import FT300SensorHandler
from geometry_msgs.msg import WrenchStamped

# Initialize the gripper
gripper = Robotiq85Gripper()
# Initialize the FT300 sensor handler
ft300_handler = FT300SensorHandler()
# Receiving data from the FT300 sensor
sensor_control_thread = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft300s_callback)
"""

class FT300SensorHandler:
    def __init__(self):
        self.data_lock = Lock()  
        self.last_gripper_action_time = 0.0  
        self.torque_z = 0.0  
        self.torque_z_count = 0  
        self.flag_open_gripper = False 
        self.gripper = Robotiq85Gripper()
        self.gripper_status = 0

    def ft300s_callback(self, data):
        """
        Callback function to handle FT300 sensor data.
        Opens the gripper if the torque threshold is exceeded.
        """
        with self.data_lock:
            try:
                current_time = time.time()
                # Ensure at least 0.1 seconds between actions
                if current_time - self.last_gripper_action_time > 0.1:
                    torque = data.wrench.torque
                    self.torque_z = float(f"{torque.z:.2f}")  # Format torque.z to 2 decimal places

                    # Check if torque exceeds the threshold
                    if self.torque_z <= -0.10 or self.torque_z >= 0.20:
                        rospy.loginfo(f"Torque_z: ({torque.z:.2f})")
                        self.torque_z_count += 1  # Increment the counter if threshold is exceeded

                        # If the threshold is exceeded 5 times consecutively, open the gripper
                        if self.torque_z_count >= 5:
                            rospy.loginfo("Torque threshold exceeded. Opening gripper...")
                            self.flag_open_gripper = True
                            self.torque_z = 0.0  # Reset torque value
                            self.gripper.open()  # Directly call gripper.open()
                            self.torque_z_count = 0  # Reset the counter
                            # Reset gripper status
                            self.gripper_status = 0
                    else:
                        # Reset the counter if the threshold is not exceeded
                        self.torque_z_count = 0
                        self.gripper_status = 0

                    # Update the last action time
                    self.last_gripper_action_time = current_time
            except Exception as e:
                rospy.logerr(f"Error in ft300s_callback: {e}")
