#!/usr/bin/env python
#
# \author  Rui Zhou rzho774@aucklanduni.ac.nz
# \date    2024-11-01

import rospy
from function_library.basis_function.sim.robot_move_sim import UR5eRobot_sim
from function_library.basis_function.sim.gripper_control_sim import Robotiq85Gripper_sim


"Run this code to test the moveit in the simulation environment"
"If you want use the camera for the simulation, please the sim.camera_sim file "
"Attention: WE HAVE TWO CAMERAS"
"Please double check the name for the camera is what you want to use, for the top camera please subscribe the azure_kinect_camera_1 ropic and for the camera mount on the gripper please subscribe the azure_kinect_camera and chanege the publish name in the camera_sim"

def main():
    rospy.init_node('ur5e_move_sim')
    robot = UR5eRobot_sim()
    gripper = Robotiq85Gripper_sim()

    try:
        robot.go_home()
        
        object_list = []
        object_list.append([0.5, 0.597, 1.2 + 0.15, 0.200, 2.231, -2.216])
        object_list.append([0.6, 0.597, 1.2, 0.200, 2.231, -2.216])
        robot.execute_cartesian_trajectory(object_list)
        
        gripper.close()
        rospy.sleep(1)
        
        target_list = []
        target_list.append([0.2, 0.2,1.2, 0.200, 2.231, -2.216])
        robot.execute_cartesian_trajectory(target_list)
        
        gripper.open()

    except rospy.ROSInterruptException:
        rospy.logerr('ROS node interrupted.')

if __name__ == '__main__':
    main()

