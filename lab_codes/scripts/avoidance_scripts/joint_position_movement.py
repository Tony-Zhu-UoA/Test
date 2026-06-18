#!/usr/bin/env python
"""
Author: Dipesh Patel (dpat353@aucklanduni.ac.nz) and Nathan Phu (npu995@aucklanduni.ac.nz)
Date: 1/05/2025
Description: Runs the live avoidance functionality UR5e to avoid both workers and initalised boundary obstacles.
            - Utlised proximity checking data to avoid closes obstacle using vector in "obs_dist" topic
            - Mimics a trajectory controller, accepting new goals from "required_pos" topic
"""

import sys
import rospy
import csv
import os
import geometry_msgs.msg as geometry_msgs
import numpy as np
import rospkg 

# Get the path of the current ROS package
rospack = rospkg.RosPack()
package_path = rospack.get_path('lab_codes')  # Replace 'cobot' with your package name

# Add the 'avoidance_scripts' folder to the Python path
sys.path.append(package_path + '/scripts/avoidance_scripts')

from sensor_msgs.msg import JointState
from avoidance_functions.UR5EJacobian import UR5EJacobian as urj
from controller_manager_msgs.srv import SwitchControllerRequest
from controller_manager_msgs.srv import LoadControllerRequest
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from ur_ikfast import ur_kinematics
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8MultiArray
from tf.transformations import euler_matrix, euler_from_quaternion
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

PI = 3.14159

# Open CSV files to write end effector and obstacle positions, for data collection
ee_csv_filename = "end_effector_positions.csv"
obstacle_csv_filename = "obstacle_positions.csv"
ee_csv_file = open(ee_csv_filename, mode='w', newline='')
obstacle_csv_file = open(obstacle_csv_filename, mode='w', newline='')
ee_csv_writer = csv.writer(ee_csv_file)
obstacle_csv_writer = csv.writer(obstacle_csv_file)
ee_csv_writer.writerow(['X', 'Y', 'Z'])  # Write header for end effector
obstacle_csv_writer.writerow(['X', 'Y', 'Z'])  # Write header for obstacle

# Accetable Boundary limits set for safe operation
min_x = 0.19
max_x = 0.8
min_y = -0.61
max_y = 0.43  # 0.6
min_z = 0.132
max_z = 0.6


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def create_transform_matrix(xyz, rpy):
    """
    Create a 4x4 homogeneous transformation matrix from XYZ translation and RPY rotation.

    Args:
    xyz (list): A tuple or list of translation (x, y, z)
    rpy (list): A tuple or list of rotation (roll, pitch, yaw)

    Returns:
    2D Array: A 4x4 homogeneous transformation matrix
    """
    tx, ty, tz = xyz

    if (len(rpy) == 4):
        rpy = euler_from_quaternion(rpy)

    roll, pitch, yaw = rpy

    # Create rotation matrix from RPY
    R = euler_matrix(roll, pitch, yaw)

    # Initialize a 4x4 identity matrix
    T = np.eye(4)

    # Set the top-left 3x3 submatrix to the rotation matrix
    T[0:3, 0:3] = R[0:3, 0:3]

    # Set the top-right 3x1 submatrix to the translation vector
    T[0:3, 3] = [tx, ty, tz]

    return T


class Avoidance:
    def __init__(self):
        """
        Initialises and avoidance object instance
        """

        # Kinematics object
        self.ur5e_arm = ur_kinematics.URKinematics('ur5e')


        # Contains the current joint and end effector cartision coordinates
        self.current_pose = np.zeros(6)
        self.current_cPose = self.joint_to_cart(self.current_pose)
        self.home_pose = np.array([0.297, -0.132, 0.250, 2.231, -2.216, 0.0])
        self.requested_cPose = self.home_pose

        # Subscribers to retrieve UR5e joints states, proximity detection informations, and recieve cartesian goals.
        self.sub = rospy.Subscriber(
            "joint_states", JointState, self.callback_joint_angles)
        self.sub_obs = rospy.Subscriber(
            "/obs_dist", Float32MultiArray, self.callback_obs)
        self.sub_rp = rospy.Subscriber(
            "/required_pos", Float32MultiArray, self.callback_requested_goal)

        # Jacobian object for carrying out jacobian calculations
        self.jac = urj()

        # Variables for collision obstacle proximity information
        self.obs_nearest_joint = None
        self.obs_abs_dist = None
        self.obs_direction = None
        self.obs_boundary = False
        self.updated_joints_at_startup = False
        self.obstacle_list = []

        # Velocity dampening variables
        self.old_W = [0, 0, 0, 0, 0, 0]
        self.max_speed = 0.1
        self.time_slice = 1

        # Publisher for sending bool flag for if there is a current goal the avoidance algorithm is executing
        self.pub_ag = rospy.Publisher(
            'active_avoidance_goal', Int8MultiArray, queue_size=5)

        # Publisher for communicating with velcioty controller
        self.pub_v = rospy.Publisher(
            '/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)

        print("Initialised Avoidance Object")

    def callback_requested_goal(self, data):
        """
        Callback function to load a new cartesian goal point(Required Postion) for UR5e to traverse to

        Args:
        data (Float32MultiArray): A float array containing the cartesian point another ROS node has sent
        """
        # Loads cartesian point into avoidance object, if it is within the accetable boundary limits.
        if ((data.data[0] > min_x) and (data.data[0] < max_x) and (data.data[1] > min_y) and (data.data[1] < max_y) and (data.data[2] > min_z) and (data.data[2] < max_z)):
            self.requested_cPose = data.data
            print(data.data)
        else:
            print(
                f"invalid position passed into avoidance server: {data.data}")
            print(
                f"invalid position passed into avoidance server: {data.data}")

    def callback_obs(self, data):
        """
        Callback function to load collision obstacle informationn from proximity checker node when active

        Args:
        data (Float32MultiArray): A float multi array containing information of the nearest collision obstacle if it exists
        """
        if data.data:
            self.obstacle_list = []
            for i in range(0, int(len(data.data)/6)):
                self.obstacle_list.append([data.data[(
                    i*6) - 6], data.data[(i*6) - 5], data.data[(i*6) - 4:(i*6) - 1], data.data[(i*6) - 1]])
                # Stores the nearest joint an existing obstacle is closest to
                self.obs_nearest_joint = data.data[0]
                # Stores the absolute distance between the nearest link and existing obstacle
                self.obs_abs_dist = data.data[1]
                # Stores the 3D Direction vector between the nearest link and existing obstacle
                self.obs_direction = data.data[2:4]
                # Stores flag to indicate whether the nearest obstacle is a boundary
                self.obs_boundary = bool(data.data[-1])
        else:
            self.obstacle_list = None

    def execute_cartesian_velocity(self, speed, joint=None, link_dist=None):
        """
        Converts a cartesian velocity to a joint velocity with singularity mitigation

        Args:
        speed (np array): The cartesian velocty to be appied in terms of xyz and rpy
        joint (int): The joint to apply the cartesian velocity around
        link_dist (float): The distance away from the joint to apply the carteisan velocity at

        Returns:
        np array: The joint velocity required to achieve the cartesian velocity given the current pose.
        """

        # Retrieves the jacobian matrix for the specified joint and link_dist and for the current joint angles
        if ((joint == None) or (link_dist is None)):
            original_jac = self.jac.jacobian(self.current_pose)
            joint = 5
            link_dist = 0.0966
        else:
            original_jac = self.jac.jacobian(
                self.current_pose, joint, abs(link_dist))

        # Singularity mitigation to alter jacobian to avoid singularities
        jac = original_jac[:, 0:joint+1]
        c = np.sqrt(np.linalg.det(np.dot(np.transpose(jac), jac)))

        zeta = 0.01 / (np.tan(3.14+c) + 0.01)

        new_J = np.dot(np.linalg.inv((np.dot(np.transpose(jac), jac) +
                       (zeta*zeta*np.eye(joint+1)))), np.transpose(jac))

        joint_vel = np.dot(new_J, np.transpose(speed))
        if (joint < 5):
            joint_vel = np.append(joint_vel, np.zeros(5-joint))
        return joint_vel

    def execute_joint_position(self, joint_vel, time=0.1):
        """
        Dampens the current joint velocity to be within acceleration and velocity limits before sendind it

        Args:
        joint (np array): The joint velocities to be applied to the UR5e
        """
        # scale all the joint speeds relatively if one or more joints exceed the max speed threshold set
        # Keep it at 0.1 for testi\ng obstacle avoidance else 0.4 for actual
        threshold_speed = 0.4
        max_joint_speed = 0

        # acceleration checks and scaling
        # finds the max requsted accel from the new joint velocity
        requested_accels = (joint_vel - self.old_W)/time
        max_requested_accel = max(abs(requested_accels))

        # if the acceleration exeeds the current limit, scales the acceerations and alterantively, changes the velocity
        if max_requested_accel > 0.6:  # 0.5
            accel_scaling_factor = 0.6/max_requested_accel
            new_accels = requested_accels * accel_scaling_factor
            joint_vel = (new_accels*time) + self.old_W

        for jv, val in enumerate(joint_vel):
            if (abs(val)) > max_joint_speed:
                max_joint_speed = abs(val)

        # print(f"sent speed is {joint_vel}")
        if (max_joint_speed > threshold_speed):
            # print("joint speed at limit")
            scaling_factor = threshold_speed / max_joint_speed
            joint_vel = joint_vel * scaling_factor

        self.old_W = joint_vel
        print(joint_vel)
        traj = Float64MultiArray()
        traj.data = joint_vel
        self.pub_v.publish(traj)

    def go_home(self):
        """
        Returns the UR5e to the home position
        """
        self.potential_field(self.home_pose)

    def callback_joint_angles(self, data):
        """
        Callback function to retrieve the current UR5e joint angles from the ROS server
        """
        if (self.updated_joints_at_startup is False):
            self.updated_joints_at_startup = True
        self.current_pose = np.array(
            [data.position[2], data.position[1], data.position[0], data.position[3], data.position[4], data.position[5]])

    def _vec2quat(self, x, y, z):
        """
        Converts rotation vector angles to quarterions

        Args:
        x,y,z (float): Rotation vector angles

        Returns:
        list: Quarterion represenation
        """
        # Create a rotation object from rotation vector in radians
        rot = Rotation.from_rotvec([x, y, z])
        # Convert to quaternions and print
        rq = rot.as_quat()
        return [rq[0], rq[1], rq[2], rq[3]]

    def _quat2vec(self, x, y, z, w):
        """
        Converts quarterions to ration vector angles

        Args:
        x,y,z,w (float): Quaterion angles

        Returns:
        list: Roation vector represenation
        """
        # Create a rotation object from rotation vector in radians
        rot = Rotation.from_quat([x, y, z, w])
        # Convert to quaternions and print

        rr = rot.as_euler('xyz', degrees=False)
        # rr = rot.as_mrp()
        return rr[0], rr[1], rr[2]

    def joint_to_cart(self, pose):
        """
        Converts UR5e joint angles into a cartesian pose

        Args:
        pose (np array or list): UR5e joint angles

        Returns:
        list: Cartesian pose of UR5e
        """
        # convert joint pose(in radians) to cartisian point
        pose_quat = self.ur5e_arm.forward(pose)
        # return pose_quat
        vec = self._quat2vec(
            pose_quat[3], pose_quat[4], pose_quat[5], pose_quat[6])
        # return vec
        return [pose_quat[0], pose_quat[1], pose_quat[2], vec[0], vec[1], vec[2]]

    def potential_field(self, position):
        """
        Potential field method to drive UR5e end effector to goal while avoiding nearby obstacles

        Args:
        position (np array or list): THe goal position
        """
        # Potential Field Algorithm Parameters

        # Maximum absolute distance the nearest obtacle and boundary have to be away from arm to avoid the obstacle
        threshold_distance = 0.25
        threshold_boundary_distance = 0.1

        # Attraction, obstacle repulsion and boundary repulsion gains
        k_attr = 1.5
        k_rep = 0.001
        k_rep_b = 0.002

        self.current_cPose = self.joint_to_cart(self.current_pose)
        abs_dist = np.linalg.norm(
            np.array(position[:3]) - np.array(self.current_cPose[:3]))
        rate = rospy.Rate(17)  # 20

        # If the target position is within boundaries
        if ((position[0] > min_x) and (position[0] < max_x) and (position[1] > min_y) and (position[1] < max_y) and (position[2] > min_z) and (position[2] < max_z)):
            #  Traverses to goal until absolute error is less than 0.2mm
            while abs_dist > 0.002:
                self.current_cPose = self.joint_to_cart(self.current_pose)
                self.current_cPose[0] = self.current_cPose[0] * -1
                self.current_cPose[1] = self.current_cPose[1] * -1

                # Calculate attraction cartesian velocity to apply to end effector
                abs_dist = np.linalg.norm(
                    np.array(position[:3]) - np.array(self.current_cPose[:3]))

                v = np.array(position[:3]) - np.array(self.current_cPose[:3])
                d_hat = v / np.linalg.norm(v)
                d_hat = d_hat / np.linalg.norm(d_hat)

                v_attr = k_attr * abs_dist

                if (v_attr > 0.25):
                    v_attr = 0.25

                v_repel = 0
                Vx = (v_attr * d_hat[0])
                Vy = (v_attr * d_hat[1])
                Vz = (v_attr * d_hat[2])

                w_attr = self.execute_cartesian_velocity(
                    [Vx, Vy, Vz, 0, 0, 0], 5, 0.0966)

                # Calculated repulsion cartesian velocity from proximity information
                w_body = np.zeros(6)
                w_bound = np.zeros(6)
                w_bound_end = np.zeros(6)
                w_total_repel = np.zeros(6)
                if (self.obstacle_list is not None):
                    for (joint, link_dist, r, ws_bool) in self.obstacle_list:
                        mag_r = np.linalg.norm(r)
                        if (mag_r != 0):
                            r_hat = r/mag_r

                            if ws_bool:
                                print("Near Boundary")
                                # add if no obstacles are nearby
                                if (mag_r <= threshold_boundary_distance):
                                    v_repel = k_rep_b * \
                                        (((1/mag_r) - (1/threshold_boundary_distance))
                                         * (1/(mag_r * mag_r)))
                                else:
                                    v_repel = 0
                            else:
                                if (mag_r <= threshold_distance):
                                    v_repel = k_rep * \
                                        (((1/mag_r) - (1/threshold_distance))
                                         * (1/(mag_r * mag_r)))
                                else:
                                    v_repel = 0

                                # psuedo code if no person obstacles and near boundary where goal is within, cause boundary to decay
                            # if()
                            if (v_repel > 0.3):
                                v_repel = 0.3
                            print(f"v_repel is {v_repel}")

                            Vx = -v_repel * r_hat[0]
                            Vy = -v_repel * r_hat[1]
                            Vz = v_repel * r_hat[2]

                        if (mag_r != 0):
                            w_repel = self.execute_cartesian_velocity(
                                [Vx, Vy, Vz, 0, 0, 0], int(joint), link_dist)
                        else:
                            print("Obstacle too close to arm, ignoring obstacle")

                        # Incorporates repelling action from boundary obstacle if close to boundaries
                        if ws_bool:
                            if (joint == 5):
                                w_bound_end = w_bound_end + w_repel
                            else:
                                w_bound = w_bound + w_repel

                        else:
                            w_body = w_body + w_repel

                    if (np.all(w_bound == 0) and np.all(w_body == 0)):
                        print(
                            "No obstacles nearby, ignore workspace obstacles near end effector")
                        w_total_repel = w_bound   #
                    else:
                        print(
                            "Person or other obstacles are within proximity of cobot")
                        w_total_repel = w_bound + w_bound_end + w_repel

                # Adds both attracting and repeling velocities
                w_total = w_total_repel + w_attr

                # Executes the current joint velocity
                self.execute_joint_position(w_total, 0.12)

                rate.sleep()
        else:
            print("Position is out of bounds")

    def avoidance_server(self):
        """
        Potential field method to drive UR5e end effector to goal while avoiding nearby obstacles. However the goal is
        is activiely changed by other ros nodes set in the request_Cpose variable. THe loop also remains active until
        interrupted

        """
        # Potential Field Algorithm Parameters
        # Maximum absolute distance the nearest obtacle and boundary have to be away from arm to avoid the obstacle
        threshold_distance = 0.2
        threshold_boundary_distance = 0.1

        # Attraction, obstacle repulsion and boundary repulsion gains
        k_attr = 3
        k_rep = 0.001
        k_rep_b = 0.0012

        self.current_cPose = self.joint_to_cart(self.current_pose)
        abs_dist = np.linalg.norm(
            np.array(self.requested_cPose[:3]) - np.array(self.current_cPose[:3]))
        rate = rospy.Rate(75)
        local_min_counter = 0

        # Continues to always traverse to requested goal until interrupted
        while not rospy.is_shutdown():
            self.current_cPose = self.joint_to_cart(self.current_pose)
            self.current_cPose[0] = self.current_cPose[0] * -1
            self.current_cPose[1] = self.current_cPose[1] * -1

            # Attraction cartesian velocity calculation
            abs_dist = np.linalg.norm(
                np.array(self.requested_cPose[:3]) - np.array(self.current_cPose[:3]))

            v = np.array(self.requested_cPose[:3]) - \
                np.array(self.current_cPose[:3])
            d_hat = v / np.linalg.norm(v)
            d_hat = d_hat / np.linalg.norm(d_hat)

            v_attr = k_attr * abs_dist

            if (v_attr > 0.25):
                v_attr = 0.25

            v_repel = 0
            Vx = (v_attr * d_hat[0])
            Vy = (v_attr * d_hat[1])
            Vz = (v_attr * d_hat[2])

            w_attr = self.execute_cartesian_velocity(
                [Vx, Vy, Vz, 0, 0, 0], 5, 0.0966)

            # Repulsion cartesian velocity calculation
            w_body = np.zeros(6)
            w_bound = np.zeros(6)
            w_bound_end = np.zeros(6)
            w_total_repel = np.zeros(6)
            if (self.obstacle_list is not None):
                for (joint, link_dist, r, ws_bool) in self.obstacle_list:
                    mag_r = np.linalg.norm(r)
                    if (mag_r != 0):

                        r_hat = r/mag_r

                        if ws_bool:
                            # print("Near Boundary")
                            # add if no obstacles are nearby
                            if (mag_r <= threshold_boundary_distance):
                                v_repel = k_rep_b * \
                                    (((1/mag_r) - (1/threshold_boundary_distance))
                                     * (1/(mag_r * mag_r)))
                            else:
                                v_repel = 0
                        else:
                            if (mag_r <= threshold_distance):
                                v_repel = k_rep * \
                                    (((1/mag_r) - (1/threshold_distance))
                                     * (1/(mag_r * mag_r)))
                            else:
                                v_repel = 0

                        if (v_repel > 0.3):
                            v_repel = 0.3

                        Vx = -v_repel * r_hat[0]
                        Vy = -v_repel * r_hat[1]
                        Vz = v_repel * r_hat[2]

                    if (mag_r != 0):
                        w_repel = self.execute_cartesian_velocity(
                            [Vx, Vy, Vz, 0, 0, 0], int(joint), link_dist)
                    else:
                        print("Obstacle too close to arm, ignoring obstacle")

                    if ws_bool:
                        if (joint == 5):
                            w_bound_end = w_bound_end + w_repel
                        else:
                            w_bound = w_bound + w_repel

                    else:
                        w_body = w_body + w_repel

                if (np.all(w_bound == 0) and np.all(w_body == 0)):
                    # print("No obstacles nearby, ignore workspace obstacles near end effector")
                    w_total_repel = w_bound   #
                else:
                    # print("Person or other obstacles are within proximity of cobot")
                    w_total_repel = w_bound + w_bound_end + w_repel

            w_total = w_total_repel + w_attr

            # Calculated velocity for end effector orientation correction
            error_X = 0
            if (self.current_cPose[3] < 3.14159 and self.current_cPose[3] > 0):
                error_X = -3.14159 + self.current_cPose[3]
            else:
                error_X = 3.14159 + self.current_cPose[3]

            error_Y = 0
            if (self.current_cPose[4] < 3.14159 and self.current_cPose[4] > 0):
                error_Y = 0 - self.current_cPose[4]
            else:
                error_Y = 0 - self.current_cPose[4]

            w = self.execute_cartesian_velocity(
                [0, 0, 0, error_Y, error_X, 0], 5, 0.0996)

            w_total = w_total + w

            status = Int8MultiArray()

            # Updates and broadcasts flag on whether the end effector has reached the current request goal postion. Executes
            # joint velocity if the end effector is more than 1mm away from goal position.
            if (abs_dist < 0.005):
                w_zero = np.array([0, 0, 0, 0, 0, 0])
                self.execute_joint_position(w_zero, (1/75))
                status.data.append(0)

            else:
                self.execute_joint_position(w_total, (1/75))
                status.data.append(1)

            if (((np.sum(np.abs(w_total_repel))) > 1) and (np.sum(np.abs(w_total)) < 2.5) and (abs_dist > 0.1)):
                local_min_counter = local_min_counter + 1

            else:
                local_min_counter = 0

            if (local_min_counter > 10):
                status.data.append(1)
            else:
                status.data.append(0)

            self.pub_ag.publish(status)

            rate.sleep()

    def servo(self, vx, vy, vz, wx, wy, wz, joint, link_Dist, time):
        """
        Executes a cartesian velocity for a set amount of time at the specified joint and link distance. (For testing jacobian)

        Args:
        vx, vw, vz, wx, wy, wz (floats): The cartesian velocty to be applied
        joint (int): The nearest joint to apply the cartesian velocity to 
        link_Dist (float): The distance away from the specified joint to apply the cartesian velocity to 
        time (float): The duration to apply the cartesian velocity for 
        """
        self.current_cPose = self.joint_to_cart(self.current_pose)
        self.current_cPose[0] = self.current_cPose[0] * -1
        self.current_cPose[1] = self.current_cPose[1] * -1

        w = self.execute_cartesian_velocity(
            [vx, vy, vz, wx, wy, wz], joint, link_Dist)
        self.execute_joint_position(w, time)
        print("Done Executing")
        self.current_cPose = self.joint_to_cart(self.current_pose)
        self.current_cPose[0] = self.current_cPose[0] * -1
        self.current_cPose[1] = self.current_cPose[1] * -1


def main_loop():
    robot = Avoidance()

    # Ensure joint angles are retrieved from ROS during startup
    while robot.updated_joints_at_startup is False:
        rospy.sleep(0.1)

    # Runs the avoidance server
    robot.avoidance_server()

    print("done")


if __name__ == "__main__":
    rospy.init_node("UR5e_Mover")
    main_loop()
