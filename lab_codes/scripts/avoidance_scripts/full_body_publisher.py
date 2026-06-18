"""
Author: Dipesh Patel (dpat353@aucklanduni.ac.nz) and Nathan Phu (npu995@aucklanduni.ac.nz)
Date: 1/05/2025
Description: Transforms and processes body tracking data from "body_tracking_data" topic and publishes
processed body tracking data to "moving sphere" topic.
    - Ensure new marker position is taken with repect to the base_link frame rather than the depth_camera frame
    - Creates linkages between each marker point point through adding cyclinders
    - Removes spheres representing right hand and right thumb, if ignore_hand (set by ROS_Callbacks script) topic 
    returns true. This ensures proximity detection node avoids checking hand distance against robot links to reduce interference
    when handing over tool.
"""

import rospy
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

# IDs of all upper body points
body_points = {
    2,  # joint spine chest
    3,  # joint _neck
    4,  # joint_clavicle left
    5,  # joint shoulder left
    6,  # joint elbow left
    7,  # joint wrist left
    8,  # joint hand left
    # 9, #joint hand tip left
    10,  # joint thumb left
    11,  # joint clavicle right
    12,  # joint shoulder right
    13,  # joint elbow right
    14,  # joint wrist right
    15,  # joint hand right
    # 16, #joint handtip right
    17,  # joint hand thumb right
    26,  # joint head
}


# IDs for left arm body points
chain_left_arm = {
    4,
    5,
    6,
    7,
    8,
    10,
}

# IDs for left arm body points
chain_right_arm = {
    11,
    12,
    13,
    14,
    15,
    17
}

# IDs for main body points
chain_middle = {
    2,
    3,
    26,

}


# Ignore hand flag
ignore_hand = False


def extract_sphere_data(marker_array):
    """
    Extracts all marker 3D positions and radius from Marker array ros message, and returns in array. 

    Args:
        marker_array (MarkerArray): The marker array representing a body.

    Returns:
        spheres (array(array)): Overall array with arrays of marker positions and radius

    """
    spheres = []
    for marker in marker_array.markers:
        if marker.type == Marker.SPHERE:
            x, y, z = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
            radius = marker.scale.x / 2
            spheres.append((x, y, z, radius))
    return spheres


def get_orientation(start, end):
    """
    Retrieve orientation of line segmenent between two defined points. 

    Args:
        start (array): x,y and z representing start point
        end (array): x,y and z representing end point
    """
    direction = np.array(end) - np.array(start)
    norm_direction = direction / np.linalg.norm(direction)

    z_axis = np.array([0, 0, 1])  # Cylinder default orientation along Z-axis
    axis = np.cross(z_axis, norm_direction)
    angle = math.acos(np.dot(z_axis, norm_direction))

    # Quaternion representing rotation around `axis` by `angle`
    qx = axis[0] * math.sin(angle / 2)
    qy = axis[1] * math.sin(angle / 2)
    qz = axis[2] * math.sin(angle / 2)
    qw = math.cos(angle / 2)
    return (qx, qy, qz, qw)


def add_cylinder_link(bodyseg, new_marker_array, id_offset):
    """
    Adds cylindrical link markers between body segment points and adds to new_marker_array

    Args:
    bodyseg (array): The array of marker coordinates corresponding to the passed in body segment
    new_marker_array (MarkerArray): New marker array to append cylinder link markers to
    id_offset (int): An integer to offset marker ids by for current body chain
    """
    for i in range(len(bodyseg) - 1):
        start = bodyseg[i]
        end = bodyseg[i + 1]

        # Compute midpoint
        mid_point = [(start[0] + end[0]) / 2, (start[1] +
                                               end[1]) / 2, (start[2] + end[2]) / 2]
        # Compute length of the cylinder
        length = math.sqrt(
            (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2 + (end[2] - start[2]) ** 2)

        # Get orientation quaternion
        orientation = get_orientation(start, end)

        # Create cylinder marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = Marker.CYLINDER
        marker.lifetime = rospy.Duration(0.5)
        marker.id =((len(bodyseg) + i + 200) * 2) + id_offset
        marker.pose.position.x = mid_point[0]
        marker.pose.position.y = mid_point[1]
        marker.pose.position.z = mid_point[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = length
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        new_marker_array.markers.append(marker)


def joint_coordinates_callback(data):
    """
    Callback function to process body tracking data from markey array topic, into only upper body points with
    connect with line segements

    Args:
        data (MarkerArray): markerarray message from body tracking topic
        end (array): x,y and z representing end point

    Raises:
        tf2_rosException: Exception raised from transforms in ROS not being setup correctly

    """
    global ignore_hand
    # Holds the transformed marker array
    new_marker_array = MarkerArray()
    if data.markers:
        # Variables to store marker positions of body segments
        left_arm_positions = []
        right_arm_positions = []
        middle_positions = []

        # Iterates through each marker, only extracting the upper body markers.
        # Does not extract hand markers if ignore_hand is true
        for original_marker in data.markers:
            if ((original_marker.id % 100) in body_points):
                if not (ignore_hand and ((original_marker.id % 100 == 17) or (original_marker.id % 100 == 15))):

                    # Create a PointStamped message for the current marker point
                    input_point = PointStamped()
                    input_point.header.frame_id = 'depth_camera_link'
                    input_point.header.stamp = rospy.Time.now()
                    input_point.point.x = original_marker.pose.position.x
                    input_point.point.y = original_marker.pose.position.y
                    input_point.point.z = original_marker.pose.position.z
                    try:
                        # Transforms the current marker to be with respect to the base_link frame of the UR5e
                        transform = tf_buffer.lookup_transform(
                            'base_link', input_point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                        transformed_point = tf2_geometry_msgs.do_transform_point(
                            input_point, transform)

                        if ((original_marker.id % 100) in chain_left_arm):
                            left_arm_positions.append(
                                [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

                        if ((original_marker.id % 100) in chain_right_arm):
                            right_arm_positions.append(
                                [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

                        if ((original_marker.id % 100) in chain_middle):
                            middle_positions.append(
                                [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

                        # New marker message to represent current transformed marker
                        marker = Marker()
                        marker.header.frame_id = 'base_link'
                        marker.header.stamp = rospy.Time.now()
                        marker.ns = "basic_shapes"
                        marker.id = (original_marker.id % 100)
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.lifetime = rospy.Duration(0.5)
                        marker.pose.position.x = transformed_point.point.x
                        marker.pose.position.y = transformed_point.point.y
                        marker.pose.position.z = transformed_point.point.z
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 0.05
                        marker.scale.y = 0.05
                        marker.scale.z = 0.05
                        marker.color.a = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        new_marker_array.markers.append(marker)

                    except (tf2_ros.TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        rospy.logerr(f"Transform error: {e}")

        # Adds cylinder links for each body segments
        add_cylinder_link(left_arm_positions, new_marker_array,1000)
        add_cylinder_link(right_arm_positions, new_marker_array,2000)
        add_cylinder_link(middle_positions, new_marker_array,3000)

    sphere_publisher.publish(new_marker_array)


def listener():
    """
    Function part of the main loop to listen to the ROS transform tree
    """
    global tf_buffer, tf_listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.spin()


def ih_cb(data):
    """
    Retrieve ignore_hand flag status and stores in gloable variable
    Args:
    data (bool): True if main script want right hand to be ignored for collision avoidance
    """
    global ignore_hand
    ignore_hand = data.data


if __name__ == '__main__':
    rospy.init_node('body_tracking_transform', anonymous=True)
    rospy.Subscriber('body_tracking_data', MarkerArray,
                     joint_coordinates_callback)
    rospy.Subscriber('ignore_hand', Bool, ih_cb)
    tf_buffer = None
    tf_listener = None
    sphere_publisher = rospy.Publisher(
        '/moving_sphere', MarkerArray, queue_size=5)

    listener()
