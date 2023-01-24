import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import CameraInfo, Image


# https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


def move_camera_pos(x, y, z, pitch=0, name="camera"):
    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    msg = ModelState()
    msg.model_name = name
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    # convert pitch to quaternion
    qx, qy, qz, qw = get_quaternion_from_euler(0, pitch, 0)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    set_state(msg)


def get_camera_info(topic="/camera/color/camera_info"):
    return rospy.wait_for_message("/camera/color/camera_info", CameraInfo)


def get_model_state(name="camera"):
    rospy.wait_for_service("/gazebo/get_model_state")
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    camera_state = get_model_state(name, "world")
    return camera_state


def get_camera_image(topic):
    bridge = CvBridge()
    msg = rospy.wait_for_message(topic, Image)
    rgb = bridge.imgmsg_to_cv2(msg, msg.encoding)
    return rgb


def get_camera_image_rgb(topic="/camera/color/image_raw"):
    return get_camera_image(topic)


def get_camera_image_depth(topic="/camera/aligned_depth_to_color/image_raw"):
    return get_camera_image(topic)
