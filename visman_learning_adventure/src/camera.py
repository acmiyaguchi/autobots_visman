from typing import Tuple, Union

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import CameraInfo, Image


# https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
def get_quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
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
    return (qx, qy, qz, qw)


def move_camera_pos(
    x: float, y: float, z: float, pitch: float = 0, name: str = "camera"
) -> None:
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


def get_camera_info(
    topic: str = "/camera/color/camera_info",
) -> Union[CameraInfo, None]:
    return rospy.wait_for_message(topic, CameraInfo)


def get_model_state(name: str = "camera") -> ModelState:
    rospy.wait_for_service("/gazebo/get_model_state")
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    camera_state = get_model_state(name, "world")
    return camera_state


def get_camera_image(topic: str) -> np.ndarray:
    bridge = CvBridge()
    msg = rospy.wait_for_message(topic, Image)
    if not msg:
        raise ValueError("No image received")
    return bridge.imgmsg_to_cv2(msg, msg.encoding)


def get_camera_image_rgb(topic: str = "/camera/color/image_raw") -> np.ndarray:
    return get_camera_image(topic)


def get_camera_image_depth(
    topic: str = "/camera/aligned_depth_to_color/image_raw",
) -> np.ndarray:
    return get_camera_image(topic)


def _compute_depth_from_corners(depth: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """Given a depth image and corners, compute the depths of the corners
    return a new array with the depth attached."""
    depths = []
    for corner in corners.tolist():
        x, y = corner
        # get all 4 points, and then do a weighted average
        x_floor, x_ceil = int(np.floor(x)), int(np.ceil(x))
        y_floor, y_ceil = int(np.floor(y)), int(np.ceil(y))
        values = [
            depth[y_floor, x_floor],
            depth[y_floor, x_ceil],
            depth[y_ceil, x_floor],
            depth[y_ceil, x_ceil],
        ]
        # for linear interpolation, we want to know how far we are from the
        # fractional point using euclidean distance
        weights = [
            np.linalg.norm([x - x_floor, y - y_floor]),
            np.linalg.norm([x - x_ceil, y - y_floor]),
            np.linalg.norm([x - x_floor, y - y_ceil]),
            np.linalg.norm([x - x_ceil, y - y_ceil]),
        ]
        # weighted average gives us a depth value
        depths.append(np.average(values, weights=weights))

    return np.array(np.concatenate([corners, np.array(depths)[:, None]], axis=1))


def find_chessboard_corners_depth(
    rgb: np.ndarray, depth: np.ndarray, pattern_size: Tuple[int, int]
) -> np.ndarray:
    """Given a color image and depth image, return the corners of the
    calibration chessboard."""
    gray = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY)
    _, corners = cv.findChessboardCorners(gray, pattern_size, None)
    if corners is None:
        raise ValueError("No chessboard found")
    return _compute_depth_from_corners(depth, corners.squeeze(1))


def generate_world_chessboard_corners(
    pattern_size: Tuple[int, int], length: float, height: float
) -> np.ndarray:
    """Generate the world coordinates of the chessboard corners.

    This assumes the use of the textured box.
    """
    n, m = pattern_size
    points = []
    for j in range(m):
        for i in range(n):
            points.append((i * length / n, j * length / m, height))
    return np.array(points)


def compute_chessboard_mask(
    rgb: np.ndarray, threshold: float = 100, iterations: int = 10
) -> np.ndarray:
    """Compute a mask for the chessboard."""

    # check that we're looking at either pure white or pure black
    mask = np.logical_or(
        np.abs(rgb - 255).sum(axis=2) < threshold,
        np.abs(rgb - 0).sum(axis=2) < threshold,
    ).astype(np.uint8)

    # we dilate half as much as we erode, to ensure we're in bounds of the surface
    kernel = np.ones((3, 3), np.uint8)
    mask = cv.dilate(mask, kernel, iterations=iterations // 2)
    mask = cv.erode(mask, kernel, iterations=iterations)
    return mask


def compute_valid_chessboard_points(mask: np.ndarray, depth: np.ndarray) -> np.ndarray:
    """Given a mask and depth image, compute the valid points."""
    valid = np.argwhere(mask)
    d = depth[valid[:, 0], valid[:, 1]]
    return np.array(np.hstack([valid, d[:, np.newaxis]]))


def compute_dlt(x_src: np.ndarray, x_dst: np.ndarray) -> np.ndarray:
    """Perform the direct linear transformation (DLT) between two sets of
    points.

    We construct a matrix A, and then solve for Ap = 0 using SVD. The smallest
    right singular vector of A is the solution to Ap = 0, and we reshape it to a
    3x4 matrix.
    """
    # TODO: vectorize
    A = []
    for src, dst in zip(x_src.tolist(), x_dst.tolist()):
        x, y, w = src
        X = np.hstack([dst, 1])
        A += [
            np.hstack([np.zeros(4), -w * X, y * X]),
            np.hstack([w * X, np.zeros(4), -x * X]),
        ]
    A = np.array(A)
    _, _, V = np.linalg.svd(A)
    # take the smallest singular value
    P = V[-1].reshape(3, 4)
    return P


def draw_aruco_grid(rows, cols, figsize=(10, 10)):
    # https://docs.opencv.org/3.4/d9/d6a/group__aruco.html#gac84398a9ed9dd01306592dd616c2c975
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)

    # 3x3 subplot
    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    for i, ax in enumerate(axes.flatten()):
        img = cv.aruco.drawMarker(aruco_dict, i, 200)
        ax.imshow(img, cmap="gray", interpolation="nearest")
        ax.axis("off")


def extract_aruco_tags(rgb, dictionary=cv.aruco.DICT_4X4_50):
    gray = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.Dictionary_get(dictionary)
    parameters = cv.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )
    frame_markers = cv.aruco.drawDetectedMarkers(rgb.copy(), corners, ids)
    return frame_markers, corners, ids


def draw_aruco_tags(rgb, frame_markers, corners, ids):
    plt.imshow(frame_markers)
    for i in range(len(ids)):
        c = corners[i][0]
        plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label="id={0}".format(ids[i]))
    plt.axis("off")
