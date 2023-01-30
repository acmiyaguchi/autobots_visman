from typing import List, Optional, Tuple, Union

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
    x: float,
    y: float,
    z: float,
    roll: float = 0,
    pitch: float = 0,
    yaw: float = 0,
    name: str = "camera",
) -> None:
    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    msg = ModelState()
    msg.model_name = name
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    # convert pitch to quaternion
    qx, qy, qz, qw = get_quaternion_from_euler(roll, pitch, yaw)
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


def _normalize(x, dims=3):
    """
    Normalization of coordinates (centroid to the origin and mean distance of sqrt(2 or 3).

    See: https://github.com/acvictor/DLT/blob/master/DLT.py
    """

    m, s = np.mean(x, 0), np.std(x)

    T = np.linalg.inv(
        np.array(
            [
                [s, 0, 0, m[0]],
                [0, s, 0, m[1]],
                [0, 0, s, m[2]],
                [0, 0, 0, 1],
            ]
        )
        if dims == 3
        else np.array(
            [
                [s, 0, m[0]],
                [0, s, m[1]],
                [0, 0, 1],
            ]
        )
    )

    x = T @ np.vstack([x.T, np.ones(x.shape[0])])
    return T, x[:dims].T


def compute_dlt(xyz: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """Perform the direct linear transformation (DLT) between two sets of
    points.

    We construct a matrix A, and then solve for Ap = 0 using SVD. The smallest
    right singular vector of A is the solution to Ap = 0, and we reshape it to a
    3x4 matrix.

    See: https://github.com/acvictor/DLT/blob/master/DLT.py
    """

    T_xyz, xyz = _normalize(xyz, dims=3)
    T_uv, uv = _normalize(uv, dims=2)

    A = []
    for src, dst in zip(xyz.tolist(), uv.tolist()):
        X = np.array(src + [1])
        x, y = dst
        A += [
            np.hstack([X, np.zeros(4), -x * X]),
            np.hstack([np.zeros(4), X, -y * X]),
        ]
    A = np.array(A)

    _, _, V = np.linalg.svd(A)
    # take the smallest singular value
    L = V[-1] / V[-1, -1]
    P = L.reshape(3, 4)

    # denormalize
    P = np.linalg.pinv(T_uv) @ P @ T_xyz
    P = P / P[-1, -1]

    return P


def draw_aruco_grid(
    rows: int, cols: int, ids: Optional[List[int]], figsize: Tuple[int, int] = (10, 10)
):
    """Draw a grid of aruco markers. Optionally apply a list of ids."""
    if not ids:
        ids = list(range(rows * cols))
    if ids and len(ids) != rows * cols:
        raise ValueError("Number of ids must match number of markers")

    # https://docs.opencv.org/3.4/d9/d6a/group__aruco.html#gac84398a9ed9dd01306592dd616c2c975
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)

    # 3x3 subplot
    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    for id, ax in zip(ids, axes.flatten()):
        img = cv.aruco.drawMarker(aruco_dict, id, 200)
        ax.imshow(img, cmap="gray", interpolation="nearest")
        ax.axis("off")


def extract_aruco_tags(rgb, dictionary=cv.aruco.DICT_4X4_50):
    gray = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.Dictionary_get(dictionary)
    parameters = cv.aruco.DetectorParameters_create()
    corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = cv.aruco.drawDetectedMarkers(rgb.copy(), corners, ids)
    return frame_markers, corners, ids


def draw_aruco_tags(frame_markers, corners, ids):
    plt.imshow(frame_markers)
    for i in range(len(ids)):
        c = corners[i][0]
        plt.scatter(
            [c[:, 0].mean()], [c[:, 1].mean()], s=5, label="id={0}".format(ids[i])
        )
    plt.axis("off")


def map_id_to_world_coords(
    border: int = 10, width: int = 175, total: int = 400, side_length=0.2
) -> np.ndarray:
    """Map the aruco tag ids to world coordinates, for our specific calibration box."""
    b, w = border, width
    rel_coords = []
    for x in [b + w / 2, b + w + 35 + w / 2]:
        for y in [b + w / 2, b + w + 35 + w / 2]:
            rel_coords.append((x / total, y / total))
    rel_coords = np.array(rel_coords)

    # let's translate the coordinates so they center around 0
    rel_coords -= 0.5

    # now scale these so they are in the range [-0.1, 0.1] corresponding to a
    # unit cube scaled by 1/5
    rel_coords *= side_length

    # now we have to map these to each of the sides
    ones = np.ones((4, 1))
    zeros = np.zeros((4, 1))

    # add a third dimension that's all 0.1
    top = np.hstack([rel_coords, ones * side_length])

    # the bottom is the same as the top, but mirrored
    # NOTE: honestly, not sure about this one, but it doesn't particularly
    # matter because we're not using the bottom side
    bottom = np.hstack([rel_coords[[1, 0, 3, 2]], zeros])

    # the front side moves the x position to -0.1, which means that our relative
    # coordinates are now the y and z coordinates.

    y_z_off = np.hstack([zeros, ones * 0.5 * side_length])

    front = np.hstack([ones * 0.5 * -side_length, rel_coords[[1, 3, 0, 2]] + y_z_off])

    # the back side is the same as the front side, but mirrored
    back = np.hstack([ones * 0.5 * side_length, rel_coords[[3, 1, 2, 0]] + y_z_off])

    # the left side is the same as the back side, but rotated 90 degrees
    # this means we can likely just transpose dimensions of the front side
    left = front[:, [1, 0, 2]]

    # the right side is the same as the left side, but mirrored
    right = back[:, [1, 0, 2]]

    # now we can combine all of these into a single array
    sides = np.vstack([top, bottom, front, back, left, right])
    return sides
