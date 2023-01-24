#!/usr/bin/env python
"""A subscriber application to the realsense topic.

Source: https://github.com/ivalab/aruco_tag_saver_ros
"""
import time
from argparse import ArgumentParser

import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def plot(rgb, depth):
    fig, axes = plt.subplots(1, 2, figsize=(9, 6))

    ax = axes.flat[0]
    ax.imshow(rgb)
    ax.set_title("color image")

    ax = axes.flat[1]
    ax.imshow(depth)
    ax.set_title("depth image")

    plt.show()


def main():
    # NOTE: roslaunch passes arguments like __name and __logs that we need to ignore
    parser = ArgumentParser()
    parser.add_argument(
        "--sleep", default=0, type=int, help="time to sleep before plotting"
    )
    args, _ = parser.parse_known_args()

    print(f"sleeping {args.sleep} seconds")
    time.sleep(args.sleep)

    rospy.init_node("realsense_subscriber")
    bridge = CvBridge()

    msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    rgb = bridge.imgmsg_to_cv2(msg, msg.encoding)

    msg = rospy.wait_for_message("/camera/depth/image_raw", Image)
    depth = bridge.imgmsg_to_cv2(msg, msg.encoding)

    plot(rgb, depth)


if __name__ == "__main__":
    main()
