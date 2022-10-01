import time

import pytest
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


@pytest.fixture()
def node():
    rospy.init_node("test_camera_box", anonymous=True)


@pytest.fixture()
def bridge():
    return CvBridge()


def test_camera_can_see_red_box(node, bridge):
    msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    assert msg
    img = bridge.imgmsg_to_cv2(msg, msg.encoding)
    # flatten (480, 640, 3)
    red, green, blue = img.reshape(-1, 3).sum(axis=0)
    assert green == blue
    assert red > blue
    assert red > green
