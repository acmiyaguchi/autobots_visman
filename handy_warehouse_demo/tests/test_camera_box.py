import time

import pytest
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import Image


@pytest.fixture()
def node():
    rospy.init_node("test_camera_box", anonymous=True)


@pytest.fixture()
def bridge():
    return CvBridge()


def move_box_abs_x(pos, x=0.3):
    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    msg = ModelState()
    msg.model_name = "box"
    msg.pose.position.x = x

    set_state(msg)


@pytest.fixture(autouse=True)
def reset_state(node):
    rospy.wait_for_service("/gazebo/get_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    origin = get_state("box", None)
    yield
    move_box_abs_x(origin.pose.position.x)


def get_total_color(bridge):
    msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    assert msg
    img = bridge.imgmsg_to_cv2(msg, msg.encoding)
    # flatten (480, 640, 3)
    red, green, blue = img.reshape(-1, 3).sum(axis=0)
    return red, green, blue


def test_camera_can_see_red_box(node, bridge):
    red, green, blue = get_total_color(bridge)
    assert green == blue
    assert red > blue
    assert red > green


def test_camera_red_increases_when_box_moves_closer(node, bridge):
    reds = []
    for pos in [x / 10 for x in range(2, 8)]:
        move_box_abs_x(pos)
        red, *_ = get_total_color(bridge)
        if reds:
            assert reds[-1] - red > 0, f"failure testing {pos}"
        reds.append(red)
    print(reds)
