import math

from builtin_interfaces.msg import Time
import pytest
from raspimouse_python_driver import Odometry
from raspimouse_python_driver.devices.light_sensor import LightSensorValues
from raspimouse_python_ros2.conversions import (
    battery_to_msg,
    light_sensors_to_msg,
    odometry_to_msg,
    odometry_to_transform,
    switches_to_msg,
    yaw_to_quaternion,
)
from sensor_msgs.msg import BatteryState

STAMP = Time(sec=12, nanosec=34)
ODOM = Odometry(x=1.0, y=-2.0, theta=math.pi / 2, linear=0.15, angular=-0.5)


def test_yaw_to_quaternion():
    q = yaw_to_quaternion(math.pi / 2)
    assert (q.x, q.y, q.z, q.w) == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))


def test_light_sensors_field_mapping():
    msg = light_sensors_to_msg(LightSensorValues(right_front=1, right=2, left=3, left_front=4))
    assert (msg.forward_r, msg.right, msg.left, msg.forward_l) == (1, 2, 3, 4)


def test_switches():
    msg = switches_to_msg([False, True, False])
    assert (msg.switch0, msg.switch1, msg.switch2) == (False, True, False)


def test_odometry_msg():
    msg = odometry_to_msg(ODOM, STAMP, 'odom', 'base_footprint')
    assert msg.header.stamp == STAMP
    assert (msg.header.frame_id, msg.child_frame_id) == ('odom', 'base_footprint')
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.0, -2.0)
    assert msg.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert (msg.twist.twist.linear.x, msg.twist.twist.angular.z) == (0.15, -0.5)


def test_odometry_transform():
    msg = odometry_to_transform(ODOM, STAMP, 'odom', 'base_footprint')
    assert (msg.header.frame_id, msg.child_frame_id) == ('odom', 'base_footprint')
    assert (msg.transform.translation.x, msg.transform.translation.y) == (1.0, -2.0)
    assert msg.transform.rotation.w == pytest.approx(math.sqrt(0.5))


def test_battery():
    msg = battery_to_msg(12.2, STAMP)
    assert msg.voltage == pytest.approx(12.2)
    assert msg.present
    assert math.isnan(msg.percentage) and math.isnan(msg.current)
    assert msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
