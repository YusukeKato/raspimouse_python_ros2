"""Conversions between driver values and ROS messages (no node needed)."""

from collections.abc import Sequence
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg
from raspimouse_msgs.msg import LightSensors, Switches
from raspimouse_python_driver import Odometry
from raspimouse_python_driver.devices.light_sensor import LightSensorValues
from sensor_msgs.msg import BatteryState


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Return the quaternion of a rotation by ``yaw`` [rad] about the z axis."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def light_sensors_to_msg(values: LightSensorValues) -> LightSensors:
    """Convert light sensor values to raspimouse_msgs/LightSensors."""
    return LightSensors(forward_r=values.right_front, forward_l=values.left_front,
                        left=values.left, right=values.right)


def switches_to_msg(pressed: Sequence[bool]) -> Switches:
    """Convert the pressed states of switches 0-2 to raspimouse_msgs/Switches."""
    return Switches(switch0=bool(pressed[0]), switch1=bool(pressed[1]),
                    switch2=bool(pressed[2]))


def odometry_to_msg(odom: Odometry, stamp: Time, frame_id: str,
                    child_frame_id: str) -> OdometryMsg:
    """Convert the driver odometry to nav_msgs/Odometry."""
    msg = OdometryMsg()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = odom.x
    msg.pose.pose.position.y = odom.y
    msg.pose.pose.orientation = yaw_to_quaternion(odom.theta)
    msg.twist.twist.linear.x = odom.linear
    msg.twist.twist.angular.z = odom.angular
    return msg


def odometry_to_transform(odom: Odometry, stamp: Time, frame_id: str,
                          child_frame_id: str) -> TransformStamped:
    """Convert the driver odometry to the transform ``frame_id`` -> ``child_frame_id``."""
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = odom.x
    msg.transform.translation.y = odom.y
    msg.transform.rotation = yaw_to_quaternion(odom.theta)
    return msg


def battery_to_msg(voltage: float, stamp: Time) -> BatteryState:
    """
    Convert the battery voltage to sensor_msgs/BatteryState.

    Only the voltage is known. With an AC adapter it is the adapter voltage.
    """
    msg = BatteryState()
    msg.header.stamp = stamp
    msg.voltage = float(voltage)
    msg.present = True
    nan = float('nan')
    msg.temperature = msg.current = msg.charge = msg.capacity = nan
    msg.design_capacity = msg.percentage = nan
    msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
    return msg
