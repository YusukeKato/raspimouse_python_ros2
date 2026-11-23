import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry as OdometryMsg
import pytest
from raspimouse_msgs.msg import Leds, LightSensors, Switches
from raspimouse_python_driver import DriveConfig, RaspiMouse, RaspiMouseConfig
from raspimouse_python_driver.hal.mock import MockBackend
from raspimouse_python_ros2.parameters import declare_and_read
from raspimouse_python_ros2.raspimouse_node import RaspiMouseNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int16
from std_srvs.srv import SetBool

CONFIG = RaspiMouseConfig()


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def robot(ros):
    """Create a node on fake hardware and return (node, backend)."""
    backend = MockBackend()
    node = RaspiMouseNode(RaspiMouse(backend=backend))
    yield node, backend
    node.destroy_node()


def test_parameters_default_to_the_driver_defaults(ros):
    node = rclpy.create_node('params')
    params = declare_and_read(node)
    assert params.driver == CONFIG
    node.destroy_node()


def test_parameters_override_the_driver_config(ros):
    node = rclpy.create_node('params', parameter_overrides=[
        Parameter('wheel_diameter', value=0.05),
        Parameter('cmd_vel_timeout', value=1.0),
        Parameter('use_pulse_counters', value=False),
    ])
    params = declare_and_read(node)
    assert params.driver.drive == DriveConfig(wheel_diameter_m=0.05, command_timeout_s=1.0)
    assert not params.driver.has_pulse_counters
    node.destroy_node()


def test_motor_power_service(robot):
    node, backend = robot
    enable = backend.outputs[CONFIG.motors.enable_pin]
    response = node.on_motor_power(SetBool.Request(data=True), SetBool.Response())
    assert response.success and enable.value
    response = node.on_motor_power(SetBool.Request(data=False), SetBool.Response())
    assert response.success and not enable.value


def test_leds(robot):
    node, backend = robot
    node.on_leds(Leds(led0=True, led1=False, led2=True, led3=False))
    assert [backend.outputs[pin].value for pin in CONFIG.led_pins] == [True, False, True, False]


def test_buzzer(robot):
    node, backend = robot
    pwm = backend.software_pwms[CONFIG.buzzer.pin]
    node.on_buzzer(Int16(data=440))
    assert pwm.frequency_hz == 440
    node.on_buzzer(Int16(data=0))
    assert not pwm.running
    node.on_buzzer(Int16(data=440))
    node.on_buzzer(Int16(data=-1))   # logged, stops
    assert not pwm.running


def test_cmd_vel_sets_the_drive_target(robot):
    node, _ = robot
    msg = TwistStamped()
    msg.twist.linear.x = 0.15
    msg.twist.angular.z = 0.5
    node.on_cmd_vel(msg)
    assert node.mouse.drive.target_velocity == (0.15, 0.5)


def test_unstamped_cmd_vel(ros):
    node = RaspiMouseNode(RaspiMouse(backend=MockBackend()), parameter_overrides=[
        Parameter('use_stamped_cmd_vel', value=False)])
    msg = Twist()
    msg.linear.x = 0.1
    node.on_cmd_vel(msg)
    assert node.mouse.drive.target_velocity == (0.1, 0.0)
    node.destroy_node()


def test_device_errors_are_logged_not_raised(robot):
    node, backend = robot
    backend.software_pwms[CONFIG.buzzer.pin].closed = True   # makes play() fail
    node.on_buzzer(Int16(data=440))                            # must not raise


def test_publishes_sensor_topics(robot):
    node, _ = robot
    received = {}
    listener = rclpy.create_node('listener')
    for msg_type, topic in ((LightSensors, 'light_sensors'), (Switches, 'switches'),
                            (OdometryMsg, 'odom'), (BatteryState, 'battery')):
        listener.create_subscription(
            msg_type, topic, lambda msg, topic=topic: received.setdefault(topic, msg), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(listener)
    deadline = time.monotonic() + 5.0
    while len(received) < 4 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    executor.shutdown()
    listener.destroy_node()
    assert set(received) == {'light_sensors', 'switches', 'odom', 'battery'}
    assert received['odom'].header.frame_id == 'odom'
    assert received['switches'].switch0 is False


def test_zero_rate_disables_a_topic(ros):
    node = RaspiMouseNode(RaspiMouse(backend=MockBackend()), parameter_overrides=[
        Parameter('light_sensors_hz', value=0.0)])
    timers = len(list(node.timers))
    node.destroy_node()
    node = RaspiMouseNode(RaspiMouse(backend=MockBackend()))
    assert len(list(node.timers)) == timers + 1
    node.destroy_node()


def test_destroy_releases_the_hardware(ros):
    backend = MockBackend()
    node = RaspiMouseNode(RaspiMouse(backend=backend), parameter_overrides=[
        Parameter('initial_motor_power', value=True)])
    assert backend.outputs[CONFIG.motors.enable_pin].value
    node.destroy_node()
    assert backend.closed
    assert not backend.outputs[CONFIG.motors.enable_pin].value
