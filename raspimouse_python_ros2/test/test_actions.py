import math
import threading
import time
from unittest import mock

from geometry_msgs.msg import TwistStamped
import pytest
from raspimouse_python_driver import RaspiMouse, RaspiMouseConfig
from raspimouse_python_driver.hal.mock import MockBackend
from raspimouse_python_msgs.action import DriveDistance, Rotate
from raspimouse_python_ros2.raspimouse_node import RaspiMouseNode
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool

# Without pulse counters the odometry follows the commands, so goals finish on fake hardware.
CONFIG = RaspiMouseConfig(has_pulse_counters=False)


@pytest.fixture
def system():
    """Run a node on fake hardware and a client node; return (node, client, backend)."""
    rclpy.init()
    backend = MockBackend()
    node = RaspiMouseNode(RaspiMouse(CONFIG, backend))
    client = rclpy.create_node('client')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node, client, backend
    executor.shutdown(timeout_sec=3.0)
    node.destroy_node()
    client.destroy_node()
    rclpy.try_shutdown()


def wait(future, timeout=15.0):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    assert event.wait(timeout), 'timed out'
    return future.result()


def send(client_node, action_type, name, goal, feedback=None):
    client = ActionClient(client_node, action_type, name)
    assert client.wait_for_server(timeout_sec=5.0)
    return wait(client.send_goal_async(goal, feedback_callback=feedback))


def power_on(node):
    node.on_motor_power(SetBool.Request(data=True), SetBool.Response())


def test_drive_distance(system):
    node, client, _ = system
    power_on(node)
    feedback = []
    handle = send(client, DriveDistance, 'drive_distance',
                  DriveDistance.Goal(distance=0.05, speed=0.15),
                  lambda msg: feedback.append(msg.feedback))
    assert handle.accepted
    result = wait(handle.get_result_async()).result
    assert result.success
    assert result.traveled == pytest.approx(0.05, abs=0.001)
    assert feedback and feedback[-1].remaining < 0.05
    assert node.mouse.drive.wheel_frequencies == (0.0, 0.0)


def test_rotate(system):
    node, client, _ = system
    power_on(node)
    handle = send(client, Rotate, 'rotate', Rotate.Goal(angle=-math.pi / 4, angular_speed=math.pi))
    result = wait(handle.get_result_async()).result
    assert result.success
    assert result.rotated == pytest.approx(-math.pi / 4, abs=math.radians(1))


def test_rejected_while_motor_power_is_off(system):
    _, client, _ = system
    handle = send(client, DriveDistance, 'drive_distance',
                  DriveDistance.Goal(distance=0.05, speed=0.15))
    assert not handle.accepted


def test_rejected_when_invalid(system):
    node, client, _ = system
    power_on(node)
    handle = send(client, DriveDistance, 'drive_distance',
                  DriveDistance.Goal(distance=0.05, speed=0.0))
    assert not handle.accepted


def test_second_goal_is_rejected_and_cmd_vel_ignored(system):
    node, client, _ = system
    power_on(node)
    first = send(client, DriveDistance, 'drive_distance',
                 DriveDistance.Goal(distance=0.3, speed=0.15))
    assert first.accepted
    second = send(client, Rotate, 'rotate', Rotate.Goal(angle=1.0, angular_speed=1.0))
    assert not second.accepted
    node.on_cmd_vel(TwistStamped())           # ignored, does not stop the goal
    assert node.mouse.drive.target_velocity[0] > 0
    wait(first.cancel_goal_async())
    result = wait(first.get_result_async()).result
    assert not result.success and result.message == 'canceled'


def test_cancel_stops_the_robot(system):
    node, client, _ = system
    power_on(node)
    handle = send(client, DriveDistance, 'drive_distance',
                  DriveDistance.Goal(distance=1.0, speed=0.15))
    wait(handle.cancel_goal_async())
    result = wait(handle.get_result_async()).result
    assert not result.success
    assert 0 <= result.traveled < 1.0
    assert node.mouse.drive.wheel_frequencies == (0.0, 0.0)


def test_motor_power_off_aborts(system):
    node, client, _ = system
    power_on(node)
    handle = send(client, DriveDistance, 'drive_distance',
                  DriveDistance.Goal(distance=1.0, speed=0.15))
    node.on_motor_power(SetBool.Request(data=False), SetBool.Response())
    result = wait(handle.get_result_async()).result
    assert not result.success
    assert 'motor power' in result.message


def test_shutdown_during_a_goal_stops_quietly(system):
    node, client, _ = system
    power_on(node)
    handle = send(client, DriveDistance, 'drive_distance',
                  DriveDistance.Goal(distance=1.0, speed=0.15))
    assert handle.accepted
    # Newer rclpy (not Jazzy) also destroys the node at shutdown, which closes the hardware.
    drive = node.mouse.drive
    # rclpy prints a traceback when the execute callback raises.
    with mock.patch('rclpy.action.server.traceback.print_exc') as print_exc:
        rclpy.shutdown()          # what SIGINT / SIGTERM does
        deadline = time.monotonic() + 5.0
        while node._motion_busy and time.monotonic() < deadline:
            time.sleep(0.01)
        time.sleep(0.2)           # let rclpy finish the goal
    assert not node._motion_busy
    assert drive.wheel_frequencies == (0.0, 0.0)
    print_exc.assert_not_called()


def test_destroy_during_a_goal_ends_it_before_closing(system):
    node, client, _ = system
    power_on(node)
    handle = send(client, DriveDistance, 'drive_distance',
                  DriveDistance.Goal(distance=1.0, speed=0.15))
    assert handle.accepted
    drive, close = node.mouse.drive, node.mouse.close
    busy_at_close = []
    node.mouse.close = lambda: (busy_at_close.append(node._motion_busy), close())
    node.destroy_node()
    assert busy_at_close == [False]
    assert drive.wheel_frequencies == (0.0, 0.0)
