"""
ROS 2 node for Raspberry Pi Mouse.

Publishes odom (+ TF), light_sensors, switches and battery; subscribes to
cmd_vel, leds and buzzer; offers the motor_power service and the
drive_distance and rotate actions. Topic names and types follow
rt-net/raspimouse2.
"""

from collections.abc import Callable
import dataclasses
import functools
import signal
import sys
import threading
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry as OdometryMsg
from raspimouse_msgs.msg import Leds, LightSensors, Switches
from raspimouse_python_driver import RaspiMouse, RaspiMouseError
from raspimouse_python_driver.hal.mock import MockBackend
from raspimouse_python_msgs.action import DriveDistance, Rotate
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from .conversions import (
    battery_to_msg,
    light_sensors_to_msg,
    odometry_to_msg,
    odometry_to_transform,
    switches_to_msg,
)
from .motion import check_goal, MotionLimits, RotationMotion, StraightMotion
from .parameters import declare_and_read

CHECK_HINT = 'run "raspimouse-cli check" to find setup problems'
# Rate of the velocity commands while an action runs [Hz].
MOTION_RATE_HZ = 50.0


def _logs_device_errors(callback: Callable) -> Callable:
    """Log driver errors in a callback instead of stopping the executor."""
    @functools.wraps(callback)
    def wrapper(self, *args, **kwargs):
        try:
            return callback(self, *args, **kwargs)
        except RaspiMouseError as e:
            self.get_logger().error(f'{callback.__name__}: {e}', throttle_duration_sec=5.0)
    return wrapper


class RaspiMouseNode(Node):
    """Node that owns the Raspberry Pi Mouse hardware."""

    def __init__(self, mouse: RaspiMouse | None = None, **kwargs) -> None:
        """
        Open the hardware and create the topics, timers and service.

        :param mouse: hardware to use (for tests); created from the parameters if None.
        :param kwargs: passed to :class:`rclpy.node.Node` (e.g. parameter_overrides).
        :raises RaspiMouseError: the hardware cannot be opened.
        """
        super().__init__('raspimouse', **kwargs)
        self.params = declare_and_read(self)
        if mouse is None:
            config, backend = self.params.driver, None
            if self.params.use_mock_backend:
                # Fake pulse counters never count: estimate the odometry from the commands.
                config = dataclasses.replace(config, has_pulse_counters=False)
                backend = MockBackend()
            mouse = RaspiMouse(config, backend)
        self.mouse = mouse
        self.mouse.open()
        if self.params.initial_motor_power:
            self.mouse.motor_power.on()

        # Outputs
        cmd_vel_type = TwistStamped if self.params.use_stamped_cmd_vel else Twist
        self.create_subscription(cmd_vel_type, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Leds, 'leds', self.on_leds, 10)
        self.create_subscription(Int16, 'buzzer', self.on_buzzer, 10)
        self.create_service(SetBool, 'motor_power', self.on_motor_power)

        # Actions. They run in their own threads (MultiThreadedExecutor), one goal at a time.
        self.motion_limits = MotionLimits.from_drive_config(self.params.driver.drive)
        self._motion_lock = threading.Lock()
        self._motion_busy = False
        action_group = ReentrantCallbackGroup()
        self._action_servers = [
            ActionServer(self, DriveDistance, 'drive_distance', self.execute_drive_distance,
                         goal_callback=self.on_drive_distance_goal,
                         cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=action_group),
            ActionServer(self, Rotate, 'rotate', self.execute_rotate,
                         goal_callback=self.on_rotate_goal,
                         cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=action_group),
        ]

        # Inputs
        p = self.params
        self.odom_pub = self.create_publisher(OdometryMsg, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if p.publish_tf else None
        self.light_sensors_pub = self.create_publisher(LightSensors, 'light_sensors', 10)
        self.switches_pub = self.create_publisher(Switches, 'switches', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        for hz, callback in ((p.odom_hz, self.publish_odometry),
                             (p.light_sensors_hz, self.publish_light_sensors),
                             (p.switches_hz, self.publish_switches),
                             (p.battery_hz if self.mouse.battery else 0.0,
                              self.publish_battery)):
            if hz > 0:
                self.create_timer(1.0 / hz, callback)

        self.get_logger().info('Raspberry Pi Mouse is ready (motor power '
                               f'{"on" if self.mouse.motor_power.is_on else "off"})')

    # --- Subscriptions and service ---------------------------------------

    @_logs_device_errors
    def on_cmd_vel(self, msg: Twist | TwistStamped) -> None:
        """Set the target velocity. Repeat at least every cmd_vel_timeout seconds."""
        twist = msg.twist if isinstance(msg, TwistStamped) else msg
        if self._motion_busy:
            self.get_logger().warning('cmd_vel ignored while an action goal runs',
                                      throttle_duration_sec=5.0)
            return
        if not self.mouse.motor_power.is_on:
            self.get_logger().warning('cmd_vel received while the motor power is off',
                                      throttle_duration_sec=5.0)
        self.mouse.drive.set_velocity(twist.linear.x, twist.angular.z)

    @_logs_device_errors
    def on_leds(self, msg: Leds) -> None:
        """Turn LED0-LED3 on or off."""
        for led, value in zip(self.mouse.leds, (msg.led0, msg.led1, msg.led2, msg.led3)):
            led.set_state(value)

    @_logs_device_errors
    def on_buzzer(self, msg: Int16) -> None:
        """Sound the buzzer at msg.data Hz; 0 stops it."""
        if msg.data < 0:
            self.get_logger().warning(f'buzzer: negative frequency {msg.data}; stopping')
            self.mouse.buzzer.stop()
            return
        self.mouse.buzzer.play(msg.data)

    def on_motor_power(self, request: SetBool.Request,
                       response: SetBool.Response) -> SetBool.Response:
        """Turn the motor power on or off. Turning it off also stops the robot."""
        try:
            if request.data:
                self.mouse.motor_power.on()
            else:
                self.mouse.drive.stop()
                self.mouse.motor_power.off()
        except RaspiMouseError as e:
            response.success = False
            response.message = str(e)
            return response
        response.success = True
        response.message = f'motor power {"on" if request.data else "off"}'
        self.get_logger().info(response.message)
        return response

    # --- Actions -----------------------------------------------------------

    def on_drive_distance_goal(self, goal: DriveDistance.Goal) -> GoalResponse:
        """Accept a valid goal while the motor power is on and no other goal runs."""
        return self._accept_goal('drive_distance', check_goal(goal.distance, goal.speed))

    def on_rotate_goal(self, goal: Rotate.Goal) -> GoalResponse:
        """Accept a valid goal while the motor power is on and no other goal runs."""
        return self._accept_goal('rotate', check_goal(goal.angle, goal.angular_speed))

    def _accept_goal(self, name: str, error: str | None) -> GoalResponse:
        if error is None and not self.mouse.motor_power.is_on:
            error = 'the motor power is off'
        with self._motion_lock:
            if error is None and self._motion_busy:
                error = 'another goal is running'
            if error is None:
                self._motion_busy = True   # reserved until the goal ends
        if error is not None:
            self.get_logger().warning(f'{name}: goal rejected: {error}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_drive_distance(self, goal_handle) -> DriveDistance.Result:
        """Drive the requested distance, then stop."""
        goal = goal_handle.request
        motion = StraightMotion(goal.distance, goal.speed, self.mouse.drive.odometry(),
                                self.motion_limits)

        def feedback():
            return DriveDistance.Feedback(traveled=motion.traveled, remaining=motion.remaining)

        def result(success, message):
            return DriveDistance.Result(success=success, traveled=motion.traveled,
                                        message=message)
        return self._run_motion('drive_distance', goal_handle, motion, feedback, result)

    def execute_rotate(self, goal_handle) -> Rotate.Result:
        """Rotate by the requested angle, then stop."""
        goal = goal_handle.request
        motion = RotationMotion(goal.angle, goal.angular_speed, self.mouse.drive.odometry(),
                                self.motion_limits)

        def feedback():
            return Rotate.Feedback(rotated=motion.rotated, remaining=motion.remaining)

        def result(success, message):
            return Rotate.Result(success=success, rotated=motion.rotated, message=message)
        return self._run_motion('rotate', goal_handle, motion, feedback, result)

    def _run_motion(self, name, goal_handle, motion, feedback, result):
        drive = self.mouse.drive
        period = 1.0 / MOTION_RATE_HZ
        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self._stop_and_settle(motion)
                    goal_handle.canceled()
                    return result(False, 'canceled')
                if not rclpy.ok(context=self.context) or not self.mouse.motor_power.is_on:
                    self._stop_and_settle(motion)
                    goal_handle.abort()
                    return result(False, 'the motor power was turned off or the node stops')
                command = motion.step(drive.odometry())
                if command is None:
                    break
                drive.set_velocity(*command)
                goal_handle.publish_feedback(feedback())
                time.sleep(period)
            self._stop_and_settle(motion)
            goal_handle.succeed()
            self.get_logger().info(f'{name}: done')
            return result(True, 'reached the target')
        except RaspiMouseError as e:
            self.get_logger().error(f'{name}: {e}')
            try:
                drive.stop()
            except RaspiMouseError:
                pass
            goal_handle.abort()
            return result(False, str(e))
        finally:
            with self._motion_lock:
                self._motion_busy = False

    def _stop_and_settle(self, motion) -> None:
        """Slow down to a stop and measure the final progress."""
        drive = self.mouse.drive
        drive.stop()
        deadline = time.monotonic() + 3.0
        while drive.wheel_frequencies != (0.0, 0.0) and time.monotonic() < deadline:
            time.sleep(0.01)
        # Let the odometry take in the last pulses (two control cycles).
        time.sleep(2.0 / self.params.driver.drive.control_rate_hz)
        motion.update(drive.odometry())

    # --- Timers ------------------------------------------------------------

    @_logs_device_errors
    def publish_odometry(self) -> None:
        """Publish odom and the odom -> base_footprint transform."""
        odom = self.mouse.drive.odometry()
        stamp = self.get_clock().now().to_msg()
        p = self.params
        self.odom_pub.publish(
            odometry_to_msg(odom, stamp, p.odom_frame_id, p.odom_child_frame_id))
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                odometry_to_transform(odom, stamp, p.odom_frame_id, p.odom_child_frame_id))

    @_logs_device_errors
    def publish_light_sensors(self) -> None:
        """Publish the light sensor values."""
        self.light_sensors_pub.publish(light_sensors_to_msg(self.mouse.light_sensors.read()))

    @_logs_device_errors
    def publish_switches(self) -> None:
        """Publish the switch states."""
        self.switches_pub.publish(
            switches_to_msg([sw.is_pressed() for sw in self.mouse.switches]))

    @_logs_device_errors
    def publish_battery(self) -> None:
        """Publish the battery (or AC adapter) voltage."""
        self.battery_pub.publish(battery_to_msg(
            self.mouse.battery.read_voltage(), self.get_clock().now().to_msg()))

    def destroy_node(self) -> None:
        """Stop the motors, cut their power and release the hardware."""
        try:
            self.mouse.close()
            self.get_logger().info('released the hardware (motors stopped, motor power off)')
        except RaspiMouseError as e:
            # Everything is released even then; report and go on shutting down.
            self.get_logger().error(f'error while closing the hardware: {e}')
        finally:
            super().destroy_node()


def main(args: list[str] | None = None) -> int:
    """Run the node until SIGINT or SIGTERM."""
    rclpy.init(args=args)
    try:
        node = RaspiMouseNode()
    except RaspiMouseError as e:
        rclpy.logging.get_logger('raspimouse').fatal(
            f'cannot open the hardware: {e}; {CHECK_HINT}')
        rclpy.try_shutdown()
        return 1
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A signal shut the context down while a callback was publishing.
        if rclpy.ok(context=node.context):
            raise
    finally:
        # Ctrl+C in a terminal reaches the node both directly and through
        # ros2 launch. Do not let a second signal interrupt releasing the
        # hardware, or the motors could keep running.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        executor.shutdown(timeout_sec=3.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
