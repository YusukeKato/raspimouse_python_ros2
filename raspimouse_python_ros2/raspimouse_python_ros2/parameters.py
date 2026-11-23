"""ROS parameters of the node and their conversion to the driver configuration."""

from dataclasses import dataclass

from raspimouse_python_driver import DriveConfig, RaspiMouseConfig
from rclpy.node import Node

_DRIVE = DriveConfig()

# Name -> default. Driver defaults come from the driver so that they stay in sync.
DEFAULTS = {
    # Hardware
    'use_mock_backend': False,       # fake hardware, e.g. to try the node on a PC
    'use_pulse_counters': True,      # False for models before V3
    'use_battery_monitor': True,     # False for models before V3
    'initial_motor_power': False,
    # Drive (see raspimouse_python_driver.DriveConfig)
    'wheel_diameter': _DRIVE.wheel_diameter_m,
    'wheel_tread': _DRIVE.wheel_tread_m,
    'pulses_per_revolution': _DRIVE.pulses_per_revolution,
    'odometry_scale_left_wheel': _DRIVE.odometry_scale_left,
    'odometry_scale_right_wheel': _DRIVE.odometry_scale_right,
    'max_wheel_frequency': _DRIVE.max_wheel_frequency_hz,
    'max_wheel_acceleration': _DRIVE.max_wheel_acceleration_hz_s,
    'cmd_vel_timeout': _DRIVE.command_timeout_s,
    # Topics
    'use_stamped_cmd_vel': True,     # geometry_msgs/TwistStamped (else Twist)
    'odom_frame_id': 'odom',
    'odom_child_frame_id': 'base_footprint',
    'publish_tf': True,
    # Publish rates [Hz]; 0 disables the topic.
    'odom_hz': 50.0,
    'light_sensors_hz': 30.0,
    'switches_hz': 10.0,
    'battery_hz': 1.0,
}


@dataclass(frozen=True)
class NodeParameters:
    """Values of all parameters of the node."""

    use_mock_backend: bool
    initial_motor_power: bool
    use_stamped_cmd_vel: bool
    odom_frame_id: str
    odom_child_frame_id: str
    publish_tf: bool
    odom_hz: float
    light_sensors_hz: float
    switches_hz: float
    battery_hz: float
    driver: RaspiMouseConfig


def declare_and_read(node: Node) -> NodeParameters:
    """Declare the parameters of ``node`` and return their values."""
    for name, default in DEFAULTS.items():
        node.declare_parameter(name, default)
    p = {name: node.get_parameter(name).value for name in DEFAULTS}
    drive = DriveConfig(
        wheel_diameter_m=float(p['wheel_diameter']),
        wheel_tread_m=float(p['wheel_tread']),
        pulses_per_revolution=float(p['pulses_per_revolution']),
        odometry_scale_left=float(p['odometry_scale_left_wheel']),
        odometry_scale_right=float(p['odometry_scale_right_wheel']),
        max_wheel_frequency_hz=float(p['max_wheel_frequency']),
        max_wheel_acceleration_hz_s=float(p['max_wheel_acceleration']),
        command_timeout_s=float(p['cmd_vel_timeout']),
    )
    driver = RaspiMouseConfig(
        has_pulse_counters=bool(p['use_pulse_counters']),
        has_battery_monitor=bool(p['use_battery_monitor']),
        drive=drive,
    )
    return NodeParameters(
        use_mock_backend=bool(p['use_mock_backend']),
        initial_motor_power=bool(p['initial_motor_power']),
        use_stamped_cmd_vel=bool(p['use_stamped_cmd_vel']),
        odom_frame_id=str(p['odom_frame_id']),
        odom_child_frame_id=str(p['odom_child_frame_id']),
        publish_tf=bool(p['publish_tf']),
        odom_hz=float(p['odom_hz']),
        light_sensors_hz=float(p['light_sensors_hz']),
        switches_hz=float(p['switches_hz']),
        battery_hz=float(p['battery_hz']),
        driver=driver,
    )
