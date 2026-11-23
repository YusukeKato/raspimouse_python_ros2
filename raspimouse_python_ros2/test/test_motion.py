import math

import pytest
from raspimouse_python_driver import Drive, DriveConfig
from raspimouse_python_driver.config import MotorConfig
from raspimouse_python_driver.devices.motor import Motors, StepperMotor
from raspimouse_python_driver.hal.mock import MockDigitalOutput, MockPwmChannel
from raspimouse_python_ros2.motion import (
    check_goal,
    MotionLimits,
    RotationMotion,
    StraightMotion,
)

DRIVE = DriveConfig()
LIMITS = MotionLimits.from_drive_config(DRIVE)
PERIOD = 1.0 / DRIVE.control_rate_hz


class Clock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def make_drive():
    """Create the real Drive on fake motors; odometry is estimated from the commands."""
    motor_config = MotorConfig()
    motors = Motors(*(
        StepperMotor(MockPwmChannel(), MockDigitalOutput(ch.direction_pin), ch, motor_config)
        for ch in (motor_config.left, motor_config.right)))
    clock = Clock()
    drive = Drive(motors, DRIVE, clock=clock, auto_start=False)
    drive.update()
    return drive, clock


def run(motion, drive, clock, max_seconds=60.0):
    """Run like the action server; return the commands that were sent."""
    commands = []
    for _ in range(round(max_seconds / PERIOD)):
        clock.now += PERIOD
        drive.update()
        command = motion.step(drive.odometry())
        if command is None:
            break
        drive.set_velocity(*command)
        commands.append(command)
    else:
        raise AssertionError('the motion did not finish')
    drive.stop()
    for _ in range(100):
        clock.now += PERIOD
        drive.update()
    motion.update(drive.odometry())
    return commands


@pytest.mark.parametrize('distance', [0.3, -0.1, 0.001])
def test_straight_stops_at_the_target(distance):
    drive, clock = make_drive()
    motion = StraightMotion(distance, 0.15, drive.odometry(), LIMITS)
    commands = run(motion, drive, clock)
    assert motion.traveled == pytest.approx(distance, abs=0.001)
    assert motion.remaining < 0.001
    assert all(abs(linear) <= 0.15 + 1e-9 and angular == 0 for linear, angular in commands)
    assert all(math.copysign(1, linear) == math.copysign(1, distance)
               for linear, _ in commands)


def test_straight_cruises_then_slows_down():
    drive, clock = make_drive()
    motion = StraightMotion(0.3, 0.15, drive.odometry(), LIMITS)
    speeds = [linear for linear, _ in run(motion, drive, clock)]
    assert max(speeds) == pytest.approx(0.15)
    assert speeds[-1] < 0.02                      # slow just before the target
    assert speeds[-1] >= LIMITS.min_linear_speed - 1e-9


def test_straight_follows_the_start_heading():
    drive, clock = make_drive()
    drive.reset_odometry(1.0, 2.0, math.pi / 2)  # facing +y
    motion = StraightMotion(0.2, 0.1, drive.odometry(), LIMITS)
    run(motion, drive, clock)
    odom = drive.odometry()
    assert (odom.x, odom.y) == pytest.approx((1.0, 2.2), abs=0.001)


@pytest.mark.parametrize('angle', [math.pi / 2, -math.pi / 2, 3 * math.pi, 0.01])
def test_rotation_stops_at_the_target(angle):
    drive, clock = make_drive()
    motion = RotationMotion(angle, math.pi, drive.odometry(), LIMITS)
    commands = run(motion, drive, clock)
    assert motion.rotated == pytest.approx(angle, abs=math.radians(1))
    assert all(linear == 0 and abs(angular) <= math.pi + 1e-9 for linear, angular in commands)


def test_slow_requested_speed_is_kept():
    drive, clock = make_drive()
    motion = StraightMotion(0.01, 0.005, drive.odometry(), LIMITS)   # below the minimum
    commands = run(motion, drive, clock)
    assert all(linear == pytest.approx(0.005) for linear, _ in commands)
    assert motion.traveled == pytest.approx(0.01, abs=0.001)


@pytest.mark.parametrize('amount, speed', [(float('nan'), 0.1), (0.1, 0.0), (0.1, -1.0),
                                           (float('inf'), 0.1), (0.1, float('nan'))])
def test_invalid_goals(amount, speed):
    assert check_goal(amount, speed) is not None
    drive, _ = make_drive()
    with pytest.raises(ValueError):
        StraightMotion(amount, speed, drive.odometry(), LIMITS)


def test_limits_from_the_drive_config():
    # 2000 Hz/s * 0.377 mm * 0.7 = 0.53 m/s^2; 20 Hz = 7.5 mm/s; one pulse = 0.377 mm
    assert LIMITS.linear_deceleration == pytest.approx(0.528, abs=0.001)
    assert LIMITS.min_linear_speed == pytest.approx(0.00754, abs=1e-5)
    assert LIMITS.distance_tolerance == pytest.approx(0.000377, abs=1e-6)
