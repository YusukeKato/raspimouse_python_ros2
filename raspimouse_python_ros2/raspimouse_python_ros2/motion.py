"""
Motions that end at a target: drive a distance, rotate by an angle.

Pure logic without ROS or hardware. Call :meth:`step` with the current
odometry at a fixed rate and send the returned (linear, angular) velocity to
the drive until :meth:`step` returns None.

Near the target the speed follows v = sqrt(2 a d), so that the robot can stop
within the remaining distance d at the deceleration a allowed by the drive.
A small minimum speed keeps the stepper motors above their 5 Hz stop
threshold, so that the robot never stalls just before the target. The
distance covered during the control latency (a command takes effect one
control cycle later and shows in the odometry one cycle after that) is
subtracted from the remaining distance, so the robot does not overshoot.
"""

from dataclasses import dataclass
import math

from raspimouse_python_driver import DriveConfig, Odometry
from raspimouse_python_driver.kinematics import meters_per_pulse, normalize_angle


@dataclass(frozen=True)
class MotionLimits:
    """Deceleration, minimum speed and stop tolerance of a motion."""

    linear_deceleration: float   # [m/s^2]
    angular_deceleration: float  # [rad/s^2]
    min_linear_speed: float      # [m/s]
    min_angular_speed: float     # [rad/s]
    distance_tolerance: float    # [m]
    angle_tolerance: float       # [rad]
    latency: float               # [s] from a command to its effect in the odometry

    @classmethod
    def from_drive_config(cls, config: DriveConfig, margin: float = 0.7,
                          min_wheel_hz: float = 20.0) -> 'MotionLimits':
        """
        Derive the limits from the drive configuration.

        :param margin: fraction of the drive's acceleration limit to plan with.
        :param min_wheel_hz: lowest wheel frequency used before the target.

        The latency is two control cycles of the drive.
        """
        mpp = meters_per_pulse(config)
        half_tread = config.wheel_tread_m / 2.0
        deceleration = config.max_wheel_acceleration_hz_s * mpp * margin
        min_speed = min_wheel_hz * mpp
        return cls(
            linear_deceleration=deceleration,
            angular_deceleration=deceleration / half_tread,
            min_linear_speed=min_speed,
            min_angular_speed=min_speed / half_tread,
            distance_tolerance=mpp,              # one pulse
            angle_tolerance=mpp / half_tread,    # one pulse of each wheel
            latency=2.0 / config.control_rate_hz,
        )


def check_goal(amount: float, speed: float) -> str | None:
    """Return why a goal (distance or angle, speed) is invalid, or None if it is valid."""
    if not math.isfinite(amount):
        return f'the target must be a finite number: {amount}'
    if not (math.isfinite(speed) and speed > 0):
        return f'the speed must be greater than 0: {speed}'
    return None


def _approach_speed(remaining: float, tolerance: float, speed: float,
                    deceleration: float, min_speed: float) -> float:
    # Brake so as to arrive at the tolerance, not at the exact target.
    braking = math.sqrt(2.0 * deceleration * max(remaining - tolerance, 0.0))
    return max(min(speed, braking), min(speed, min_speed))


class StraightMotion:
    """Drive straight along the heading at the start."""

    def __init__(self, distance: float, speed: float, start: Odometry,
                 limits: MotionLimits) -> None:
        """
        Plan the motion.

        :param distance: [m] negative drives backward.
        :param speed: [m/s] greater than 0.
        :raises ValueError: invalid distance or speed.
        """
        error = check_goal(distance, speed)
        if error:
            raise ValueError(error)
        self._sign = 1.0 if distance >= 0 else -1.0
        self._target = abs(distance)
        self._speed = speed
        self._start = start
        self._limits = limits
        self._last_speed = 0.0
        self.traveled = 0.0

    @property
    def remaining(self) -> float:
        """Return the distance left [m], 0 once reached."""
        return max(0.0, self._target - self._sign * self.traveled)

    def update(self, odom: Odometry) -> None:
        """Measure the progress along the start heading."""
        dx, dy = odom.x - self._start.x, odom.y - self._start.y
        self.traveled = dx * math.cos(self._start.theta) + dy * math.sin(self._start.theta)

    def step(self, odom: Odometry) -> tuple[float, float] | None:
        """Return the (linear, angular) velocity to send, or None when the target is reached."""
        self.update(odom)
        lim = self._limits
        # Distance still to come from earlier commands (control latency).
        remaining = self._target - self._sign * self.traveled - self._last_speed * lim.latency
        if remaining <= lim.distance_tolerance:
            return None
        self._last_speed = _approach_speed(remaining, lim.distance_tolerance, self._speed,
                                           lim.linear_deceleration, lim.min_linear_speed)
        return self._sign * self._last_speed, 0.0


class RotationMotion:
    """Rotate in place; angles beyond pi (several turns) are fine."""

    def __init__(self, angle: float, angular_speed: float, start: Odometry,
                 limits: MotionLimits) -> None:
        """
        Plan the motion.

        :param angle: [rad] positive is counterclockwise.
        :param angular_speed: [rad/s] greater than 0.
        :raises ValueError: invalid angle or speed.
        """
        error = check_goal(angle, angular_speed)
        if error:
            raise ValueError(error)
        self._sign = 1.0 if angle >= 0 else -1.0
        self._target = abs(angle)
        self._speed = angular_speed
        self._limits = limits
        self._last_theta = start.theta
        self._last_speed = 0.0
        self.rotated = 0.0

    @property
    def remaining(self) -> float:
        """Return the angle left [rad], 0 once reached."""
        return max(0.0, self._target - self._sign * self.rotated)

    def update(self, odom: Odometry) -> None:
        """Accumulate the rotation since the last call."""
        self.rotated += normalize_angle(odom.theta - self._last_theta)
        self._last_theta = odom.theta

    def step(self, odom: Odometry) -> tuple[float, float] | None:
        """Return the (linear, angular) velocity to send, or None when the target is reached."""
        self.update(odom)
        lim = self._limits
        # Angle still to come from earlier commands (control latency).
        remaining = self._target - self._sign * self.rotated - self._last_speed * lim.latency
        if remaining <= lim.angle_tolerance:
            return None
        self._last_speed = _approach_speed(remaining, lim.angle_tolerance, self._speed,
                                           lim.angular_deceleration, lim.min_angular_speed)
        return 0.0, self._sign * self._last_speed
