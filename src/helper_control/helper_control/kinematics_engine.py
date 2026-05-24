import math


class KinematicsEngine:
    """Differential-drive velocity/RPM conversion helper."""

    def __init__(self, config):
        self.cfg = config
        self.m_to_rpm = (
            60.0 * self.cfg.GEAR_RATIO
        ) / (2.0 * math.pi * self.cfg.WHEEL_RADIUS)
        self.rpm_to_m = (
            2.0 * math.pi * self.cfg.WHEEL_RADIUS
        ) / (60.0 * self.cfg.GEAR_RATIO)

    def inverse_kinematics(self, linear_v, angular_w):
        linear_v = max(
            min(float(linear_v), self.cfg.MAX_LINEAR_VEL),
            -self.cfg.MAX_LINEAR_VEL,
        )
        angular_w = max(
            min(float(angular_w), self.cfg.MAX_ANGULAR_VEL),
            -self.cfg.MAX_ANGULAR_VEL,
        )

        v_l = linear_v - (angular_w * self.cfg.TRACK_WIDTH / 2.0)
        v_r = linear_v + (angular_w * self.cfg.TRACK_WIDTH / 2.0)

        rpm_l = v_l * self.m_to_rpm
        rpm_r = v_r * self.m_to_rpm

        max_calculated_rpm = max(abs(rpm_l), abs(rpm_r))
        if max_calculated_rpm > self.cfg.MAX_RPM:
            scale = self.cfg.MAX_RPM / max_calculated_rpm
            rpm_l *= scale
            rpm_r *= scale

        return int(round(rpm_l)), int(round(rpm_r))

    def forward_kinematics(self, left_rpm, right_rpm):
        v_l = left_rpm * self.rpm_to_m
        v_r = right_rpm * self.rpm_to_m

        current_v = (v_r + v_l) / 2.0
        current_w = (v_r - v_l) / self.cfg.TRACK_WIDTH

        return current_v, current_w
