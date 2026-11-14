"""Conservative visual departure cue; not a physical contact measurement.

Require three near-foot observations followed by three observations showing
the ball moving away along the kick direction. Robot-frame distance alone
would incorrectly trigger when the robot walks away from a stationary ball.
This detector does not change/filter the targets sent to the native skill.
"""
import math


class BallDeparture:
    def __init__(self, reference_yaw):
        self.direction = (math.cos(reference_yaw), math.sin(reference_yaw))
        self.last_stamp = None
        self.near = []
        self.anchor = None
        self.last_near = None
        self.away = []
        self.confirmed = False

    def observe(self, ball_odom, pose, stamp):
        if self.confirmed or not all(math.isfinite(v) for v in (*ball_odom, *pose, stamp)):
            return None
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return None
        gap = None if self.last_stamp is None else stamp - self.last_stamp
        self.last_stamp = stamp
        if gap is not None and gap > .3:
            self.near.clear()
            self.away.clear()
        if self.last_near is not None and stamp - self.last_near > 1.2:
            self.anchor = None
        radius = math.hypot(ball_odom[0] - pose[0], ball_odom[1] - pose[1])
        point = (stamp, tuple(ball_odom), radius)
        if radius <= .45:
            self.near.append(point)
            self.near = self.near[-3:]
            if len(self.near) == 3 and self.near[-1][0] - self.near[0][0] >= .06:
                closest = min(self.near, key=lambda p: p[2])
                if self.anchor is None or closest[2] < self.anchor[2]:
                    self.anchor = closest
                self.last_near = stamp
        else:
            self.near.clear()
        if self.anchor is None:
            return None
        advance = sum((ball_odom[i] - self.anchor[1][i]) * self.direction[i] for i in (0, 1))
        if radius < .4 or radius - self.anchor[2] < .25 or advance < .25:
            self.away.clear()
            return None
        self.away.append(stamp)
        if len(self.away) < 3 or self.away[-1] - self.away[0] < .06:
            return None
        self.confirmed = True
        return dict(observation_monotonic_s=stamp, near_observation_monotonic_s=self.last_near,
                    ball_range_m=radius, forward_displacement_est_m=advance,
                    evidence='visual_departure_not_measured_contact')
