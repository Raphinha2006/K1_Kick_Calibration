"""Wire schema copied from AAA RobotClient and the installed Booster SDK.

Power is the numeric desired-range input in AAA's Kick.msg, not a measured force
or a verified distance in metres. Only empirical calibration supplies its scale.
"""
import math

CHANGE_MODE, MOVE, ROTATE_HEAD, GET_UP = 2000, 2001, 2004, 2008
GET_MODE, GET_ROBOT_INFO, VISUAL_KICK = 2017, 2022, 2038
PREPARE_MODE, SOCCER_MODE = 1, 4


def power_value(text):
    value = float(text)
    if not math.isfinite(value) or not 1.0 <= value <= 10.0:
        raise ValueError('power 必须是 1–10 的有限小数；不会自动裁剪。')
    return value


def distance_value(text):
    value = float(text)
    if not math.isfinite(value) or value < 0:
        raise ValueError('距离必须是非负有限数，单位米。')
    return value


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def target(ball_odom, pose, reference_yaw, power, reference_m):
    """Keep a trial's physical direction while the native skill changes heading.

    The odometry frame is only a local estimate, not soccer-field localization.
    Translation is unscaled; no person-pass odometry correction is applied here.
    """
    px, py, yaw = pose
    dx, dy = ball_odom[0] - px, ball_odom[1] - py
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = c * dx + s * dy, -s * dx + c * dy
    direction = wrap(reference_yaw - yaw)
    return dict(x=x, y=y, dir=direction,
                goal_x=x + reference_m * math.cos(direction),
                goal_y=y + reference_m * math.sin(direction),
                robot_theta_to_field=wrap(yaw - reference_yaw), power=power)
