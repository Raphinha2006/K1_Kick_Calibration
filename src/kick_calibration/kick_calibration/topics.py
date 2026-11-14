"""Explicit raw recording list; no image streams and no inferred contact signal."""
TOPICS = {
    '/low_state': 'booster_interface/msg/LowState',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/odom': 'nav_msgs/msg/Odometry',
    '/odometer_state': 'booster_interface/msg/Odometer',
    '/battery_state': 'booster_interface/msg/BatteryState',
    '/robot_states': 'booster_interface/msg/RobotStatesMsg',
    '/fall_down_recovery_state': 'booster_interface/msg/RawBytesMsg',
    '/booster_soccer/detection': 'vision_interface/msg/Detections',
    '/LocoApiTopicReq': 'booster_msgs/msg/RpcReqMsg',
    '/LocoApiTopicResp': 'booster_msgs/msg/RpcRespMsg',
    '/kick_ball': 'brain/msg/Kick',
    '/kick_calibration/events': 'std_msgs/msg/String',
    '/kick_calibration/preview_rpc': 'booster_msgs/msg/RpcReqMsg',
    '/kick_calibration/preview_target': 'brain/msg/Kick',
}
REQUIRED = ('/low_state', '/odometer_state')
