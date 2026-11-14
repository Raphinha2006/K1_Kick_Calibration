#!/usr/bin/env python3
"""Hardware-free transport/recording acceptance test. Requires built ROS packages.

scripts/test_ros.sh forces a separate DDS domain. All controller output remains
on preview topics even in this domain. No real actuator request is published.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from booster_interface.msg import (LowState, MotorState, Odometer, RawBytesMsg,
                                   RobotStatesMsg, BatteryState)
from booster_msgs.msg import RpcReqMsg, RpcRespMsg
from vision_interface.msg import Detections, DetectedObject
from brain.msg import Kick
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry

from kick_calibration.analyze import summarize
from kick_calibration.storage import atomic_json, read_trials, write_csv


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '171':
        raise SystemExit('Run scripts/test_ros.sh; isolated ROS_DOMAIN_ID=171 required.')
    root = Path(__file__).resolve().parents[1]
    folder = root / 'logs' / ('isolated_' + time.strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('calibration_fake_robot')
    forbidden = []
    previews = []
    targets = []
    responses = node.create_publisher(RpcRespMsg, '/LocoApiTopicResp', 100)
    pubs = {topic: node.create_publisher(cls, topic, 100) for topic, cls in (
        ('/low_state', LowState), ('/odometer_state', Odometer), ('/robot_states', RobotStatesMsg),
        ('/fall_down_recovery_state', RawBytesMsg), ('/booster_soccer/detection', Detections),
        ('/battery_state', BatteryState), ('/joint_states', JointState), ('/imu/data', Imu), ('/odom', Odometry))}

    def rpc(m):
        previews.append(m)
        api = json.loads(m.header)['api_id']
        body = {'mode': 4} if api == 2017 else {}
        responses.publish(RpcRespMsg(uuid=m.uuid, header='{"status":0}', body=json.dumps(body)))

    subs = [node.create_subscription(RpcReqMsg, '/kick_calibration/preview_rpc', rpc, 100),
            node.create_subscription(Kick, '/kick_calibration/preview_target', targets.append, 100),
            node.create_subscription(RpcReqMsg, '/LocoApiTopicReq', forbidden.append, 100),
            node.create_subscription(Kick, '/kick_ball', forbidden.append, 100)]
    joint_names = ['joint_' + str(i) for i in range(22)]
    last_slow = 0
    feedback_count = 0

    def feedback():
        nonlocal last_slow, feedback_count
        feedback_count += 1
        stamp = node.get_clock().now().to_msg()
        low = LowState(motor_state_serial=[MotorState(q=float(i)*.01, dq=.1, ddq=.2,
                   tau_est=.3, temperature=35, lost=0, reserve=[0, 300]) for i in range(22)])
        pubs['/low_state'].publish(low)
        pubs['/odometer_state'].publish(Odometer(x=0., y=0., theta=0.))
        joint = JointState(name=joint_names, position=[0.]*22, velocity=[.1]*22, effort=[.3]*22)
        joint.header.stamp = stamp
        pubs['/joint_states'].publish(joint)
        imu, odom = Imu(), Odometry()
        imu.header.stamp = odom.header.stamp = stamp
        odom.pose.pose.orientation.w = 1.
        pubs['/imu/data'].publish(imu)
        pubs['/odom'].publish(odom)
        if time.monotonic() - last_slow >= .05:
            last_slow = time.monotonic()
            pubs['/robot_states'].publish(RobotStatesMsg(current_mode=4, current_body_control=12))
            pubs['/fall_down_recovery_state'].publish(RawBytesMsg(msg=[0, 1, 12]))
            pubs['/battery_state'].publish(BatteryState(voltage=51.5, current=1.5, soc=80.))
            msg = Detections(detected_objects=[DetectedObject(label='Ball', confidence=98.,
                                       position_projection=[.6, 0., 0.])])
            msg.header.stamp = stamp
            pubs['/booster_soccer/detection'].publish(msg)

    node.create_timer(1/300, feedback)
    processes = []
    streams = []

    def spin_until(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.002)
            if any(p.poll() is not None for p in processes):
                raise AssertionError('Child exited early; inspect ' + str(folder))
            if predicate():
                return
        raise AssertionError('Timeout; inspect ' + str(folder))

    def has_event(kind):
        file = folder/'events.jsonl'
        return file.exists() and any(json.loads(v)['kind'] == kind for v in file.read_text().splitlines())

    def launch(arguments, log):
        stream = (folder/log).open('w')
        streams.append(stream)
        proc = subprocess.Popen(arguments, stdin=subprocess.PIPE, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True, text=True)
        processes.append(proc)
        return proc

    try:
        recorder = launch([sys.executable, '-m', 'kick_calibration.recorder', '--session', str(folder)], 'recorder.log')
        spin_until(lambda: (folder/'recorder_health.json').exists() and
                   json.loads((folder/'recorder_health.json').read_text()).get('ready'))
        controller = launch([sys.executable, '-c',
            'import sys; from kick_calibration.runtime import run; run(sys.argv[1],sys.argv[2],True)',
            str(folder), str(root/'config/calibration.json')], 'terminal.log')
        spin_until(lambda: has_event('session_idle'))
        spin_until(lambda: (folder/'controller_health.json').exists() and
                   json.loads((folder/'controller_health.json').read_text()).get('recorder_ok'))
        # Discovery and receipt in the controller must complete before prepare.
        # Recorder readiness alone does not imply a newly created controller
        # has already received its first LowState callback.
        spin_until(lambda: all(pubs[t].get_subscription_count() >= 2 for t in
                   ('/low_state', '/odometer_state', '/fall_down_recovery_state', '/robot_states')))
        after_discovery = feedback_count
        spin_until(lambda: feedback_count - after_discovery >= 100)
        controller.stdin.write('prepare\n')
        controller.stdin.flush()
        spin_until(lambda: has_event('prepared'))
        controller.stdin.write('kick 3.125\n')
        controller.stdin.flush()
        spin_until(lambda: has_event('stopped'), 12)
        trials = read_trials(folder)
        assert len(trials) == 1 and trials[0]['outcome'] == 'window_complete', trials
        assert targets and all(t.power == 3.125 for t in targets)
        controller.stdin.write('distance 2.8\nquit\n')
        controller.stdin.flush()
        deadline = time.monotonic() + 5
        while controller.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert controller.poll() == 0, 'controller did not close cleanly'
        processes.remove(controller)
        os.killpg(recorder.pid, signal.SIGINT)
        recorder.wait(timeout=6)
        processes.remove(recorder)
        assert recorder.returncode == 0
        summary = summarize(folder)
        row = read_trials(folder)[0]
        assert row['manual_distance_m'] == 2.8 and row['power'] == 3.125
        assert summary[row['trial_id']]['data_complete']
        assert summary[row['trial_id']]['during_message_counts']['/low_state'] > 350
        assert summary[row['trial_id']]['pre_battery_voltage'] == 51.5
        assert write_csv(folder, training_only=True) == 0, 'dry-run must never become training data'
        assert not forbidden, 'dry-run touched real actuator topics'
        starts = [i for i,m in enumerate(previews) if json.loads(m.header)['api_id'] == 2038 and json.loads(m.body)['start']]
        assert len(starts) == 1
        following = [m for m in previews[starts[0]+1:] if json.loads(m.header)['api_id'] != 2004]
        assert json.loads(following[0].body)['start'] is False
        assert json.loads(following[1].header)['api_id'] == 2001
        result = dict(passed=True, raw_low_count=summary[row['trial_id']]['during_message_counts']['/low_state'],
                      target_count=len(targets), native_start_count=len(starts), real_actuator_messages=0)
        atomic_json(folder/'acceptance.json', result)
        print(json.dumps(result, indent=2))
        print('Evidence: ' + str(folder))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=7)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
