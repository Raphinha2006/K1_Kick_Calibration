"""Offline, per-trial summaries; never estimates ground-truth distance/contact."""
import json
import math
from pathlib import Path

from .storage import atomic_json, read_trials, write_csv


def summarize(folder):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    folder = Path(folder)
    trials = read_trials(folder)
    if not (folder / 'raw' / 'metadata.yaml').exists():
        raise ValueError('Bag 未完整关闭，保留原始文件；不要将本轮自动导出为训练数据。')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder / 'raw'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    windows = {}
    for row in trials:
        start = row.get('request_unix_ns')
        stop = row.get('stop_request_unix_ns')
        if start and stop:
            windows[row['trial_id']] = dict(row=row, start=start, stop=stop, pre={}, counts={}, gaps={},
                                            last={}, low_count=0, sums=[0.] * 22,
                                            ddq_sums=[0.] * 22, dq_peak=[0.] * 22,
                                            tau_peak=[0.] * 22, temp_max=[-128] * 22,
                                            lost_first=None, lost_last=None,
                                            gyro_peak=0., acc_peak=0., published_power=[],
                                            start_seen=False, stop_seen=False, zero_seen=False)
    wanted = {'/low_state', '/battery_state', '/odom', '/odometer_state',
              '/robot_states', '/fall_down_recovery_state', '/joint_states', '/imu/data',
              '/kick_ball', '/kick_calibration/preview_target', '/LocoApiTopicReq',
              '/kick_calibration/preview_rpc', '/LocoApiTopicResp'}
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(raw, types[topic])
        for w in windows.values():
            # Most recent *pre-action* sample, with its age. Never filled from
            # samples at/after the action request (prevents feature leakage).
            if timestamp < w['start']:
                w['pre'][topic] = (timestamp, msg)
            # Confirm raw command presence too. The stop/zero RPCs follow the
            # action window and must still be present in the finalized bag.
            if topic in ('/LocoApiTopicReq', '/kick_calibration/preview_rpc'):
                for prefix in ('start', 'stop', 'zero'):
                    if msg.uuid == w['row'].get(prefix + '_uuid'):
                        w[prefix + '_seen'] = True
            if not w['start'] <= timestamp <= w['stop']:
                continue
            w['counts'][topic] = w['counts'].get(topic, 0) + 1
            if topic in w['last']:
                w['gaps'][topic] = max(w['gaps'].get(topic, 0), timestamp - w['last'][topic])
            w['last'][topic] = timestamp
            if topic in ('/kick_ball', '/kick_calibration/preview_target'):
                w['published_power'].append(msg.power)
            if topic != '/low_state' or len(msg.motor_state_serial) != 22:
                continue
            motors = msg.motor_state_serial
            w['low_count'] += 1
            for i, m in enumerate(motors):
                for key, value in (('sums', m.dq * m.dq), ('ddq_sums', m.ddq * m.ddq)):
                    w[key][i] += value
                w['dq_peak'][i] = max(w['dq_peak'][i], abs(m.dq))
                w['tau_peak'][i] = max(w['tau_peak'][i], abs(m.tau_est))
                w['temp_max'][i] = max(w['temp_max'][i], m.temperature)
            lost = [m.lost for m in motors]
            if w['lost_first'] is None:
                w['lost_first'] = lost
            w['lost_last'] = lost
            w['gyro_peak'] = max(w['gyro_peak'], math.sqrt(sum(v * v for v in msg.imu_state.gyro)))
            w['acc_peak'] = max(w['acc_peak'], math.sqrt(sum(v * v for v in msg.imu_state.acc)))
    out = {}
    topic_stats = json.loads((folder / 'topic_statistics.json').read_text())
    for row in trials:
        trial_id = row['trial_id']
        if trial_id not in windows:
            continue
        w = windows[trial_id]
        result = dict(during_message_counts=w['counts'], during_max_receipt_gap_s={
                      k: v / 1e9 for k, v in w['gaps'].items()},
                      pre_sample_age_s={k: (w['start'] - v[0]) / 1e9 for k, v in w['pre'].items()})
        low = w['pre'].get('/low_state')
        if low:
            motors = low[1].motor_state_serial
            for name in ('q', 'dq', 'ddq', 'tau_est', 'temperature', 'lost', 'mode'):
                convert = int if name in ('temperature', 'lost', 'mode') else float
                result['pre_joint_' + name] = [convert(getattr(m, name)) for m in motors]
            result['pre_imu_rpy'] = [float(v) for v in low[1].imu_state.rpy]
            result['pre_imu_gyro'] = [float(v) for v in low[1].imu_state.gyro]
            result['pre_imu_acc'] = [float(v) for v in low[1].imu_state.acc]
        battery = w['pre'].get('/battery_state')
        if battery:
            for name in ('voltage', 'current', 'soc', 'average_voltage'):
                result['pre_battery_' + name] = float(getattr(battery[1], name))
        if w['low_count']:
            result.update(during_joint_dq_rms_rad_s=[math.sqrt(v / w['low_count']) for v in w['sums']],
                          during_joint_ddq_rms_rad_s2=[math.sqrt(v / w['low_count']) for v in w['ddq_sums']],
                          during_joint_abs_dq_peak_rad_s=w['dq_peak'],
                          during_joint_abs_tau_est_peak_nm=w['tau_peak'],
                          during_joint_temperature_max_c=w['temp_max'],
                          during_motor_lost_first=w['lost_first'], during_motor_lost_last=w['lost_last'],
                          during_imu_gyro_norm_peak=w['gyro_peak'], during_imu_acc_norm_peak=w['acc_peak'])
        jumps = [j for j in topic_stats['clock_jumps'] if w['start'] <= j['unix_ns'] <= w['stop']]
        result['during_recorded_power_min'] = min(w['published_power'], default=None)
        result['during_recorded_power_max'] = max(w['published_power'], default=None)
        result['raw_command_sequence_present'] = all(w[k + '_seen'] for k in ('start', 'stop', 'zero'))
        result['raw_command_presence'] = {k: w[k + '_seen'] for k in ('start', 'stop', 'zero')}
        result['data_complete'] = bool(w['low_count'] and w['counts'].get('/odometer_state')
                                       and w['published_power'] and all(p == row['power'] for p in w['published_power'])
                                       and result['raw_command_sequence_present']
                                       and not topic_stats['failure'] and not jumps)
        result['during_clock_jump_count'] = len(jumps)
        out[trial_id] = result
    atomic_json(folder / 'summary.json', out)
    write_csv(folder)
    return out
