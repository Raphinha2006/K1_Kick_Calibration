"""Dedicated unthrottled rosbag2 writer. No action publishers or FK work.

Bag timestamps are host receipt wall times, NOT sensor sample/contact times.
Topic counters describe received messages. DDS loss reports and motor `lost`
have different meanings; neither establishes total acquisition losslessness.
"""
import argparse
import json
from pathlib import Path
import signal
import time

from .storage import atomic_json
from .topics import REQUIRED, TOPICS


def main():
    import rclpy
    import rosbag2_py
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    from rclpy.qos_event import SubscriptionEventCallbacks, UnsupportedEventTypeError

    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    args = parser.parse_args()
    folder = Path(args.session)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node('kick_calibration_raw_recorder')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(folder / 'raw'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    command_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
    running = True
    failed = None
    stats = {}
    last_mono, last_wall = time.monotonic_ns(), time.time_ns()
    clock_jumps = []
    low_shape = {}
    joint_names = []
    subscriptions = []

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def report(closed=False):
        now = time.monotonic_ns()
        counts = {k: v['count'] for k, v in stats.items()}
        # Ready only after samples have actually been committed, not just graph discovery.
        ready = not closed and failed is None and all(
            counts.get(k, 0) >= 3 and now - stats[k]['last_mono_ns'] < 800_000_000 for k in REQUIRED)
        atomic_json(folder / 'recorder_health.json', dict(
            ready=ready, closed=closed, failure=failed, heartbeat_monotonic_ns=now,
            counts=counts, low_shape=low_shape,
            dds_loss_reports={k: v['dds_lost_reported'] for k, v in stats.items()}))

    def callback(topic, msg):
        nonlocal last_mono, last_wall, low_shape, joint_names
        mono, wall = time.monotonic_ns(), time.time_ns()
        # Write first. A failed write never counts as a recorded observation.
        writer.write(topic, serialize_message(msg), wall)
        st = stats[topic]
        if st['count']:
            interval = mono - st['last_mono_ns']
            st['interval_sum_ns'] += interval
            st['interval_min_ns'] = min(st['interval_min_ns'], interval)
            st['interval_max_ns'] = max(st['interval_max_ns'], interval)
        else:
            st['first_mono_ns'], st['first_unix_ns'] = mono, wall
        st['count'] += 1
        st['last_mono_ns'], st['last_unix_ns'] = mono, wall
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if st['last_source_ns'] is not None and stamp <= st['last_source_ns']:
                st['source_nonincreasing_count'] += 1
            st['last_source_ns'] = stamp
        if topic == '/low_state':
            low_shape = {'motor_state_serial': len(msg.motor_state_serial),
                         'motor_state_parallel': len(msg.motor_state_parallel)}
        if topic == '/joint_states' and not joint_names:
            joint_names = list(msg.name)
        if abs((wall - last_wall) - (mono - last_mono)) > 50_000_000:
            clock_jumps.append(dict(unix_ns=wall, monotonic_ns=mono,
                                    discrepancy_ns=(wall - last_wall) - (mono - last_mono)))
        last_mono, last_wall = mono, wall

    try:
        schemas = {}
        for topic, typename in TOPICS.items():
            message_class = get_message(typename)  # Fail early on a wrong overlay.
            schemas[topic] = dict(type=typename, fields=message_class.get_fields_and_field_types())
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename,
                                                        serialization_format='cdr'))
            stats[topic] = dict(count=0, interval_sum_ns=0, interval_min_ns=10**20,
                                interval_max_ns=0, last_mono_ns=0, last_source_ns=None,
                                source_nonincreasing_count=0, dds_lost_reported=0,
                                dds_loss_event_supported=True)

            def lost(info, name=topic):
                stats[name]['dds_lost_reported'] += info.total_count_change

            cb = lambda m, name=topic: callback(name, m)
            selected_qos = command_qos if topic in (
                '/LocoApiTopicReq', '/kick_ball', '/kick_calibration/events',
                '/kick_calibration/preview_rpc', '/kick_calibration/preview_target') else qos
            try:
                sub = node.create_subscription(message_class, topic, cb, selected_qos,
                      event_callbacks=SubscriptionEventCallbacks(message_lost=lost))
            except UnsupportedEventTypeError:
                stats[topic]['dds_loss_event_supported'] = False
                sub = node.create_subscription(message_class, topic, cb, selected_qos)
            subscriptions.append(sub)
        atomic_json(folder / 'message_schemas.json', schemas)
        node.create_timer(.5, report)
        report()
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exc:
        failed = repr(exc)
        raise
    finally:
        report(closed=True)
        for st in stats.values():
            if st['count'] <= 1:
                st['interval_min_ns'] = None
                st['mean_receipt_hz'] = None
            else:
                st['mean_receipt_hz'] = (st['count'] - 1) * 1e9 / st['interval_sum_ns']
        atomic_json(folder / 'topic_statistics.json', dict(
            topics=stats, clock_jumps=clock_jumps, failure=failed, joint_names_observed=joint_names,
            timestamp_semantics='rosbag: receipt Unix ns; statistics: monotonic receipt ns',
            loss_semantics='DDS reported loss != total logger loss; motor lost != DDS loss'))
        node.destroy_node()
        rclpy.try_shutdown()
        del writer  # Finalize metadata.yaml and sqlite DB before parent analyzes it.


if __name__ == '__main__':
    main()
