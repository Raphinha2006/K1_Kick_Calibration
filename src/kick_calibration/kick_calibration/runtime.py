"""ROS transport and a terminal serviced on the same executor as the controller."""
import codecs
from collections import deque
import json
import math
import os
from pathlib import Path
import selectors
import signal
import sys
import time

from .controller import Config, Controller
from .storage import Store, atomic_json

HELP = '''prepare              原生准备/起身，完成后等待 kick
kick <1..10>         一次原生技能尝试；不换算力度
distance <米>        当前试验的人工自然滚动直线距离
invalid <原因>       当前试验作废；例如未触球/多次触球/碰停
stop / status / quit 停止动作 / 查看状态 / 退出
每脚后人工复位机器人朝向与球位；必须先填标签，才允许下一脚。'''


def run(folder, config_path, dry_run):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from booster_interface.msg import LowState, Odometer, RawBytesMsg, RobotStatesMsg
    from booster_msgs.msg import RpcReqMsg, RpcRespMsg
    from brain.msg import Kick
    from vision_interface.msg import Detections
    from std_msgs.msg import String

    config = Config(**json.loads(Path(config_path).read_text()))
    folder = Path(folder)
    store = Store(folder)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Node('kick_calibration_controller')
    request_topic = '/kick_calibration/preview_rpc' if dry_run else '/LocoApiTopicReq'
    target_topic = '/kick_calibration/preview_target' if dry_run else '/kick_ball'
    command_pub = node.create_publisher(RpcReqMsg, request_topic, 10)
    target_pub = node.create_publisher(Kick, target_topic, 10)
    event_pub = node.create_publisher(String, '/kick_calibration/events', 100)
    own_request_ids = deque(maxlen=2048)
    external_motion_time = -100.0
    recorder_matched_since = None

    class IO:
        def request(self, request_id, api_id, body):
            own_request_ids.append(request_id)
            msg = RpcReqMsg(uuid=request_id, header=json.dumps({'api_id': api_id}), body=json.dumps(body))
            command_pub.publish(msg)

        def target(self, values):
            msg = Kick(**values)
            msg.header.stamp = node.get_clock().now().to_msg()
            target_pub.publish(msg)

        def event(self, event):
            event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
            kind = event['kind']
            if kind == 'prepared':
                print('\n准备完成；输入 kick <power> 开始一脚。', flush=True)
            elif kind == 'trial_created':
                print('\n试验 %s，power=%s' % (event['trial_id'], event['power']), flush=True)
            elif kind == 'stopping':
                print('\n退出原生技能并停车：' + event['reason'], flush=True)
            elif kind == 'stopped':
                print('\n' + ('退出/零速度响应已确认。' if event['confirmed'] else '未确认退出/停车响应；查看 status。')
                      + (' 输入 distance <米> 或 invalid <原因>。' if event['trial_id'] else ''), flush=True)
            elif kind == 'rpc_timeout':
                print('\n接口响应超时：' + event['purpose'], flush=True)

    io = IO()
    core = Controller(config, io, store, dry_run)
    last_source = -1
    stopping = False
    failure = None

    def response(m):
        try:
            header, body = json.loads(m.header), json.loads(m.body or '{}')
            status = header.get('status')
            if not isinstance(status, int) or not isinstance(body, dict):
                return
            core.response(m.uuid, status, body)
        except (ValueError, TypeError):
            return

    def low(m):
        serial = m.motor_state_serial
        if len(serial) == 22 and all(math.isfinite(v.q) and math.isfinite(v.dq) for v in serial):
            core.update('low', True)

    def odom(m):
        if all(math.isfinite(v) for v in (m.x, m.y, m.theta)):
            core.update('pose', (m.x, m.y, m.theta))

    def recovery(m):
        if len(m.msg) >= 3 and m.msg[0] <= 3 and m.msg[1] <= 1:
            core.update('recovery', (m.msg[0], bool(m.msg[1]), m.msg[2]))

    def detection(m):
        nonlocal last_source
        stamp_ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= last_source or not 0 <= age <= config.frame_timeout_s:
            return
        last_source = stamp_ns  # Replayed/empty frames never refresh a previous ball.
        if not core.fresh('pose', config.robot_timeout_s):
            return
        candidates = [o for o in m.detected_objects if o.label == 'Ball'
                      and math.isfinite(o.confidence) and o.confidence >= config.confidence_threshold
                      and len(o.position_projection) >= 2
                      and all(math.isfinite(v) for v in o.position_projection[:2])
                      and .05 <= math.hypot(*o.position_projection[:2]) <= 20]
        if not candidates:
            return
        best = max(candidates, key=lambda o: o.confidence)
        bx, by = best.position_projection[:2]
        x, y, yaw = core.value('pose')
        c, s = math.cos(yaw), math.sin(yaw)
        core.update('ball', (x + c * bx - s * by, y + s * bx + c * by), time.monotonic() - age)

    def other_request(m):
        nonlocal external_motion_time
        if m.uuid in own_request_ids:
            return
        try:
            api = json.loads(m.header).get('api_id')
        except (ValueError, AttributeError):
            return
        # The robot's bridge and bare-DDS clients keep idle publishers alive
        # and send read-only status queries. Endpoint count is not motion.
        if isinstance(api, int) and api not in (2011, 2017, 2018, 2022):
            external_motion_time = time.monotonic()
            core.event('external_action_request', uuid=m.uuid, api_id=api)

    qos = qos_profile_sensor_data
    subs = [node.create_subscription(LowState, '/low_state', low, qos),
            node.create_subscription(Odometer, '/odometer_state', odom, qos),
            node.create_subscription(RawBytesMsg, '/fall_down_recovery_state', recovery, qos),
            node.create_subscription(RobotStatesMsg, '/robot_states',
                                     lambda m: core.update('mode', m.current_mode), qos),
            node.create_subscription(Detections, '/booster_soccer/detection', detection, qos),
            node.create_subscription(RpcReqMsg, '/LocoApiTopicReq', other_request, 100),
            node.create_subscription(RpcRespMsg, '/LocoApiTopicResp', response, 100)]

    def health():
        nonlocal recorder_matched_since
        matched = all(any(info.node_name == 'kick_calibration_raw_recorder'
                          for info in node.get_subscriptions_info_by_topic(topic))
                      for topic in (request_topic, target_topic, '/kick_calibration/events'))
        if not matched:
            recorder_matched_since = None
        elif recorder_matched_since is None:
            recorder_matched_since = time.monotonic()
        try:
            h = json.loads((folder / 'recorder_health.json').read_text())
            age = (time.monotonic_ns() - h['heartbeat_monotonic_ns']) / 1e9
            core.recorder_ok = (h.get('ready', False) and 0 <= age < config.record_timeout_s
                                and recorder_matched_since is not None
                                and time.monotonic() - recorder_matched_since >= .5)
        except (OSError, ValueError, KeyError):
            core.recorder_ok = False
        core.external_controller = (time.monotonic() - external_motion_time < 2.0
                                    or node.count_publishers('/kick_ball') > (0 if dry_run else 1))
        atomic_json(folder / 'controller_health.json', core.status())

    def command(line):
        nonlocal stopping
        parts = line.strip().split(maxsplit=1)
        if not parts:
            return
        cmd, argument = parts[0], parts[1] if len(parts) > 1 else None
        if cmd in ('help', '?'):
            print(HELP, flush=True)
        elif cmd == 'prepare' and argument is None:
            core.prepare()
        elif cmd == 'kick' and argument is not None:
            core.kick(argument)
        elif cmd == 'distance' and argument is not None:
            print('Saved ' + core.label(distance=argument), flush=True)
        elif cmd == 'invalid' and argument:
            print('Saved ' + core.label(invalid=argument), flush=True)
        elif cmd == 'stop' and argument is None:
            core.stop()
        elif cmd == 'status' and argument is None:
            print(json.dumps(core.status(), ensure_ascii=False, indent=2), flush=True)
        elif cmd == 'quit' and argument is None:
            stopping = True
        else:
            raise ValueError('输入 help 查看命令。')

    def interrupt(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    selector = selectors.DefaultSelector()
    selector.register(sys.stdin, selectors.EVENT_READ)
    node.create_timer(.25, health)
    node.create_timer(.05, core.tick)
    # DDS reader/writer discovery must complete before showing the prompt.
    # This sends no RPC and prevents the first trial's start from missing the bag.
    warmup_deadline = time.monotonic() + 3
    while time.monotonic() < warmup_deadline and not stopping:
        rclpy.spin_once(node, timeout_sec=.02)
        if core.recorder_ok and core.fresh('low', config.robot_timeout_s) and core.fresh('pose', config.robot_timeout_s):
            break
    print(('DRY RUN：只发布 preview；不会准备、起身或踢球。\n' if dry_run else 'REAL：启动待命，输入 prepare 才准备机器人。\n')
          + HELP + '\nSession: ' + str(folder), flush=True)
    print('calib> ', end='', flush=True)
    decoder = codecs.getincrementaldecoder(sys.stdin.encoding or 'utf-8')()
    input_buffer = ''
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            if selector.select(0):
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    stopping = True
                    break
                input_buffer += decoder.decode(chunk)
                while '\n' in input_buffer and not stopping:
                    line, input_buffer = input_buffer.split('\n', 1)
                    try:
                        health()
                        command(line)
                    except ValueError as exc:
                        print(str(exc), flush=True)
                    print('calib> ', end='', flush=True)
    except Exception as exc:
        failure = repr(exc)
        raise
    finally:
        selector.close()
        try:
            core.stop('shutdown' if failure is None else 'controller_exception')
            deadline = time.monotonic() + 2 * config.rpc_timeout_s + 1
            while core.state == 'STOPPING' and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            core.event('session_exit', controller_failure=failure)
        except Exception:
            # Disk/journal failure must not prevent native stop. It is abnormal
            # and stays unlabelled, never eligible for training.
            import uuid
            io.request(str(uuid.uuid4()), 2038, {'start': False, 'version': 1})
            end = time.monotonic() + .3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            io.request(str(uuid.uuid4()), 2001, {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0})
            rclpy.spin_once(node, timeout_sec=.1)
            raise
        finally:
            store.close()
            node.destroy_node()
            rclpy.try_shutdown()
