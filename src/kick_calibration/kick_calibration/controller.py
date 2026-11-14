"""Deterministic state machine; no ROS imports and no background motion.

IO is injected so the *same* controller can be exercised without robot topics.
Every start, including after a label, requires a new terminal command.
"""
from dataclasses import dataclass, fields
import math
import time
import uuid

from .protocol import (CHANGE_MODE, GET_MODE, GET_UP, MOVE, PREPARE_MODE,
                       ROTATE_HEAD, SOCCER_MODE, VISUAL_KICK, distance_value,
                       power_value, target)


@dataclass(frozen=True)
class Config:
    kick_duration_s: float = 2.5
    target_reference_m: float = 6.0
    rpc_timeout_s: float = 2.0
    prepare_timeout_s: float = 35.0
    robot_timeout_s: float = 0.8
    recovery_timeout_s: float = 2.5
    ball_timeout_s: float = 1.0
    frame_timeout_s: float = 0.5
    confidence_threshold: float = 50.0
    record_timeout_s: float = 2.0
    visual_kick_version: int = 1
    getup_version: int = 1

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid config: ' + f.name)
        if self.visual_kick_version != 1 or self.getup_version != 1:
            raise ValueError('This demo uses the validated native V2 interfaces (wire value 1).')
        if self.confidence_threshold > 100:
            raise ValueError('confidence_threshold must be <= 100')


class Controller:
    def __init__(self, config, io, store, dry_run, clock=time.monotonic):
        self.c, self.io, self.store, self.dry_run, self.clock = config, io, store, dry_run, clock
        self.state = 'IDLE'
        self.feedback = {}
        self.trial = None
        self.pending = None
        self.prep_phase = None
        self.prep_started = 0
        self.phase_since = 0
        self.kick_since = 0
        self.stop_since = 0
        self.ready_after_stop = False
        self.stop_status = self.zero_status = None
        self.last_head = -100.0
        self.target_count = 0
        self.last_target_fresh = None
        self.external_controller = False
        self.recorder_ok = False
        self.event('session_idle', dry_run=dry_run)

    def event(self, kind, **data):
        event = self.store.event(kind, trial_id=self.trial['trial_id'] if self.trial else None, **data)
        self.io.event(event)

    def update(self, name, value, at=None):
        self.feedback[name] = (self.clock() if at is None else at, value)

    def value(self, name, default=None):
        return self.feedback.get(name, (None, default))[1]

    def fresh(self, name, timeout):
        t = self.feedback.get(name, (None, None))[0]
        return t is not None and 0 <= self.clock() - t <= timeout

    def reason(self, ball=False, soccer=True):
        if not self.recorder_ok:
            return '原始录制未就绪/心跳过期'
        if self.external_controller:
            return '检测到其他动作指令发布者'
        if not self.fresh('low', self.c.robot_timeout_s):
            return '关节反馈过期或不是 22 个串联关节'
        if not self.fresh('pose', self.c.robot_timeout_s):
            return '里程计反馈过期'
        if not self.fresh('recovery', self.c.recovery_timeout_s):
            return '恢复状态反馈过期'
        recovery, available, planner = self.value('recovery')
        if recovery != 0:
            return '未站立；需要 prepare'
        if soccer and (not self.fresh('mode', self.c.recovery_timeout_s)
                       or self.value('mode') != SOCCER_MODE or planner not in (12, 22)):
            return '原生踢球所需模式/规划器未就绪；需要 prepare'
        if ball and not self.fresh('ball', self.c.ball_timeout_s):
            return '没有新鲜有效的球观测'
        return None

    def _send(self, api, body, purpose):
        request_id = str(uuid.uuid4())
        self.pending = dict(uuid=request_id, api=api, body=body, purpose=purpose,
                            at=self.clock())
        self.event('rpc_request', uuid=request_id, api_id=api, body=body, purpose=purpose,
                   dry_run=self.dry_run)
        if self.trial and purpose in ('kick_start', 'kick_stop', 'zero'):
            prefix = {'kick_start': 'start', 'kick_stop': 'stop', 'zero': 'zero'}[purpose]
            self.trial[prefix + '_uuid'] = request_id
            self.trial[{'kick_start': 'request_unix_ns', 'kick_stop': 'stop_request_unix_ns',
                        'zero': 'zero_request_unix_ns'}[purpose]] = time.time_ns()
            self.store.save(self.trial)
        self.io.request(request_id, api, body)

    def _head(self, pitch, yaw):
        # Head requests do not replace the pending body/skill RPC.
        self.last_head = self.clock()
        request_id = str(uuid.uuid4())
        body = {'pitch': max(.25, min(1.1, pitch)), 'yaw': max(-1.0, min(1.0, yaw))}
        self.event('head_request', uuid=request_id, body=body, dry_run=self.dry_run)
        self.io.request(request_id, ROTATE_HEAD, body)

    def prepare(self):
        if self.state not in ('IDLE', 'READY', 'FAULT') or self.trial:
            raise ValueError('先结束当前动作并给当前样本填距离或 invalid。')
        if not self.recorder_ok or self.external_controller:
            raise ValueError('录制未就绪或有其他控制程序。')
        if not self.fresh('low', self.c.robot_timeout_s):
            raise ValueError('等待有效的 22 关节反馈。')
        self.state = 'PREPARING'
        self.prep_started = self.clock()
        self.prep_phase = 'query'
        self._send(GET_MODE, {}, 'prepare_query')

    def kick(self, text):
        power = power_value(text)
        if self.state != 'READY' or self.trial:
            raise ValueError('需要先 prepare；上一脚必须先填 distance 或 invalid。')
        reason = self.reason(ball=True)
        if reason:
            raise ValueError(reason)
        now = self.clock()
        self.trial = dict(trial_id='trial_' + uuid.uuid4().hex[:16], power=power,
                          dry_run=self.dry_run, valid=None, manual_distance_m=None,
                          invalid_reason=None, outcome='running', recording_ok=True,
                          reference_yaw_odom_rad=self.value('pose')[2],
                          target_reference_m=self.c.target_reference_m,
                          visual_kick_version=self.c.visual_kick_version,
                          configured_window_s=self.c.kick_duration_s,
                          pre_ball_age_s=now - self.feedback['ball'][0],
                          pre_pose=list(self.value('pose')))
        self.store.save(self.trial)  # Commit identity/input before any actuation.
        self.event('trial_created', power=power)
        self.kick_since = now
        self.target_count = 0
        self.last_target_fresh = None
        self.ready_after_stop = False
        self.state = 'STARTING'
        self._target()
        self._send(VISUAL_KICK, {'start': True, 'version': self.c.visual_kick_version}, 'kick_start')

    def _target(self):
        fresh = self.fresh('ball', self.c.ball_timeout_s)
        if self.last_target_fresh is not fresh:
            self.event('target_freshness', fresh=fresh)
            self.last_target_fresh = fresh
        if not fresh:
            return  # Same as AAA: never invent a ball or publish stale detections.
        values = target(self.value('ball'), self.value('pose'),
                        self.trial['reference_yaw_odom_rad'], self.trial['power'],
                        self.c.target_reference_m)
        self.io.target(values)
        self.target_count += 1
        if self.target_count == 1:
            self.trial['first_target'] = values
            self.store.save(self.trial)
            self.event('first_target', target=values)
        # The native skill owns body motion. Only the original head RPC is used.
        if self.clock() - self.last_head >= .2:
            self._head(math.atan2(.8, math.hypot(values['x'], values['y'])),
                       math.atan2(values['y'], values['x']))

    def stop(self, reason='operator_stop'):
        if self.state == 'STOPPING':
            return
        if self.state in ('IDLE', 'AWAIT_LABEL', 'FAULT') and self.pending is None:
            return
        self.ready_after_stop = reason == 'window_complete'
        if self.trial:
            self.trial.update(outcome=reason, action_window_s=self.clock() - self.kick_since,
                              target_message_count=self.target_count)
        self.state = 'STOPPING'
        self.stop_status = self.zero_status = None
        self.stop_since = self.clock()
        self.event('stopping', reason=reason)
        # Always exit VisualKick first; Move(0) is sent after its response/timeout.
        self._send(VISUAL_KICK, {'start': False, 'version': self.c.visual_kick_version}, 'kick_stop')

    def _zero(self):
        self._send(MOVE, {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0}, 'zero')

    def _stopped(self):
        self.pending = None
        if self.trial:
            if any(self.trial.get(k) != 0 for k in ('start_status', 'stop_status', 'zero_status')):
                self.ready_after_stop = False
            self.trial['stop_sequence_s'] = self.clock() - self.stop_since
            self.store.save(self.trial)
            self.state = 'AWAIT_LABEL'
        else:
            self.state = 'IDLE' if self.ready_after_stop else 'FAULT'
        self.event('stopped', ready_after_label=self.ready_after_stop,
                   confirmed=self.stop_status == 0 and self.zero_status == 0)

    def response(self, request_id, status, body):
        if not self.pending or request_id != self.pending['uuid']:
            return
        pending, self.pending = self.pending, None
        purpose = pending['purpose']
        self.event('rpc_response', uuid=request_id, api_id=pending['api'], purpose=purpose,
                   status=status, body=body, latency_s=self.clock() - pending['at'])
        if self.trial and purpose in ('kick_start', 'kick_stop', 'zero'):
            prefix = {'kick_start': 'start', 'kick_stop': 'stop', 'zero': 'zero'}[purpose]
            self.trial[prefix + '_status'] = status
            self.trial[prefix + '_response_unix_ns'] = time.time_ns()
            self.store.save(self.trial)
        if purpose == 'kick_stop':
            self.stop_status = status
            if status != 0:
                self.ready_after_stop = False
            self._zero()
        elif purpose == 'zero':
            self.zero_status = status
            if status != 0:
                self.ready_after_stop = False
            self._stopped()
        elif purpose == 'kick_start':
            if status == 0:
                self.state = 'KICKING'
            else:
                self.stop('firmware_rejected')
        elif purpose.startswith('prepare_'):
            if purpose == 'prepare_soccer' and status == 501:
                # The native get-up can finish in soccer mode itself. A 501
                # mode reply requires verification, never a blind retry.
                self.prep_phase = 'verify_soccer'
                self._send(GET_MODE, {}, 'prepare_verify_soccer')
                return
            if status != 0:
                self.stop('prepare_rejected')
            elif purpose == 'prepare_verify_soccer':
                if body.get('mode') != SOCCER_MODE:
                    self.stop('prepare_rejected')
                else:
                    self.update('mode', SOCCER_MODE)
                    self.prep_phase, self.phase_since = 'wait_soccer', self.clock()
            elif purpose == 'prepare_query':
                mode = body.get('mode')
                if not isinstance(mode, int) or mode < 0:
                    self.stop('invalid_mode_response')
                    return
                self.update('mode', mode)
                if mode == SOCCER_MODE and self.reason() is None:
                    self._prepared()
                elif mode != PREPARE_MODE:
                    self.prep_phase = 'prepare_ack'
                    self._send(CHANGE_MODE, {'mode': PREPARE_MODE}, 'prepare_mode')
                else:
                    self.prep_phase, self.phase_since = 'wait_stand', self.clock()
            else:
                self.prep_phase = {'prepare_mode': 'wait_stand', 'prepare_getup': 'wait_getup',
                                   'prepare_soccer': 'wait_soccer'}[purpose]
                self.phase_since = self.clock()

    def _prepared(self):
        self.state, self.prep_phase = 'READY', None
        self.event('prepared')
        self._head(.9, 0.0)

    def tick(self):
        now = self.clock()
        if self.state in ('PREPARING', 'STARTING', 'KICKING') and (
                not self.recorder_ok or self.external_controller):
            if self.trial:
                self.trial['recording_ok'] = self.recorder_ok
            self.stop('recording_failed' if not self.recorder_ok else 'external_controller')
        if self.pending and now - self.pending['at'] > self.c.rpc_timeout_s:
            pending, self.pending = self.pending, None
            self.event('rpc_timeout', uuid=pending['uuid'], purpose=pending['purpose'])
            if pending['purpose'] == 'kick_stop':
                self.ready_after_stop = False
                self._zero()
            elif pending['purpose'] == 'zero':
                self.ready_after_stop = False
                self._stopped()
            else:
                self.stop('rpc_timeout')
        if self.state == 'PREPARING' and self.pending is None:
            if now - self.prep_started > self.c.prepare_timeout_s:
                self.stop('prepare_timeout')
                return
            if not self.fresh('recovery', self.c.recovery_timeout_s):
                return
            recovery, available, planner = self.value('recovery')
            if self.prep_phase == 'wait_stand':
                if recovery == 2 and available and planner in (1, 2) and now - self.phase_since >= 2:
                    self.prep_phase = 'getup_ack'
                    self._send(GET_UP, {'version': self.c.getup_version}, 'prepare_getup')
                elif recovery == 0 and planner in (2, 8, 20, 12, 22) and now - self.phase_since >= 2:
                    self.prep_phase = 'soccer_ack'
                    self._send(CHANGE_MODE, {'mode': SOCCER_MODE}, 'prepare_soccer')
            elif (self.prep_phase == 'wait_getup' and recovery == 0
                  and planner in (8, 20, 12, 22) and now - self.phase_since >= 1):
                if self.reason() is None:
                    self._prepared()
                else:
                    self.prep_phase = 'soccer_ack'
                    self._send(CHANGE_MODE, {'mode': SOCCER_MODE}, 'prepare_soccer')
            elif self.prep_phase == 'wait_soccer' and self.reason() is None:
                self._prepared()
        if self.state in ('STARTING', 'KICKING'):
            reason = self.reason()
            if reason:
                self.event('feedback_guard', reason=reason)
                self.stop('feedback_stale_or_not_ready')
            elif now - self.kick_since >= self.c.kick_duration_s:
                self.stop('window_complete')
            else:
                self._target()

    def label(self, distance=None, invalid=None):
        if self.state != 'AWAIT_LABEL' or not self.trial:
            raise ValueError('当前没有已结束、等待标签的试验。')
        if invalid is None and (self.trial['outcome'] != 'window_complete'
                                or not self.ready_after_stop):
            raise ValueError('本次异常/中断，使用 invalid <原因>，不会进入训练导出。')
        self.trial.update(manual_distance_m=distance_value(distance) if invalid is None else None,
                          valid=invalid is None, invalid_reason=invalid, label_unix_ns=time.time_ns())
        self.event('manual_label', distance_m=self.trial['manual_distance_m'], invalid_reason=invalid)
        self.store.save(self.trial)
        result = self.trial['trial_id']
        self.trial = None
        self.state = 'READY' if self.ready_after_stop else 'IDLE'
        return result

    def status(self):
        return dict(state=self.state, trial_id=self.trial['trial_id'] if self.trial else None,
                    power=self.trial['power'] if self.trial else None, dry_run=self.dry_run,
                    outcome=self.trial.get('outcome') if self.trial else None,
                    response_status={k: self.trial.get(k + '_status') for k in ('start', 'stop', 'zero')}
                    if self.trial else {},
                    recorder_ok=self.recorder_ok, readiness=self.reason(ball=True),
                    feedback_age_s={k: round(self.clock() - v[0], 3)
                                    for k, v in self.feedback.items()})
