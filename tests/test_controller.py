import json
import math
from pathlib import Path
import tempfile
import unittest

from kick_calibration.controller import Config, Controller
from kick_calibration.protocol import (CHANGE_MODE, GET_MODE, GET_UP, MOVE, VISUAL_KICK,
                                       decode_response, power_value, target)
from kick_calibration.storage import Store, read_trials, relabel, write_csv, atomic_json, recover


class FakeIO:
    def __init__(self):
        self.requests, self.targets, self.events = [], [], []

    def request(self, request_id, api, body):
        self.requests.append((request_id, api, body))

    def target(self, values):
        self.targets.append(values)

    def event(self, event):
        self.events.append(event)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.folder = Path(self.tmp.name)
        self.store = Store(self.folder)
        self.io = FakeIO()
        self.now = 100.0
        self.core = Controller(Config(), self.io, self.store, False, lambda: self.now)
        self.refresh()

    def tearDown(self):
        self.store.close()
        self.tmp.cleanup()

    def refresh(self, **overrides):
        feedback = dict(low=True, pose=(0., 0., 0.), recovery=(0, True, 12), mode=4, ball=(.6, 0.))
        feedback.update(overrides)
        for key, value in feedback.items():
            self.core.update(key, value)
        self.core.recorder_ok = True

    def ack(self, status=0, body=None):
        self.assertIsNotNone(self.core.pending)
        self.core.response(self.core.pending['uuid'], status, body or {})

    def prepare(self):
        self.core.prepare()
        self.assertEqual(self.core.pending['api'], GET_MODE)
        self.ack(body={'mode': 4})
        self.assertEqual(self.core.state, 'READY')

    def finish(self):
        self.now += 7.01
        self.refresh()
        self.core.tick()
        self.assertEqual(self.core.state, 'STOPPING')
        self.assertEqual(self.core.pending['body']['start'], False)
        self.ack()
        self.assertEqual(self.core.pending['api'], MOVE)
        self.ack()
        self.assertEqual(self.core.state, 'AWAIT_LABEL')

    def test_startup_never_actuates(self):
        self.core.tick()
        self.assertEqual(self.io.requests, [])
        with self.assertRaises(ValueError): self.core.kick('3')

    def test_native_response_status_does_not_require_object_body(self):
        for body in ('', '{}', 'null', '"ok"', 'OK'):
            self.assertEqual(decode_response('{"status":0}', body)[0], 0)
        for header in ('null', '{}', '{"status":"0"}', 'bad'):
            with self.assertRaises(ValueError): decode_response(header, '')

    def test_power_range_and_identity(self):
        for value in ('nan', 'inf', '-inf', '0', '.9', '10.01', 'oops'):
            with self.assertRaises(ValueError): power_value(value)
        self.prepare()
        self.core.kick('3.125')
        self.assertEqual(self.io.targets[0]['power'], 3.125)
        self.assertEqual(self.core.pending['body'], {'start': True, 'version': 1})
        self.ack()
        self.now += .2
        self.refresh()
        self.core.tick()
        self.assertTrue(all(t['power'] == 3.125 for t in self.io.targets))

    def test_direction_compensates_turn_and_goal_stays_six_metres(self):
        result = target((.6, 0), (0, 0, math.pi/2), 0, 8.25, 6)
        self.assertAlmostEqual(result['dir'], -math.pi/2)
        self.assertAlmostEqual(result['goal_x'], 0)
        self.assertAlmostEqual(result['goal_y'], -6.6)
        self.assertAlmostEqual(math.hypot(result['goal_x']-result['x'], result['goal_y']-result['y']), 6)
        self.assertAlmostEqual(result['robot_theta_to_field'], math.pi/2)
        self.assertEqual(result['power'], 8.25)

    def test_wrap_boundary(self):
        result = target((1, 0), (0, 0, -math.pi+.01), math.pi-.01, 1, 6)
        self.assertAlmostEqual(result['dir'], -.02)

    def test_label_gate_and_no_auto_next_attempt(self):
        self.prepare()
        self.core.kick('2.75')
        trial_id = self.core.trial['trial_id']
        with self.assertRaises(ValueError): self.core.kick('4')
        with self.assertRaises(ValueError): self.core.label(distance='2')
        self.ack()
        self.finish()
        with self.assertRaises(ValueError): self.core.kick('4')
        with self.assertRaises(ValueError): self.core.label(distance='nan')
        self.assertEqual(self.core.label(distance='2.8'), trial_id)
        count = len(self.io.requests)
        self.core.tick()
        self.assertEqual(len(self.io.requests), count)
        self.assertEqual(read_trials(self.folder)[0]['manual_distance_m'], 2.8)

    def test_rejected_start_exits_before_zero_and_cannot_label_valid(self):
        self.prepare()
        self.core.kick('4')
        self.ack(status=501)
        self.assertEqual(self.core.pending['body'], {'start': False, 'version': 1})
        self.ack()
        self.assertEqual(self.core.pending['api'], MOVE)
        self.ack()
        with self.assertRaises(ValueError): self.core.label(distance='2')
        self.core.label(invalid='firmware rejected')
        self.assertEqual(self.core.state, 'IDLE')

    def test_feedback_stale_stops_native_without_nonzero_walk(self):
        self.prepare()
        self.core.kick('3')
        self.ack()
        self.now += .9
        self.core.tick()
        self.assertEqual(self.core.state, 'STOPPING')
        self.ack()
        self.ack()
        moves = [r[2] for r in self.io.requests if r[1] == MOVE]
        self.assertEqual(moves, [{'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0}])

    def test_recording_failure_stops_and_marks_sample(self):
        self.prepare()
        self.core.kick('3')
        self.ack()
        self.core.recorder_ok = False
        self.core.tick()
        self.assertFalse(self.core.trial['recording_ok'])
        self.assertEqual(self.core.trial['outcome'], 'recording_failed')
        self.assertEqual(self.core.state, 'STOPPING')

    def test_recording_failure_prevents_prepare_and_kick(self):
        self.core.recorder_ok = False
        with self.assertRaises(ValueError): self.core.prepare()
        self.core.recorder_ok = True
        self.prepare()
        self.core.recorder_ok = False
        with self.assertRaises(ValueError): self.core.kick('3')

    def test_external_controller_stops(self):
        self.prepare()
        self.core.kick('3')
        self.ack()
        self.core.external_controller = True
        self.core.tick()
        self.assertEqual(self.core.trial['outcome'], 'external_controller')

    def test_shutdown_and_late_responses_do_not_restart(self):
        self.prepare()
        self.core.kick('3')
        start_id = self.core.pending['uuid']
        self.core.stop('shutdown')
        self.core.response(start_id, 0, {})
        self.assertEqual(self.core.state, 'STOPPING')
        self.ack()
        self.ack()
        self.assertEqual(self.core.trial['outcome'], 'shutdown')

    def test_stop_timeout_still_sends_zero(self):
        self.prepare()
        self.core.kick('3')
        self.ack()
        self.core.stop()
        self.now += 2.1
        self.core.tick()
        self.assertEqual(self.core.pending['api'], MOVE)
        self.ack()
        self.assertFalse(self.core.ready_after_stop)

    def test_start_timeout_and_cancel_prepare(self):
        self.prepare()
        self.core.kick('3')
        self.now += 2.1
        self.refresh()
        self.core.tick()
        self.assertEqual(self.core.pending['body']['start'], False)
        self.assertEqual(self.core.trial['outcome'], 'rpc_timeout')

    def test_getup_only_follows_explicit_prepare_with_bounded_sequence(self):
        self.refresh(mode=1, recovery=(2, True, 2))
        self.core.tick()
        self.assertFalse(self.io.requests)
        self.core.prepare()
        self.ack(body={'mode': 1})
        self.now += 2.1
        self.refresh(mode=1, recovery=(2, True, 2))
        self.core.tick()
        self.assertEqual(self.core.pending['api'], GET_UP)
        self.ack()
        self.now += 1.1
        self.refresh(mode=1, recovery=(0, True, 23))
        self.core.tick()
        self.assertEqual(self.core.pending['api'], CHANGE_MODE)
        self.assertEqual(self.core.pending['body'], {'mode': 4})
        self.ack()
        self.refresh()
        self.core.tick()
        self.assertEqual(self.core.state, 'READY')

    def test_no_mode_switch_or_walk_during_native_kick(self):
        self.prepare()
        count = len(self.io.requests)
        self.core.kick('5')
        self.ack()
        for _ in range(10):
            self.now += .05
            self.refresh()
            self.core.tick()
        self.assertFalse(any(api in (MOVE, CHANGE_MODE, GET_UP) for _, api, _ in self.io.requests[count:]))

    def test_lost_ball_is_not_fabricated_and_does_not_claim_contact(self):
        self.prepare()
        self.core.kick('5')
        self.ack()
        count = len(self.io.targets)
        self.now += 1.1
        self.refresh()
        del self.core.feedback['ball']
        self.core.tick()
        self.assertEqual(len(self.io.targets), count)
        self.assertEqual(self.core.state, 'KICKING')

    def test_training_excludes_unlabelled_invalid_dryrun_and_incomplete(self):
        self.prepare()
        self.core.kick('5')
        self.ack()
        self.finish()
        trial_id = self.core.trial['trial_id']
        self.assertEqual(write_csv(self.folder, True), 0)
        self.core.label(distance='3.7')
        self.assertEqual(write_csv(self.folder, True), 0)  # Not analyzed yet.
        atomic_json(self.folder/'summary.json', {trial_id: {'data_complete': True}})
        self.assertEqual(write_csv(self.folder, True), 1)
        relabel(self.folder, trial_id, invalid='two contacts')
        self.assertEqual(write_csv(self.folder, True), 0)
        relabel(self.folder, trial_id, distance='3.9')
        self.assertEqual(write_csv(self.folder, True), 1)
        row = read_trials(self.folder)[0]
        row['dry_run'] = True
        self.store.save(row)
        self.assertEqual(write_csv(self.folder, True), 0)

    def test_recover_marks_interrupted_trial_without_inventing_result(self):
        self.prepare()
        self.core.kick('3')
        recover(self.folder)
        row = read_trials(self.folder)[0]
        self.assertEqual(row['outcome'], 'unclean_shutdown')
        self.assertFalse(row['valid'])
        self.assertIsNone(row['manual_distance_m'])


if __name__ == '__main__':
    unittest.main()
