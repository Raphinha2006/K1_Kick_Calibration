"""Durable trial labels. Raw observations live in rosbag, never in Git."""
import csv
import json
import math
import os
from pathlib import Path
import time

from .protocol import distance_value


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('w') as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    # Persist the rename as well as file contents on Linux/macOS.
    fd = os.open(str(path.parent), os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def read_trials(folder):
    return [json.loads(p.read_text()) for p in sorted((Path(folder) / 'trials').glob('*.json'))]


def training_eligible(row):
    return (row.get('valid') is True and not row.get('dry_run', True)
            and row.get('start_status') == 0 and row.get('stop_status') == 0
            and row.get('zero_status') == 0 and row.get('outcome') == 'window_complete'
            and row.get('recording_ok') is True
            and isinstance(row.get('manual_distance_m'), (int, float))
            and math.isfinite(row['manual_distance_m']) and row['manual_distance_m'] >= 0)


def write_csv(folder, training_only=False):
    folder = Path(folder)
    rows = read_trials(folder)
    summaries = {}
    summary_path = folder / 'summary.json'
    if summary_path.exists():
        summaries = json.loads(summary_path.read_text())
    rows = [{**r, **summaries.get(r['trial_id'], {})} for r in rows]
    rows = [r for r in rows if not training_only or (training_eligible(r) and r.get('data_complete') is True)]
    columns = ['trial_id', 'power', 'manual_distance_m', 'valid', 'invalid_reason',
               'dry_run', 'outcome', 'start_status', 'stop_status', 'zero_status',
               'recording_ok', 'request_unix_ns', 'start_response_unix_ns',
               'stop_request_unix_ns', 'stop_response_unix_ns', 'action_window_s',
               'reference_yaw_odom_rad', 'target_reference_m', 'visual_kick_version']
    extra = sorted(set().union(*(r.keys() for r in rows)) - set(columns)) if rows else []
    destination = folder / ('training.csv' if training_only else 'trials.csv')
    temporary = destination.with_name(destination.name + '.tmp')
    with temporary.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns + extra)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, ensure_ascii=False, allow_nan=False)
                             if isinstance(v, (dict, list)) else v for k, v in row.items()})
        f.flush()
        os.fsync(f.fileno())
    os.replace(temporary, destination)
    return len(rows)


class Store:
    def __init__(self, folder):
        self.folder = Path(folder)
        (self.folder / 'trials').mkdir(parents=True, exist_ok=True)
        self.journal = (self.folder / 'events.jsonl').open('a', buffering=1)

    def event(self, kind, **fields):
        event = dict(kind=kind, unix_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(), **fields)
        self.journal.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        self.journal.flush()
        os.fsync(self.journal.fileno())
        return event

    def save(self, row):
        atomic_json(self.folder / 'trials' / (row['trial_id'] + '.json'), row)
        write_csv(self.folder)

    def close(self):
        self.journal.close()


def relabel(folder, trial_id, distance=None, invalid=None):
    matches = [r for r in read_trials(folder) if r['trial_id'] == trial_id]
    if len(matches) != 1:
        raise ValueError('找不到唯一的 trial ID。')
    row = matches[0]
    if row.get('outcome') in (None, 'running'):
        raise ValueError('试验尚未结束；中断进程留下的记录请先标 invalid。' if invalid is None
                         else '先使用 recover 将中断试验标记为无效。')
    if invalid is None and row.get('outcome') != 'window_complete':
        raise ValueError('异常/中断试验只能标记为 invalid。')
    previous = {k: row.get(k) for k in ('valid', 'manual_distance_m', 'invalid_reason')}
    row.update(valid=invalid is None, manual_distance_m=distance_value(distance) if invalid is None else None,
               invalid_reason=invalid, label_unix_ns=time.time_ns())
    store = Store(folder)
    try:
        store.event('manual_relabel', trial_id=trial_id, previous=previous,
                    distance_m=row['manual_distance_m'], invalid_reason=invalid)
        store.save(row)
    finally:
        store.close()


def recover(folder):
    store = Store(folder)
    try:
        for row in read_trials(folder):
            if row.get('outcome') == 'running':
                row.update(outcome='unclean_shutdown', valid=False, recording_ok=False,
                           invalid_reason='unclean_shutdown; native completion unverified')
                store.event('recover_interrupted_trial', trial_id=row['trial_id'])
                store.save(row)
    finally:
        store.close()
