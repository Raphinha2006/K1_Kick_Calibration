"""Own only this session's processes; terminal input always stays in the parent."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import time
import uuid

from .storage import atomic_json


def terminate(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--no-vision', action='store_true', help='Reuse an already running vision source only')
    args = parser.parse_args()
    def interrupt_startup(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt_startup)
    root = Path(args.root).resolve()
    (root / 'logs').mkdir(exist_ok=True)
    lock = (root / 'logs' / 'session.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('已有 calibration 会话运行。')
    # Check old demos without stopping or modifying them.
    rows = subprocess.run(['ps', '-eo', 'args='], text=True, stdout=subprocess.PIPE, check=True).stdout.splitlines()
    processes = []
    for row in rows:
        try:
            processes.append(Path(shlex.split(row)[0]).name)
        except (ValueError, IndexError):
            continue
    conflicts = sorted(set(processes) & {'brain_node', 'person_pass_node', 'game_controller_node'})
    if conflicts:
        raise SystemExit('先停止其他 demo：' + ', '.join(conflicts))
    if 'vision_node' in processes and not args.no_vision:
        raise SystemExit('已有 Vision；先停止它或使用 --no-vision 明确复用。')
    folder = root / 'logs' / (time.strftime('calib_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6])
    folder.mkdir()
    atomic_json(root / 'logs/latest.json', dict(session=str(folder)))
    recorder = vision = None
    opened = []
    old_handlers = {}
    try:
        from .metadata import capture
        capture(root, folder, not args.real)
        if not args.no_vision:
            stream = (folder / 'vision.log').open('w')
            opened.append(stream)
            vision = subprocess.Popen([str(root / 'install/vision/lib/vision/vision_node'),
                str(root / 'src/vision/config/vision.yaml'), str(root / 'config/no_vision_override.yaml'),
                str(root / 'config/no_user_override.yaml'), '--ros-args', '-p', 'save_data:=false', '-p', 'save_depth:=false'],
                cwd=root, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        stream = (folder / 'recorder.log').open('w')
        opened.append(stream)
        recorder = subprocess.Popen([sys.executable, '-m', 'kick_calibration.recorder', '--session', str(folder)],
                                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 12
        ready = False
        while time.monotonic() < deadline:
            if recorder.poll() is not None:
                raise RuntimeError('录制进程退出；查看 ' + str(folder / 'recorder.log'))
            if vision is not None and vision.poll() is not None:
                raise RuntimeError('Vision 退出；查看 ' + str(folder / 'vision.log'))
            health_path = folder / 'recorder_health.json'
            if health_path.exists() and json.loads(health_path.read_text()).get('ready'):
                ready = True
                break
            time.sleep(.1)
        if not ready:
            raise RuntimeError('12 秒内未录到必需反馈；没有发送任何动作。查看 recorder_health.json。')
        from .runtime import run
        old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        run(folder, root / 'config/calibration.json', not args.real)
    finally:
        # Ignore repeat Ctrl+C while the controller has already performed its stop sequence.
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        terminate(recorder)
        terminate(vision)
        for stream in opened:
            stream.close()
        try:
            from .analyze import summarize
            from .storage import write_csv
            summarize(folder)
            print('训练导出有效样本数：', write_csv(folder, training_only=True), flush=True)
        except Exception as exc:
            print('原始数据保留；摘要尚未生成：' + str(exc), file=sys.stderr, flush=True)
        print('数据目录：' + str(folder), flush=True)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        lock.close()


if __name__ == '__main__':
    main()
