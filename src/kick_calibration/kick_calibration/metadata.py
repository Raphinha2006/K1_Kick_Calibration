import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import time

from .storage import atomic_json


def capture(root, folder, dry_run):
    root, folder = Path(root), Path(folder)
    def output(args):
        return subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              timeout=10).stdout.strip()

    # Copy exact experiment configurations and small interpretation schemas;
    # hash model binaries instead of duplicating them into each data session.
    paths = [root / 'config/calibration.json', root / 'src/vision/config/vision.yaml',
             root / 'config/no_vision_override.yaml', root / 'config/no_user_override.yaml',
             root / 'src/interface/brain/msg/Kick.msg',
             Path('/opt/booster/Gait/configs/K1/robot.urdf'), Path('/opt/booster/version.txt'),
             Path('/usr/local/include/booster/robot/b1/b1_loco_api.hpp'),
             Path('/usr/local/include/booster/robot/b1/b1_api_const.hpp'),
             Path('/usr/local/include/booster/robot/common/robot_shared.hpp'),
             Path('/opt/booster/Gait/configs/K1/common_module_options.lua')]
    paths += sorted((root / 'src/vision/model').glob('*.engine'))
    # Firmware options resolve lib/... relative to /opt/booster, not Gait/.
    paths += sorted(Path('/opt/booster/lib/motion').glob('k1_*policy*.bin'))
    fingerprint_dir = folder / 'metadata_files'
    fingerprint_dir.mkdir()
    fingerprints = []
    for index, path in enumerate(paths):
        if not path.is_file():
            fingerprints.append(dict(path=str(path), available=False))
            continue
        checksum = hashlib.sha256()
        with path.open('rb') as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b''):
                checksum.update(chunk)
        row = dict(path=str(path), available=True, size=path.stat().st_size, sha256=checksum.hexdigest())
        if path.stat().st_size <= 2_000_000:
            saved = fingerprint_dir / (str(index) + '_' + path.name)
            shutil.copy2(path, saved)
            row['saved_as'] = str(saved.relative_to(folder))
        fingerprints.append(row)
    schema_dir = Path('/opt/booster/BoosterRos2Interface/install/booster_interface/share/booster_interface/msg')
    for path in sorted(schema_dir.glob('*.msg')):
        if path.stem in ('LowState', 'MotorState', 'ImuState', 'BatteryState', 'RobotStatesMsg', 'Odometer', 'RawBytesMsg'):
            shutil.copy2(path, fingerprint_dir / path.name)
    mapping = ['Head_Yaw', 'Head_Pitch', 'Left_Shoulder_Pitch', 'Left_Shoulder_Roll', 'Left_Elbow_Pitch',
               'Left_Elbow_Yaw', 'Right_Shoulder_Pitch', 'Right_Shoulder_Roll', 'Right_Elbow_Pitch',
               'Right_Elbow_Yaw']
    # Confirm against SDK JointIndex, and preserve the runtime JointState names too.
    mapping += [side + '_' + suffix for side in ('Left', 'Right') for suffix in
                ('Hip_Pitch', 'Hip_Roll', 'Hip_Yaw', 'Knee_Pitch', 'Ankle_Pitch', 'Ankle_Roll')]
    atomic_json(folder / 'metadata.json', dict(
        created_unix_ns=time.time_ns(), dry_run=dry_run, hostname=platform.node(),
        platform=platform.platform(), python=platform.python_version(),
        git_commit=output(['git', '-C', str(root), 'rev-parse', 'HEAD']),
        git_status=output(['git', '-C', str(root), 'status', '--porcelain']),
        ros_distro=os.environ.get('ROS_DISTRO'), ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
        rmw=os.environ.get('RMW_IMPLEMENTATION', 'default'),
        joint_serial_index_to_name=dict(enumerate(mapping)),
        units=dict(q='rad', dq='rad/s', ddq='rad/s^2', tau_est='estimated Nm',
                   manual_distance='m', power='native numeric input, physical scale not assumed'),
        ground_truth='straight-line distance: actual kick-out point to natural stopping point',
        timestamp_notes='LowState has no header; bag time is receipt time, not contact time',
        fingerprints=fingerprints))
