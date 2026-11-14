#!/usr/bin/env python3
"""Verify separately stored vendor models/libraries against the copied baseline."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
rows = json.loads((root/'vendor_reference/runtime_artifacts.json').read_text())
failures = []
for row in rows:
    path = root/row['path']
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']:
        failures.append(row['path'])
if failures:
    raise SystemExit('Missing/mismatched vendor artifacts:\n'+'\n'.join(failures))
print('Verified vendor artifacts:', len(rows))
