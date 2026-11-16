#!/usr/bin/env bash
set -eo pipefail
python3 - <<'STOP_PY'
import os
from pathlib import Path
import signal
import time

script = '/home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/scripts/run_prior_replay.py'
pids = []
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        args = (entry / 'cmdline').read_bytes().split(b'\0')
        if script.encode() in args and args and b'python' in Path(os.fsdecode(args[0])).name.encode():
            pids.append(int(entry.name))
    except (OSError, ValueError):
        pass
for pid in pids:
    try:
        os.kill(pid, signal.SIGINT)
        print(f'Stopping previous Hector localization session: {pid}', flush=True)
    except ProcessLookupError:
        pass
end = time.monotonic() + 45
while pids and time.monotonic() < end:
    remaining = []
    for pid in pids:
        try:
            if Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z':
                remaining.append(pid)
        except OSError:
            pass
    pids = remaining
    if pids:
        time.sleep(.2)
if pids:
    raise SystemExit(f'Previous session is still cleaning up: {pids}; try again after it exits.')
STOP_PY
