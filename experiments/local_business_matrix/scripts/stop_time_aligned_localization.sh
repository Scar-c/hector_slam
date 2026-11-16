#!/usr/bin/env bash
set -eo pipefail
python3 - "$@" <<'PY'
import os,sys,signal,time
from pathlib import Path
algorithm=sys.argv[1] if len(sys.argv)>1 else 'all'
if algorithm not in ['hector','amcl','all']:raise SystemExit('Use hector, amcl or all')
script=b'/home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/scripts/run_time_aligned_replay.py'
pids=[]
for entry in Path('/proc').iterdir():
 if not entry.name.isdigit():continue
 try:
  args=(entry/'cmdline').read_bytes().split(b'\0')
  if script not in args or not args or b'python' not in Path(os.fsdecode(args[0])).name.encode():continue
  method=args[args.index(b'--algorithm')+1].decode() if b'--algorithm' in args else 'hector'
  if algorithm=='all' or method==algorithm:pids.append(int(entry.name))
 except (OSError,ValueError,IndexError):pass
for pid in pids:
 try:os.kill(pid,signal.SIGINT);print('Stopping aligned localization session:',pid,flush=True)
 except ProcessLookupError:pass
end=time.monotonic()+40
while pids and time.monotonic()<end:
 remaining=[]
 for pid in pids:
  try:
   if Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[0]!='Z':remaining.append(pid)
  except OSError:pass
 pids=remaining
 if pids:time.sleep(.2)
if pids:raise SystemExit('Previous session is still cleaning up: '+str(pids))
PY
