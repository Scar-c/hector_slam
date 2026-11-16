#!/usr/bin/env python3
"""Stage public shell entries without changing unrelated localization scripts."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1]
profiles=json.loads((ROOT/'scripts/matrix_profiles.json').read_text())
out=ROOT/'public_matrix';out.mkdir(exist_ok=True)
for name,p in profiles.items():
    config='amcl_ekf_time_aligned.rviz' if p['algorithm']=='amcl' else ('hector_ekf_time_aligned.rviz' if p['ekf'] else 'hector.rviz')
    script='''#!/usr/bin/env bash
set -eo pipefail
MATRIX_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MATRIX_BAG_ROOT="$(cd "$MATRIX_SCRIPTS/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash run_@PROFILE@_localization.sh [NEW_OUTPUT_DIRECTORY] [--initializer business_fft|hector_search] [--no-rviz] [--no-hold] [--rate 1]'
  echo 'Profile @PROFILE@; EKF=@EKF@ pre=@PRE@ desk=@DESK@ time=@TIME@. Default initializer: business_fft.'
  echo 'Opens RViz, waits 5 seconds, replays from the beginning, saves the existing three-panel plot, then holds RViz. Ctrl+C exits.'
  exit 0
fi
MATRIX_INITIALIZER=business_fft
MATRIX_PREVIOUS=''
for MATRIX_ARGUMENT in "$@"; do
  if [[ "$MATRIX_PREVIOUS" == --initializer ]]; then MATRIX_INITIALIZER="$MATRIX_ARGUMENT"; fi
  if [[ "$MATRIX_ARGUMENT" == --initializer=* ]]; then MATRIX_INITIALIZER="${MATRIX_ARGUMENT#*=}"; fi
  MATRIX_PREVIOUS="$MATRIX_ARGUMENT"
done
case "$MATRIX_INITIALIZER" in business_fft|hector_search) ;; *) echo 'Invalid initializer' >&2; exit 2;; esac
if [[ $# -gt 0 && "$1" != --* ]]; then
  MATRIX_OUTPUT="$1"
  shift
else
  MATRIX_OUTPUT="$MATRIX_BAG_ROOT/results/@PROFILE@_${MATRIX_INITIALIZER}_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
fi
if [[ -e "$MATRIX_OUTPUT" ]]; then echo "Result directory already exists: $MATRIX_OUTPUT" >&2; exit 2; fi
bash "$MATRIX_SCRIPTS/stop_localization_matrix.sh" @PROFILE@
export ROS_DOMAIN_ID=@DOMAIN@
exec bash @ROOT@/scripts/run_localization_matrix.sh \
  --profile @PROFILE@ --expected-domain @DOMAIN@ --output "$MATRIX_OUTPUT" \
  --rviz-config "$MATRIX_SCRIPTS/@CONFIG@" "$@"
'''
    replacements={'PROFILE':name,'EKF':str(p['ekf']).lower(),'PRE':str(p['pre']).lower(),'DESK':str(p['desk']).lower(),'TIME':str(p['time']).lower(),'DOMAIN':str(p['domain']),'CONFIG':config,'ROOT':str(ROOT)}
    for key,value in replacements.items():script=script.replace('@'+key+'@',value)
    path=out/f'run_{name}_localization.sh';path.write_text(script);path.chmod(0o755)
(out/'run_hector_ekf_localization.sh').write_text('''#!/usr/bin/env bash
# Compatibility: pre = EKF motion prediction used by the Hector matcher.
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_hector_ekf_pre_localization.sh" "$@"
''')
(out/'stop_localization_matrix.sh').write_text('''#!/usr/bin/env bash
set -e
python3 - "${1:-all}" <<'PY_STOP'
import os,signal,sys,time
from pathlib import Path
wanted=sys.argv[1]
runner='/home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/scripts/run_localization_matrix.py'
targets=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    try:args=(entry/'cmdline').read_bytes().decode().split('\\0')
    except (OSError,UnicodeError):continue
    if runner not in args or '--profile' not in args:continue
    profile=args[args.index('--profile')+1]
    if wanted!='all' and profile!=wanted:continue
    targets.append(int(entry.name))
for pid in targets:
    print('Stopping matrix replay',pid,flush=True)
    try:os.kill(pid,signal.SIGINT)
    except ProcessLookupError:pass
end=time.monotonic()+15
while time.monotonic()<end:
    live=[]
    for pid in targets:
        try:
            if Path('/proc',str(pid),'stat').read_text().split()[2]!='Z':live.append(pid)
        except FileNotFoundError:pass
    if not live:break
    time.sleep(.1)
else:raise SystemExit('Previous replay is still cleaning up; wait and retry.')
PY_STOP
''')
for path in out.glob('*.sh'):path.chmod(0o755)
lines=['# 初始化来源：默认业务 FFT；也可在每条命令后加 --initializer hector_search。',
       '# pre=EKF运动预测初值；desk=扫描内逐束去畸变；time=前后雷达统一参考时刻。',
       '# 默认打开RViz，等待5秒后从头播放；全部自动生成三模块图到results。','']
for name in profiles:
    lines += [name,f'bash /home/lc/slam_11_13/scripts/run_{name}_localization.sh','']
lines += ['# 原有命令保留为 hector_ekf_pre 的兼容入口：','bash /home/lc/slam_11_13/scripts/run_hector_ekf_localization.sh','',
          '# 切换为同一份自写Hector搜索初值，例如：','bash /home/lc/slam_11_13/scripts/run_amcl_ekf_pre_localization.sh --initializer hector_search',
          '# 关闭本矩阵会话：','bash /home/lc/slam_11_13/scripts/stop_localization_matrix.sh all','']
(out/'test.txt').write_text('\n'.join(lines))
print('Staged',len(list(out.glob('*.sh'))),'shell entries in',out)
