#!/usr/bin/env python3
"""Reuse the original plotting script to collect validated 1x matrix images."""
import argparse,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('results',type=Path);p.add_argument('--require-complete',action='store_true');args=p.parse_args()
profiles=json.loads((ROOT/'scripts/matrix_profiles.json').read_text());overview=Path('/home/lc/slam_11_13/overview');overview.mkdir(exist_ok=True)
rows=[]
for method in ['business_fft','hector_search']:
 for name,features in profiles.items():
    folder=args.results/method/name
    if not (folder/'summary.json').exists():continue
    summary=json.loads((folder/'summary.json').read_text())
    if summary['replay_rate']!=1 or not summary.get('all_sensor_messages_received'):continue
    matrix=name+'__'+method;target=overview/(matrix+'.png');meta=overview/(matrix+'.json')
    current=json.loads(meta.read_text()) if meta.exists() else {}
    if not target.exists() or current.get('source_result')!=str(folder.resolve()):
        algorithm='business' if features['algorithm']=='amcl' else ('hector_ekf' if features['ekf'] else 'hector')
        subprocess.run(['python3','/home/lc/slam_11_13/scripts/plot_localization.py','--method',algorithm,'--output',str(folder)],check=True)
    rows.append((matrix,folder,features))
if args.require_complete and len(rows)!=24:raise SystemExit(f'Only {len(rows)}/24 complete 1x cases')
text=['# 定位矩阵结果图','',f'当前收集 {len(rows)}/24 组。全部来自1倍速完整回放；图片由已有 plot_localization.py 生成。',
      '','命名：`配置名__初始化来源.png`；右上角图例显示配置名、初始化来源和 EKF/pre/desk/time 开关。',
      '','business_fft = 业务FFT核心；hector_search = 自写Hector候选搜索初始化。两种来源各自使用统一缓存初值。',
      '','| 矩阵名 | 图片 | 原始结果 |','|---|---|---|']
for matrix,folder,_ in rows:text.append(f'| {matrix} | [查看]({matrix}.png) | [{folder.name}]({folder}) |')
(overview/'README.md').write_text('\n'.join(text)+'\n')
print('Collected',len(rows),'matrix images in',overview)
