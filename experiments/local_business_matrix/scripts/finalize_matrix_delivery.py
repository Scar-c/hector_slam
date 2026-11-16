#!/usr/bin/env python3
"""Package only fully verified 1x matrix results and their source provenance."""
import csv,hashlib,json,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
results=Path(sys.argv[1]);support=Path('/tmp/localization_matrix_support_path.txt').read_text().strip();support=Path(support)
report=json.loads((results/'matrix_validation.json').read_text());audit=json.loads((results/'offline_parameter_audit.json').read_text())
assert report['passed'] and report['cases']==24 and report['rate']==1
assert audit['complete'] and audit['verified_cases']==24
rows=sorted(report['results'],key=lambda r:(r['initializer'],list(json.loads((ROOT/'scripts/matrix_profiles.json').read_text())).index(r['profile'])))
for r in rows:
 assert r['passed']
 m=Path('/home/lc/slam_11_13/overview')/(r['profile']+'__'+r['initializer']+'.json');meta=json.loads(m.read_text())
 # A user may replay a profile while packaging; overview intentionally keeps its latest image.
 assert meta['replay_rate']==1 and meta['profile']==r['profile'] and meta['initializer']==r['initializer']
 assert (Path(r['output'])/'localization_overview.png').exists()
fields=['initializer','profile','poses','business_poses','scan_frames','beam_agreement','seed_sha256','output']
with (results/'matrix_summary.csv').open('w') as f:
 writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows({k:r[k] for k in fields} for r in rows)
lines=['# 定位矩阵：1倍速验收结果','','24/24 组完整回放通过。所有回放速度均为1倍速，两个独立ROS域各自运行；扫描几何与参数审计在离线完成。','',
       '两种初始化分别共享同一缓存位姿。全部先验地图保持不变，EKF配置一致；没有增加地图校正量低通。','',
       '三模块图统一保存在 [overview](/home/lc/slam_11_13/overview/README.md)，文件名为 `配置名__初始化来源.png`，右上角图例标明实际配置。','',
       '命令见 [test.txt](/home/lc/slam_11_13/test.txt)，完整说明见 [README_LOCALIZATION_MATRIX.md](/home/lc/slam_11_13/scripts/README_LOCALIZATION_MATRIX.md)。','',
       '验证范围：原始传感器消息收齐，发布地图与PGM/YAML一致且只有一个版本；Hector先验各层不更新；扫描由原始数据和记录TF重建通过；实际启动参数与处理开关符合矩阵；同一来源的初始位姿/SHA一致；Hector地图校正为直接TF组合；全部图片生成。该验收不是真值精度评估。','',
       'desk遵守LaserScan逐束时间模型；驱动真实曝光方向与偏移尚未物理标定。此前加速回放目录已标记为无效诊断记录，不属于本次验收。','',
       '| 初始化 | 配置 | 原始定位帧 | 业务位姿帧 | 扫描帧 | 重建一致率 | 图片 |','|---|---|---:|---:|---:|---:|---|']
for r in rows:
 name=r['profile']+'__'+r['initializer'];lines.append(f"| {r['initializer']} | {r['profile']} | {r['poses']} | {r['business_poses']} | {r['scan_frames']} | {r['beam_agreement']:.6%} | [查看](/home/lc/slam_11_13/overview/{name}.png) |")
(results/'README.md').write_text('\n'.join(lines)+'\n')
for cache_key in {r['cache_key'] for r in rows}:
 target=support/'shared_initialization'/cache_key
 shutil.copytree(ROOT/'initialization_cache'/cache_key,target,dirs_exist_ok=True)
tests=support/'test_reports';tests.mkdir(exist_ok=True)
for p in (ROOT/'build/hector_mapping/test_results').rglob('*.xml'):shutil.copy2(p,tests/p.name)
for p in (ROOT/'initializers/build/test_results').rglob('*.xml'):shutil.copy2(p,tests/p.name)
shutil.copy2(ROOT/'README_LOCALIZATION_MATRIX.md','/home/lc/slam_11_13/scripts/README_LOCALIZATION_MATRIX.md')
shutil.copy2(results/'matrix_validation.json',support/'matrix_validation_1x.json')
shutil.copy2(results/'offline_parameter_audit.json',support/'offline_parameter_audit_1x.json')
archive=support/'hector_prior_matrix_source_1x.tar.gz'
subprocess.run(['tar','-czf',str(archive),'--exclude=initializers/build','--exclude=__pycache__','-C',str(ROOT),'src','scripts','initializers','dependencies/fftw/prefix','dependencies/evo_requirements.txt','dependencies/evo_runtime_versions.json','README_LOCALIZATION_MATRIX.md','public_matrix'],check=True)
manifest={'playback_rate':1,'verified_cases':24,'results':str(results),'overview':'/home/lc/slam_11_13/overview','source_archive':str(archive),'source_archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'plot_script':'/home/lc/slam_11_13/scripts/plot_localization.py','plot_script_sha256':hashlib.sha256(Path('/home/lc/slam_11_13/scripts/plot_localization.py').read_bytes()).hexdigest(),'same_cached_seed_per_method':report['same_cached_seed_within_initializer'],'audit':str(results/'offline_parameter_audit.json')}
(support/'final_delivery_manifest_1x.json').write_text(json.dumps(manifest,indent=2))
(support/'README.md').write_text('# 最终交付\n\n有效源码快照为 `hector_prior_matrix_source_1x.tar.gz`，验收记录为 `matrix_validation_1x.json`。此前较早源码快照保留供回溯，最终入口强制仅允许1倍速回放。`legacy_entries/`保留覆盖前的脚本及绘图脚本。\n')
print(json.dumps(manifest,indent=2))
