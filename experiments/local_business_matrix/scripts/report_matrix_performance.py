#!/usr/bin/env python3
"""Offline reports and charts from Linux CPU/PSS resource samples."""
import argparse,csv,json,os
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('results',type=Path);p.add_argument('--require-complete',action='store_true');args=p.parse_args()
report_path=args.results/'performance_results.json';progress=args.results/'performance_progress.json'
report=json.loads(report_path.read_text()) if report_path.exists() else {'results':json.loads(progress.read_text())}
results=[r for r in report['results'] if r['initializer']=='business_fft'];environment=json.loads((args.results/'benchmark_environment.json').read_text());profiles=json.loads((ROOT/'scripts/matrix_profiles.json').read_text())
if args.require_complete:assert len(results)==12 and all(r['passed'] and r['full_bag'] for r in results)
rows=[];details=[]
for case in results:
    if not case['passed']:continue
    for window,w in case['windows'].items():
        for group,g in w['groups'].items():
            if not g['cpu_percent'] or not g['pss_mib']:continue
            details.append({'profile':case['profile'],'initializer':case['initializer'],'window':window,'group':group,'cpu_mean_percent':g['cpu_percent']['mean'],'cpu_p95_percent':g['cpu_percent']['p95'],'cpu_peak_percent':g['cpu_percent']['peak'],'logical_cores_mean':g['logical_cores']['mean'],'machine_cpu_mean_percent':g['machine_cpu_percent']['mean'],'pss_mean_mib':g['pss_mib']['mean'],'pss_p95_mib':g['pss_mib']['p95'],'pss_peak_mib':g['pss_mib']['peak'],'threads_peak':g['threads_peak'],'window_seconds':w['window_seconds'],'covered_seconds':w['covered_seconds'],'missing_pss_samples':g['missing_pss_samples']})
    w=case['windows']['steady'];g=w['groups']['pipeline'];local=w['groups']['localizer'];harness=w['groups']['harness'];monitor=w['groups']['monitor'];host=w['host_logical_cores']['mean']
    rows.append({'profile':case['profile'],'initializer':case['initializer'],'pipeline_cpu_mean_percent':g['cpu_percent']['mean'],'pipeline_cpu_p95_percent':g['cpu_percent']['p95'],'pipeline_cpu_peak_percent':g['cpu_percent']['peak'],'logical_cores_mean':g['logical_cores']['mean'],'logical_cores_p95':g['logical_cores']['p95'],'machine_cpu_mean_percent':g['machine_cpu_percent']['mean'],'pipeline_pss_mean_mib':g['pss_mib']['mean'],'pipeline_pss_p95_mib':g['pss_mib']['p95'],'pipeline_pss_peak_mib':g['pss_mib']['peak'],'localizer_cpu_mean_percent':local['cpu_percent']['mean'],'localizer_pss_mean_mib':local['pss_mib']['mean'],'harness_cpu_mean_percent':harness['cpu_percent']['mean'],'harness_pss_mean_mib':harness['pss_mib']['mean'],'monitor_cpu_mean_percent':monitor['cpu_percent']['mean'],'host_busy_mean_percent':w['host_busy_percent']['mean'],'other_host_logical_cores_estimate':host-g['logical_cores']['mean']-harness['logical_cores']['mean']-monitor['logical_cores']['mean'],'steady_window_seconds':w['window_seconds'],'samples':w['samples'],'output':case['output']})
for row in rows:
    run=json.loads((Path(row['output'])/'summary.json').read_text())
    row.update(aligned_scans=run['aligned_scans'],raw_localizer_poses=run['poses'],business_poses=run['business_poses'],all_sensor_messages_received=run.get('all_sensor_messages_received'))
if not rows:raise SystemExit('No successful cases available')
rows.sort(key=lambda r:(list(profiles).index(r['profile']),r['initializer']))
for file,data in [('performance_summary.csv',rows),('performance_processes_and_phases.csv',details)]:
 with (args.results/file).open('w') as f:
    writer=csv.DictWriter(f,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
pairs=[('hector','hector_ekf_pre','Hector: add EKF + prediction'),('hector_ekf_pre','hector_ekf_time_pre','Hector: add time to pre'),('hector_ekf_pre','hector_ekf_pre_desk','Hector: add desk to pre'),('hector_ekf_pre','hector_ekf_desk_pre_time','Hector: add desk+time to pre'),('hector_ekf_time','hector_ekf_time_desk','Hector: add desk to time'),('amcl_ekf_pre','amcl_ekf_time_pre','AMCL: add time'),('amcl_ekf_pre','amcl_ekf_desk_pre','AMCL: add desk'),('amcl_ekf_pre','amcl_ekf_desk_pre_time','AMCL: add desk+time')]
deltas=[]
for method in ['business_fft']:
 for base,target,label in pairs:
    a=next((r for r in rows if r['profile']==base and r['initializer']==method),None);b=next((r for r in rows if r['profile']==target and r['initializer']==method),None)
    if a and b:deltas.append({'comparison':label,'initializer':method,'baseline':base,'target':target,'cpu_percentage_point_delta':b['pipeline_cpu_mean_percent']-a['pipeline_cpu_mean_percent'],'cpu_relative_percent_delta':(b['pipeline_cpu_mean_percent']/a['pipeline_cpu_mean_percent']-1)*100,'logical_cores_delta':b['logical_cores_mean']-a['logical_cores_mean'],'pss_mib_delta':b['pipeline_pss_mean_mib']-a['pipeline_pss_mean_mib']})
if deltas:
 with (args.results/'performance_overhead.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=list(deltas[0]));writer.writeheader();writer.writerows(deltas)
lines=['# 定位矩阵 CPU / 逻辑核 / PSS 实测','',f"已完成 {len(rows)}/12 组。测试机：{environment['cpu_model']}，{environment['logical_cpus']} 个逻辑CPU，governor={environment['governor']}。",'',
'全部串行、1倍速完整bag回放，关闭RViz，保持原有定位节点配置。主统计区间为开始播放5秒后到播放结束前1秒；完整播放、启动、后处理阶段另存。每秒采样一次，峰值为1秒采样观测峰值。','',
'CPU口径：/proc/PID/stat的utime+stime差分除以实际单调时钟间隔；100% CPU等于1个逻辑核，逻辑核数是时间平均使用量，不是绑定核数或线程数。本机整机CPU占比还需除以16。未修改CPU频率、绑核或终止用户其它任务；逐核整机负载和允许CPU列表随原始数据保存。','',
'PSS口径：实际读取/proc/PID/smaps_rollup的Pss，逐PID相加，单位MiB；没有用RSS代替PSS。PSS受系统中共享库使用者影响，本表为本机实际运行结果。','',
'主表 pipeline 包括定位器、扫描合并/补偿、EKF、robot_state_publisher、业务base_pose，以及AMCL需要的map_server/置信度节点。定位器单进程也单独统计。测试用Python记录程序、bag播放器和性能采样器分开列出，不计入pipeline；原有MCAP记录仍开启，因此定位节点的DDS发布负载包含记录订阅者。','',
'初始化固定使用此前缓存的业务FFT位姿；此处比较正常定位资源，不包含重新执行FFT/全局搜索的冷启动代价。每种配置一次完整回放，小差异不解释为稳定显著差异。','',
'## 正常定位进程组','', '| 配置 | 初始化 | CPU均值 [%] | CPU P95 [%] | CPU采样峰值 [%] | 平均逻辑核 | PSS均值 [MiB] | PSS采样峰值 [MiB] |','|---|---|---:|---:|---:|---:|---:|---:|']
for r in rows:lines.append(f"| {r['profile']} | {r['initializer']} | {r['pipeline_cpu_mean_percent']:.2f} | {r['pipeline_cpu_p95_percent']:.2f} | {r['pipeline_cpu_peak_percent']:.2f} | {r['logical_cores_mean']:.3f} | {r['pipeline_pss_mean_mib']:.2f} | {r['pipeline_pss_peak_mib']:.2f} |")
lines+=['','## 定位器本体与测试开销','','| 配置 | 初始化 | 定位器CPU [%] | 定位器PSS [MiB] | 回放+记录CPU [%] | 回放+记录PSS [MiB] |','|---|---|---:|---:|---:|---:|']
for r in rows:lines.append(f"| {r['profile']} | {r['initializer']} | {r['localizer_cpu_mean_percent']:.2f} | {r['localizer_pss_mean_mib']:.2f} | {r['harness_cpu_mean_percent']:.2f} | {r['harness_pss_mean_mib']:.2f} |")
lines+=['','## 功能开销差值','','CPU差值以百分点表示，例如20%到25%为+5个百分点，等于+0.05个逻辑核。Hector基线到pre的变化同时包含EKF和预测，不能单独归因于EKF。','', '| 对比 | 初始化 | CPU差值 [百分点] | 逻辑核差值 | PSS差值 [MiB] |','|---|---|---:|---:|---:|']
for d in deltas:lines.append(f"| {d['comparison']} | {d['initializer']} | {d['cpu_percentage_point_delta']:+.2f} | {d['logical_cores_delta']:+.3f} | {d['pss_mib_delta']:+.2f} |")
lines+=['','完整数据：performance_summary.csv、performance_processes_and_phases.csv、performance_overhead.csv；每组目录含逐秒resource_samples.jsonl、performance_summary.json及原运行日志/MCAP。localizer/pipeline/harness是分组别名，不能把别名与单进程行再次求和。','']
(args.results/'README.md').write_text('\n'.join(lines))
# Scientific summary plotted directly from measured CPU/PSS values.
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'matplotlib_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
names=[name for name in profiles if any(r['profile']==name for r in rows)]
fig,axes=plt.subplots(1,2,figsize=(16,9),sharey=True,layout='constrained');y=np.arange(len(names));height=.36
for ax,key,p95,title,label in [(axes[0],'pipeline_cpu_mean_percent','pipeline_cpu_p95_percent','Localization pipeline CPU','CPU [%], 100% = one logical CPU'),(axes[1],'pipeline_pss_mean_mib','pipeline_pss_p95_mib','Localization pipeline PSS','Proportional set size [MiB]')]:
 for j,(method,color) in enumerate([('business_fft','tab:blue')]):
    data=[next((r for r in rows if r['profile']==name and r['initializer']==method),None) for name in names];values=[r[key] if r else np.nan for r in data];high=[r[p95] if r else np.nan for r in data]
    ax.barh(y+j*height,values,height=height,label=method,color=color);ax.scatter(high,y+j*height,marker='x',s=18,color='black',label='P95' if j==0 else None,zorder=3)
 ax.set_title(title);ax.set_xlabel(label);ax.grid(axis='x',alpha=.25);ax.set_axisbelow(True);ax.legend(loc='upper right',fontsize=8)
axes[0].set_yticks(y,names);axes[0].invert_yaxis();secondary=axes[0].secondary_xaxis('top',functions=(lambda x:x/100,lambda x:x*100));secondary.set_xlabel('Equivalent logical CPUs')
fig.suptitle(f'Localization matrix | serial 1x playback | {len(rows)}/12 cases | RViz off\nPipeline excludes bag player, recorder and monitor; steady playback; black x = P95',fontsize=11)
chart=Path('/home/lc/slam_11_13/overview/localization_matrix__performance.png');fig.savefig(chart,dpi=160);plt.close(fig)
print('Saved',len(rows),'performance rows and',chart)
