#!/usr/bin/env python3
"""Reuse the 24-case evo protocol for the one requested semantic profile."""
import copy
import csv
import hashlib
import json
import os
from pathlib import Path
import numpy as np
import evo
from evo.core import sync, metrics
from evo.core.trajectory import Plane
from evo.main_ape import ape
from evo.tools import file_interface
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
NEW=ROOT/'runs/fixed_final_1x_complete'
OLD=Path('/home/lc/slam_11_13/results/localization_matrix_validation_1x_20261113/business_fft/hector_ekf_desk_pre_time')
REFROOT=Path('/home/lc/slam_11_13/results/business_single/lo')
OUT=ROOT/'comparison'
OUT.mkdir(exist_ok=True)
ns=json.loads((NEW/'summary.json').read_text());bs=json.loads((OLD/'summary.json').read_text())
assert ns['initialization_sha256']==bs['initialization_sha256']
assert ns['features']==bs['features'] and ns['replay_rate']==bs['replay_rate']==1
for path in [REFROOT/'map.yaml',REFROOT/'map.pgm',Path('/home/lc/slam_11_13/metadata.yaml'),Path('/home/lc/slam_11_13/slam_11_13_0.mcap'),Path('/home/lc/ZHUXING_V2WS-master/src/algorithm/odom_fusion/config/ekf.yaml')]:
    assert ns['input_hashes'][str(path)]==bs['input_hashes'][str(path)]==hashlib.sha256(path.read_bytes()).hexdigest()
assert ns['all_sensor_messages_received'] and ns['published_map_versions']==1 and ns['published_map_equals_pgm_yaml']
sources={'original_24case':OLD,'fixed_final':NEW}
rows=[];series={};associated={}
for reference_name,file in [('mapping_online','trajectory_online.tum'),('mapping_at_save','trajectory_at_save.tum')]:
    ref=file_interface.read_tum_trajectory_file(str(REFROOT/file))
    for name,folder in sources.items():
        for role,filename in [('business','trajectory_business.tum'),('raw_hector','trajectory_hector.tum')]:
            estimate=file_interface.read_tum_trajectory_file(str(folder/filename))
            r,e=sync.associate_trajectories(ref,estimate,max_diff=.04,sync_method=sync.SyncMethod.nearest_time)
            r.project(Plane.XY);e.project(Plane.XY)
            result=ape(r,e,metrics.PoseRelation.translation_part,align=False,correct_scale=False)
            result.info.update(reference_source=str(REFROOT/file),estimate_source=str(folder/filename),xy_projection=True,alignment=False,max_time_difference_seconds=.04)
            pair=OUT/reference_name/name/role;pair.mkdir(parents=True,exist_ok=True)
            file_interface.save_res_file(str(pair/'evo_ape.zip'),result,confirm_overwrite=False)
            errors=result.np_arrays['error_array'];dt=abs(r.timestamps-e.timestamps)
            np.savetxt(pair/'errors.csv',np.column_stack([r.timestamps,e.timestamps,dt,errors]),delimiter=',',header='reference_timestamp,estimate_timestamp,time_difference_s,xy_ape_m',comments='',fmt='%.9f')
            stats={k:float(v) for k,v in result.stats.items()};stats['p95']=float(np.quantile(errors,.95))
            row=dict(reference=reference_name,version=name,role=role,matched_poses=e.num_poses,rmse_m=stats['rmse'],mean_m=stats['mean'],p95_m=stats['p95'],max_m=stats['max'],max_time_difference_ms=float(dt.max()*1000),source=str(folder/filename))
            rows.append(row)
            if reference_name=='mapping_online':
                series[name,role]=(e.timestamps,errors)
                associated[name,role]={float(t):float(err) for t,err in zip(r.timestamps,errors)}
            (pair/'stats.json').write_text(json.dumps(stats,indent=2)+'\n')
with (OUT/'ate_comparison.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
common=[]
for role in ['business','raw_hector']:
    a,b=associated['original_24case',role],associated['fixed_final',role]
    times=sorted(set(a)&set(b))
    common.append(dict(role=role,common_reference_nodes=len(times),original_rmse_m=float(np.sqrt(np.mean([a[t]**2 for t in times]))),fixed_rmse_m=float(np.sqrt(np.mean([b[t]**2 for t in times])))))
result=dict(evo_version=evo.__version__,semantic_profile='hector_ekf_desk_pre_time',initializer='business_fft',initialization_sha256=ns['initialization_sha256'],replay_rate=1,reference_is_ground_truth=False,protocol='XY APE, no alignment/scale/time shift, evo nearest_time <=40ms',same_inputs_and_ekf_verified=True,rows=rows,common_reference_sensitivity=common)
(OUT/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':11,'axes.unicode_minus':False})
fig,axes=plt.subplots(2,2,figsize=(15,10),constrained_layout=True)
ref=np.loadtxt(REFROOT/'trajectory_online.tum');start=ref[0,0]
ax=axes[0,0];ax.plot(ref[:,1],ref[:,2],'--',c='gray',label='原始建图在线轨迹（参考）',lw=1.2)
for name,color,label in [('original_24case','tab:blue','原24组：同配置'),('fixed_final','tab:orange','最终修复版：同语义')]:
    tr=np.loadtxt(sources[name]/'trajectory_business.tum');ax.plot(tr[:,1],tr[:,2],c=color,label=label,lw=1.2)
ax.set_aspect('equal');ax.set(xlabel='地图 x [m]',ylabel='地图 y [m]',title='业务 /base_pose 轨迹');ax.legend(fontsize=9)
for ax,role,title in [(axes[0,1],'business','业务连续定位 XY APE'),(axes[1,0],'raw_hector','Hector 原始输出 XY APE')]:
    for name,color,label in [('original_24case','tab:blue','原24组：同配置'),('fixed_final','tab:orange','最终修复版：同语义')]:
        t,errors=series[name,role];ax.plot(t-start,errors*100,c=color,label=label,lw=1)
    ax.set(xlabel='相对建图首帧时间 [s]',ylabel='XY APE [cm]',title=title);ax.grid(alpha=.25);ax.legend(fontsize=9)
ax=axes[1,1];labels=['业务 RMSE','业务最大','原始 RMSE','原始最大'];pos=np.arange(4)
for j,(name,color,label) in enumerate([('original_24case','tab:blue','原24组：同配置'),('fixed_final','tab:orange','最终修复版：同语义')]):
    selected={r['role']:r for r in rows if r['reference']=='mapping_online' and r['version']==name}
    values=[selected['business']['rmse_m'],selected['business']['max_m'],selected['raw_hector']['rmse_m'],selected['raw_hector']['max_m']]
    bars=ax.bar(pos+(j-.5)*.36,np.array(values)*100,.36,label=label,color=color)
    ax.bar_label(bars,fmt='%.2f',fontsize=9)
ax.set_xticks(pos,labels);ax.set(ylabel='误差 [cm]',title='同语义：EKF + pre + time + desk | FFT 初始化');ax.grid(axis='y',alpha=.25);ax.legend(fontsize=9);ax.margins(y=.18)
fig.suptitle('最终修复版 Hector 固定先验地图定位 vs 原24组同配置\n同 bag / 地图 / EKF / FFT 初值；1倍速；evo ≤40ms；无空间对齐',fontsize=14)
fig.savefig(OUT/'hector_ekf_desk_pre_time__fixed_final_comparison.png',dpi=160);plt.close(fig)
print(json.dumps(result,indent=2))
