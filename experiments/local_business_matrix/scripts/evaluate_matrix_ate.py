#!/usr/bin/env python3
"""Offline ATE from actual evo, against original mapping and saved-map nodes."""
import argparse,copy,csv,hashlib,json,os
from pathlib import Path
import numpy as np
import evo
from evo.core import sync,metrics
from evo.core.trajectory import Plane
from evo.main_ape import ape
from evo.tools import file_interface
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--results',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--max-time-diff',type=float,default=.04);args=p.parse_args()
report=json.loads((args.results/'matrix_validation.json').read_text());assert report['passed'] and report['cases']==24 and report['rate']==1
args.output.mkdir(parents=True,exist_ok=False)
reference_root=Path('/home/lc/slam_11_13/results/business_single/lo')
references={'mapping_online':reference_root/'trajectory_online.tum','mapping_at_save':reference_root/'trajectory_at_save.tum'}
reference_trajs={key:file_interface.read_tum_trajectory_file(path) for key,path in references.items()}
rows=[];input_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in references.values()}
profiles=json.loads((ROOT/'scripts/matrix_profiles.json').read_text())
for case in report['results']:
    name=case['profile'];method=case['initializer'];profile=profiles[name];folder=Path(case['output']);matrix=name+'__'+method
    raw=folder/('trajectory_amcl.tum' if profile['algorithm']=='amcl' else 'trajectory_hector.tum')
    displayed=folder/'trajectory_business.tum' if profile['ekf'] else raw
    for role,path in [('displayed',displayed),('raw_localizer',raw)]:
        estimate=file_interface.read_tum_trajectory_file(path);input_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        for reference_key,reference in reference_trajs.items():
            ref,est=sync.associate_trajectories(reference,estimate,max_diff=args.max_time_diff,offset_2=0,sync_method=sync.SyncMethod.nearest_time)
            dt=np.abs(ref.timestamps-est.timestamps);pairdir=args.output/reference_key/matrix/role;pairdir.mkdir(parents=True)
            np.savetxt(pairdir/'associated_timestamps.txt',np.column_stack([ref.timestamps,est.timestamps,dt]),fmt='%.9f',header='reference_timestamp estimate_timestamp absolute_difference_seconds')
            all_stats={}
            for mode in ['xy_no_alignment','xy_rigid_aligned','xyz_no_alignment']:
                r=copy.deepcopy(ref);e=copy.deepcopy(est)
                if mode.startswith('xy_'):r.project(Plane.XY);e.project(Plane.XY)
                result=ape(r,e,metrics.PoseRelation.translation_part,align=mode=='xy_rigid_aligned',correct_scale=False)
                result.info.update(matrix=matrix,reference_source=str(references[reference_key]),estimate_source=str(path),role=role,association_max_time_diff_seconds=args.max_time_diff,association_method='evo nearest_time; no interpolation, offset=0',projection_before_alignment=mode.startswith('xy_'))
                stats={k:float(v) for k,v in result.stats.items()};stats['p95']=float(np.quantile(result.np_arrays['error_array'],.95));all_stats[mode]=stats
                file_interface.save_res_file(pairdir/(mode+'.zip'),result)
                error=np.asarray(result.np_arrays['error_array']);np.savetxt(pairdir/(mode+'_error.txt'),np.column_stack([est.timestamps,error]),fmt='%.9f',header='estimate_timestamp translation_error_m')
                (pairdir/(mode+'_stats.json')).write_text(json.dumps(stats,indent=2))
            row={'reference':reference_key,'initializer':method,'profile':name,'matrix':matrix,'role':role,'matched_poses':est.num_poses,'estimate_poses':estimate.num_poses,'association_dt_mean_ms':float(dt.mean()*1000),'association_dt_p95_ms':float(np.quantile(dt,.95)*1000),'association_dt_max_ms':float(dt.max()*1000),'paired_span_seconds':float(est.timestamps[-1]-est.timestamps[0]),'estimate_max_gap_seconds':float(np.max(np.diff(estimate.timestamps))),'xy_rmse_m':all_stats['xy_no_alignment']['rmse'],'xy_mean_m':all_stats['xy_no_alignment']['mean'],'xy_median_m':all_stats['xy_no_alignment']['median'],'xy_std_m':all_stats['xy_no_alignment']['std'],'xy_min_m':all_stats['xy_no_alignment']['min'],'xy_max_m':all_stats['xy_no_alignment']['max'],'xy_p95_m':all_stats['xy_no_alignment']['p95'],'aligned_xy_rmse_m':all_stats['xy_rigid_aligned']['rmse'],'aligned_xy_mean_m':all_stats['xy_rigid_aligned']['mean'],'aligned_xy_max_m':all_stats['xy_rigid_aligned']['max'],'xyz_rmse_m':all_stats['xyz_no_alignment']['rmse'],'estimate_file':str(path),'evo_results':str(pairdir)}
            rows.append(row)
    print('Evaluated',matrix,flush=True)
fields=list(rows[0])
with (args.output/'ate_all.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
main=[r for r in rows if r['reference']=='mapping_online' and r['role']=='displayed']
main.sort(key=lambda r:r['xy_rmse_m'])
with (args.output/'ate_main_ranking.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(main)
settings={'evo_version':evo.__version__,'cases':24,'rows':len(rows),'primary_reference':str(references['mapping_online']),'secondary_reference':str(references['mapping_at_save']),'primary_metric':'Translation ATE/APE RMSE in XY, no alignment, no scale correction','secondary_metric':'XY projection first, then rigid Umeyama alignment with scale fixed to 1','additional_metric':'3D unaligned translation ATE, including fixed vertical-frame offset','association':{'method':'evo nearest_time','max_difference_seconds':args.max_time_diff,'time_offset_seconds':0,'interpolation':False},'reference_is_ground_truth':False,'input_sha256':input_hashes,'results':rows}
(args.output/'ate_results.json').write_text(json.dumps(settings,indent=2))
lines=['# evo ATE：24组定位矩阵','','主参考是建图在线原始轨迹 `trajectory_online.tum`（1172帧），附表使用地图保存时节点 `trajectory_at_save.tum`（677帧，与三模块图虚线一致）。两份参考均为 map -> base_link；它们是建图参考轨迹，不是外部真值。','',f'使用 evo {evo.__version__} 的真实 `associate_trajectories` 和 `ape` 实现。全部被评估轨迹来自已通过验收的1倍速完整回放；这里仅做离线处理。','',f'时间戳按 evo nearest_time 匹配，最大差 {args.max_time_diff*1000:.0f} ms，不做时间偏移搜索、不插值、不改尺度。原始建图轨迹约15Hz，40ms覆盖半个采样周期；每行保留匹配数和实际时间差。','',
'主指标为共同地图坐标系下不对齐的 XY 平移ATE（米）。建图参考Z固定为-0.29m，当前2D定位Z为0，故主指标投影到XY；完整3D未对齐ATE也保留在CSV中，避免隐藏高度差。附指标在XY投影后做刚体Umeyama对齐，尺度固定1；它会消除全局平移/旋转，不能替代共同地图坐标系的主指标。','',
'主表使用图片实际绘制的轨迹：带EKF使用共同业务 /base_pose，纯Hector使用其原始输出。`raw_localizer`附表另列Hector /slam_out_pose、AMCL /amcl_pose；AMCL原始输出在静止时可能长时间不更新，采样数不同，不能把该表的较小ATE直接解读为更好的连续定位。','',
'[evo指标定义](https://github.com/MichaelGrupp/evo/wiki/Metrics)；[时间匹配与投影](https://github.com/MichaelGrupp/evo/wiki/evo_traj)。','',
'## 主表：在线原始建图参考，图中轨迹','','| 排名 | 矩阵 | XY RMSE [m] | 均值 [m] | P95 [m] | 最大 [m] | 对齐后RMSE [m] | 匹配帧 |','|---:|---|---:|---:|---:|---:|---:|---:|']
for i,r in enumerate(main,1):lines.append(f"| {i} | {r['matrix']} | {r['xy_rmse_m']:.6f} | {r['xy_mean_m']:.6f} | {r['xy_p95_m']:.6f} | {r['xy_max_m']:.6f} | {r['aligned_xy_rmse_m']:.6f} | {r['matched_poses']} |")
for reference,role,title in [('mapping_online','raw_localizer','在线原始建图参考，定位器原始输出'),('mapping_at_save','displayed','保存时建图节点参考，图中轨迹'),('mapping_at_save','raw_localizer','保存时建图节点参考，定位器原始输出')]:
 lines+=['',f'## 附表：{title}','','| 矩阵 | XY RMSE [m] | 对齐后RMSE [m] | 匹配帧 |','|---|---:|---:|---:|']
 for r in sorted((r for r in rows if r['reference']==reference and r['role']==role),key=lambda r:r['xy_rmse_m']):lines.append(f"| {r['matrix']} | {r['xy_rmse_m']:.6f} | {r['aligned_xy_rmse_m']:.6f} | {r['matched_poses']} |")
lines+=['','每组目录保存原生evo结果ZIP、逐时刻误差、统计JSON和关联时间戳，可用evo_res重新读取。完整数据见 ate_all.csv，主表排序见 ate_main_ranking.csv。','']
(args.output/'README.md').write_text('\n'.join(lines))
# Summary chart uses evo-produced values, not another ATE implementation.
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'matplotlib_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(16,9),sharey=True,layout='constrained')
y=np.arange(len(profiles));height=.36
for ax,key,title in zip(axes,['xy_rmse_m','aligned_xy_rmse_m'],['ATE RMSE in shared map frame (XY, no alignment)','ATE RMSE after rigid alignment (XY, scale = 1)']):
 for j,(method,color) in enumerate([('business_fft','tab:blue'),('hector_search','tab:orange')]):
  values=[next(r[key] for r in main if r['profile']==name and r['initializer']==method) for name in profiles]
  ax.barh(y+(j-.5)*height,values,height=height,label=method,color=color)
 ax.set_title(title,fontsize=11);ax.set_xlabel('Translation RMSE [m]');ax.grid(axis='x',alpha=.25);ax.set_axisbelow(True);ax.legend(loc='lower right',fontsize=9)
axes[0].set_yticks(y,list(profiles));axes[0].invert_yaxis()
fig.suptitle('Localization matrix vs original online mapping trajectory | evo '+evo.__version__+' | all replays at 1x\nReference is mapping output, not ground truth; associations within 40 ms; displayed trajectory',fontsize=11)
chart=Path('/home/lc/slam_11_13/overview/localization_matrix__ATE_RMSE.png');fig.savefig(chart,dpi=160);plt.close(fig)
print('ATE evaluation complete:',args.output,'chart:',chart,flush=True)
