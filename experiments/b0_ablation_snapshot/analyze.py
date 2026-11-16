import argparse,csv,json,os
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();OUT=args.output;ROOT=Path(__file__).resolve().parent;M=json.loads((ROOT/'outputs/metrics.json').read_text());lookup={r['case']:r for r in M['custom']}
selected=[('Fully corrected','11111'),('Remove XYZ retention','01111'),('Remove shared attitude','10111'),('Remove wider height filter','11011'),('Remove voxel filter','11101'),('Remove ray-origin correction','11110'),('Keep old XYZ/shared; height+voxel only','00110'),('Shared attitude + height + voxel','01110'),('XYZ + height + voxel','10110')]
rows=[dict(label=label,**lookup['custom_'+bits]) for label,bits in selected]
with (OUT/'custom_selected.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
os.environ.setdefault('MPLCONFIGDIR','/tmp/b0_core_ablation_mpl');import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(13,7),layout='constrained');values=[r['rmse_m'] for r in rows];ax.barh(range(len(rows)),values,color=['tab:green' if v<.15 else 'tab:orange' for v in values]);ax.set_yticks(range(len(rows)),[r['label'] for r in rows]);ax.invert_yaxis();ax.set(xlabel='Position difference RMSE [m]',title='Custom deskew pipeline, fixed recorded input, first30s\nMap=0.05m/3levels; insertion=0.2m/0.08rad; no parameter changes')
for i,v in enumerate(values):ax.text(v+.01,i,f'{v:.4f}m',va='center')
ax.set_xlim(0,1.25);ax.grid(axis='x',alpha=.2);fig.savefig(OUT/'b0_hector_core_ablation_custom.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained');ref=np.loadtxt('/home/lc/slam_11_13/results/b0_hector_diagnosis_20261116/carto30_verified/trajectory.tum')
for name,color,label in [('official_L2_voxel0_height0','tab:orange','Official launch, voxel off'),('official_L2_voxel0.025_height0','tab:blue','Only add voxel=0.025m')]:
 a=np.loadtxt(ROOT/'outputs'/f'{name}.tum');r=np.c_[np.interp(a[:,0],ref[:,0],ref[:,1]),np.interp(a[:,0],ref[:,0],ref[:,2])];e=np.linalg.norm(a[:,1:3]-r,axis=1);axes[0].plot(a[:,1],a[:,2],color=color,label=label);axes[1].plot(a[:,0]-1405076449.715589,e,color=color,label=label)
axes[0].plot(ref[:,1],ref[:,2],'k--',label='Cartographer online reference');axes[0].set(aspect='equal',xlabel='x [m]',ylabel='y [m]',title='Unchanged upstream core, raw scan input');axes[1].set(xlim=(0,30),xlabel='Bag time [s]',ylabel='Position difference [m]',title='Official map/insertion parameters unchanged')
for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.savefig(OUT/'b0_hector_core_ablation_upstream_offline.png',dpi=160);plt.close(fig)
