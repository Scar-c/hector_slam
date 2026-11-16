"""Compare recorded online poses and controlled fixed-input ablations; no alignment."""
import argparse,csv,json,os,shutil
from pathlib import Path
import numpy as np,yaml
from PIL import Image
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
OUT=args.output;ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[1];T0=1405076449.715589
reference=Path('/home/lc/slam_11_13/results/b0_hector_diagnosis_20261116/carto30_verified')
c=np.loadtxt(reference/'trajectory.tum');old=np.loadtxt(WS/'test/b0_hector_deskew/diagnostics/data/live.tum');new=np.loadtxt(OUT/'live30/trajectory_hector.tum')
metrics=[];traces={}
def measure(name,a,kind):
 mask=(a[:,0]>=c[0,0])&(a[:,0]<=c[-1,0]);a=a[mask]
 r=np.column_stack([np.interp(a[:,0],c[:,0],c[:,i]) for i in [1,2,3]])
 error=np.linalg.norm(a[:,1:4]-r,axis=1);traces[name]=(a[:,0]-T0,error)
 metrics.append(dict(case=name,kind=kind,poses=len(a),rmse_m=float(np.sqrt(np.mean(error**2))),max_m=float(error.max()),end_m=float(error[-1])))
 return a,r,error
measure('Original live',old,'ROS 1x');a,r,error=measure('Corrected live',new,'ROS 1x')
np.savetxt(OUT/'pose_comparison_30s.csv',np.c_[a[:,0]-T0,r,a[:,1:4],error],delimiter=',',header='bag_seconds,carto_x,carto_y,carto_z,hector_x,hector_y,hector_z,position_difference_m',comments='')
probe=WS/'test/b0_hector_imu_investigation'
for name in ['no_internal_imu','yaw_only','full3d_native','full3d_carto','full3d_carto_height','voxel','adaptive','native_voxel','carto_oldheight_voxel','native_wide_voxel','voxel_origin','voxel_05','voxel_10','one_level','two_levels','soft_map']:
 path=probe/(name+'.tum')
 if path.exists():measure(name,np.loadtxt(path),'offline fixed-input ablation')
with (OUT/'comparison.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=metrics[0].keys());w.writeheader();w.writerows(metrics)
(OUT/'comparison.json').write_text(json.dumps({'reference':str(reference),'ground_truth':False,'time_association':'Linear interpolation at Hector timestamp; reference-covered interval only; no spatial alignment, no scale fitting','metrics':metrics},indent=2))
os.environ.setdefault('MPLCONFIGDIR','/tmp/b0_cloud_matplotlib')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig=plt.figure(figsize=(14,10),layout='constrained');g=fig.add_gridspec(2,2,height_ratios=[3,1.15]);axs=[fig.add_subplot(g[0,0]),fig.add_subplot(g[0,1])]
for ax,title in zip(axs,['Original Hector input: first 30 s','Corrected Hector input: first 30 s']):
 cfg=yaml.safe_load((reference/'map.yaml').read_text());im=np.array(Image.open(reference/'map.pgm'));x,y,_=cfg['origin'];res=cfg['resolution']
 ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=[x,x+im.shape[1]*res,y,y+im.shape[0]*res],origin='upper',alpha=.8)
 ax.plot(c[:,1],c[:,2],'--',color='tab:blue',lw=1.1,label='Cartographer online (comparison reference)')
 pose=old if ax is axs[0] else new
 ax.plot(pose[:,1],pose[:,2],color='tab:orange' if ax is axs[0] else 'tab:green',lw=1.1,label='Hector online, ROS replay 1x')
 ax.scatter(*pose[0,1:3],color='orange',s=35,label='Start');ax.scatter(*pose[-1,1:3],color='red',marker='x',s=40,label='End')
 ax.set(xlim=(-19,2),ylim=(-10,2),aspect='equal',xlabel='Map x [m]',ylabel='Map y [m]',title=title)
 ax.legend(loc='upper right',fontsize=7,title='2D rebin + independent IMU + z[-1,1]' if ax is axs[0] else 'Full XYZ + shared IMU reference\nz[-0.8,2.0]m + 3D voxel=0.025m\nOriginal Hector core: 3 map levels',title_fontsize=8)
ax=fig.add_subplot(g[1,:])
for name,color in [('Original live','tab:orange'),('Corrected live','tab:green')]:t,e=traces[name];ax.plot(t,e,color=color,label=name)
ax.axvspan(6,8,alpha=.1,color='red');ax.set(xlabel='Bag time [s]',ylabel='Position difference [m]',xlim=(0,30));ax.grid(alpha=.25);ax.legend()
fig.suptitle('b0 | same bag, same extrinsics | Cartographer online comparison, no spatial alignment')
fig.savefig(OUT/'b0_hector_gravity_cloud_comparison_30s.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(18,7),layout='constrained')
for ax,title,path,trajectory in [(axes[0],'Cartographer, online',reference,c),(axes[1],'Original Hector',None,old),(axes[2],'Corrected Hector, online 1x',OUT/'live30',new)]:
 if path is None:im=np.array(Image.open(WS/'test/b0_hector_deskew/diagnostics/replay_results/original.pgm'));extent=[-102.425,102.375,-102.425,102.375]
 else:
  cfg=yaml.safe_load((path/'map.yaml').read_text());im=np.array(Image.open(path/'map.pgm'));x,y,_=cfg['origin'];res=cfg['resolution'];extent=[x,x+im.shape[1]*res,y,y+im.shape[0]*res]
 ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=extent,origin='upper');ax.plot(trajectory[:,1],trajectory[:,2],lw=.7,color='tab:green')
 ax.set(xlim=(-24,25),ylim=(-24,9),aspect='equal',xlabel='Map x [m]',ylabel='Map y [m]',title=title)
fig.savefig(OUT/'b0_hector_gravity_cloud_maps_30s.png',dpi=160);plt.close(fig)
print(json.dumps(metrics[:2],indent=2))
