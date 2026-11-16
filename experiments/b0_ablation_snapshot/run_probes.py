"""Fixed-input factorial ablations and direct transfer to pristine upstream core."""
import csv,itertools,json,subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[1];OUT=ROOT/'outputs';OUT.mkdir(exist_ok=True)
C=np.loadtxt('/home/lc/slam_11_13/results/b0_hector_diagnosis_20261116/carto30_verified/trajectory.tum');DATA=WS/'test/b0_hector_deskew/diagnostics/data';POINTS=WS/'test/b0_hector_imu_investigation'
def measure(path):
 a=np.loadtxt(path);mask=(a[:,0]>=C[0,0])&(a[:,0]<=C[-1,0]);a=a[mask];r=np.c_[np.interp(a[:,0],C[:,0],C[:,1]),np.interp(a[:,0],C[:,0],C[:,2])];e=np.linalg.norm(a[:,1:3]-r,axis=1)
 return dict(rmse_m=float(np.sqrt(np.mean(e*e))),max_m=float(e.max()),end_m=float(e[-1]))
def run(cmd,name):
 with (OUT/(name+'.log')).open('w') as f:subprocess.run([str(x) for x in cmd],stdout=f,stderr=subprocess.STDOUT,check=True)
def upstream(voxel,height,levels=2):
 name=f'official_L{levels}_voxel{voxel}_height{int(height)}';lo,hi=(-.8,2.) if height else (-1.,1.)
 cmd=[ROOT/'build/upstream_probe',DATA/'rawscans.bin',OUT/(name+'.tum'),.05,.4,.06,.4,levels,2048,voxel,lo,hi];run(cmd,name)
 row=dict(case=name,kind='upstream_fixed_raw_input',voxel=voxel,height=height,levels=levels,**measure(OUT/(name+'.tum')));print(row,flush=True);return row
native=[upstream(v,h) for v,h in [(0,False),(.025,False),(0,True),(.025,True),(.05,False),(.1,False)]]
a=np.loadtxt(OUT/'official_L2_voxel0_height0.tum');b=np.loadtxt('/home/lc/slam_11_13/results/b0_official_launch_20261116/official_launch/trajectory.tum');assert len(a)==len(b);e=np.linalg.norm(a[:,1:3]-b[:,1:3],axis=1);assert e.max()<.005;validation={'upstream_baseline_vs_live_max_m':float(e.max())};print('validation',validation,flush=True)
def custom(bits):
 xyz,shared,height,voxel,origin=bits;name='custom_'+''.join(map(str,bits));lo,hi=(-.8,2.) if height else (-1.,1.)
 cmd=[ROOT/'build/custom_probe',POINTS,OUT/name,shared,lo,hi,origin,.025 if voxel else 0,0,.4,.9,3,xyz];run(cmd,name)
 row=dict(case=name,kind='custom_fixed_recorded_deskew_input',xyz=xyz,shared=shared,height=height,voxel=voxel,origin=origin,**measure(OUT/(name+'.tum')));return row
base=custom((0,0,0,0,0));full=custom((1,1,1,1,1))
for case,ref in [(base,WS/'test/b0_hector_deskew/diagnostics/replay_results/original.tum'),(full,POINTS/'voxel_origin.tum')]:
 a=np.loadtxt(OUT/(case['case']+'.tum'));b=np.loadtxt(ref);assert len(a)==len(b);e=np.linalg.norm(a[:,1:3]-b[:,1:3],axis=1);validation[case['case']+'_vs_previous_offline_max_m']=float(e.max());print('validation',case['case'],e.max(),flush=True);assert e.max()<.005
rows=[]
with ThreadPoolExecutor(max_workers=2) as pool:
 for row in pool.map(custom,itertools.product([0,1],repeat=5)):rows.append(row);print(row,flush=True)
for name,data in [('upstream',native),('custom',rows)]:
 with (OUT/(name+'_metrics.csv')).open('w') as f:w=csv.DictWriter(f,fieldnames=data[0].keys());w.writeheader();w.writerows(data)
(OUT/'metrics.json').write_text(json.dumps(dict(validation=validation,upstream=native,custom=rows),indent=2))
