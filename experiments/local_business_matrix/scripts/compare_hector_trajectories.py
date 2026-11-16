#!/usr/bin/env python3
"""Compare trajectory roughness at the same 0.1s sampling interval, not accuracy."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml

def measure(path,grid):
    data=np.atleast_2d(np.loadtxt(path))
    xy=np.stack([np.interp(grid,data[:,0],data[:,j]) for j in [1,2]],axis=1)
    second=np.linalg.norm(np.diff(xy,n=2,axis=0),axis=1)
    return {'second_difference_rms_m':float(np.sqrt(np.mean(second**2))),
            'second_difference_p95_m':float(np.quantile(second,.95)),
            'maximum_sample_step_m':float(np.linalg.norm(np.diff(xy,axis=0),axis=1).max()),'samples':len(grid)}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--result',type=Path,required=True)
    p.add_argument('--bag',type=Path,default=Path('/home/lc/slam_11_13'));a=p.parse_args()
    metadata=yaml.safe_load((a.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start=metadata['starting_time']['nanoseconds_since_epoch']/1e9
    grid=start+np.arange(1,64,.1)  # Common moving segment, excluding startup and parked tail.
    stats={'definition':'Euclidean second position difference at 0.1s, bag seconds 1..64. Not accuracy; turns also contribute.',
           'hector_before_ekf':measure(a.baseline/'trajectory_hector.tum',grid),
           'hector_with_ekf_raw_matching':measure(a.result/'trajectory_hector.tum',grid),
           'hector_with_ekf_business_tf':measure(a.result/'trajectory_business.tum',grid)}
    before=stats['hector_before_ekf']['second_difference_rms_m']
    for key in ['hector_with_ekf_raw_matching','hector_with_ekf_business_tf']:
        stats[key]['rms_reduction_fraction_vs_baseline']=1-stats[key]['second_difference_rms_m']/before
    (a.result/'roughness_comparison.json').write_text(json.dumps(stats,indent=2))
    print(json.dumps(stats,indent=2))
if __name__=='__main__':main()
