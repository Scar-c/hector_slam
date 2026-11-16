#!/usr/bin/env python3
"""Full-bag functional validation, two isolated DDS domains at a time."""
import argparse,concurrent.futures,hashlib,json,os,subprocess,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
profiles=json.loads((ROOT/'scripts/matrix_profiles.json').read_text())
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--rate',type=float,choices=[1.0],default=1.0)
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
locks={name:threading.Lock() for name in profiles}
def run(case):
    method,name=case;profile=profiles[name];folder=args.output/method/name;folder.parent.mkdir(exist_ok=True)
    with locks[name],(folder.parent/(name+'.log')).open('w') as log:
        env=os.environ.copy();env['ROS_DOMAIN_ID']=str(profile['domain'])
        command=['bash',str(ROOT/'scripts/run_localization_matrix.sh'),'--profile',name,'--initializer',method,'--expected-domain',str(profile['domain']),'--output',str(folder),'--rate',str(args.rate),'--no-rviz','--no-hold']
        start=time.monotonic();r=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    result={'profile':name,'initializer':method,'returncode':r.returncode,'output':str(folder),'elapsed_seconds':time.monotonic()-start,'checks':{}}
    try:
        s=json.loads((folder/'summary.json').read_text());p=json.loads((folder/'profile.json').read_text());i=json.loads((folder/'initialization.json').read_text())
        checks=result['checks']
        checks['all_inputs']=s['all_sensor_messages_received']
        checks['map_fixed']=s['published_map_versions']==1 and s['published_map_equals_pgm_yaml'] and s['source_files_unchanged']
        checks['real_switches']=s['features']==profile and s['scan_alignment_validation']['per_beam_deskew']==profile['desk'] and s['scan_alignment_validation']['frame_time_alignment']==profile['time']
        checks['scan_geometry']=s['scan_alignment_validation']['mean_beam_agreement_fraction']>=.999
        checks['poses']=s['poses']>100 and (not profile['ekf'] or s['business_poses']>700)
        checks['plot']=(folder/'localization_overview.png').stat().st_size>10000
        checks['initializer']=i['method']==method and s['initialization_sha256']==i['result_sha256']
        if profile['ekf'] and profile['algorithm']=='hector':checks['direct_map_correction']=s['tf_composition']['maximum_position_error_m']<1e-4
        result.update(poses=s['poses'],business_poses=s['business_poses'],scan_frames=s['aligned_scans'],beam_agreement=s['scan_alignment_validation']['mean_beam_agreement_fraction'],seed_sha256=i['result_sha256'],seed=[i[k] for k in ['initial_pose_x','initial_pose_y','initial_pose_yaw']],cache_key=i['cache_key'])
    except Exception as e:result['validation_error']=str(e)
    result['passed']=r.returncode==0 and result['checks'] and all(result['checks'].values()) and 'validation_error' not in result
    return result
results=[]
cases=[(method,name) for method in ['business_fft','hector_search'] for name in profiles]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    futures=[pool.submit(run,case) for case in cases]
    for f in concurrent.futures.as_completed(futures):
        result=f.result();results.append(result)
        (args.output/'validation_progress.json').write_text(json.dumps(results,indent=2))
        print(f"{len(results)}/24 {'PASS' if result['passed'] else 'FAIL'} {result['initializer']} {result['profile']}",flush=True)
shared={method:len({r.get('seed_sha256') for r in results if r['initializer']==method})==1 for method in ['business_fft','hector_search']}
report={'passed':all(r['passed'] for r in results) and all(shared.values()),'cases':len(results),'rate':args.rate,'same_cached_seed_within_initializer':shared,'results':results,'scope':'Functional full-bag validation at 1x playback; not a ground-truth accuracy benchmark.'}
(args.output/'matrix_validation.json').write_text(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
