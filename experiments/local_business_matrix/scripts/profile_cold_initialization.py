#!/usr/bin/env python3
"""Offline resource measurement of the existing two-method cache generator."""
import argparse,hashlib,json,os,subprocess,time
from pathlib import Path
from benchmark_localization_matrix import ROOT,process_snapshot
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
cache=ROOT/'initialization_cache/034c6911704011d10085';cloud=cache/'input_cloud.xy';map_yaml=Path('/home/lc/slam_11_13/results/business_single/lo/map.yaml');exe=ROOT/'initializers/build/shared_initializers'
log=(args.output/'initializer.log').open('w');start=time.monotonic();child=subprocess.Popen([str(exe),str(map_yaml),str(cloud),str(args.output)],stdout=log,stderr=subprocess.STDOUT);samples=[]
while True:
    now=time.monotonic();snapshot=process_snapshot(child.pid)
    if snapshot is not None:samples.append({'elapsed_seconds':now-start,**snapshot})
    waited,status,usage=os.wait4(child.pid,os.WNOHANG)
    if waited:
        child.returncode=os.waitstatus_to_exitcode(status);end=time.monotonic();break
    time.sleep(.005)
log.close()
assert child.returncode==0
for method in ['business_fft','hector_search']:
    original=json.loads((cache/(method+'.json')).read_text());actual=json.loads((args.output/(method+'.json')).read_text())
    assert all(abs(original[k]-actual[k])<1e-12 for k in ['initial_pose_x','initial_pose_y','initial_pose_yaw'])
pss=[s['pss_kib']/1024 for s in samples if s['pss_kib'] is not None];cpu=usage.ru_utime+usage.ru_stime;elapsed=end-start
result={'scope':'Existing cache-miss generator computes both business_fft and hector_search together; not an isolated per-method benchmark','offline':True,'sample_interval_target_seconds':.005,'elapsed_seconds':elapsed,'cpu_user_seconds':usage.ru_utime,'cpu_system_seconds':usage.ru_stime,'cpu_total_seconds':cpu,'average_cpu_percent':cpu/elapsed*100,'average_logical_cores':cpu/elapsed,'sampled_pss_peak_mib':max(pss) if pss else None,'sampled_pss_samples':len(pss),'maximum_rss_mib_for_reference_only':usage.ru_maxrss/1024,'returned_poses_equal_cached':True,'executable_sha256':hashlib.sha256(exe.read_bytes()).hexdigest(),'input_cloud_sha256':hashlib.sha256(cloud.read_bytes()).hexdigest(),'samples':samples}
(args.output/'cold_initialization_performance.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
