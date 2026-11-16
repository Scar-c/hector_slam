#!/usr/bin/env python3
"""Serial 1x localization benchmarks using Linux process CPU time and PSS."""
import argparse,hashlib,json,os,platform,signal,subprocess,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PROFILES=json.loads((ROOT/'scripts/matrix_profiles.json').read_text())
HZ=os.sysconf('SC_CLK_TCK')
PIPELINE={'robot','merger','ekf','base_pose','map_server','amcl','hector','amcl_confidence'}

def process_snapshot(pid):
    try:
        text=Path(f'/proc/{pid}/stat').read_text();fields=text[text.rfind(')')+2:].split()
        ticks=int(fields[11])+int(fields[12]);start=int(fields[19]);threads=int(fields[17]);cpu=int(fields[36])
        values={}
        try:
            for line in Path(f'/proc/{pid}/smaps_rollup').read_text().splitlines()[1:]:
                k,_,v=line.partition(':')
                if v.strip():values[k]=int(v.split()[0])
            pss=values['Pss']
        except (OSError,KeyError,ValueError):pss=None
        return {'pid':pid,'start_ticks':start,'cpu_ticks':ticks,'pss_kib':pss,'rss_kib':values.get('Rss'),'threads':threads,'last_cpu':cpu}
    except (OSError,ValueError,IndexError):return None

def descendants(pid):
    result=[];todo=[pid];seen=set()
    while todo:
        parent=todo.pop()
        if parent in seen:continue
        seen.add(parent);result.append(parent)
        try:todo.extend(map(int,Path(f'/proc/{parent}/task/{parent}/children').read_text().split()))
        except OSError:pass
    return result

def host_snapshot():
    result={}
    for line in Path('/proc/stat').read_text().splitlines():
        if line.startswith('cpu'):
            values=line.split();v=list(map(int,values[1:9]));result[values[0]]={'total':sum(v),'idle':v[3]+v[4]}
    return result

def load_json(path,default):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return default

def summarize(samples,groups,start,end):
    selected=[]
    for s in samples:
        overlap=max(0,min(s['monotonic_seconds'],end)-max(s['monotonic_seconds']-s['dt_seconds'],start))
        if overlap>0:selected.append((s,overlap))
    result={'window_seconds':end-start,'covered_seconds':sum(w for _,w in selected),'samples':len(selected),'groups':{}}
    if not selected:return result
    def stats(values,weights):
        a=np.array(values,float);w=np.array(weights,float)
        return {'mean':float(np.average(a,weights=w)),'p95':float(np.quantile(a,.95)),'peak':float(a.max())}
    for group in groups:
        present=[(s['groups'][group],w) for s,w in selected if group in s['groups']]
        if not present:continue
        valid=[(g,w) for g,w in present if g['cpu_complete']]
        cpu=stats([g['cpu_percent'] for g,w in valid],[w for g,w in valid]) if valid else None
        mem=[(g,w) for g,w in present if g['pss_complete']]
        pss=stats([g['pss_mib'] for g,w in mem],[w for g,w in mem]) if mem else None
        result['groups'][group]={'cpu_percent':cpu,'logical_cores':{k:v/100 for k,v in cpu.items()} if cpu else None,'machine_cpu_percent':{k:v/os.cpu_count() for k,v in cpu.items()} if cpu else None,'pss_mib':pss,'cpu_time_seconds':sum(g['cpu_percent']/100*w for g,w in valid),'missing_cpu_samples':len(present)-len(valid),'missing_pss_samples':len(present)-len(mem),'threads_peak':max(g['threads'] for g,w in present)}
    host=[(s,w) for s,w in selected if s['host_logical_cores'] is not None]
    if host:
        result['host_logical_cores']=stats([s['host_logical_cores'] for s,w in host],[w for s,w in host])
        result['host_busy_percent']=stats([s['host_logical_cores']/os.cpu_count()*100 for s,w in host],[w for s,w in host])
    return result

def run_case(args,method,name,index):
    out=args.output/method/name;out.parent.mkdir(exist_ok=True)
    domain=args.domain_base+list(PROFILES).index(name)
    env=os.environ.copy();env['ROS_DOMAIN_ID']=str(domain)
    command=['bash',str(ROOT/'scripts/run_localization_matrix.sh'),'--profile',name,'--initializer',method,'--expected-domain',str(domain),'--output',str(out),'--rate','1','--no-rviz','--no-hold','--no-plot','--performance-markers']
    if args.duration:command+=['--duration',str(args.duration)]
    previous={};host_previous=host_snapshot();last_time=time.monotonic();samples=[];group_names=set();affinity={}
    log=(out.parent/(name+'.log')).open('w')
    runner=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    start=time.monotonic();next_sample=start
    try:
        while runner.poll() is None:
            now=time.monotonic()
            if now<next_sample:time.sleep(min(.05,next_sample-now));continue
            manifest=load_json(out/'runtime_processes.json',[])
            roots={int(p['pid']):p['name'] for p in manifest}
            assignments={runner.pid:'runner',os.getpid():'monitor'}
            for pid,label in roots.items():
                for child in descendants(pid):assignments[child]=label
            snapshot=[];dt=now-last_time
            for pid,label in assignments.items():
                proc=process_snapshot(pid)
                if proc is None:continue
                key=(pid,proc['start_ticks']);old=previous.get(key)
                cpu=(proc['cpu_ticks']-old)/HZ/dt*100 if old is not None and dt>0 else None
                previous[key]=proc['cpu_ticks'];proc.update(name=label,cpu_percent=cpu);snapshot.append(proc)
                if str(pid) not in affinity:
                    try:affinity[str(pid)]={'name':label,'cpus_allowed_list':next(x.split(':',1)[1].strip() for x in Path(f'/proc/{pid}/status').read_text().splitlines() if x.startswith('Cpus_allowed_list:'))}
                    except (OSError,StopIteration):pass
            h=host_snapshot();per_cpu={}
            for k,v in h.items():
                old=host_previous.get(k)
                if old:
                    total=v['total']-old['total'];busy=total-(v['idle']-old['idle']);per_cpu[k]=busy/total*100 if total else 0
            logical=per_cpu.get('cpu',0)/100*os.cpu_count()
            phase=load_json(out/'performance_phase.json',{}).get('phase','starting')
            members={label:[p for p in snapshot if p['name']==label] for label in {p['name'] for p in snapshot}}
            members['pipeline']=[p for p in snapshot if p['name'] in PIPELINE]
            members['harness']=[p for p in snapshot if p['name'] in {'runner','player'}]
            members['localizer']=[p for p in snapshot if p['name'] in {'hector','amcl'}]
            groups={}
            for label,parts in members.items():
                if not parts:continue
                groups[label]={'cpu_percent':sum(p['cpu_percent'] or 0 for p in parts),'cpu_complete':all(p['cpu_percent'] is not None for p in parts),'pss_mib':sum(p['pss_kib'] or 0 for p in parts)/1024,'pss_complete':all(p['pss_kib'] is not None for p in parts),'threads':sum(p['threads'] for p in parts)}
            group_names.update(groups)
            sample={'monotonic_seconds':now,'elapsed_seconds':now-start,'dt_seconds':dt,'phase':phase,'processes':snapshot,'groups':groups,'host_logical_cores':logical,'host_per_cpu_percent':per_cpu}
            samples.append(sample)
            if out.exists():
                with (out/'resource_samples.jsonl').open('a') as f:f.write(json.dumps(sample)+'\n')
            host_previous=h;last_time=now;next_sample=now+args.interval
        runner.wait()
    finally:
        if runner.poll() is None:
            os.kill(runner.pid,signal.SIGINT)
            try:runner.wait(timeout=30)
            except subprocess.TimeoutExpired:os.kill(runner.pid,signal.SIGTERM);runner.wait(timeout=10)
        log.close()
    phase=load_json(out/'performance_phase.json',{});events={e['phase']:e['monotonic_ns']/1e9 for e in phase.get('events',[])}
    result={'profile':name,'initializer':method,'output':str(out),'returncode':runner.returncode,'replay_rate':1,'domain':domain,'sample_interval_seconds':args.interval,'full_bag':not bool(args.duration),'window_policy':'steady = playing+5s to playback_end-1s; active = all playback; no RViz','pss_source':'/proc/PID/smaps_rollup Pss, summed across distinct process PIDs; KiB/1024=MiB','cpu_source':'delta(/proc/PID/stat utime+stime)/SC_CLK_TCK/delta(monotonic); 100%=one logical CPU','affinity':affinity,'events':events,'windows':{}}
    if 'playing' in events and 'draining' in events:
        a,b=events['playing'],events['draining']
        for label,left,right in [('active',a,b),('steady',a+5,b-1),('initialization',events.get('initializing',start),a)]:
            result['windows'][label]=summarize(samples,sorted(group_names),left,right)
        if 'postprocessing' in events and 'complete' in events:result['windows']['postprocessing']=summarize(samples,sorted(group_names),events['postprocessing'],events['complete'])
    summary=load_json(out/'summary.json',{})
    result['functional_checks']={'sensor_input_complete':summary.get('all_sensor_messages_received') if not args.duration else None,'fixed_map':summary.get('published_map_versions')==1 and summary.get('published_map_equals_pgm_yaml',False),'scan_geometry':summary.get('scan_alignment_validation',{}).get('mean_beam_agreement_fraction',0)>=.999,'poses':summary.get('poses',0),'initialization_sha256':summary.get('initialization_sha256')}
    steady=result['windows'].get('steady',{});pipe=steady.get('groups',{}).get('pipeline',{})
    result['passed']=bool(runner.returncode==0 and pipe.get('cpu_percent') and pipe.get('pss_mib') and steady.get('samples',0)>=3 and pipe.get('missing_pss_samples',1)==0 and result['functional_checks']['fixed_map'] and result['functional_checks']['scan_geometry'] and (args.duration or result['functional_checks']['sensor_input_complete']))
    (out/'performance_summary.json').write_text(json.dumps(result,indent=2))
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--profiles',nargs='+',choices=list(PROFILES),default=list(PROFILES));p.add_argument('--initializers',nargs='+',choices=['business_fft','hector_search'],default=['business_fft']);p.add_argument('--resume',action='store_true');p.add_argument('--domain-base',type=int,default=210);p.add_argument('--interval',type=float,default=1);p.add_argument('--duration',type=float,default=0);args=p.parse_args()
    if not 0<=args.domain_base<=221:p.error('Domain base must be in [0,221]')
    if args.interval<.2 or args.duration<0:p.error('Interval >=0.2 seconds and duration >=0 required')
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=args.resume)
    cpuinfo=next((x.split(':',1)[1].strip() for x in Path('/proc/cpuinfo').read_text().splitlines() if x.startswith('model name')),'unknown')
    governor=Path('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor')
    paths=[ROOT/'scripts/run_localization_matrix.py',Path(__file__),ROOT/'scripts/matrix_profiles.json',ROOT/'install/hector_mapping/lib/hector_mapping/hector_mapping_node',ROOT/'install/hector_mapping/lib/hector_mapping/time_aligned_scan_merger_node',ROOT/'install/hector_mapping/lib/hector_mapping/matrix_base_pose_from_tf',Path('/opt/ros/jazzy/lib/nav2_amcl/amcl'),Path('/opt/ros/jazzy/lib/robot_localization/ekf_node')]
    metadata={'cpu_model':cpuinfo,'logical_cpus':os.cpu_count(),'benchmark_cpu_affinity':sorted(os.sched_getaffinity(0)),'kernel':platform.release(),'platform':platform.platform(),'clock_ticks_per_second':HZ,'governor':governor.read_text().strip() if governor.exists() else None,'sequential':True,'replay_rate':1,'rviz':False,'recording':'Existing replay/recording harness stays on and is measured separately from localization nodes. Node DDS publishing load includes recorder subscriptions.','initialization':'Shared cached seed, not cold FFT/search; initializer execution cost is excluded from steady-state results.','source_sha256':{str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}}
    results=[]
    if args.resume:
        original=load_json(args.output/'benchmark_environment.json',{})
        if not original:p.error('Resume requires an existing benchmark_environment.json')
        for path,digest in original['source_sha256'].items():
            if path!=str(Path(__file__)) and hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:
                p.error('Measured executable/config changed since previous cases: '+path)
        source=args.output/'benchmark_source/benchmark_localization_matrix.py'
        previous_code=source.read_text().split('def main():')[0]
        measurement_code=Path(__file__).read_text().split('def main():')[0]
        previous_code=previous_code.replace('domain=180+list(PROFILES).index(name)','domain=args.domain_base+list(PROFILES).index(name)')
        if previous_code!=measurement_code:p.error('Sampling implementation changed; do not mix measurements')
        previous=load_json(args.output/'performance_progress.json',[])
        backup=args.output/'performance_progress_before_fft_scope.json'
        if not backup.exists():backup.write_text(json.dumps(previous,indent=2))
        supplemental=[r for r in previous if r['initializer'] not in args.initializers or r['profile'] not in args.profiles]
        if supplemental:(args.output/'performance_supplementary_non_fft.json').write_text(json.dumps(supplemental,indent=2))
        results=[r for r in previous if r['initializer'] in args.initializers and r['profile'] in args.profiles and r['passed']]
        original.update(primary_initializers=args.initializers,primary_profiles=args.profiles,resume_controller_sha256=metadata['source_sha256'][str(Path(__file__))],sampling_implementation_unchanged=True,dds_domain_base=args.domain_base)
        metadata=original
        (args.output/'benchmark_source/benchmark_localization_matrix_fft_resume.py').write_bytes(Path(__file__).read_bytes())
    (args.output/'benchmark_environment.json').write_text(json.dumps(metadata,indent=2))
    (args.output/'performance_progress.json').write_text(json.dumps(results,indent=2))
    cases=[(method,name) for name in args.profiles for method in args.initializers]
    completed={(r['initializer'],r['profile']) for r in results}
    for index,(method,name) in enumerate(cases,1):
        if (method,name) in completed:
            print('Keeping completed case:',name,method,flush=True);continue
        folder=args.output/method/name
        if folder.exists():
            diagnostics=args.output/'diagnostics';diagnostics.mkdir(exist_ok=True)
            preserved=diagnostics/(method+'__'+name+'__interrupted_'+str(time.time_ns()))
            folder.rename(preserved)
            log=folder.parent/(name+'.log')
            if log.exists():log.rename(preserved/'interrupted_runner.log')
            (preserved/'INTERRUPTED_FOR_SCOPE_CHANGE.md').write_text('Interrupted when narrowing scope to cached business FFT; preserved as diagnostic only.\n')
        result=run_case(args,method,name,index);results.append(result)
        (args.output/'performance_progress.json').write_text(json.dumps(results,indent=2))
        pipe=result.get('windows',{}).get('steady',{}).get('groups',{}).get('pipeline',{})
        print(f"{index}/{len(cases)} {'PASS' if result['passed'] else 'FAIL'} {name} {method} CPU={(pipe.get('cpu_percent') or {}).get('mean')}% PSS={(pipe.get('pss_mib') or {}).get('mean')}MiB",flush=True)
    report={'passed':all(r['passed'] for r in results),'cases':len(results),'environment':metadata,'results':results}
    (args.output/'performance_results.json').write_text(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
