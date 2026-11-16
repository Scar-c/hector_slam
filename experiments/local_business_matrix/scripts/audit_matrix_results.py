#!/usr/bin/env python3
"""Audit actual node parameters, recorded flags and shared seed files offline."""
import argparse,hashlib,json
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('folder',type=Path);args=p.parse_args()
profiles=json.loads((ROOT/'scripts/matrix_profiles.json').read_text());rows=[];methods={}
for method in ['business_fft','hector_search']:
    methods[method]=[]
    for name,profile in profiles.items():
        folder=args.folder/method/name
        if not (folder/'summary.json').exists():continue
        summary=json.loads((folder/'summary.json').read_text());seed=json.loads((folder/'seed.json').read_text());initial=json.loads((folder/'initialization.json').read_text())
        runtime=json.loads((folder/'runtime_processes.json').read_text());nodes={x['name']:x for x in runtime}
        flags=[json.loads(line) for line in (folder/'scan_alignment_status.jsonl').read_text().splitlines()]
        commands=nodes['merger']['command']
        assert 'apply_frame_alignment:='+str(profile['time']).lower() in commands
        assert 'apply_beam_deskew:='+str(profile['desk']).lower() in commands
        assert bool('ekf' in nodes)==profile['ekf']
        assert bool('base_pose' in nodes)==profile['ekf']
        assert all(x['time_alignment']==profile['time'] and x['beam_deskew']==profile['desk'] for x in flags)
        assert seed['offset']==[0,0,0]
        assert seed['pose']==[initial[k] for k in ['initial_pose_x','initial_pose_y','initial_pose_yaw']]
        assert summary['replay_rate']==1
        if profile['algorithm']=='hector':
            params=yaml.safe_load((folder/'hector.yaml').read_text())['hector_mapping']['ros__parameters']
            assert params['use_odom_pose_start_estimate']==profile['pre']
            assert params['allow_odom_tf_without_pose_prior']==(profile['ekf'] and not profile['pre'])
            assert not params['global_relocalization'] and params['localization_only']
        else:
            assert 'amcl' in nodes and 'map_server' in nodes
            assert hashlib.sha256((folder/'amcl.yaml').read_bytes()).hexdigest()==hashlib.sha256((ROOT.parents[1]/'src/zx_localization/config/amcl.yaml').read_bytes()).hexdigest()
        entry={'initializer':method,'profile':name,'seed_sha256':initial['result_sha256'],'seed':seed['pose'],'ekf_config_sha256':hashlib.sha256((folder/'ekf.yaml').read_bytes()).hexdigest() if profile['ekf'] else None,'map_pgm_sha256':hashlib.sha256((folder/'map.pgm').read_bytes()).hexdigest(),'runtime_features_checked':True}
        methods[method].append(entry);rows.append(entry)
    if methods[method]:assert len({(x['seed_sha256'],tuple(x['seed'])) for x in methods[method]})==1
if rows:assert len({x['map_pgm_sha256'] for x in rows})==1
if any(x['ekf_config_sha256'] for x in rows):assert len({x['ekf_config_sha256'] for x in rows if x['ekf_config_sha256']})==1
report={'complete':len(rows)==24,'verified_cases':len(rows),'actual_parameters_checked':True,'shared_map_and_ekf_config':True,'same_seed_within_initializer':True,'cases':rows}
(args.folder/'offline_parameter_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2))
