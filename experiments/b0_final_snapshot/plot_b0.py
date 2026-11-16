#!/usr/bin/env python3
"""Offline plot of saved business localization, mapping and health outputs."""
from pathlib import Path
import argparse
import json
import os
import shutil
import numpy as np
import yaml
from PIL import Image
root = Path(__file__).resolve().parent.parent
os.environ.setdefault('MPLCONFIGDIR', str(root / 'results/.matplotlib_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=root / 'results/amcl_localization')
parser.add_argument('--mapping', type=Path, default=root / 'results/business_single/lo')
parser.add_argument('--bag', type=Path, default=root)
parser.add_argument('--method', choices=['business', 'hector', 'hector_ekf', 'pgm_fcsm'], default='business')
parser.add_argument('--baseline', type=Path)
args = parser.parse_args()
output, mapping = args.output, args.mapping
# Matrix metadata is optional, preserving the existing business/FCSM plot paths.
matrix_summary = json.loads((output / 'summary.json').read_text()) if (output / 'profile.json').exists() and (output / 'summary.json').exists() else {}
matrix_name = 'b0_hector_gravity_cloud'
matrix_legend = 'Hector + shared IMU reference\nXYZ -> gravity -> z[-0.8,2.0]m\n3D voxel=0.025m; 3 map levels'
if matrix_summary.get('profile') and matrix_summary.get('initializer'):
    matrix_name = matrix_summary['profile'] + '__' + matrix_summary['initializer']
    features = matrix_summary['features']
    flags = ' '.join(key + '=' + ('on' if features[key] else 'off') for key in ['ekf', 'pre', 'desk', 'time'])
    matrix_legend = matrix_summary['profile'] + '\ninit=' + matrix_summary['initializer'] + '\n' + flags
is_hector = args.method in ['hector', 'hector_ekf']
is_ekf = args.method == 'hector_ekf'
is_fcsm = args.method == 'pgm_fcsm'
info = yaml.safe_load((output / 'map.yaml').read_text())
image = np.array(Image.open(output / info['image']))
reference = np.empty((0, 8))
business = np.atleast_2d(np.loadtxt(output / ('trajectory_hector.tum' if args.method == 'hector' else 'trajectory_business.tum')))
amcl = business if is_fcsm else (np.atleast_2d(np.loadtxt(output / 'trajectory_hector.tum')) if is_hector else np.atleast_2d(np.loadtxt(output / 'trajectory_amcl.tum')))
statuses = [json.loads(line) for line in (output / ('deskew_status.jsonl' if is_hector else 'localization_status.jsonl')).read_text().splitlines()]
times = np.array([m['reference_stamp_ns'] * 1e-9 for m in statuses])
start = float(yaml.safe_load((args.bag / 'metadata.yaml').read_text())[
    'rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']) * 1e-9
origin = info['origin']
extent = [origin[0], origin[0] + image.shape[1]*info['resolution'],
          origin[1], origin[1] + image.shape[0]*info['resolution']]
fig = plt.figure(figsize=(12, 9), constrained_layout=True)
grid = fig.add_gridspec(2, 2, height_ratios=[3, 1])
for index, title in enumerate(['Hector map from b0 backpack', 'Localization trajectory detail']):
    ax = fig.add_subplot(grid[0, index])
    ax.imshow(image, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper')
    if args.baseline:
        baseline = np.atleast_2d(np.loadtxt(args.baseline))
        ax.plot(baseline[:,1], baseline[:,2], ':', color='tab:orange', linewidth=.8, alpha=.65, label='Original Hector input pipeline')
    plotted = business[:,1:3].copy()
    if is_fcsm:
        # Do not draw an invented straight trajectory across a localization outage.
        plotted[np.r_[False, np.diff(business[:,0]) > .5], :] = np.nan
    ax.plot(plotted[:,0], plotted[:,1], color='tab:green', linewidth=1.2, label='PGM FCSM /base_pose' if is_fcsm else ('Hector + EKF /base_pose' if is_ekf else ('Hector + IMU + historical pose deskew' if is_hector else 'Business /base_pose')))
    ax.scatter(business[0,1], business[0,2], color='orange', marker='o', s=35, label='Start')
    ax.scatter(business[-1,1], business[-1,2], color='red', marker='x', s=45, label='End')
    ax.set(title=title, xlabel='Map x [m]', ylabel='Map y [m]', aspect='equal')
    if index:
        bounds = np.vstack([business[:,1:3], reference[:,1:3]]) if is_fcsm else business[:,1:3]
        ax.set_xlim(bounds[:,0].min()-1, bounds[:,0].max()+1)
        ax.set_ylim(bounds[:,1].min()-1, bounds[:,1].max()+1)
        ax.legend(loc='upper right' if matrix_name else 'best', fontsize=8, title=matrix_legend, title_fontsize=8)
ax = fig.add_subplot(grid[1, :])
ax.step(times-start, [float(m.get('score', 0)) if is_fcsm else (float(m['translation_valid']) if is_hector else m['confidence']) for m in statuses], where='post', color='tab:purple',
        label='FCSM score (not accuracy)' if is_fcsm else ('Historical translation prediction available' if is_hector else 'Business confidence (not accuracy)'))
if is_fcsm:
    ax.step(times-start, [float(m.get('inliers', 0)) for m in statuses], where='post', color='tab:green', alpha=.7, label='Endpoint inlier ratio')
    for event in json.loads((output / 'events.json').read_text()) if (output / 'events.json').exists() else []:
        ax.axvline(event['bag_time'], color='tab:red', linestyle='--', label='Map reload' if 'reload_map' in event['service'] else 'Force relocalization')
else:
    ax.axvline(amcl[-1,0]-start, color='tab:red', linestyle='--', label='Last Hector pose update' if is_hector else 'Last AMCL pose update')
ax.set(xlabel='Bag time [s]', ylabel='Match score / ratio' if is_fcsm else ('Prediction available (0/1)' if is_hector else 'Confidence'), ylim=(-.05, 1.1), xlim=(0, max(times[-1]-start, 1)))
ax.grid(alpha=.25)
ax.legend(loc='lower left', fontsize=8)
fig.savefig(output / 'localization_overview.png', dpi=160)
plt.close(fig)
if matrix_summary.get('profile') and matrix_summary.get('replay_rate') == 1.0 and matrix_summary.get('all_sensor_messages_received', True):
    overview = root / 'overview'
    overview.mkdir(parents=True, exist_ok=True)
    target = overview / (matrix_name + '.png')
    temporary = overview / ('.' + matrix_name + '.' + str(os.getpid()) + '.png')
    shutil.copyfile(output / 'localization_overview.png', temporary)
    temporary.replace(target)
    provenance = {'matrix': matrix_name, 'source_result': str(output.resolve()), 'profile': matrix_summary['profile'], 'initializer': matrix_summary['initializer'], 'features': matrix_summary['features'], 'replay_rate': matrix_summary['replay_rate'], 'initialization_sha256': matrix_summary.get('initialization_sha256')}
    (overview / (matrix_name + '.json')).write_text(json.dumps(provenance, indent=2))
    print(target)
print(output / 'localization_overview.png')

summary=json.loads((output/"summary.json").read_text())
if summary["full_bag"]:
    target=Path("/home/lc/slam_11_13/overview/b0_hector_deskew.png")
    target.parent.mkdir(exist_ok=True,parents=True)
    shutil.copyfile(output/"localization_overview.png",target)
