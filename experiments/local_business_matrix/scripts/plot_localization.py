#!/usr/bin/env python3
"""Offline plot of saved business localization, mapping and health outputs."""
from pathlib import Path
import argparse
import json
import os
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
parser.add_argument('--method', choices=['business', 'hector', 'hector_ekf'], default='business')
parser.add_argument('--baseline', type=Path)
args = parser.parse_args()
output, mapping = args.output, args.mapping
is_hector = args.method != 'business'
is_ekf = args.method == 'hector_ekf'
info = yaml.safe_load((output / 'map.yaml').read_text())
image = np.array(Image.open(output / info['image']))
reference = np.loadtxt(mapping / 'trajectory_at_save.tum')
business = np.atleast_2d(np.loadtxt(output / ('trajectory_hector.tum' if args.method == 'hector' else 'trajectory_business.tum')))
amcl = np.atleast_2d(np.loadtxt(output / 'trajectory_hector.tum')) if is_hector else np.loadtxt(output / 'trajectory_amcl.tum')
statuses = [json.loads(line) for line in (output / ('status.jsonl' if is_hector else 'localization_status.jsonl')).read_text().splitlines()]
times = np.array([m['stamp'] if is_hector else m['header']['stamp']['sec'] + m['header']['stamp']['nanosec'] * 1e-9
                  for m in statuses])
start = float(yaml.safe_load((args.bag / 'metadata.yaml').read_text())[
    'rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']) * 1e-9
origin = info['origin']
extent = [origin[0], origin[0] + image.shape[1]*info['resolution'],
          origin[1], origin[1] + image.shape[0]*info['resolution']]
fig = plt.figure(figsize=(12, 9), constrained_layout=True)
grid = fig.add_gridspec(2, 2, height_ratios=[3, 1])
for index, title in enumerate(['Saved production map', 'Localization trajectory detail']):
    ax = fig.add_subplot(grid[0, index])
    ax.imshow(image, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper')
    ax.plot(reference[:,1], reference[:,2], '--', color='tab:blue', linewidth=1,
            label='Mapping nodes (not ground truth)')
    if args.baseline:
        baseline = np.atleast_2d(np.loadtxt(args.baseline))
        ax.plot(baseline[:,1], baseline[:,2], ':', color='tab:orange', linewidth=.8, alpha=.65, label='Hector before EKF')
    ax.plot(business[:,1], business[:,2], color='tab:green', linewidth=1.2, label='Hector + EKF /base_pose' if is_ekf else ('Hector scan + IMU' if is_hector else 'Business /base_pose'))
    ax.scatter(business[0,1], business[0,2], color='orange', marker='o', s=35, label='Start')
    ax.scatter(business[-1,1], business[-1,2], color='red', marker='x', s=45, label='End')
    ax.set(title=title, xlabel='Map x [m]', ylabel='Map y [m]', aspect='equal')
    if index:
        ax.set_xlim(business[:,1].min()-1, business[:,1].max()+1)
        ax.set_ylim(business[:,2].min()-1, business[:,2].max()+1)
        ax.legend(loc='best', fontsize=8)
ax = fig.add_subplot(grid[1, :])
ax.step(times-start, [m['score'] if is_hector else m['confidence'] for m in statuses], where='post', color='tab:purple',
        label='Hector match score (not accuracy)' if is_hector else 'Business confidence (not accuracy)')
ax.axvline(amcl[-1,0]-start, color='tab:red', linestyle='--', label='Last Hector pose update' if is_hector else 'Last AMCL pose update')
ax.set(xlabel='Bag time [s]', ylabel='Match score' if is_hector else 'Confidence', ylim=(-.05, 1.1), xlim=(0, yaml.safe_load((args.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9))
ax.grid(alpha=.25)
ax.legend(loc='lower left', fontsize=8)
fig.savefig(output / 'localization_overview.png', dpi=160)
plt.close(fig)
print(output / 'localization_overview.png')
