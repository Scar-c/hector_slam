#!/usr/bin/env python3
"""Offline plot of saved business localization, mapping and health outputs."""
from pathlib import Path
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

output = root / 'results/amcl_localization'
mapping = root / 'results/business_single/lo'
info = yaml.safe_load((output / 'map.yaml').read_text())
image = np.array(Image.open(output / info['image']))
reference = np.loadtxt(mapping / 'trajectory_at_save.tum')
business = np.loadtxt(output / 'trajectory_business.tum')
amcl = np.loadtxt(output / 'trajectory_amcl.tum')
statuses = [json.loads(line) for line in (output / 'localization_status.jsonl').read_text().splitlines()]
times = np.array([m['header']['stamp']['sec'] + m['header']['stamp']['nanosec'] * 1e-9
                  for m in statuses])
start = float(yaml.safe_load((root / 'metadata.yaml').read_text())[
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
    ax.plot(business[:,1], business[:,2], color='tab:green', linewidth=1.2, label='Business /base_pose')
    ax.scatter(business[0,1], business[0,2], color='orange', marker='o', s=35, label='Start')
    ax.scatter(business[-1,1], business[-1,2], color='red', marker='x', s=45, label='End')
    ax.set(title=title, xlabel='Map x [m]', ylabel='Map y [m]', aspect='equal')
    if index:
        ax.set_xlim(business[:,1].min()-1, business[:,1].max()+1)
        ax.set_ylim(business[:,2].min()-1, business[:,2].max()+1)
        ax.legend(loc='best', fontsize=8)
ax = fig.add_subplot(grid[1, :])
ax.step(times-start, [m['confidence'] for m in statuses], where='post', color='tab:purple',
        label='Business confidence (not accuracy)')
ax.axvline(amcl[-1,0]-start, color='tab:red', linestyle='--', label='Last AMCL pose update')
ax.set(xlabel='Bag time [s]', ylabel='Confidence', ylim=(-.05, 1.1), xlim=(0, 77.35))
ax.grid(alpha=.25)
ax.legend(loc='lower left', fontsize=8)
fig.savefig(output / 'localization_overview.png', dpi=160)
plt.close(fig)
print(output / 'localization_overview.png')
