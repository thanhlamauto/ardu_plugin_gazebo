#!/usr/bin/env python3
"""Plot per-frame center-patch monocular depth against sparse LiDAR truth."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial',type=Path,nargs='+')
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    fig,axes=plt.subplots(1,2,figsize=(10,4.2),layout='constrained')
    for index,path in enumerate(args.trial):
        rows=[json.loads(line) for line in (path/'center_patch_depth_pairs.jsonl').open()]
        truth=np.array([row['lidar_depth_m'] for row in rows])
        predicted=np.array([row['monocular_depth_m'] for row in rows])
        label=f'Run {index+1} (n={len(rows)})'
        axes[0].scatter(truth,predicted,s=15,alpha=.65,label=label)
        axes[1].scatter(truth,predicted-truth,s=15,alpha=.65,label=label)
    domain=np.linspace(3,7.5,100)
    axes[0].plot(domain,domain,color='black',lw=1.5,label='Ideal')
    axes[0].set(xlabel='LiDAR axial depth (m)',ylabel='Monocular estimate (m)',
                xlim=(3,7.5),ylim=(3,7.5),title='Center of obstacle image')
    axes[1].axhline(0,color='black',lw=1.5)
    axes[1].axhspan(-.5,.5,color='gray',alpha=.15,label='±0.5 m')
    axes[1].set(xlabel='LiDAR axial depth (m)',ylabel='Estimate − LiDAR (m)',
                xlim=(3,7.5),title='Range-dependent bias')
    for axis in axes:
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=180)
    plt.close(fig)


if __name__=='__main__':main()
