"""Graph/map coverage diagnostic. Uncalibrated overlays are explicitly provisional."""
import argparse
import csv
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
from .engine import load
from .core.io import read_pcd
from .geometry import Coverage, ROS_FROM_CI, load_calibration


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration', default='')
    p.add_argument('--output', required=True)
    p.add_argument('--map-path', default='')
    a=p.parse_args()
    share=Path(get_package_share_directory('ugv_relocalization'))
    graph=load(share/'config/graph.yaml')
    points=read_pcd(a.map_path or share/'maps/reference_map.pcd')[0]
    points=points[np.isfinite(points).all(axis=1)]
    T=load_calibration(a.calibration) if a.calibration else ROS_FROM_CI
    mapped=points @ T[:3,:3].T + T[:3,3]
    support=Coverage(points)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax=plt.subplots(figsize=(12,8))
    ax.scatter(mapped[::3,0],mapped[::3,1],s=.15,c='0.65',label='reference map')
    rows=[]
    for edge in graph['edges']:
        start,end=np.asarray(graph['nodes'][edge['u']]),np.asarray(graph['nodes'][edge['v']])
        xy=np.linspace(start,end,max(2,int(np.linalg.norm(end-start)/.5)+1))
        graph3=np.column_stack([xy,np.zeros(len(xy))])
        ref=(graph3-T[:3,3]) @ T[:3,:3] @ ROS_FROM_CI[:3,:3].T
        ok=[support.contains(x[:2]) for x in ref]
        status=('covered' if all(ok) else 'odometry_only') if a.calibration else 'uncalibrated'
        rows.append(dict(edge_id=edge['edge_id'],u=edge['u'],v=edge['v'],status=status,
                         supported_fraction=float(np.mean(ok)) if a.calibration else 'unknown'))
        ax.plot(xy[:,0],xy[:,1],color={'covered':'green','odometry_only':'red','uncalibrated':'orange'}[status],lw=1.2)
    for k,xy in graph['nodes'].items():
        if not k.startswith('__'):
            ax.text(*xy,k,fontsize=8)
    ax.set_title('Graph coverage: green=supported, red=odometry only' if a.calibration else
                 'PROVISIONAL: no graph calibration; orange edges have UNKNOWN coverage')
    ax.set(xlabel='graph X (m)',ylabel='graph Y (m)',aspect='equal')
    out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(out,dpi=180,bbox_inches='tight')
    with out.with_suffix('.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    print(out)
