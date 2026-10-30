"""Audit whether a bag can independently identify its graph start node."""
import argparse
import csv
import numpy as np
import json
import sqlite3
from pathlib import Path
import yaml


def trajectory_candidates(trajectory, graph):
    """Weak graph-shape cross-check: named start nodes ±2m, 12 initial yaw basins.

    Assumes the driven trajectory follows graph edges. This is not surveyed alignment.
    """
    from scipy.spatial import cKDTree
    from scipy.optimize import least_squares
    with open(trajectory) as f:
        rows=list(csv.DictReader(f))
    xy=np.array([[float(r['z']),float(r['x'])] for r in rows])
    xy=(xy-xy[0])[::4]
    samples=[]
    for e in graph['edges']:
        a,b=np.asarray(graph['nodes'][e['u']]),np.asarray(graph['nodes'][e['v']])
        samples.extend(np.linspace(a,b,max(2,int(np.linalg.norm(a-b)/.25)+1)))
    tree=cKDTree(samples)
    ranked=[]
    for node,origin in graph['nodes'].items():
        if node.startswith('__'):
            continue
        origin=np.asarray(origin)
        best=None
        for yaw in np.radians(np.arange(0,360,30)):
            def residual(v):
                c,s=np.cos(v[2]),np.sin(v[2])
                return tree.query(xy @ np.array([[c,s],[-s,c]])+v[:2])[0]
            fit=least_squares(residual,[*origin,yaw],
                bounds=([*(origin-2.),yaw-np.pi/12],[*(origin+2.),yaw+np.pi/12]),
                loss='soft_l1',max_nfev=60)
            d=residual(fit.x)
            candidate=dict(node=node,rmse_m=float(np.sqrt(np.mean(d*d))),
                median_m=float(np.median(d)),p90_m=float(np.percentile(d,90)),
                start_xy=fit.x[:2].tolist(),yaw_deg=float(np.degrees(fit.x[2])))
            if best is None or candidate['rmse_m']<best['rmse_m']:
                best=candidate
        ranked.append(best)
    ranked.sort(key=lambda c:c['rmse_m'])
    return dict(method='heuristic trajectory-to-graph fit; not a calibration',
        assumptions='start within ±2 m per axis of named node, trajectory near graph edges; yaw multistart in 30 degree basins',
        trajectory_samples=len(xy),ranked_named_nodes=ranked,
        conclusion='No node is accepted by this heuristic; require surveyed calibration and an independent landmark cross-check.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag')
    p.add_argument('--output', required=True)
    p.add_argument('--trajectory', default='', help='optional processed aft_mapped_to_init.csv for a weak graph-shape check')
    a=p.parse_args()
    bag=Path(a.bag)
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    topics=metadata['topics_with_message_count']
    graph_topics=[t['topic_metadata'] for t in topics if t['topic_metadata']['type'] in
                  ('sensor_msgs/msg/NavSatFix','geographic_msgs/msg/GeoPoseStamped') or
                  t['topic_metadata']['name'] in ('/lego/odometry','/initialpose')]
    report=dict(bag=str(bag),topics=[t['topic_metadata'] for t in topics], graph_evidence_topics=graph_topics,
        conclusion='No absolute graph start evidence: node match is underdetermined without calibration or a surveyed landmark.'
        if not graph_topics else 'Candidate absolute topics present; inspect coordinates and datum before matching nodes.',
        lidar_first_header_ns=None, lidar_first_record_ns=None)
    # Read only a single message (never copy a 16.8 GiB bag).
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from .ros_common import stamp_to_ns
    for db in sorted(bag.glob('*.db3')):
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
            row=con.execute("SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name='/rslidar_points' ORDER BY m.timestamp LIMIT 1").fetchone()
        if row:
            msg=deserialize_message(row[1],PointCloud2)
            report.update(lidar_first_header_ns=stamp_to_ns(msg.header.stamp),lidar_first_record_ns=row[0])
            break
    if a.trajectory:
        from ament_index_python.packages import get_package_share_directory
        share=Path(get_package_share_directory('ugv_relocalization'))
        graph=yaml.safe_load((share/'config/graph.yaml').read_text())
        report['trajectory_graph_crosscheck']=trajectory_candidates(a.trajectory,graph)
    Path(a.output).parent.mkdir(parents=True,exist_ok=True)
    Path(a.output).write_text(json.dumps(report,indent=2)+'\n')
    print(report['conclusion'])
