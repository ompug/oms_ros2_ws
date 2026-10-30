"""Export the provenance-recorded YAML graph into the previous stack's CSV interface."""
import csv
from pathlib import Path


def export_graph(graph, directory):
    root=Path(directory); root.mkdir(parents=True,exist_ok=True)
    nodes=root/'nodes.csv'; edges=root/'edges.csv'
    with nodes.open('w') as f:
        w=csv.writer(f); w.writerow(['node_id','x','y','yaw'])
        for key,xy in graph['nodes'].items():
            w.writerow([key,*xy,graph['node_yaw'][key]])
    with edges.open('w') as f:
        w=csv.DictWriter(f,fieldnames=['edge_id','u','v','length','status']);w.writeheader()
        w.writerows(graph['edges'])
    return str(nodes),str(edges)
