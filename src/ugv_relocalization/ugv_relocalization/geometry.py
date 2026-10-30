"""Graph/refmap transforms. T_graph_refmap always uses raw camera_init axes."""
import math
import numpy as np
import yaml
from scipy.spatial import cKDTree
from .core.frames import pose_T, xyz_rpy, invert_T

R_PERM = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
ROS_FROM_CI = np.eye(4)
ROS_FROM_CI[:3, :3] = R_PERM


def planar(T):
    a = xyz_rpy(T)
    return pose_T(a[0], a[1], 0., yaw=a[5])


def graph_pose(graph, node, facing):
    a, b = graph['nodes'][node], graph['nodes'][facing]
    if np.linalg.norm(np.array(a)-b) < 1.e-6:
        raise ValueError('start and facing nodes must differ')
    return pose_T(a[0], a[1], 0., yaw=math.atan2(b[1]-a[1], b[0]-a[0]))


def validate_transform(T):
    T = np.asarray(T, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1])
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1.)):
        raise ValueError('invalid rigid transform')
    return T


def load_calibration(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    if c.get('parent') != 'map' or c.get('child') != 'camera_init_reference':
        raise ValueError('calibration frames do not match graph/reference camera_init')
    return validate_transform(c['T_graph_refmap'])


def calibration_from_pose(T_graph_base, T_refros_lidar, T_base_lidar):
    T_refros_base = T_refros_lidar @ invert_T(T_base_lidar)
    return planar(T_graph_base) @ invert_T(planar(T_refros_base)) @ ROS_FROM_CI


class Coverage:
    """Conservative local point-support test; sparse regions are odometry-only."""
    def __init__(self, map_ci, radius=5., min_points=30):
        self.xy = (map_ci @ R_PERM.T)[:, :2]
        self.tree = cKDTree(self.xy)
        self.radius, self.min_points = radius, min_points

    def contains(self, xy):
        return len(self.tree.query_ball_point(np.asarray(xy), self.radius)) >= self.min_points
