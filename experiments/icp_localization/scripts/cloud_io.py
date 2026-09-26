"""Small, dependency-light PCD and ROS bag helpers for this experiment."""
import pathlib
import numpy as np


def read_pcd(path):
    path = pathlib.Path(path)
    with path.open('rb') as stream:
        header = {}
        while True:
            line = stream.readline().decode('ascii').strip()
            if not line:
                raise ValueError(f'Incomplete PCD header: {path}')
            if line.startswith('#'):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        if any(c != 1 for c in counts):
            raise ValueError('Multi-count PCD fields are unsupported')
        dtype = np.dtype([(f, {'F': 'f', 'I': 'i', 'U': 'u'}[t] + str(s))
                          for f, t, s in zip(fields, types, sizes)])
        count = int(header['POINTS'][0])
        encoding = header['DATA'][0].lower()
        if encoding == 'binary':
            points = np.fromfile(stream, dtype=dtype, count=count)
        elif encoding == 'ascii':
            rows = np.loadtxt(stream, dtype=np.float64).reshape(-1, len(fields))
            points = np.empty(len(rows), dtype=dtype)
            for i, field in enumerate(fields):
                points[field] = rows[:, i]
        else:
            raise ValueError(f'Unsupported PCD DATA {encoding}')
    if len(points) != count:
        raise ValueError(f'Expected {count} points, found {len(points)}')
    return header, points


def write_pcd(path, xyz, intensity=None):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('xyz must be Nx3')
    names = ['x', 'y', 'z'] + (['intensity'] if intensity is not None else [])
    points = np.empty(len(xyz), dtype=[(name, '<f4') for name in names])
    for i, name in enumerate(names[:3]):
        points[name] = xyz[:, i]
    if intensity is not None:
        points['intensity'] = np.asarray(intensity, dtype=np.float32)
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS ' + ' '.join(names) +
              '\nSIZE ' + ' '.join(['4'] * len(names)) +
              '\nTYPE ' + ' '.join(['F'] * len(names)) +
              '\nCOUNT ' + ' '.join(['1'] * len(names)) +
              f'\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    with path.open('wb') as stream:
        stream.write(header.encode('ascii'))
        points.tofile(stream)


def xyz_array(points):
    return np.column_stack([points[name] for name in ('x', 'y', 'z')])


def bag_messages(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {entry.name: get_message(entry.type) for entry in reader.get_all_topics_and_types()
             if entry.name in topics}
    missing = set(topics) - set(types)
    if missing:
        raise ValueError(f'Missing topics in {path}: {sorted(missing)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, payload, bag_stamp = reader.read_next()
        yield topic, deserialize_message(payload, types[topic]), bag_stamp


def message_stamp_ns(msg):
    stamp = msg.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def cloud_arrays(msg):
    from sensor_msgs_py import point_cloud2
    fields = [field.name for field in msg.fields]
    if not {'x', 'y', 'z'} <= set(fields):
        raise ValueError(f'Cloud lacks XYZ: {fields}')
    names = ('x', 'y', 'z', 'intensity') if 'intensity' in fields else ('x', 'y', 'z')
    pts = point_cloud2.read_points(msg, field_names=names, skip_nans=False)
    xyz = np.column_stack([pts[name] for name in names[:3]]).astype(np.float32)
    intensity = np.asarray(pts['intensity'], np.float32) if len(names) == 4 else None
    return xyz, intensity
