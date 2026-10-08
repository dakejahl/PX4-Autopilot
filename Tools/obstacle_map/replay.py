#!/usr/bin/env python3
"""
Replay a flight log through the obstacle map and write a 3D viewer as one HTML page.

The map is rebuilt from the logged range_image tiles and the estimated pose, by the same C++
code the vehicle runs, built on first use into ~/.cache/px4_obstacle_map. The page shows the
flight path, the setpoints, what Collision Prevention was given, the map's occupied voxels as
they appear and clear, and, for SIH logs, the simulated trunks.

Log range_image with SDLOG_PROFILE bit 7 (Computer Vision and Avoidance), which SITL sets.

    Tools/obstacle_map/replay.py log.ulg -o flight.html
"""

import argparse
import ctypes
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit('pyulog is required: pip install pyulog')

PX4_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
CORE_SOURCES = [
    'Tools/obstacle_map/replay_core.cpp',
    'src/lib/obstacle_sim/obstacle_sim.cpp',
    'src/lib/range_image/RangeImageGeometry.cpp',
    'src/modules/obstacle_map/ObstacleGrid.cpp',
]
CORE_HEADERS = [
    'src/lib/obstacle_sim/obstacle_sim.h',
    'src/lib/range_image/RangeImageGeometry.hpp',
    'src/modules/obstacle_map/ObstacleGrid.hpp',
]
CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'px4_obstacle_map')

# ObstacleMap.hpp and module.yaml defaults, for logs that do not carry them
DEFAULT_GRID = (64, 32)
DEFAULT_PARAMS = {'OMAP_VOX_SIZE': 0.15, 'OMAP_VEH_HGT': 0.5, 'OMAP_HIT': 2, 'OMAP_MISS': 1,
                  'OMAP_OCC_THR': 3, 'OMAP_FREE_THR': -2, 'OMAP_LO_MIN': -4}
BINS = 72
# trunks within this distance of the flight path are drawn
TREE_SEARCH_RADIUS = 15.0


def build_core():
    """Compile the map and world sources into a shared library, cached by their content."""
    digest = hashlib.sha1()

    for path in CORE_SOURCES + CORE_HEADERS:
        with open(os.path.join(PX4_ROOT, path), 'rb') as f:
            digest.update(f.read())

    library = os.path.join(CACHE_DIR, 'replay_core_%s.so' % digest.hexdigest()[:12])

    if not os.path.exists(library):
        os.makedirs(CACHE_DIR, exist_ok=True)
        compiler = os.environ.get('CXX', 'c++')
        command = [compiler, '-O2', '-std=c++17', '-shared', '-fPIC', '-I', os.path.join(PX4_ROOT, 'src'),
                   '-o', library + '.tmp'] + [os.path.join(PX4_ROOT, path) for path in CORE_SOURCES]
        subprocess.run(command, check=True)
        os.replace(library + '.tmp', library)

    return library


class Core:
    """ctypes wrapper of replay_core.cpp"""

    def __init__(self):
        lib = ctypes.CDLL(build_core())
        c_float_p = ctypes.POINTER(ctypes.c_float)
        lib.grid_create.restype = ctypes.c_void_p
        lib.grid_create.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.POINTER(ctypes.c_int)]
        lib.grid_destroy.argtypes = [ctypes.c_void_p]
        lib.grid_recenter.argtypes = [ctypes.c_void_p, c_float_p]
        lib.grid_shift_origin.argtypes = [ctypes.c_void_p, c_float_p]
        lib.grid_clear.argtypes = [ctypes.c_void_p]
        lib.grid_insert_tile.restype = ctypes.c_int
        lib.grid_insert_tile.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), c_float_p, c_float_p, ctypes.c_int,
                                         c_float_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_ushort), ctypes.c_int,
                                         ctypes.c_float, ctypes.c_float, ctypes.c_float]
        lib.grid_occupied.restype = ctypes.c_int
        lib.grid_occupied.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int]
        lib.grid_state.restype = ctypes.c_int
        lib.grid_state.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        lib.grid_center.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
        lib.grid_sector_distances.argtypes = [ctypes.c_void_p, c_float_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                              ctypes.c_float, ctypes.c_float, ctypes.c_int, c_float_p]
        lib.world_create.restype = ctypes.c_void_p
        lib.world_create.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        lib.world_destroy.argtypes = [ctypes.c_void_p]
        lib.world_trees_near.restype = ctypes.c_int
        lib.world_trees_near.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, c_float_p,
                                         ctypes.c_int]
        lib.world_raycast.restype = ctypes.c_float
        lib.world_raycast.argtypes = [ctypes.c_void_p, c_float_p, c_float_p, ctypes.c_float]
        lib.world_nearest_trunk.restype = ctypes.c_float
        lib.world_nearest_trunk.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                            ctypes.c_float]
        lib.world_box_count.restype = ctypes.c_int
        lib.world_box_count.argtypes = [ctypes.c_void_p]
        lib.world_box.argtypes = [ctypes.c_void_p, ctypes.c_int, c_float_p]
        lib.world_nearest_box.restype = ctypes.c_float
        lib.world_nearest_box.argtypes = [ctypes.c_void_p, c_float_p]
        self.lib = lib


def floats(values):
    array = np.ascontiguousarray(values, dtype=np.float32)
    return array, array.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


class Grid:
    def __init__(self, core, size_xy, size_z, voxel_size, weights):
        self.lib = core.lib
        self.voxel_size = voxel_size
        self.size_xy = size_xy
        self.size_z = size_z
        w = (ctypes.c_int * 5)(*weights)
        self.handle = self.lib.grid_create(size_xy, size_z, voxel_size, w)

        if not self.handle:
            raise ValueError('invalid grid size %d x %d' % (size_xy, size_z))

        self._buffer = (ctypes.c_int * (3 * size_xy * size_xy * size_z))()

    def __del__(self):
        if getattr(self, 'handle', None):
            self.lib.grid_destroy(self.handle)

    def recenter(self, position):
        _, p = floats(position)
        self.lib.grid_recenter(self.handle, p)

    def shift_origin(self, delta):
        _, p = floats(delta)
        self.lib.grid_shift_origin(self.handle, p)

    def clear(self):
        self.lib.grid_clear(self.handle)

    def insert_tile(self, info, pose, first_zone, ranges, lsb, range_min, range_max):
        geometry_int = (ctypes.c_int * 5)(*info['geometry_int'])
        _, geometry_float = floats(info['geometry_float'])
        row_angle_array, row_angle = floats(info['row_angle'])
        pose_array, pose_p = floats(pose)
        ranges_array = np.ascontiguousarray(ranges, dtype=np.uint16)
        return self.lib.grid_insert_tile(self.handle, geometry_int, geometry_float, row_angle, len(info['row_angle']),
                                         pose_p, first_zone, ranges_array.ctypes.data_as(ctypes.POINTER(ctypes.c_ushort)),
                                         len(ranges_array), lsb, range_min, range_max)

    def occupied(self):
        count = self.lib.grid_occupied(self.handle, self._buffer, len(self._buffer) // 3)
        return np.frombuffer(self._buffer, dtype=np.int32, count=3 * count).reshape(-1, 3).copy()

    def state(self, north, east, down):
        return self.lib.grid_state(self.handle, north, east, down)

    def center(self):
        c = (ctypes.c_int * 3)()
        self.lib.grid_center(self.handle, c)
        return list(c)

    def sector_distances(self, position, yaw, footprint_radius, half_height, margin, max_range):
        _, p = floats(position)
        out, out_p = floats(np.zeros(BINS))
        self.lib.grid_sector_distances(self.handle, p, yaw, footprint_radius, half_height, margin, max_range, BINS, out_p)
        return out


class World:
    def __init__(self, core, world_type, seed, cell_size, density, clear_radius):
        self.lib = core.lib
        self.handle = self.lib.world_create(world_type, seed, cell_size, density, clear_radius)

    def __del__(self):
        if getattr(self, 'handle', None):
            self.lib.world_destroy(self.handle)

    def trees_near(self, north, east, radius, max_trees=256):
        out, out_p = floats(np.zeros(4 * max_trees))
        count = self.lib.world_trees_near(self.handle, north, east, radius, out_p, max_trees)
        return out[:4 * count].reshape(-1, 4)

    def raycast(self, origin, direction, max_range):
        _, o = floats(origin)
        _, d = floats(direction)
        return self.lib.world_raycast(self.handle, o, d, max_range)

    def nearest_trunk(self, north, east, down, search_radius):
        return self.lib.world_nearest_trunk(self.handle, north, east, down, search_radius)

    def boxes(self):
        """(min north, east, down, max north, east, down) per box"""
        result = []

        for i in range(self.lib.world_box_count(self.handle)):
            out, out_p = floats(np.zeros(6))
            self.lib.world_box(self.handle, i, out_p)
            result.append(out.tolist())

        return result

    def nearest_box(self, point):
        _, p = floats(point)
        return self.lib.world_nearest_box(self.handle, p)


def quaternion_to_dcm(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_multiply(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def yaw_of(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Log:
    """The topics and parameters a replay needs"""

    def __init__(self, path):
        topics = ['range_image', 'range_image_info', 'vehicle_local_position', 'vehicle_attitude',
                  'vehicle_local_position_groundtruth', 'trajectory_setpoint', 'obstacle_distance',
                  'obstacle_map_status', 'collision_constraints']
        ulog = ULog(path, topics)
        self.data = {}

        for dataset in ulog.data_list:
            if dataset.multi_id == 0:
                self.data[dataset.name] = dataset.data

        self.params = ulog.initial_parameters
        self.start = ulog.start_timestamp

        for name in ['range_image', 'range_image_info', 'vehicle_local_position', 'vehicle_attitude']:
            if name not in self.data:
                sys.exit('%s is not in the log: log with SDLOG_PROFILE bit 7 and the obstacle map running' % name)

    def param(self, name, default=None):
        return self.params.get(name, DEFAULT_PARAMS.get(name, default))

    def column(self, topic, name):
        return self.data[topic][name]

    def array(self, topic, name, count):
        return np.stack([self.data[topic]['%s[%d]' % (name, i)] for i in range(count)], axis=1)


class PoseHistory:
    """
    Estimated pose as ObstacleMap::positionAt and attitudeAt see it while handling a tile: only
    samples published before the tile, none from before the last local frame reset, newest
    position extrapolated with its velocity for at most 50 ms.
    """

    MAX_EXTRAPOLATION_US = 50000

    def __init__(self, log):
        lpos = log.data['vehicle_local_position']
        self.p_published = lpos['timestamp'].astype(np.int64)
        self.p_time = lpos['timestamp_sample'].astype(np.int64)
        self.position = np.stack([lpos['x'], lpos['y'], lpos['z']], axis=1)
        self.velocity = np.stack([lpos['vx'], lpos['vy'], lpos['vz']], axis=1)
        self.valid = (lpos['xy_valid'] > 0) & (lpos['z_valid'] > 0)
        counters = np.stack([lpos['xy_reset_counter'], lpos['z_reset_counter'], lpos['heading_reset_counter']], axis=1)
        changed = np.any(np.diff(counters, axis=0) != 0, axis=1)
        heading_changed = np.diff(counters[:, 2]) != 0
        # index of the first sample after the latest reset, for every sample
        self.p_segment = np.maximum.accumulate(np.where(np.concatenate([[True], changed]), np.arange(len(changed) + 1), 0))
        self.heading_reset_times = self.p_published[1:][heading_changed]

        att = log.data['vehicle_attitude']
        self.q_published = att['timestamp'].astype(np.int64)
        self.q_time = att['timestamp_sample'].astype(np.int64)
        self.q = log.array('vehicle_attitude', 'q', 4)

    def latest(self, now):
        """Index of the newest local position published by now, None before the first"""
        i = np.searchsorted(self.p_published, now, side='right') - 1
        return i if i >= 0 else None

    def position_at(self, t, now):
        newest = self.latest(now)

        if newest is None:
            return None

        start = self.p_segment[newest]
        index = np.arange(start, newest + 1)[self.valid[start:newest + 1]]

        if len(index) == 0:
            return None

        times = self.p_time[index]

        if t >= times[-1]:
            dt = min(t - times[-1], self.MAX_EXTRAPOLATION_US) * 1e-6
            velocity = self.velocity[index[-1]]
            return self.position[index[-1]] + (velocity * dt if np.all(np.isfinite(velocity)) else 0.0)

        k = np.searchsorted(times, t, side='right')

        if k == 0:
            return self.position[index[0]]

        t0, t1 = times[k - 1], times[k]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        return self.position[index[k - 1]] + (self.position[index[k]] - self.position[index[k - 1]]) * a

    def attitude_at(self, t, now):
        end = np.searchsorted(self.q_published, now, side='right')
        resets = self.heading_reset_times[self.heading_reset_times <= now]
        begin = np.searchsorted(self.q_published, resets[-1], side='left') if len(resets) else 0

        if end <= begin:
            return None

        times = self.q_time[begin:end]
        i = np.searchsorted(times, t, side='right')

        if i >= len(times):
            return self.q[end - 1]

        if i == 0:
            return self.q[begin]

        t0, t1 = times[i - 1], times[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        q0, q1 = self.q[begin + i - 1], self.q[begin + i]

        if np.dot(q0, q1) < 0:
            q1 = -q1

        q = q0 * (1 - a) + q1 * a
        return q / np.linalg.norm(q)


def sensor_infos(log):
    """range_image_info messages keyed by (device_id, config_id)"""
    infos = {}
    data = log.data['range_image_info']

    for k in range(len(data['timestamp'])):
        key = (int(data['device_id'][k]), int(data['config_id'][k]))
        row_count = int(data['row_angle_count'][k])
        infos[key] = {
            'geometry_int': [int(data[name][k]) for name in ['num_rows', 'num_cols', 'projection', 'range_type', 'zone_order']],
            'geometry_float': [float(data[name][k]) for name in ['x_start', 'x_step', 'y_start', 'y_step']],
            'row_angle': [float(data['row_angle[%d]' % i][k]) for i in range(row_count)],
            'q_body_sensor': np.array([data['q_body_sensor[%d]' % i][k] for i in range(4)]),
            'position_body': np.array([data['position_body[%d]' % i][k] for i in range(3)]),
            'lsb': data['range_lsb_mm'][k] * 1e-3,
            'range_min': float(data['range_min'][k]),
            'range_max': float(data['range_max'][k]),
        }

    return infos


def replay(log, core, keyframe_interval_us):
    """Insert every tile into a fresh map; return occupied voxel snapshots and the map geometry."""
    status = log.data.get('obstacle_map_status')
    size_xy, size_z = DEFAULT_GRID

    if status is not None:
        size_xy, size_z = int(status['grid_size_xy'][0]), int(status['grid_size_z'][0])

    voxel_size = float(log.param('OMAP_VOX_SIZE'))
    weights = [int(log.param(name)) for name in ['OMAP_HIT', 'OMAP_MISS', 'OMAP_OCC_THR', 'OMAP_FREE_THR', 'OMAP_LO_MIN']]
    grid = Grid(core, size_xy, size_z, voxel_size, weights)
    poses = PoseHistory(log)
    infos = sensor_infos(log)

    tiles = log.data['range_image']
    lpos = log.data['vehicle_local_position']
    ranges = log.array('range_image', 'ranges', 200)
    order = np.argsort(tiles['timestamp'], kind='stable')

    snapshots = []
    next_snapshot = 0
    reset_index = 0
    counters = None

    for k in order:
        t = int(tiles['timestamp'][k])

        # the module recentres on, and applies the resets of, the local position it has seen by now
        while reset_index < len(lpos['timestamp']) and lpos['timestamp'][reset_index] <= t:
            current = (lpos['xy_reset_counter'][reset_index], lpos['z_reset_counter'][reset_index],
                       lpos['heading_reset_counter'][reset_index])

            if counters is not None and current != counters:
                if current[2] != counters[2]:
                    grid.clear()

                else:
                    delta = [0.0, 0.0, 0.0]

                    if current[0] != counters[0]:
                        delta[0:2] = [lpos['delta_xy[0]'][reset_index], lpos['delta_xy[1]'][reset_index]]

                    if current[1] != counters[1]:
                        delta[2] = lpos['delta_z'][reset_index]

                    grid.shift_origin(delta)

            counters = current
            reset_index += 1

        key = (int(tiles['device_id'][k]), int(tiles['config_id'][k]))

        newest = poses.latest(t)

        # the module drops tiles it cannot describe or place, as here
        if key not in infos or newest is None or not poses.valid[newest]:
            continue

        info = infos[key]
        grid.recenter(poses.position[newest])

        t_sample = int(tiles['timestamp_sample'][k])
        q = poses.attitude_at(t_sample, t)
        position = poses.position_at(t_sample, t)

        if q is None or position is None:
            continue

        q_body_sensor = info['q_body_sensor'] if np.linalg.norm(info['q_body_sensor']) > 1e-6 else np.array([1., 0, 0, 0])
        rotation = quaternion_to_dcm(quaternion_multiply(q, q_body_sensor / np.linalg.norm(q_body_sensor)))
        origin = position + quaternion_to_dcm(q) @ info['position_body']
        pose = np.concatenate([origin, rotation.reshape(-1)])
        count = min(int(tiles['num_ranges'][k]), 200)
        grid.insert_tile(info, pose, int(tiles['first_zone'][k]), ranges[k, :count], info['lsb'], info['range_min'],
                         info['range_max'])

        if t >= next_snapshot:
            snapshots.append((t, grid.occupied()))
            next_snapshot = t + keyframe_interval_us

    return grid, snapshots


def world_from_log(log, core):
    world_type = int(log.param('SIH_WLD_TYPE', 0))

    if world_type == 0:
        return None

    return World(core, world_type, int(log.param('SIH_WLD_SEED', 0)), float(log.param('SIH_WLD_SPACING', 5.0)),
                 float(log.param('SIH_WLD_DENSITY', 0.5)), float(log.param('SIH_WLD_CLEAR', 4.0)))


def resample(times, values, period_us):
    """Every period_us from the first to the last sample, nearest earlier sample"""
    if len(times) == 0:
        return np.zeros(0, dtype=np.int64), values[:0]

    grid_times = np.arange(times[0], times[-1] + 1, period_us, dtype=np.int64)
    index = np.clip(np.searchsorted(times, grid_times, side='right') - 1, 0, len(times) - 1)
    return grid_times, values[index]


def scene(log, core, keyframe_interval_us, track_period_us):
    grid, snapshots = replay(log, core, keyframe_interval_us)
    start = int(min(snapshots[0][0], log.data['vehicle_local_position']['timestamp'][0])) if snapshots else int(
        log.data['vehicle_local_position']['timestamp'][0])

    def seconds(t):
        return np.round((np.asarray(t, dtype=np.int64) - start) * 1e-6, 3)

    # occupied voxels as additions and removals between keyframes, so the page stays small
    frames = []
    previous = set()

    for t, voxels in snapshots:
        current = set(map(tuple, voxels.tolist()))
        frames.append({'t': float(seconds(t)), 'add': [list(v) for v in current - previous],
                       'del': [list(v) for v in previous - current]})
        previous = current

    lpos = log.data['vehicle_local_position']
    track_t, track = resample(lpos['timestamp'], np.stack([lpos['x'], lpos['y'], lpos['z']], axis=1), track_period_us)
    att = log.data['vehicle_attitude']
    _, quats = resample(att['timestamp'], log.array('vehicle_attitude', 'q', 4), track_period_us)
    att_idx = np.clip(np.searchsorted(att['timestamp'], track_t, side='right') - 1, 0, len(att['timestamp']) - 1)
    yaws = np.array([yaw_of(q) for q in log.array('vehicle_attitude', 'q', 4)[att_idx]])

    result = {
        'voxel_size': grid.voxel_size,
        'track': {'t': seconds(track_t).tolist(), 'p': np.round(track, 3).tolist(), 'yaw': np.round(yaws, 3).tolist()},
        'frames': frames,
        'cp_dist': float(log.param('CP_DIST', -1.0)),
        'band': float(log.param('OMAP_VEH_HGT')),
    }

    truth = log.data.get('vehicle_local_position_groundtruth')

    if truth is not None:
        t, p = resample(truth['timestamp'], np.stack([truth['x'], truth['y'], truth['z']], axis=1), track_period_us)
        result['truth'] = {'t': seconds(t).tolist(), 'p': np.round(p, 3).tolist()}

    setpoint = log.data.get('trajectory_setpoint')

    if setpoint is not None:
        t, p = resample(setpoint['timestamp'], log.array('trajectory_setpoint', 'position', 3), track_period_us)
        _, v = resample(setpoint['timestamp'], log.array('trajectory_setpoint', 'velocity', 3), track_period_us)
        result['setpoint'] = {'t': seconds(t).tolist(), 'p': np.where(np.isfinite(p), np.round(p, 3), None).tolist(),
                              'v': np.where(np.isfinite(v), np.round(v, 3), None).tolist()}

    constraints = log.data.get('collision_constraints')

    if constraints is not None:
        t, original = resample(constraints['timestamp'], log.array('collision_constraints', 'original_setpoint', 2),
                               track_period_us)
        _, adapted = resample(constraints['timestamp'], log.array('collision_constraints', 'adapted_setpoint', 2),
                              track_period_us)
        result['constraints'] = {'t': seconds(t).tolist(), 'original': np.round(original, 2).tolist(),
                                 'adapted': np.round(adapted, 2).tolist()}

    obstacle = log.data.get('obstacle_distance')

    if obstacle is not None:
        t, d = resample(obstacle['timestamp'], log.array('obstacle_distance', 'distances', BINS), track_period_us)
        result['obstacle_distance'] = {'t': seconds(t).tolist(), 'max': int(obstacle['max_distance'][-1]),
                                       'd': d.astype(int).tolist()}

    world = world_from_log(log, core)

    if world is not None:
        # the trunks along the flight, gathered around points a few metres apart
        found = {}

        for x, y in track[::max(1, len(track) // 400), :2]:
            for tree in world.trees_near(float(x), float(y), TREE_SEARCH_RADIUS):
                found[(round(float(tree[0]), 3), round(float(tree[1]), 3))] = tree

        result['trees'] = np.round(np.array(list(found.values())).reshape(-1, 4), 3).tolist()
        result['boxes'] = np.round(np.array(world.boxes()).reshape(-1, 6), 3).tolist()

        if truth is not None:
            clearance = [min(world.nearest_trunk(float(x), float(y), float(z), 20.0), world.nearest_box([x, y, z]))
                         for x, y, z in result['truth']['p']]
            result['truth']['clearance'] = [round(c, 3) if np.isfinite(c) else None for c in clearance]

    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('ulog', help='log with range_image, range_image_info and the vehicle pose')
    parser.add_argument('-o', '--output', help='HTML file to write, default next to the log')
    parser.add_argument('--keyframe', type=float, default=0.1, help='map snapshot interval [s], default 0.1')
    parser.add_argument('--track', type=float, default=0.05, help='path and setpoint sample interval [s], default 0.05')
    parser.add_argument('--json', action='store_true', help='also write the scene as JSON')
    args = parser.parse_args()

    log = Log(args.ulog)
    core = Core()
    data = scene(log, core, int(args.keyframe * 1e6), int(args.track * 1e6))
    data['title'] = os.path.basename(args.ulog)

    output = args.output or os.path.splitext(args.ulog)[0] + '_obstacle_map.html'

    with open(os.path.join(os.path.dirname(__file__), 'replay_viewer.html')) as f:
        page = f.read()

    with open(output, 'w') as f:
        # the template is a fragment, so it can also be published where a host page wraps it
        f.write('<!doctype html>\n' + page.replace('/*SCENE_JSON*/null', json.dumps(data, separators=(',', ':'))))

    if args.json:
        with open(os.path.splitext(output)[0] + '.json', 'w') as f:
            json.dump(data, f)

    voxels = sum(len(frame['add']) for frame in data['frames'])
    print('%s: %.0f s, %d map keyframes, %d voxel additions, %d trunks' % (
        output, data['track']['t'][-1] if data['track']['t'] else 0, len(data['frames']), voxels, len(data.get('trees', []))))


if __name__ == '__main__':
    main()
