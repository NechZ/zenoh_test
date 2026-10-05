#!/usr/bin/env python3
"""ROS-side counterpart of zconsumer: same metrics, printed in the same format.

Usage: probe_ros.py <secs> [cloud|all]
  cloud: only /ouster/points   all: also both camera image topics
Latency = callback entry time - header.stamp (both are system-clock based).
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class Probe(Node):
    def __init__(self, with_cams):
        super().__init__('probe_ros')
        self.s = {}
        self.create_subscription(PointCloud2, '/ouster/points', lambda m: self.cloud('cloud', m), qos_profile_sensor_data)
        if with_cams:
            for i, t in enumerate(('/my_camera/image_raw', '/my_camera_2/image_raw')):
                self.create_subscription(Image, t, lambda m, i=i: self.image('cam%d' % i, m), qos_profile_sensor_data)

    def rec(self, key, m, arrival, cols=None):
        st = self.s.setdefault(key, {'stamp': [], 'lat': [], 'cols': []})
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        st['stamp'].append(stamp)
        st['lat'].append((arrival - stamp) * 1000.0)
        if cols is not None:
            st['cols'].append(cols)

    def image(self, key, m):
        self.rec(key, m, time.time())

    def cloud(self, key, m):
        arrival = time.time()
        a = pc2.read_points(m, field_names=['x', 'y'], skip_nans=False)
        x = np.asarray(a['x']).reshape(m.height, m.width)
        y = np.asarray(a['y']).reshape(m.height, m.width)
        fin = np.isfinite(x) & ((x != 0) | (y != 0))
        self.rec(key, m, arrival, fin.any(axis=0).mean())


def main():
    secs = float(sys.argv[1])
    rclpy.init()
    p = Probe(len(sys.argv) > 2 and sys.argv[2] == 'all')
    end = time.time() + secs
    t0 = time.time()
    while time.time() < end:
        rclpy.spin_once(p, timeout_sec=0.05)
    el = time.time() - t0
    for key, st in sorted(p.s.items()):
        t = np.array(st['stamp'])
        d = np.diff(t) * 1000.0
        lat = np.array(st['lat'])
        line = ('CONSUMER %-12s n=%d (%.1f Hz) | stamp gaps: <50ms=%d, 50-150ms=%d, >=150ms=%d | latency ms p50=%.1f p99=%.1f'
                % (key, len(t), (len(t) - 1) / max(t[-1] - t[0], 1e-9), (d < 50).sum(), ((d >= 50) & (d < 150)).sum(), (d >= 150).sum(),
                   np.percentile(lat, 50), np.percentile(lat, 99)))
        if st['cols']:
            c = np.array(st['cols'])
            line += ' | cols_present<99.5%%: %d/%d (min %.3f)' % ((c < 0.995).sum(), len(c), c.min())
        print(line)


if __name__ == '__main__':
    main()
