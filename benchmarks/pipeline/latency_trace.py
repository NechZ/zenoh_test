#!/usr/bin/env python3
"""Break the ROS pipeline latency into stages.

Needs the instrumented drivers (patches/latency-trace.patch) and BENCH_TRACE=1, e.g.:

    BENCH_TRACE=1 ros2 launch sensor_benchmark benchmark_drivers.launch.py record:=false 2>&1 | tee /tmp/trace.log
    benchmarks/pipeline/latency_trace.py /tmp/trace.log

Cloud stages (per scan):
  stamp_offset  actual first packet at the cloud node minus the message stamp (positive = stamp is earlier)
  hop           pcap node publishes the first packet -> cloud node receives it
  assembly      first packet -> scan complete (sensor sweep, ~110 ms at 9 Hz; unavoidable)
  queue         scan complete -> processing thread picks it up
  process       build the point cloud(s)
  publish       publish() calls
  consume       published -> in-process consumer callback
Camera stages (per frame):
  wait          stamp (taken before the blocking grab) -> frame retrieved from the camera
  copy          pixels copied into the ROS message
  rest          after the copy -> published
  consume       published -> in-process consumer callback
All times in milliseconds, medians and 95th percentiles.
"""
import re
import statistics
import sys

LINE = re.compile(r'TRACE (\w+) (.*)$')


def parse(path):
    recs = {'ouster': [], 'pcap': [], 'cam': [], 'consumer': []}
    for line in open(path, errors='replace'):
        m = LINE.search(line)
        if not m:
            continue
        kind, rest = m.groups()
        kv = dict(p.split('=', 1) for p in rest.split() if '=' in p)
        if kind in recs:
            recs[kind].append({k: (int(v) if v.lstrip('-').isdigit() else v) for k, v in kv.items()})
    return recs


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * p))]


def row(name, vals, note=''):
    if not vals:
        print('  %-14s (no data)' % name)
        return
    print('  %-14s median %7.2f   p95 %7.2f   max %7.2f ms  %s' % (name, statistics.median(vals), pct(vals, 0.95), max(vals), note))


def cloud(recs):
    cons = {r['stamp']: r['arrive'] for r in recs['consumer'] if r.get('what') == 'cloud'}
    pcap = sorted(r['first_pkt_pub'] for r in recs['pcap'])
    cols = {k: [] for k in ('stamp_offset', 'hop', 'assembly', 'queue', 'process', 'publish', 'consume',
                            'since_complete', 'since_first_pkt', 'since_stamp')}
    import bisect
    for r in recs['ouster']:
        if not (r['first_pkt'] and r['built'] and r['published']):
            continue
        arrive = cons.get(r['stamp'])
        if arrive is None:
            continue
        i = bisect.bisect_right(pcap, r['first_pkt']) - 1
        if i >= 0 and 0 <= r['first_pkt'] - pcap[i] < 300e6:
            cols['hop'].append((r['first_pkt'] - pcap[i]) / 1e6)
        cols['stamp_offset'].append((r['first_pkt'] - r['stamp']) / 1e6)
        cols['assembly'].append((r['complete'] - r['first_pkt']) / 1e6)
        cols['queue'].append((r['dequeue'] - r['complete']) / 1e6)
        cols['process'].append((r['built'] - r['dequeue']) / 1e6)
        cols['publish'].append((r['published'] - r['built']) / 1e6)
        cols['consume'].append((arrive - r['published']) / 1e6)
        cols['since_complete'].append((arrive - r['complete']) / 1e6)
        cols['since_first_pkt'].append((arrive - r['first_pkt']) / 1e6)
        cols['since_stamp'].append((arrive - r['stamp']) / 1e6)
    print('CLOUD  (%d scans matched)' % len(cols['assembly']))
    for k in ('stamp_offset', 'hop', 'assembly', 'queue', 'process', 'publish', 'consume'):
        row(k, cols[k])
    print('  -- totals')
    row('since_stamp', cols['since_stamp'], '<- what the benchmark probes reported')
    row('since_first_pkt', cols['since_first_pkt'], '<- true scan-start to consumer')
    row('since_complete', cols['since_complete'], '<- what a consumer waits after the scan is done')


def cameras(recs):
    for name, what in (('my_camera', 'cam1'), ('my_camera_2', 'cam2')):
        cons = {r['stamp']: r['arrive'] for r in recs['consumer'] if r.get('what') == what}
        cols = {k: [] for k in ('wait', 'copy', 'rest', 'consume', 'since_stamp', 'since_retrieved')}
        for r in (x for x in recs['cam'] if x.get('name') == name):
            arrive = cons.get(r['stamp'])
            if arrive is None or not r['retrieved']:
                continue
            cols['wait'].append((r['retrieved'] - r['stamp']) / 1e6)
            cols['copy'].append((r['copied'] - r['retrieved']) / 1e6)
            cols['rest'].append((r['published'] - r['copied']) / 1e6)
            cols['consume'].append((arrive - r['published']) / 1e6)
            cols['since_stamp'].append((arrive - r['stamp']) / 1e6)
            cols['since_retrieved'].append((arrive - r['retrieved']) / 1e6)
        print('CAMERA %s  (%d frames matched)' % (name, len(cols['wait'])))
        for k in ('wait', 'copy', 'rest', 'consume'):
            row(k, cols[k])
        print('  -- totals')
        row('since_stamp', cols['since_stamp'], '<- what the benchmark probes reported')
        row('since_retrieved', cols['since_retrieved'], '<- true frame-available to consumer')


if __name__ == '__main__':
    recs = parse(sys.argv[1])
    print('parsed: %s' % {k: len(v) for k, v in recs.items()})
    cloud(recs)
    cameras(recs)
