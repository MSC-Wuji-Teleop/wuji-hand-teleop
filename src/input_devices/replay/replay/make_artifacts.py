#!/usr/bin/env python3
"""make_artifacts: post-run evidence from a run directory (spec_1, 10/11).

Reads the run's mcap bag and events.jsonl and writes, into the same run
directory:

    command_vs_actual.npz   commanded vs measured series per device
    tracking_summary.json   per-joint RMSE, max error, lag, pass/fail
    fault_log.jsonl         the filtered (warn/error/fault) view of events

Proposed pass criteria (UNRATIFIED, spec_1 logging section): zero faults,
arm RMSE <= 0.15 rad and max error <= 0.35 rad, hand RMSE <= 0.15 rad, all
comm ages inside watchdog bounds for the full clip. The numbers live in
THRESHOLDS below until ratified.

Bag reading needs rosbag2_py (container). The computation functions are
pure numpy and unit-tested in the venv.

Usage:
    make_artifacts --run-dir ~/wuji_runs/<run>/
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

THRESHOLDS = {
    'arm_rmse_rad': 0.15,
    'arm_max_error_rad': 0.35,
    'hand_rmse_rad': 0.15,
}

PAIRS = [
    # (label, commanded topic, measured topic)
    ('left_arm', '/left_arm/joint_commands', '/left_arm/joint_states'),
    ('right_arm', '/right_arm/joint_commands', '/right_arm/joint_states'),
    ('left_hand', '/left_hand/joint_commands', '/left_hand/joint_states'),
    ('right_hand', '/right_hand/joint_commands', '/right_hand/joint_states'),
]


# ------------------------------------------------------- pure computation

def tracking_stats(cmd_t: np.ndarray, cmd_q: np.ndarray,
                   meas_t: np.ndarray, meas_q: np.ndarray) -> dict:
    """Per-joint RMSE / max error with measured interpolated onto the
    command timeline, plus a lag estimate (positive = measured lags)."""
    if cmd_t.size < 2 or meas_t.size < 2:
        return {'rmse': None, 'max_error': None, 'lag_s': None,
                'note': 'insufficient samples'}
    lo = max(cmd_t[0], meas_t[0])
    hi = min(cmd_t[-1], meas_t[-1])
    sel = (cmd_t >= lo) & (cmd_t <= hi)
    if sel.sum() < 2:
        return {'rmse': None, 'max_error': None, 'lag_s': None,
                'note': 'no timeline overlap'}
    t = cmd_t[sel]
    cmd = cmd_q[sel]
    meas = np.column_stack([
        np.interp(t, meas_t, meas_q[:, j]) for j in range(meas_q.shape[1])
    ])
    err = meas - cmd
    rmse = np.sqrt(np.mean(err ** 2, axis=0))
    max_err = np.max(np.abs(err), axis=0)

    # Lag: cross-correlate the mean-abs-velocity envelopes on a uniform
    # 100 Hz grid over +/- 0.5 s. Diagnostic, not a gate.
    grid = np.arange(lo, hi, 0.01)
    if grid.size >= 20:
        def envelope(ts, qs):
            v = np.gradient(np.column_stack([
                np.interp(grid, ts, qs[:, j]) for j in range(qs.shape[1])
            ]), 0.01, axis=0)
            e = np.mean(np.abs(v), axis=1)
            e = e - e.mean()
            return e

        a = envelope(cmd_t, cmd_q)
        b = envelope(meas_t, meas_q)
        max_shift = min(50, grid.size // 2)
        shifts = range(-max_shift, max_shift + 1)
        scores = [float(np.dot(a[max(0, -s):grid.size - max(0, s)],
                               b[max(0, s):grid.size - max(0, -s)]))
                  for s in shifts]
        lag_s = 0.01 * list(shifts)[int(np.argmax(scores))]
    else:
        lag_s = None

    return {
        'rmse': [float(x) for x in rmse],
        'max_error': [float(x) for x in max_err],
        'lag_s': lag_s,
    }


def summarize(series: dict, events: list, manifest: dict) -> dict:
    """Assemble tracking_summary.json from per-device series + events."""
    faults = [e for e in events if e.get('severity') == 'fault']
    per_device = {}
    worst = {'arm_rmse': 0.0, 'arm_max': 0.0, 'hand_rmse': 0.0}
    for label, data in series.items():
        stats = tracking_stats(data['cmd_t'], data['cmd_q'],
                               data['meas_t'], data['meas_q'])
        per_device[label] = {'joint_names': data.get('names'), **stats}
        if stats['rmse'] is not None:
            if 'arm' in label:
                worst['arm_rmse'] = max(worst['arm_rmse'], max(stats['rmse']))
                worst['arm_max'] = max(worst['arm_max'], max(stats['max_error']))
            else:
                worst['hand_rmse'] = max(worst['hand_rmse'], max(stats['rmse']))

    checks = {
        'zero_faults': len(faults) == 0,
        'arm_rmse_ok': worst['arm_rmse'] <= THRESHOLDS['arm_rmse_rad'],
        'arm_max_error_ok': worst['arm_max'] <= THRESHOLDS['arm_max_error_rad'],
        'hand_rmse_ok': worst['hand_rmse'] <= THRESHOLDS['hand_rmse_rad'],
    }
    return {
        'sample': manifest.get('sample'),
        'method': manifest.get('method'),
        'scope': manifest.get('scope'),
        'speed_scale': manifest.get('speed_scale'),
        'thresholds': {**THRESHOLDS, 'note': 'proposed, unratified'},
        'devices': per_device,
        'worst': {k: float(v) for k, v in worst.items()},
        'fault_count': len(faults),
        'checks': checks,
        'pass': all(checks.values()),
    }


def write_fault_log(events: list, out_path: Path) -> int:
    """fault_log.jsonl: the filtered view of events (section 10)."""
    kept = [e for e in events if e.get('severity') in ('warn', 'error', 'fault')]
    with open(out_path, 'w') as f:
        for e in kept:
            f.write(json.dumps(e, sort_keys=True) + '\n')
    return len(kept)


# ------------------------------------------------------------ bag reading

def extract_series(bag_dir: Path) -> dict:
    """Read the four command/measure topic pairs from the mcap bag.
    Container-only (rosbag2_py)."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import JointState

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                ConverterOptions('', ''))
    wanted = {}
    for label, cmd_topic, meas_topic in PAIRS:
        wanted[cmd_topic] = (label, 'cmd')
        wanted[meas_topic] = (label, 'meas')

    acc = {label: {'cmd': [], 'meas': [], 'names': None}
           for label, _, _ in PAIRS}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in wanted:
            continue
        label, kind = wanted[topic]
        msg = deserialize_message(raw, JointState)
        acc[label][kind].append((t_ns * 1e-9, list(msg.position)))
        if kind == 'cmd' and msg.name and acc[label]['names'] is None:
            acc[label]['names'] = list(msg.name)

    series = {}
    for label, data in acc.items():
        if not data['cmd'] or not data['meas']:
            continue
        cmd_t = np.array([t for t, _ in data['cmd']])
        cmd_q = np.array([q for _, q in data['cmd']])
        meas_t = np.array([t for t, _ in data['meas']])
        meas_q = np.array([q for _, q in data['meas']])
        if cmd_q.ndim != 2 or meas_q.ndim != 2 \
                or cmd_q.shape[1] != meas_q.shape[1]:
            continue
        series[label] = {'cmd_t': cmd_t, 'cmd_q': cmd_q,
                         'meas_t': meas_t, 'meas_q': meas_q,
                         'names': data['names']}
    return series


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args(argv)

    run_dir = args.run_dir.expanduser()
    manifest_path = run_dir / 'run_manifest.json'
    events_path = run_dir / 'events.jsonl'
    bag_dir = run_dir / 'bag'
    if not manifest_path.exists():
        print(f'{manifest_path} not found: not a run directory', file=sys.stderr)
        return 1
    manifest = json.loads(manifest_path.read_text())
    events = []
    if events_path.exists():
        for line in events_path.read_text().splitlines():
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue

    n = write_fault_log(events, run_dir / 'fault_log.jsonl')
    print(f'fault_log.jsonl: {n} entries')

    if not bag_dir.exists():
        print('no bag/ in this run dir; wrote fault_log only', file=sys.stderr)
        return 2
    series = extract_series(bag_dir)
    if not series:
        print('bag holds no command/measure pairs', file=sys.stderr)
        return 2

    npz_payload = {}
    for label, data in series.items():
        for key in ('cmd_t', 'cmd_q', 'meas_t', 'meas_q'):
            npz_payload[f'{label}_{key}'] = data[key]
    np.savez(run_dir / 'command_vs_actual.npz', **npz_payload)
    print(f'command_vs_actual.npz: {sorted(series)}')

    summary = summarize(series, events, manifest)
    (run_dir / 'tracking_summary.json').write_text(
        json.dumps(summary, indent=1, sort_keys=True) + '\n')
    print(f"tracking_summary.json: pass={summary['pass']} "
          f"(checks {summary['checks']})")
    return 0


if __name__ == '__main__':
    sys.exit(main())
