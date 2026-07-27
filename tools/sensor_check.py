#!/usr/bin/env python3
"""Read the Mega's serial stream and report what the new v10 sensors produce.

Prints the boot banners verbatim, then summarises the I, and U, lines: rates,
means and spreads. Enough to tell "the IMU is alive and sane" from "the IMU is
alive and reporting nonsense" without needing ROS running.
"""
import statistics
import sys
import time

import serial

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0

ser = serial.Serial('/dev/arduino', 115200, timeout=1.0)
print('--- opened, waiting for boot (gyro calibration takes ~2 s) ---')

imu, sonar, banners, errors = [], [], [], []
t0 = time.monotonic()
while time.monotonic() - t0 < DURATION:
    raw = ser.readline()
    if not raw:
        continue
    line = raw.decode('ascii', errors='replace').strip()
    if not line:
        continue
    if line.startswith('S,'):
        banners.append(line)
        print(f'  banner: {line}')
    elif line.startswith('E,'):
        errors.append(line)
    elif line.startswith('I,'):
        try:
            imu.append([float(x) for x in line[2:].split(',')])
        except ValueError:
            errors.append(f'malformed IMU: {line}')
    elif line.startswith('U,'):
        try:
            sonar.append(float(line[2:]))
        except ValueError:
            errors.append(f'malformed sonar: {line}')

elapsed = time.monotonic() - t0
ser.close()

print(f'\n--- {elapsed:.1f} s elapsed ---')

print(f'\nIMU: {len(imu)} samples  ({len(imu)/elapsed:.1f} Hz, expect ~50)')
if imu:
    names = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
    units = ['m/s2'] * 3 + ['rad/s'] * 3
    for i, (n, u) in enumerate(zip(names, units)):
        col = [s[i] for s in imu]
        mean = statistics.mean(col)
        sd = statistics.pstdev(col)
        print(f'  {n:3s} mean {mean:+8.4f}  sd {sd:7.4f}  {u}')
    az = statistics.mean([s[2] for s in imu])
    print(f'\n  gravity check: az = {az:+.3f} m/s2 '
          f'(want ~ +9.81 with the board level and +Z up)')
    gz_sd = statistics.pstdev([s[5] for s in imu])
    gz_mean = statistics.mean([s[5] for s in imu])
    print(f'  yaw-rate bias : gz = {gz_mean:+.5f} rad/s at rest '
          f'(want |gz| < 0.005 after calibration)')
    print(f'  yaw-rate noise: sd = {gz_sd:.5f} rad/s '
          f'(feeds the EKF covariance choice)')

print(f'\nSonar: {len(sonar)} samples  ({len(sonar)/elapsed:.1f} Hz, expect ~15)')
if sonar:
    good = [d for d in sonar if d > 0]
    noecho = len(sonar) - len(good)
    print(f'  valid {len(good)}, no-echo {noecho}')
    if good:
        print(f'  range  min {min(good):.3f} m  max {max(good):.3f} m  '
              f'median {statistics.median(good):.3f} m')

if errors:
    print(f'\nErrors/diagnostics ({len(errors)}):')
    for e in errors[:10]:
        print(f'  {e}')
