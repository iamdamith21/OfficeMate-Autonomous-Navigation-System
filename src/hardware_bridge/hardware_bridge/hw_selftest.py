#!/usr/bin/env python3
"""
hw_selftest.py — bench test for every component in wiring_diagram_v3.

Talks straight to the Mega over USB serial (no ROS needed), exercises each
peripheral, and prints a PASS/FAIL report. Run this after wiring the new
chassis and before committing to the custom PCB layout.

Usage (robot on a bench, firmware v3 flashed):
  python3 hw_selftest.py [port] [--motors] [--rfid] [--no-doors]

  port       serial device, default /dev/arduino
  --motors   pulse all four motors  ⚠ WHEELS MUST BE OFF THE GROUND
  --rfid     wait 15 s for you to scan a tag
  --no-doors skip the servo door open/close cycle

What it checks (from the firmware's boot banner + live streams):
  MPU6050   present, ~50 Hz, gravity ≈ 9.8 on Z, gyro ≈ 0 at rest
  HC-SR04   ~15 Hz, plausible range (or clear no-echo behaviour)
  INA219    ~1 Hz, pack voltage in the 3S window, current plausible
  FC-51 IR  reporting a state (cover/uncover it and rerun to see it flip)
  LCD       gets a "SELFTEST" line — confirm visually
  MFRC522   detected at boot (+ live tag scan with --rfid)
  SG90 ×2   full open/close cycle with firmware acks
  Motors    fwd / rev / spin pulses with --motors — watch the wheels
"""
import argparse
import sys
import time
from collections import defaultdict

import serial

BAUD = 115200


class Tester:
    def __init__(self, port):
        self.results = {}   # name -> (verdict, detail)
        self.ser = serial.Serial(port, BAUD, timeout=0.2)

    def record(self, name, ok, detail):
        self.results[name] = ('PASS' if ok else 'FAIL', detail)

    def manual(self, name, detail):
        self.results[name] = ('CHECK', detail)

    def skip(self, name, detail):
        self.results[name] = ('SKIP', detail)

    def send(self, line):
        self.ser.write((line + '\n').encode('ascii'))

    def read_lines(self, duration_s):
        end = time.monotonic() + duration_s
        out = []
        while time.monotonic() < end:
            raw = self.ser.readline()
            if raw:
                out.append(raw.decode('ascii', errors='replace').strip())
        return out

    # ── phases ──────────────────────────────────────────────────────────────
    def boot(self):
        print('>> Resetting board (DTR) and waiting for boot banner ...')
        self.ser.dtr = False
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.dtr = True
        lines = []
        end = time.monotonic() + 15
        ready = None
        while time.monotonic() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='replace').strip()
            lines.append(line)
            if line.startswith('S,READY'):
                ready = line
                break
        errors = [l for l in lines if l.startswith('E,')]
        self.record('serial link + firmware',
                    ready is not None,
                    ready or 'no S,READY within 15 s — right firmware/port?')
        for dev, tag in [('MPU6050', 'MPU6050'), ('INA219', 'INA219'),
                         ('LCD', 'LCD'), ('MFRC522 boot', 'RFID')]:
            missing = any(tag in e for e in errors)
            if missing:
                self.record(dev + ' detected', False,
                            next(e for e in errors if tag in e))
        return ready is not None

    def passive_streams(self, seconds=6):
        print(f'>> Sampling sensor streams for {seconds} s (keep robot still) ...')
        lines = self.read_lines(seconds)
        by = defaultdict(list)
        for l in lines:
            by[l[:1]].append(l)

        # IMU
        n = len(by['I'])
        hz = n / seconds
        if n:
            try:
                last = by['I'][-1].split(',')[1:]
                ax, ay, az, gx, gy, gz = (float(x) for x in last)
                grav_ok = abs(az - 9.81) < 1.5 and abs(ax) < 1.5 and abs(ay) < 1.5
                gyro_ok = max(abs(gx), abs(gy), abs(gz)) < 0.1
                self.record('MPU6050 rate', 40 <= hz <= 60, f'{hz:.0f} Hz')
                self.record('MPU6050 gravity on Z', grav_ok,
                            f'a=({ax:.2f},{ay:.2f},{az:.2f}) m/s²'
                            + ('' if grav_ok else ' — module tilted/mounted wrong axis?'))
                self.record('MPU6050 gyro at rest', gyro_ok,
                            f'g=({gx:.3f},{gy:.3f},{gz:.3f}) rad/s')
            except ValueError:
                self.record('MPU6050 rate', False, f'unparseable: {by["I"][-1]!r}')
        elif 'MPU6050 detected' not in self.results:
            self.record('MPU6050 rate', False, 'no I lines received')

        # Sonar
        n = len(by['U'])
        hz = n / seconds
        if n:
            vals = [float(l.split(',')[1]) for l in by['U'][-10:]]
            ok_vals = all(v == -1.0 or 0.02 <= v <= 2.0 for v in vals)
            self.record('HC-SR04 rate', 10 <= hz <= 20, f'{hz:.0f} Hz')
            self.record('HC-SR04 range sane', ok_vals,
                        f'last readings: {vals[-3:]} m (-1 = no echo)')
        else:
            self.record('HC-SR04', False,
                        'no U lines — TRIG/ECHO on D30/D31? VCC on 5V rail?')

        # Battery
        if by['B']:
            v, a = (float(x) for x in by['B'][-1].split(',')[1:])
            self.record('INA219 pack voltage', 8.0 <= v <= 13.0,
                        f'{v:.2f} V ({"3S window OK" if 9.0 <= v <= 12.6 else "outside 9.0-12.6"})')
            self.record('INA219 current sane', -8.0 <= a <= 8.0, f'{a:.2f} A idle')
        elif 'INA219 detected' not in self.results:
            self.record('INA219 stream', False, 'no B lines received')

        # IR
        if by['D']:
            state = by['D'][-1].split(',')[1]
            self.manual('FC-51 IR',
                        f'reports {"OCCUPIED" if state == "1" else "EMPTY"} — '
                        'wave a paper inside the compartment and rerun to see it flip')
        else:
            self.record('FC-51 IR', False, 'no D lines — OUT on D32?')

    def lcd(self):
        stamp = time.strftime('%H:%M:%S')
        self.send('L,SELFTEST ' + stamp)
        self.manual('LCD 20x4', f'line 2 should now read "SELFTEST {stamp}" '
                    '(line 1 = battery)')

    def doors(self):
        print('>> Cycling doors: OPEN ...')
        self.send('O')
        acks = [l for l in self.read_lines(5) if l.startswith('A,DOORS')]
        opened = any(l.endswith('OPEN') for l in acks)
        time.sleep(1)
        print('>> Cycling doors: CLOSE ...')
        self.send('C')
        acks = [l for l in self.read_lines(5) if l.startswith('A,DOORS')]
        closed = any(l.endswith('CLOSED') for l in acks)
        self.record('SG90 doors open', opened, 'ack received' if opened
                    else 'no OPEN ack — servo power? signal on D11/D12?')
        self.record('SG90 doors close', closed, 'ack received' if closed
                    else 'no CLOSED ack')
        if opened and closed:
            self.manual('SG90 doors travel',
                        'confirm both doors physically swung fully open and shut '
                        '(adjust DOOR_*_OPEN/CLOSED angles in firmware if not)')

    def rfid(self):
        print('>> SCAN AN RFID TAG NOW (15 s) ...')
        tags = [l for l in self.read_lines(15) if l.startswith('R,')]
        self.record('MFRC522 tag scan', bool(tags),
                    f'UID {tags[0][2:]}' if tags
                    else 'no tag seen — hold the tag against the antenna')

    def motors(self):
        print('>> MOTOR TEST: forward / reverse / spin — wheels OFF the ground!')
        for name, v, w in [('forward', 0.08, 0.0),
                           ('reverse', -0.08, 0.0),
                           ('spin-left', 0.0, 0.8)]:
            print(f'   pulsing {name} for 1.5 s ...')
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                self.send(f'V,{v:.3f},{w:.3f}')
                time.sleep(0.05)
            self.send('V,0.000,0.000')
            time.sleep(0.8)
        self.manual('4× drive motors',
                    'confirm: forward = all 4 wheels forward; reverse = all '
                    'backward; spin-left = right side fwd + left side back. '
                    'Any wheel wrong way → swap its two wires at the L298N.')

    # ── report ──────────────────────────────────────────────────────────────
    def report(self):
        print('\n' + '═' * 66)
        print(' COMPONENT SELF-TEST REPORT')
        print('═' * 66)
        icon = {'PASS': '✅ PASS ', 'FAIL': '❌ FAIL ',
                'CHECK': '👁  CHECK', 'SKIP': '⏭  SKIP '}
        fails = 0
        for name, (verdict, detail) in self.results.items():
            fails += verdict == 'FAIL'
            print(f' {icon[verdict]}  {name:<26} {detail}')
        print('═' * 66)
        print(f' {fails} failure(s). '
              f'{"Fix wiring before the PCB design." if fails else "Wiring validated — good to base the PCB on."}')
        return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?', default='/dev/arduino')
    ap.add_argument('--motors', action='store_true',
                    help='pulse motors (wheels off the ground!)')
    ap.add_argument('--rfid', action='store_true',
                    help='wait for a live tag scan')
    ap.add_argument('--no-doors', action='store_true')
    args = ap.parse_args()

    t = Tester(args.port)
    if t.boot():
        t.passive_streams()
        t.lcd()
        if args.no_doors:
            t.skip('SG90 doors', '--no-doors')
        else:
            t.doors()
        if args.rfid:
            t.rfid()
        else:
            t.skip('MFRC522 tag scan', 'run with --rfid and a tag in hand')
        if args.motors:
            t.motors()
        else:
            t.skip('4× drive motors', 'run with --motors, wheels off the ground')
    sys.exit(1 if t.report() else 0)


if __name__ == '__main__':
    main()
