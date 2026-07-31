#!/usr/bin/env python3
"""Drive left_servo_probe over serial.

  servo_drive.py A            attach at 90 (no movement)
  servo_drive.py G175         eased move
  servo_drive.py A G175 G90   several in sequence

Waits out the bootloader before writing: this Mega stays in its bootloader if
written to immediately after the port opens, which looks like a dead sketch.
"""
import sys
import time

import serial

PORT = '/dev/ttyACM0'
BOOT_WAIT_S = 2.5

cmds = sys.argv[1:] or ['?']

# DTR must be dropped BEFORE the port opens. pyserial asserts DTR on open,
# which resets the Mega, re-runs setup() and wipes the sketch's belief about
# where the horn is. That matters for more than convenience: after a reset a
# bare 'A' would command 90 deg from wherever the horn physically sits, and the
# Servo library does that in one uneased jump at ~600 deg/s.
s = serial.Serial()
s.port = PORT
s.baudrate = 115200
s.timeout = 0.4
s.dtr = False
s.rts = False
s.open()
time.sleep(BOOT_WAIT_S)
s.reset_input_buffer()

# Ask for state first so we always see the sketch is alive.
def send(c):
    s.write((c + '\n').encode())
    s.flush()

def drain(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        line = s.readline()
        if not line:
            continue
        txt = line.decode('ascii', 'replace').strip()
        if txt:
            print('   ' + txt, flush=True)

send('?')
drain(1.0)

for c in cmds:
    print(f'>> {c}', flush=True)
    send(c)
    # A move of the full 85 deg band takes ~3.4 s plus the 400 ms release.
    drain(6.0 if c.startswith('G') else 1.5)

s.close()
