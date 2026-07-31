#!/usr/bin/env bash
# rfid_check.sh [seconds] — is the MFRC522 healthy, and can it read a tag?
#
# Run with the robot PARKED and the doors idle: rfidTick() deliberately refuses
# to poll while any wheel turns or a door moves, because a card poll can block
# ~25 ms and would disturb the software PWM and the 20 ms servo frames.
#
# Takes exclusive use of the serial port, so it stops arduino_bridge and puts it
# back afterwards. Closing the port resets the Mega, which zeroes the drop
# counters -- that is why the counters are read AFTER a measured idle window
# rather than immediately, so the number means "drops per <window>".
#
# WHAT GOOD LOOKS LIKE, versus the broken baseline measured 2026-08-01:
#
#                        broken (Mega 3V3 pin)     healthy
#   TxControl drops      218                       0, or a couple
#   TReload              0/0                       0/1000
#   VersionReg unstable  5/20                      0/20
#   tapping a card       nothing                   R,<UID> line
#
# TReload reading 0/0 is the tell: the timer config the firmware wrote has
# vanished, which is the whole chip browning out, not a dropped SPI write.
set -u
SECS="${1:-45}"

for p in $(ps -eo pid,cmd | grep -E "arduino_bridge" | grep -v grep | awk '{print $1}'); do
    kill "$p" 2>/dev/null
done
sleep 3

python3 - "$SECS" <<'PYEOF'
import sys, time, serial
secs = float(sys.argv[1])
s = serial.Serial()
s.port = '/dev/ttyACM0'
s.baudrate = 115200
s.timeout = 0.4
# DTR must be low BEFORE open or the Mega resets and setup() re-runs.
s.dtr = False
s.rts = False
s.open()
time.sleep(2.0)
s.reset_input_buffer()

print(f'--- polling {secs:.0f}s. TAP THE CARD NOW (repeatedly) ---', flush=True)
tags = []
end = time.time() + secs
while time.time() < end:
    ln = s.readline()
    if not ln:
        continue
    t = ln.decode('ascii', 'replace').strip()
    if t.startswith('R,'):
        tags.append(t)
        print('  TAG: ' + t, flush=True)

print(f'\ntags read: {len(tags)}')
s.write(b'Z\n')
s.flush()
end = time.time() + 8
print('--- reader health ---')
for _ in range(4000):
    if time.time() > end:
        break
    ln = s.readline()
    if not ln:
        continue
    t = ln.decode('ascii', 'replace').strip()
    if 'rfid' in t.lower() or 'MFRC' in t:
        print('  ' + t, flush=True)
s.close()
PYEOF

echo "--- restarting arduino_bridge ---"
setsid nohup "$HOME/fw_testing/start_bridge.sh" </dev/null \
    >"$HOME/fw_testing/bridge.log" 2>&1 &
sleep 12
grep -q "Serial ready" "$HOME/fw_testing/bridge.log" \
    && echo "arduino_bridge back up" \
    || echo "WARNING: bridge did not report Serial ready - check bridge.log"
