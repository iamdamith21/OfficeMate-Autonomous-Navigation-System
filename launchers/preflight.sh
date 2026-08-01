#!/usr/bin/env bash
# preflight.sh — run BEFORE a demo. Fails loudly rather than silently.
#
# Ordered by how badly each one ruins a demonstration, and every check reports
# the number it judged on, so a marginal result is visible rather than rounded
# into a tick.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null

pass=0; fail=0
ok()   { echo "  [ OK ] $*"; pass=$((pass+1)); }
bad()  { echo "  [FAIL] $*"; fail=$((fail+1)); }
info() { echo "         $*"; }

echo "=== 1. LiDAR link ==="
car=$(cat /sys/class/net/eth0/carrier 2>/dev/null || echo 0)
if [[ "$car" == "1" ]] && ping -c1 -W2 192.168.10.160 >/dev/null 2>&1; then
    ok "lidar reachable (carrier 1, ping replies)"
else
    bad "lidar unreachable (carrier=$car)"
    # These two states need different repairs, so name which one it is.
    [[ "$car" == "1" ]] \
        && info "carrier 1 + no ping = powered but network stack dead -> power-cycle the lidar" \
        || info "carrier 0 = no electrical link at all -> cable or power"
fi

echo "=== 2. Arduino ==="
if [[ -e /dev/ttyACM0 ]]; then ok "/dev/ttyACM0 present"; else bad "/dev/ttyACM0 MISSING"; fi
holder=$(fuser /dev/ttyACM0 2>/dev/null)
[[ -n "$holder" ]] && info "port held by PID(s):$holder (kill by PID, never pkill -f over ssh)"

echo "=== 3. Stale ROS nodes ==="
n=$(pgrep -cf 'ltme_node|slam_toolbox|nav2|rf2o|amcl|joint_state_publisher' 2>/dev/null | head -1); n=${n:-0}
if [[ "$n" -eq 0 ]]; then ok "no stale nodes"; else bad "$n stale node(s) -> ~/fw_testing/navkill.sh"; fi

echo "=== 4. CPU ==="
# Ubuntu's crash uploader has been measured at 75.8% here, which starves the
# control loop and has nothing to do with the robot.
w=$(pgrep -c whoopsie 2>/dev/null | head -1); w=${w:-0}
[[ "$w" -gt 0 ]] && bad "whoopsie running -> kill it, clear /var/crash" || ok "no whoopsie"
load=$(cut -d' ' -f1 /proc/loadavg)
awk -v l="$load" 'BEGIN{exit !(l<2.0)}' && ok "load $load" || bad "load $load (>2 idle is trouble)"

echo "=== 5. Firmware + MOTOR POWER UNDER LOAD ==="
if [[ -e /dev/ttyACM0 && -z "$holder" ]]; then
python3 - <<'PY'
import serial, time, sys
try:
    s = serial.Serial(); s.port='/dev/ttyACM0'; s.baudrate=115200; s.timeout=0.4
    s.dtr=False; s.rts=False; s.open()
    # Explicit reset pulse then wait out the ~1.5 s bootloader window, or the
    # port can sit enumerated but mute.
    time.sleep(0.4); s.dtr=True; s.rts=True; time.sleep(0.1); s.dtr=False; s.rts=False
    time.sleep(3.0); s.reset_input_buffer()
    ver=None; volt=[]
    end=time.time()+6
    while time.time()<end:
        t=s.readline().decode('ascii','replace').strip()
        if t.startswith('S,READY'): ver=t
        if t.startswith('B,'):
            try: volt.append(float(t.split(',')[1]))
            except Exception: pass
    print('  [ OK ] firmware %s' % ver if ver else '  [FAIL] no S,READY banner')

    # THE important one. The pack has been measured collapsing 11.6 -> 8.22 V
    # under a pivot, which stalls the motors without turning them.
    idle = max(volt) if volt else None
    s.reset_input_buffer()
    end=time.time()+2.5; under=[]
    while time.time()<end:
        s.write(b'V,0,0.45\n'); s.flush()
        t0=time.time()
        while time.time()-t0<0.2:
            t=s.readline().decode('ascii','replace').strip()
            if t.startswith('B,'):
                try: under.append(float(t.split(',')[1]))
                except Exception: pass
    s.write(b'V,0,0\n'); s.flush()
    if idle and under:
        lo=min(under)
        print('  idle %.2f V -> under load %.2f V (sag %.2f V)' % (idle, lo, idle-lo))
        print('  [ OK ] motor rail holds' if lo>10.5 else
              '  [FAIL] RAIL COLLAPSES to %.2f V - robot will NOT drive' % lo)
    else:
        print('         no battery telemetry (B, lines) - check INA219')
    s.close()
except Exception as e:
    print('  [FAIL] serial check: %s' % e)
PY
else
    echo "  (skipped - port missing or busy)"
fi

echo
echo "=== summary: $pass ok, $fail failed ==="
[[ "$fail" -eq 0 ]] || echo "Fix the FAILs above before demonstrating."
