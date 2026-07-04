#!/usr/bin/env python3
import sys, os, time, termios
port = sys.argv[1] if len(sys.argv) > 1 else '/dev/sllidar'
try:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = list(termios.tcgetattr(fd))
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    os.write(fd, bytes([0xa5, 0x25]))
    time.sleep(0.05)
    os.write(fd, bytes([0xa5, 0x40]))
    time.sleep(0.3)
    termios.tcflush(fd, termios.TCIFLUSH)
    os.close(fd)
    time.sleep(0.3)
    print('LiDAR reset complete')
except Exception as e:
    print(f'LiDAR reset skipped ({e})')
