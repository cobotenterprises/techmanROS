#!/usr/bin/env python

import sys, signal, subprocess, time


timeout_before_kill = 1.0  # [s]
timeout_after_kill = 1.0  # [s]


def signal_handler(sig, frame):
    time.sleep(timeout_before_kill)
    subprocess.call("killall -q gzclient & killall -q gzserver", shell=True)
    time.sleep(timeout_after_kill)
    subprocess.call("killall -9 -q gzclient & killall -9 -q gzserver", shell=True)
    sys.exit(0)


if __name__ == "__main__":
   signal.signal(signal.SIGINT, signal_handler)
   cmd = ' '.join(sys.argv[1:])
   subprocess.call(cmd, shell=True)
   while True: time.sleep(1)
