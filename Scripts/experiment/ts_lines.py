#!/usr/bin/env python3
"""Prefix every stdin line with the wall-clock time (same clock as ROS time when use_sim_time=false)."""
import sys
import time

for line in iter(sys.stdin.readline, ""):
    sys.stdout.write("%.6f %s" % (time.time(), line))
    sys.stdout.flush()
