#!/usr/bin/env python3

import subprocess
import sys
import time

def loop_play_bag(bag_file, delay):
    while True:
        print("Playing ROS bag: ", bag_file)
        subprocess.call(['rosbag', 'play', bag_file])
        print("ROS bag play finished. Restarting after delay.")
        time.sleep(delay)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: loop_play_rosbag.py <bag_file> <delay_in_seconds>")
        sys.exit(1)

    bag_file = sys.argv[1]
    delay = float(sys.argv[2])
    
    loop_play_bag(bag_file, delay)
