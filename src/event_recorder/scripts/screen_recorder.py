#!/usr/bin/env python3

import subprocess
import os
from datetime import datetime
from std_msgs.msg import String
import rospy
from screeninfo import get_monitors

# Configuration
RECORDING_DIR = '/tmp/recordings'
BUFFER_DURATION = 5
BUFFER_FILE = "/tmp/rolling_buffer.mp4"


def get_screen_size():
    monitor = get_monitors()[0]
    return monitor.width, monitor.height

def trigger_callback(msg):
    output_file = datetime.now().strftime(os.path.join(RECORDING_DIR, 'trigger_%H_%M_%d_%m_%Y.mp4'))
    
    if not os.path.exists(RECORDING_DIR):
        os.makedirs(RECORDING_DIR)
    
    if os.path.exists(BUFFER_FILE):
        os.system(f"cp {BUFFER_FILE} {output_file}")
        print(f"Saved recording to {output_file}")
    else:
        print("Buffer file does not exist!")

if __name__ == '__main__':
    rospy.init_node('trigger_recorder')

    width, height = get_screen_size()
    FFMPEG_CMD = [
        'ffmpeg', '-f', 'x11grab', '-video_size', f'{width}x{height}', '-framerate', '25',
        '-i', ':0.0', '-t', str(BUFFER_DURATION), '-y', BUFFER_FILE
    ]

    ffmpeg_process = subprocess.Popen(FFMPEG_CMD, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    rospy.Subscriber('/camera_obstacle_topic', String, trigger_callback)

    rospy.spin()
