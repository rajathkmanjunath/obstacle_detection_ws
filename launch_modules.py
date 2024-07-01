#!/usr/bin/env python3

import subprocess

open_new_tab = ["gnome-terminal","--tab", "-e"]
package_launch_names = [
    ['camera_od', 'camera_od.launch'],
    ['lidar_camera_gt', 'transform_point_cloud.launch'],
    ['obstacle_detector', 'detector.launch'],
    ['camera_lidar_logger', 'topics_logger.launch'],
    ['event_recorder', 'record_event.launch']
]

def start_ros_launch(package, launch_file):
    cmd = f"sleep 1; cd ~/obstacle_detection_ws;source devel/setup.bash;roslaunch {package} {launch_file} --wait"
    return subprocess.Popen(['gnome-terminal', '--tab', '--', 'bash', '-c', f"{cmd}; exec bash"])

if __name__ == '__main__':
    subprocess.Popen(['gnome-terminal', '--tab', '--', 'bash', '-c', "roscore; exec bash"])
    for pair in package_launch_names:
        print(pair[0], pair[1])
        package, launch = pair[0], pair[1]
        start_ros_launch(package, launch)