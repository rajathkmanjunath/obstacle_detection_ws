#!/usr/bin/env python

from  datetime import datetime
import os
import rospy
from std_msgs.msg import String
import time
from threading import Timer

global detected, brake_name, brake_status
service_name = 'LIDAR_THR'
directory = '/tmp/log'

class TopicLogger:
    def __init__(self):
        self.local_map_times = []
        self.camera_od_times = []
        self.logging_interval = 3  # seconds
        self.pause_duration = 5  # seconds
        self.evaluation_frequency = 1 # seconds
        self.last_sound_time = time.time()

        rospy.Subscriber('camera_obstacle_topic', String, self.lidar_callback)
        # TODO: Rename the topics to follow camel case
        rospy.Subscriber('camera_od_topic', String, self.camera_od_callback)

        # rospy.Timer(rospy.Duration(self.evaluation_frequency), self.check_and_play_sound)

    def lidar_callback(self, msg):
        detected = True
        self.local_map_times.append(time.time())
        self.log_time('Object detected on lidar')

    def camera_od_callback(self, msg):
        self.camera_od_times.append(time.time())
        self.log_time('Object detected on Camera')

    def log_time(self, message):
        with open(os.path.join(directory, datetime.now().strftime('obstacles_log_%H_%M_%d_%m_%Y.csv')), 'a') as f:
            f.write("%s: %f\n" % (message, time.time()))

def brake_status_callback(msg):
    global brake_status,brake_name
    brake_list = msg.data.split(",")
    if(str(brake_name) in brake_list):
        brake_status = True
    else:
        brake_status= False

if __name__ == '__main__':
    rospy.init_node('topic_logger', anonymous=True)

    if not os.path.exists(directory):
        os.makedirs(directory)
    topic_logger = TopicLogger()
    rospy.spin()
    '''
    rospy.wait_for_service('emergencyStop')
    emergencyStopSrv=rospy.ServiceProxy('emergencyStop',emergencyStop)
    rospy.Subscriber("/brake_status",String,brake_status_callback,queue_size=1)

    while not rospy.is_shutdown():
        if(detected == True):
            if(brake_status == False):
                print("Trying to Brake")
                emergencyStopSrv(1,0,service_name)
        else:
            if(brake_status == True):
                print("Releasing to Brake")
                emergencyStopSrv(0,0,service_name)
        detected = False
        rate.sleep()
    '''
