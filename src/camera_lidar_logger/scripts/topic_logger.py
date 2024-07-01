#!/usr/bin/env python3

from  datetime import datetime
import os
import rospy
from std_msgs.msg import String
import time
from threading import Timer
from camera_lidar_logger.srv import emergencyStop

service_name = 'CAMERA_THR'
directory = '/tmp/log'
file_name = os.path.join(directory, datetime.now().strftime('obstacles_log_%H_%M_%d_%m_%Y.csv'))

class TopicLogger:
    '''
    Class to log the obstacle detection by lidar and camera.
    '''
    def __init__(self):
        self.local_map_times = []
        self.camera_od_times = []

        rospy.Subscriber('/lidar_obstacle_topic', String, self.lidar_callback)
        rospy.Subscriber('/camera_obstacle_topic', String, self.camera_od_callback)

    def lidar_callback(self, msg):
        self.local_map_times.append(time.time())
        self.log_time('Object detected on lidar')

    def camera_od_callback(self, msg):
        global detected
        detected = True
        self.camera_od_times.append(time.time())
        self.log_time('Object detected on Camera')

    def log_time(self, message):
        with open(file_name, 'a') as f:
            f.write("%s: %f\n" % (message, time.time()))

    @staticmethod
    def brake_status_callback(msg):
        global brake_status
        brake_list = msg.data.split(",")
        if(str(service_name) in brake_list):
            brake_status = True
        else:
            brake_status= False

if __name__ == '__main__':
    global detected, brake_status
    detected = False
    brake_status = False
    rospy.init_node('topic_logger', anonymous=True)

    if not os.path.exists(directory):
        os.makedirs(directory)
    topic_logger = TopicLogger()
    rate = rospy.Rate(10)
    
    rospy.wait_for_service('emergencyStop')
    rospy.loginfo("Waiting for Emergency stop service")
    emergencyStopSrv=rospy.ServiceProxy('emergencyStop', emergencyStop)
    rospy.Subscriber("/brake_status",String,TopicLogger.brake_status_callback,queue_size=1)

    while not rospy.is_shutdown():
        if(detected == True):
            if(brake_status == False):
                emergencyStopSrv(1,0,service_name)
        else:
            if(brake_status == True):
                print("Releasing to Brake")
                emergencyStopSrv(0,0,service_name)
        detected = False
        rate.sleep()
