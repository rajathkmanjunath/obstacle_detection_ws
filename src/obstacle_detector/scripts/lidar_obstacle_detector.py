#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import numpy as np

def point_cloud_callback(msg):
    point_array = np.array(list(pc2.read_points(msg, skip_nans=True)))
    obstacle_detected = (point_array.size != 0) 

    if obstacle_detected:
        obstacle_msg = String()
        obstacle_msg.data = "Obstacle detected!"
        pub.publish(obstacle_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_obstacle_detector')
    
    pub = rospy.Publisher('/lidar_obstacle_topic', String, queue_size=10)
    rospy.Subscriber('/local_map_transformed', PointCloud2, point_cloud_callback)
    
    rospy.spin()
