#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

def point_cloud_callback(msg):
    # Placeholder logic to detect obstacles in the point cloud
    obstacle_detected = False  # Implement your obstacle detection logic here

    if obstacle_detected:
        obstacle_msg = String()
        obstacle_msg.data = "Obstacle detected on Lidar!"
        pub.publish(obstacle_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_obstacle_detector')
    
    pub = rospy.Publisher('lidar_obstacle_topic', String, queue_size=10)
    rospy.Subscriber('local_map_transformed', PointCloud2, point_cloud_callback)
    
    rospy.spin()
