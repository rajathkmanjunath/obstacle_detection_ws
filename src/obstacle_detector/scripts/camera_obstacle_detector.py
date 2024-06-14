#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from camera_od.msg import CameraODArray  # Adjust import based on your custom message type

def camera_callback(msg):
    for obstacle in msg.detections:
        if obstacle.y2 <= 480:
            obstacle_msg = String()
            obstacle_msg.data = f"Obstacle detected on camera: {obstacle.class_name}"
            pub.publish(obstacle_msg)

if __name__ == '__main__':
    rospy.init_node('camera_obstacle_detector')
    
    pub = rospy.Publisher('camera_obstacle_topic', String, queue_size=10)
    rospy.Subscriber('camera_od_topic', CameraODArray, camera_callback)
    
    rospy.spin()
