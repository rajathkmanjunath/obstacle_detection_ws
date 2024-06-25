#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from camera_od.msg import CameraODArray


filter_obstacles = ['person', 'chair']
def camera_callback(msg):
    for obstacle in msg.detections:
        if obstacle.y2 >= 260.0 and obstacle.class_name in filter_obstacles:
            ineq = 0.5 * obstacle.x2 - 2.0 * obstacle.y2 + 650
            rospy.loginfo(str(ineq))
            # rospy.loginfo(str(obstacle.x2) + " " + str(obstacle.y2))
            
            obstacle_msg = String()
            obstacle_msg.data = f"Obstacle detected on camera: {obstacle.class_name}"
            pub.publish(obstacle_msg)

if __name__ == '__main__':
    rospy.init_node('camera_obstacle_detector')
    
    pub = rospy.Publisher('/camera_obstacle_topic', String, queue_size=10)
    rospy.Subscriber('/camera_od_topic', CameraODArray, camera_callback)
    
    rospy.spin()
