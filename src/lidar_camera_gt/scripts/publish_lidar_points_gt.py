#!/usr/bin/env python3

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np

class OccupancyGridTransformer:
    def __init__(self):
        self.occupancy_grid_sub = rospy.Subscriber('/local_map', OccupancyGrid, self.occupancy_grid_callback)
        self.point_cloud_pub = rospy.Publisher('/local_map_transformed', PointCloud2, queue_size=1)
        self.tf_listener = tf.TransformListener()

    def occupancy_grid_callback(self, msg):
        try:
            self.tf_listener.waitForTransform('/base_link', '/velodyne', rospy.Time(0), rospy.Duration(0.1))
            
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/velodyne', rospy.Time(0))
            
            transform_matrix = tf.transformations.quaternion_matrix(rot)
            transform_matrix[0][3] = trans[0]
            transform_matrix[1][3] = trans[1]
            transform_matrix[2][3] = trans[2]

            transformed_points = []
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            for y in range(height):
                for x in range(width):
                    i = x + y * width
                    if msg.data[i] > 0:
                        x_coord = origin_x + x * resolution
                        y_coord = origin_y + y * resolution
                        z_coord = 0
                        pt = [x_coord, y_coord, z_coord, 1]
                        transformed_pt = transform_matrix.dot(pt)
                        
                        if (0 <= transformed_pt[0] <= 3.52) and -1 <= transformed_pt[1] <= 1:
                            transformed_points.append(transformed_pt[:3])

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "base_link"
            
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            
            transformed_cloud = pc2.create_cloud(header, fields, transformed_points)
            self.point_cloud_pub.publish(transformed_cloud)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('Transform error: %s', e)

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_transformer')
    transformer = OccupancyGridTransformer()
    rospy.spin()