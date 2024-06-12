#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import numpy as np

class OccupancyGridThresholdCalibrator:
    def __init__(self):
        self.occupancy_grid_sub = rospy.Subscriber('/local_map', OccupancyGrid, self.occupancy_grid_callback)
        self.tf_listener = tf.TransformListener()

    def occupancy_grid_callback(self, msg):
        try:
            # Wait for the transform from 'velodyne' to 'base_link' to be available
            self.tf_listener.waitForTransform('/base_link', '/velodyne', rospy.Time(0), rospy.Duration(1.0))
            
            # Get the transformation
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/velodyne', rospy.Time(0))
            
            # Create a transformation matrix
            transform_matrix = tf.transformations.quaternion_matrix(rot)
            transform_matrix[0][3] = trans[0]
            transform_matrix[1][3] = trans[1]
            transform_matrix[2][3] = trans[2]

            # Convert OccupancyGrid to points and transform them
            transformed_points = []
            min_x_point = [float(np.inf), float(np.inf), float(np.inf)]
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            for y in range(height):
                for x in range(width):
                    i = x + y * width
                    if msg.data[i] > 0:  # Occupied cell
                        x_coord = origin_x + x * resolution
                        y_coord = origin_y + y * resolution
                        z_coord = 0  # Assume the grid is on the ground plane
                        pt = [x_coord, y_coord, z_coord, 1]
                        transformed_pt = transform_matrix.dot(pt)
                        
                        # Apply filtering for points within specified range
                        if -1.5 <= transformed_pt[1] <= 1.5 and transformed_pt[0] > 0:
                            transformed_points.append(transformed_pt[:3])
                            if transformed_pt[0] < min_x_point[0]:
                                min_x_point = transformed_pt

            print(min_x_point)
            rospy.logfatal('######## The point x, y, and z of the min points are %s ########', min_x_point)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('Transform error: %s', e)

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_threshold_calibrator')
    transformer = OccupancyGridThresholdCalibrator()
    rospy.spin()

