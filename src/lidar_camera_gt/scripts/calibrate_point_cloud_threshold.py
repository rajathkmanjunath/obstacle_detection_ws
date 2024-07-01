#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
import numpy as np

class OccupancyGridThresholdCalibrator:
    def __init__(self):
        self.occupancy_grid_sub = rospy.Subscriber('/local_map', OccupancyGrid, self.occupancy_grid_callback)
        self.tf_listener = tf.TransformListener()

    def occupancy_grid_callback(self, msg):
        try:
            self.tf_listener.waitForTransform('/base_link', '/velodyne', rospy.Time(0), rospy.Duration(1.0))
            
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/velodyne', rospy.Time(0))
            
            transform_matrix = tf.transformations.quaternion_matrix(rot)
            transform_matrix[0][3] = trans[0]
            transform_matrix[1][3] = trans[1]
            transform_matrix[2][3] = trans[2]

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
                    if msg.data[i] > 0:
                        x_coord = origin_x + x * resolution
                        y_coord = origin_y + y * resolution
                        z_coord = 0
                        pt = [x_coord, y_coord, z_coord, 1]
                        transformed_pt = transform_matrix.dot(pt)
                        
                        if -1.5 <= transformed_pt[1] <= 1.5 and transformed_pt[0] > 0:
                            transformed_points.append(transformed_pt[:3])
                            if transformed_pt[0] < min_x_point[0]:
                                min_x_point = transformed_pt

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('Transform error: %s', e)

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_threshold_calibrator')
    transformer = OccupancyGridThresholdCalibrator()
    rospy.spin()

