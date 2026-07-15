#!/usr/bin/env python3

import rospy
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from sklearn.cluster import DBSCAN

from ros_numpy import numpify, msgify
from sensor_msgs.msg import PointCloud2


class PointsClusterer:
    def __init__(self):
        # Parameters
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon')
        self.cluster_min_samples = rospy.get_param('~cluster_min_samples')

        # TODO 1: Create self.clusterer using DBSCAN with the parameters above.

        # Publishers
        self.clustered_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def points_callback(self, msg):

        # TODO 1: Extract points from the message and cluster them.
        #         - Use numpify(msg) to convert the PointCloud2 message to a numpy array
        #         - Use structured_to_unstructured()
        #           to get an (N, 3) array of point coordinates
        #         - Run self.clusterer.fit_predict(points) to get cluster labels
        #         - Skip empty point clouds (0 points) - DBSCAN cannot cluster them

        # TODO 2: Publish the clustered points as a PointCloud2 message.
        #         - Concatenate points with labels (as a new column)
        #         - Filter out noise points (label == -1)
        #         - Convert to structured array with unstructured_to_structured()
        #         - Use msgify(PointCloud2, data) to create the message
        #         - Set header stamp and frame_id from the input message
        #         - Publish with self.clustered_pub

        pass

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('points_clusterer', log_level=rospy.INFO)
    node = PointsClusterer()
    node.run()