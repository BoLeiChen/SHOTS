#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import octomap_msgs.msg
import octomap
import numpy as np

class OctomapExplorer:
    def __init__(self):
        rospy.init_node("octomap_explorer")
        self.octree = octomap.OcTree()
        self.resolution = 0.0
        self.metric_min = self.metric_max = 0.0
        
        rospy.Subscriber("/husky/octomap_binary", octomap_msgs.msg.Octomap, self.octree_callback)
        
    def octree_callback(self, msg):
        self.octree = octomap.OcTree()
        self.octree.deserialize(msg.data)
        self.resolution = msg.resolution
        self.metric_min = (msg.origin.position.x, msg.origin.position.y, msg.origin.position.z)
        self.metric_max = self.metric_min + (msg.octree.size*(msg.resolution))^3
        region_min = np.array([-5, -5, 0])
        region_max = np.array([5, 5, 2.5])
        region_volume = (region_max - region_min).prod()

        keys = octree.search(region_min, region_max)
        n_explored = len(keys)

        explored_ratio = n_explored / (n_total * region_volume)
        print(explored_ratio)

if __name__ == "__main__": 
    explorer = OctomapExplorer()
    rospy.spin()


