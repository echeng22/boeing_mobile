#!/usr/bin/env python
import matplotlib.pyplot as plt
import os
import rosbag

filename_odom = "odom_1.bag"
filename_vive = "vive_1.bag"
filepath = os.path.dirname(os.path.realpath(filename_odom))
bag_odom = rosbag.Bag(filepath + '/bag/' + filename_odom)
bag_vive = rosbag.Bag(filepath + '/bag/' + filename_vive)
xpoints_odom = []
ypoints_odom = []
xpoints_vive = []
ypoints_vive = []
for topic, msg, t in bag_odom.read_messages(topics=['/odom']):
    xpoints_odom.append(msg.position.x)
    ypoints_odom.append(msg.position.y)
for topic, msg, t in bag_vive.read_messages(topics=['/tracker_pose_filter']):
    xpoints_vive.append(msg.pose.position.x)
    ypoints_vive.append(msg.pose.position.y)
plt.plot(xpoints_odom, ypoints_odom, 'ro')
plt.plot(xpoints_vive, ypoints_vive, 'bx')
bag_odom.close()
plt.show()