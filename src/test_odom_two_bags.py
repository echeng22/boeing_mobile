#!/usr/bin/env python
import matplotlib.pyplot as plt
import os
import rosbag

filename_odom = "odom_6.bag"
filename_vive = "vive_6.bag"
# filename = "vive_odom_bag_4.bag"
filepath = os.path.dirname(os.path.realpath(filename_odom))
bag_odom = rosbag.Bag(filepath + '/bag/' + filename_odom)
bag_vive = rosbag.Bag(filepath + '/bag/' + filename_vive)
# bag = rosbag.Bag(filepath + '/bag/' + filename)
xpoints_odom = []
ypoints_odom = []
xpoints_vive = []
ypoints_vive = []

vive_offset_x = 0
vive_offset_y = 0
vive_offset_z = 0
count = 0
for topic, msg, t in bag_vive.read_messages(topics=['/tracker_pose_filter']):
    if count < 1:
        # print msg.pose.position.x
        # print msg.pose.position.y
        # print msg.pose.position.z
        vive_offset_x = msg.pose.position.x
        vive_offset_y = msg.pose.position.y
        # vive_offset_z = msg.pose.position.z
        count = 1
    # xpoints_vive.append(msg.pose.position.x)
    # ypoints_vive.append(msg.pose.position.y)
    # xpoints_vive.append((msg.pose.position.x - vive_offset_x)*2.44)
    xpoints_vive.append((msg.pose.position.x - vive_offset_x))    
    ypoints_vive.append((msg.pose.position.y - vive_offset_y))

count = 0
for topic, msg, t in bag_odom.read_messages(topics=['/odom']):
    xpoints_odom.append(msg.position.x)
    ypoints_odom.append(msg.position.y)
    # if count < 1:
    #     print msg.position.x
    #     print msg.position.y
    #     print msg.position.z
    #     count = 1


# plt.plot(xpoints_odom, ypoints_odom, 'ro')
# plt.plot(xpoints_vive, ypoints_vive, 'bx')

plt.plot(xpoints_odom[0], ypoints_odom[0], 'ro')
plt.plot(xpoints_vive[0], ypoints_vive[0], 'bx')
# bag.close()
plt.show()