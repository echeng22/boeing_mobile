#!/usr/bin/env python
import matplotlib.pyplot as plt
import os
import rosbag

filename = "square_odom.bag"
filepath = os.path.dirname(os.path.realpath(filename))
bag = rosbag.Bag(filepath + '/bag/' + filename)
xpoints = []
ypoints = []
for topic, msg, t in bag.read_messages(topics=['/odom']):
    xpoints.append(msg.position.x)
    ypoints.append(msg.position.y)
    # print msg.position.x
plt.plot(xpoints, ypoints, 'ro')

bag.close()
plt.show()