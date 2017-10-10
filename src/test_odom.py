#!/usr/bin/env python
import os
import rosbag

filename = "square_odom.bag"
filepath = os.path.dirname(os.path.realpath(filename))
bag = rosbag.Bag(filepath + '/bag/' + filename)
for topic, msg, t in bag.read_messages(topics=['/odom']):
    print msg
bag.close()
