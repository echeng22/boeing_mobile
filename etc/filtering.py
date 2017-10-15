#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import scipy.linalg

# added by Chainatee
import tf
import tf.transformations as tr

class Filter(object):

    def __init__(self):
        # create publisher and subscriber
        self.sub = rospy.Subscriber("tracker_pose", PoseStamped, self.posecb)
        self.pub = rospy.Publisher("tracker_pose_filter", PoseStamped, queue_size = 1)

        self.num = 2
	self.num_ore = 11
        self.threshold = 0.1	
	self.threshold_ore = 5
	self.count_pose = 0
	self.count_ore = 0

        self.posex = np.array([])
        self.posey = np.array([])
        self.posez = np.array([])
	self.quat_ave = np.array([0,0,0,0])
        self.quat_arr = np.zeros((1, 4))

        # added by Chainatee for transforming the coordinate based on tf between lighthouse and odom_meas
        self.tf_listener = tf.TransformListener()
        # get 4*4 transformation from lighthouse and odom_meas
        # self.tf_listener.waitForTransform("tracker", "odom_meas", rospy.Time(), rospy.Duration(0.5))
        # self.T_lo_p, self.T_lo_q = self.tf_listener.lookupTransform("tracker", "odom_meas", rospy.Time())
        # print self.T_lo_p
        # print self.T_lo_q
        # self.T_lo_p = np.array([-1.1935, -0.2019, 1.8069]) 
        # self.T_lo_q = np.array([-0.9913, -0.0842, -0.0834, -0.0577])
        # print "first: ", self.T_lo_q
        # self.T_lo_q = np.array([-self.T_lo_q[3], -self.T_lo_q[0], -self.T_lo_q[1], -self.T_lo_q[2]])
        # self.T_lo = tr.euler_matrix(*tr.euler_from_quaternion(self.T_lo_q))
        # print "equal: ", tr.quaternion_from_matrix(self.T_lo[0:2, 0:2])
        # print "1: ", self.T_lo
        # self.T_lo[0:3,-1] = self.T_lo_p
        # print "2: ", self.T_lo
        # print "test2: ", self.T_lo_p




    def posecb(self, pose):
        #### added by Chainatee
        # pos = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        # quat = np.array([pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z])
        # T_lt = tr.euler_matrix(*tr.euler_from_quaternion(quat))
        # T_lt[0:3,-1] = pos
        # T_ot = np.linalg.inv(self.T_lo) * T_lt # inv of lo and lt to get ot
        # new_quat = tr.quaternion_from_matrix(T_ot[0:2, 0:2])
        # new_pos = T_ot[3, 0:2]
        # print T_ot
        # p = pose.pose
        # p.position.x = T_ot[0, 3]
        # p.position.y = T_ot[1, 3]
        # p.position.z = T_ot[2, 3]
        # print p.position
        # self.tf_listener.transformPose("odom_meas", )
        ####

        ######
        self.tf_listener.waitForTransform("lighthouse", "odom_meas", rospy.Time(), rospy.Duration(0.5))
        # self.T_lo_p, self.T_lo_q = self.tf_listener.lookupTransform("tracker", "odom_meas", rospy.Time())
        # p = pose.pose
        # p.position.x = self.T_lo_p[0]
        # p.position.y = self.T_lo_p[1]
        # p.position.z = self.T_lo_p[2]
        ######


        #########
        try:
            pose = self.tf_listener.transformPose("odom_meas", pose)
        except (tf.ExtrapolationException):
            return 
        print "x: ", pose.pose.position.x
        print "y: ", pose.pose.position.y
        print "z: ", pose.pose.position.z

        x = pose.pose.position.x
        z = pose.pose.position.z
        pose.pose.position.z = x
        pose.pose.position.x = z
        # print "-------------------"
        #########
        
        p = pose.pose
        pub_p = PoseStamped()
	# position.x and y
	if len(self.posex) < self.num:
	    self.quat_arr = np.append(self.quat_arr, [[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]], axis = 0)
            self.posex = np.append(self.posex, [p.position.x])
	    self.posey = np.append(self.posey, [p.position.y])
	    self.posez = np.append(self.posez, [p.position.z])
	    if self.quat_arr[0][0] == 0:
		self.quat_arr = np.delete(self.quat_arr, 0, axis = 0)
        else:
	    # filtering the position data
	    old_distance = math.sqrt((self.posex[1] - self.posex[0]) * (self.posex[1] - self.posex[0]) + (self.posey[1] - self.posey[0]) * (self.posey[1] - self.posey[0]))
	    new_distance = math.sqrt((p.position.x - self.posex[1]) * (p.position.x - self.posex[1]) + (p.position.y - self.posey[1]) * (p.position.y - self.posey[1]))
	    if new_distance < old_distance + self.threshold and new_distance > old_distance - self.threshold:
                # delete the first x value 
                self.posex = np.delete(self.posex, 0)
                # add new x value
                self.posex = np.append(self.posex, [p.position.x])
		# delete the first y value
                self.posey = np.delete(self.posey, 0)
                # add new y value
                self.posey = np.append(self.posey, [p.position.y])
		# delete the first z value
                self.posez = np.delete(self.posez, 0)
                # add new z value
                self.posez = np.append(self.posez, [p.position.z])
		self.count_pose = 0
	    else:
		self.count_pose += 1

	    # if the filter data doesn't change for a long time
	    if self.count_pose > 10:
	    	pub_p.pose.position.x = p.position.x
	    	pub_p.pose.position.y = p.position.y
	    	pub_p.pose.position.z = p.position.z
		self.posex = [p.position.x, p.position.x]
		self.posey = [p.position.y, p.position.y]
		self.posez = [p.position.z, p.position.z]
		self.count_pose = 0
	    else:
		pub_p.pose.position.x = self.filtering(self.posex)
	    	pub_p.pose.position.y = self.filtering(self.posey)
	    	pub_p.pose.position.z = self.filtering(self.posez)

	    
	    # filtering the orientation data
	    x_ang_old, y_ang_old, z_ang_old = self.quaternion_to_euler_angle(self.quat_arr[0][3], self.quat_arr[0][0], self.quat_arr[0][1], self.quat_arr[0][2])
	    x_ang_new, y_ang_new, z_ang_new = self.quaternion_to_euler_angle(self.quat_arr[1][3], self.quat_arr[1][0], self.quat_arr[1][1], self.quat_arr[1][2])
	    x_ang, y_ang, z_ang = self.quaternion_to_euler_angle(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
	    # print "dev"
	    # print self.quat_arr[0][3], self.quat_arr[0][0], self.quat_arr[0][1], self.quat_arr[0][2]
	    # print self.quat_arr[1][3], self.quat_arr[1][0], self.quat_arr[1][1], self.quat_arr[1][2]
	    # print x_ang_old, y_ang_old, z_ang_old
	    # print x_ang_new, y_ang_new, z_ang_new
	    # print x_ang, y_ang, z_ang
	    # if the error is small then 10 degrees
	    if (abs((x_ang - x_ang_new) - (x_ang_new - x_ang_old)) < self.threshold_ore) and (abs((y_ang - y_ang_new) - (y_ang_new - y_ang_old)) < self.threshold_ore) and (abs((z_ang - z_ang_new) - (z_ang_new - z_ang_old)) < self.threshold_ore):
		# delete the first array 
                self.quat_arr = np.delete(self.quat_arr, 0, axis = 0)
		# add new array
		self.quat_arr = np.append(self.quat_arr, [[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]], axis = 0)
		self.count_ore = 0
	    else:
		self.count_ore += 1

	    if self.count_ore > 10:
		pub_p.pose.orientation.x = p.orientation.x
		pub_p.pose.orientation.y = p.orientation.y
		pub_p.pose.orientation.z = p.orientation.z
		pub_p.pose.orientation.w = p.orientation.w
	    	self.quat_arr = [[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]]
		self.count_ore = 0
	    else:
		pub_p.pose.orientation.x = self.quat_arr[1][0]
		pub_p.pose.orientation.y = self.quat_arr[1][1]
		pub_p.pose.orientation.z = self.quat_arr[1][2]
		pub_p.pose.orientation.w = self.quat_arr[1][3]
	     	'''
	 	# average the data
		self.quat_arr = np.array(self.quat_arr)
		Qadj = np.dot(self.quat_arr.T, self.quat_arr)
		(eig, evec) = scipy.linalg.eigh(Qadj, eigvals=(3,3))
		quat_ave = evec.ravel()
		print quat_ave
		pub_p.pose.orientation.x = quat_ave[0]
		pub_p.pose.orientation.y = quat_ave[1]
		pub_p.pose.orientation.z = quat_ave[2]
		pub_p.pose.orientation.w = quat_ave[3]
		'''

        pub_p.header.frame_id="odom_meas"
        # pub_p.header.frame_id="lighthouse"

        self.pub.publish(pub_p)

    # irr
    def filtering(self, arr):

        result = 0.25 * arr[0] + 0.75  * arr[1]
        return result

    # use quaternion to calculate euler angles
    def quaternion_to_euler_angle(self, w, x, y, z):
	ysqr =y * y
	
	t0 = 2 * (w * x + y * z)
	t1 = 1 - 2 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = 2 * (w * y - z * x)
	t2 = 1 if t2 > 1 else t2
	t2 = -1 if t2 < -1 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = 2 * (w * z + x * y)
	t4 = 1 - 2 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z


def main():

    rospy.init_node('filtering')
    filt = Filter()
    rospy.spin()

if __name__=="__main__":
    main()
