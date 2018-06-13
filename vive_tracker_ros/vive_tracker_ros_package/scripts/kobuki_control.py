#!/usr/bin/env python
import os
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg import Pose
from math import pow,atan2,sqrt
import tf
import time
import numpy as np

class TurtlebotControl():
    def __init__(self,ns=None):
        #Creating our node,publisher and subscriber
        rospy.init_node('kobuki_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
	self.ns = ns
        self.tflistener = tf.TransformListener()
        self.pose = Pose().position
	self.theta = 0
	self.monitor = Twist()
        self.rate = rospy.Rate(10)
	self.timestamp = time.time()

    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    def get_pose_from_tf(self):
	try:
	    t = self.tflistener.getLatestCommonTime(self.ns+'_tracker_frame', self.ns+'_world_frame')
            position, quaternion = self.tflistener.lookupTransform(self.ns+'_world_frame', self.ns+'_tracker_frame', t)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
	self.pose.x = position[0]
	self.pose.y = position[1]
	self.pose.z = position[2]
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	self.theta = yaw
	# Rotating frame to tranform Tracker orientation to Kobukis'
	self.pose.x,self.pose.y = -self.pose.y,self.pose.x
	self.theta = self.theta
	return True

    def get_distance(self, goal):
        distance = sqrt(pow((goal.x - self.pose.x), 2) + pow((goal.y - self.pose.y), 2))
        return distance

    def move2goal(self, xpose, ypose, tol):
        goal_pose = Pose().position
        goal_pose.x = xpose
        goal_pose.y = ypose
        distance_tolerance = tol
        vel_msg = Twist()
	dist = 10000
	dest_heading = 0
	vel_msg.linear.x = 0.0
	vel_msg.linear.y = 0
        vel_msg.linear.z = 0
	vel_msg.angular.x = 0
        vel_msg.angular.y = 0
	M = 0.9
	N = 1-M
	
        while dist >= distance_tolerance and not rospy.is_shutdown():
	    if not self.get_pose_from_tf()==True:
		print 'No tf received'
		continue
	    dist = self.get_distance(goal_pose)
	    dest_heading = self.wrap_angle((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.theta))
	    #Porportional Controller
            #linear velocity in the x-axis:
            x = 0.3*dist if abs(dest_heading)<0.7 else 0.0
	    vel_msg.linear.x = M*vel_msg.linear.x + N*x
            #angular velocity in the z-axis:
            vel_msg.angular.z = 1.3 * dest_heading

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    nspace = os.environ['ROS_NAMESPACE']
    x = TurtlebotControl(ns=nspace)
    time.sleep(2)
    try:
        while not rospy.is_shutdown():
            #x.move2goal(1.0,-0.4,0.2)
            #x.move2goal(1.0,0.4,0.2)
            #x.move2goal(0.0,0.4,0.2)
            #x.move2goal(0.0,-0.4,0.2)
	    #time.sleep(0.1)
	    x.move2goal(0.0,0.0,0.2)
    except rospy.ROSInterruptException:
	print 'error......'
	pass
