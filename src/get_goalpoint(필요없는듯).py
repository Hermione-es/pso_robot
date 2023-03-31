#!/usr/bin/env python
#-*- encoding: utf-8 -*-

import rospy
from    nav_msgs.msg import Odometry
from    math    import sqrt,pow
from    geometry_msgs.msg import Twist,Point
import ros_numpy

class get_goalpoint():
	def __init__(self,robotname):
		self.robotname=robotname
		self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.callback)

	def callback(self,msg):
		self.robot_pose_x=msg.pose.pose.position.x
		self.robot_pose_y=msg.pose.pose.position.y

	def euclidean_distance(self, goal_point_x,goal_point_y):
		distance= sqrt(pow((goal_point_x - self.robot_pose_x), 2) +
						pow((goal_point_y - self.robot_pose_y), 2))
		return distance
	
	def PrintArray(self):
		min_distance=1000
		point=Point()
		for p in self.xyz_array:
			if self.euclidean_distance(p[0],p[1]) < min_distance:
				min_distance=self.euclidean_distance(p[0],p[1])
				point.x=self.robot_pose_x-p[0]
				point.y=self.robot_pose_y-p[1]
		print (" x : %f  y: %f" %(point.x,point.y))
		print (" distance : %f " %(min_distance))	
		return point