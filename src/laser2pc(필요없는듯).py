#!/usr/bin/env python
import rospy
from 	sensor_msgs.msg import PointCloud2 as pc2
from 	sensor_msgs.msg import LaserScan
from	laser_geometry 	import LaserProjection
import 	sys

args=rospy.myargv(argv=sys.argv)
robotname= args[1]
class Laser2pc(object):
	
	def __init__(self,robotname):
		self.robotname= robotname
		self.laserProj=LaserProjection()
		self.pcPub= rospy.Publisher("/{}/LaserPointCloud".format(self.robotname),pc2, queue_size=1)
		self.lsersub=rospy.Subscriber("/{}/laser_scan".format(self.robotname),LaserScan,self.laserCallback)


	def laserCallback(self,data):
		cloud_out=self.laserProj.projectLaser(data)
		self.pcPub.publish(cloud_out)


if __name__ == '__main__':
	rospy.init_node("laser2pointcloud")
	l2pc=Laser2pc(robotname)
	rospy.spin()