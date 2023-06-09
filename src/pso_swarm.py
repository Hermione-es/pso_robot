#! /usr/bin/env python2.7
import	rospy
import 	sys
from 	geometry_msgs.msg import Twist,Point,Pose
from 	nav_msgs.msg import Odometry
from    std_msgs.msg import Float32
from    Laser_Class_origin import Laser_ClosestPoint
from 	robot_class import *
import	smach
import  smach_ros
import  time
import  os

def get_Pbest():
	pass

def callback(msg):
	Gbest.x=msg.position.x
	Gbest.y=msg.position.y

def euclidean_distance( goal_point_x,goal_point_y,robot_pose_x, robot_pose_y):
	distance= sqrt(pow((goal_point_x - robot_pose_x), 2) +
	pow((goal_point_y - robot_pose_y), 2))
	return distance


#PSO state class
class PSO(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished', 'failed'],output_keys=['next_point_out'])
		self.epoch=0
		self.Pbest=Point()
		self.pb_dist=Point()
		self.gb_dist=Point()

	def execute(self, userdata):
		obst=l.closest_point()
		obst_d=r.euclidean_distance(obst)
		goal_dist=r.euclidean_distance(goal)
		#get Pbest
		self.Pbest=r.get_Pbest(goal) #Pbest
		next_point=userdata.next_point_out=r.get_next_point(Pbest, Gbest, obst,obst_d,self.pb_dist,self.gb_dist)
		
		self.epoch=self.epoch+1
		rospy.loginfo('Epoch:%s' , self.epoch)
		return 'finished'


#Go2Point state class
class Go2Point(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished','failed','finished2'],input_keys=['next_point_in'])
		self.next_point_obs=Point()

	def execute(self, userdata):
		next_point=userdata.next_point_in
		if r.euclidean_distance(goal)>2:
			if r.euclidean_distance(next_point)>0.4:
				goal_angular_speed=r.angular_vel(next_point)
				goal_linear_speed = r.linear_vel(next_point) #linear_vel 
				speed.linear.x=goal_linear_speed
				speed.angular.z=goal_angular_speed
		
				pub.publish(speed)
			return 'finished' # goal>2 && next_point>0.4
		else:
			return 'finished2' # goal<=2

#Wait state class		
class Wait(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished', 'failed'])

	def execute(self, userdata):
		r.stop()
		print( "WorkingTime: {} sec".format(time.time()-start))
		rospy.sleep(10)
		return 'finished'


if __name__ == '__main__':
	args=rospy.myargv(argv=sys.argv)
	robotname= args[1]
	rospy.init_node("PSO_node")
	start = time.time()
	Gbest=Point()

	# Subscribe to topic to get global best point
	sub=rospy.Subscriber('/get_Gbest',Pose,callback)

	global l
	global r
	global next_point
	global goal
	global pub
	global speed

	# Initialize objects
	l=Laser_ClosestPoint(robotname)
	r=robot(robotname)

	# Define goal point and personal best
	speed=Twist()
	goal=Point()
	goal.x,goal.y=get_goal()
	Pbest=10000
	next_point=Point()

	# Initialize publishers
	pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
	
	# Wait for node to fully initialize
	rate = rospy.Rate(1)
	rate.sleep()
	
	# Define state machine and its outcomes		
	sm =smach.StateMachine(outcomes=['I_have_finished'])

	# Create and start the introspection server for visualising state machine
	sis = smach_ros.IntrospectionServer('server_name', sm, '/{}/SM_ROOT'.format(robotname))
	sis.start()

	# Adding states
	with sm:
		smach.StateMachine.add('PSO', PSO(),
                                transitions={'finished': 'Go2Point', 'failed': 'PSO'}, remapping={'next_point_out':'sm_next_point'})
		smach.StateMachine.add('Go2Point', Go2Point(),
                                transitions={'finished': 'PSO', 'failed': 'Go2Point', 'finished2' : 'Wait'},remapping={'next_point_in':'sm_next_point'})
		smach.StateMachine.add('Wait', Wait(),
                                transitions={'finished': 'Wait', 'failed': 'Wait'})
	# Start State_Machine
	outcome =sm.execute()
	sis.stop()
	rospy.spin()
