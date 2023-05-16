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


class PSO(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished', 'failed'],output_keys=['next_point_out'])
		self.epoch=0
		self.Pbest=Point()
		self.pb_dist=Point()
		self.gb_dist=Point()

	def execute(self, userdata):
		obst=l.closest_point()
		obst_dist=r.euclidean_distance(obst)
		goal_dist=r.euclidean_distance(goal)
		#get Pbest
		self.Pbest=r.get_Pbest(goal) #Pbest
		next_point=userdata.next_point_out=r.get_next_point(self.Pbest,Gbest,obst,self.pb_dist,self.gb_dist)
		optimum = Point()
		optimum.x = next_point.x-r.robot_pose_x
		optimum.y = next_point.y-r.robot_pose_y

					
		# userdata.next_point_out=r.get_next_point(self.Pbest,Gbest,obst)
		# rospy.loginfo('X: %s Y:%s' , self.Pbest.x ,self.Pbest.y )
		self.epoch=self.epoch+1
		obstacle.publish(obst_dist)
		# rospy.loginfo('Epoch:%s' , self.epoch )
		return 'finished'


class Go2Point(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished','failed','finished2'],input_keys=['next_point_in'])
		# goal=Point()
		# goal.x,goal.y=get_goal()
		self.next_point_obs=Point()


	def execute(self, userdata):
			# local=0
			# previous=Point()
			# previous.x=r.robot_pose_x
			# previous.y=r.robot_pose_y
			# optimum=Point()

			next_point=userdata.next_point_in
			# rospy.loginfo('X: %s Y:%s' , next_point.x,next_point.y )
			# self.next_point_obs=next_point
			if r.euclidean_distance(goal)>2:
				if r.euclidean_distance(next_point)>0.4:
					# steer_vec=r.avoid_obstacle(next_point)
					# self.next_point_obs.x=steer_vec[0]
					# self.next_point_obs.y=steer_vec[1]
					
					goal_angular_speed=r.angular_vel(next_point)

					goal_linear_speed = r.linear_vel(next_point) #linear_vel 
					obst_angular_speed=r.get_apf_vel(next_point) #get_apf_vel 

					speed.linear.x=goal_linear_speed
					speed.angular.z=goal_angular_speed #+ obst_angular_speed
					# rospy.loginfo('X obst %s Y obst %s' ,self.next_point_obs.x,self.next_point_obs.y )
					pub.publish(speed)
					# next_point=userdata.next_point_in
					# steer_vec=r.avoid_obstacle(next_point)
					# self.next_point_obs.x=steer_vec[0]
					# self.next_point_obs.y=steer_vec[1]
					
				# 	optimum.x = r.robot_pose_x
				# 	optimum.y = r.robot_pose_y

				
				# if  optimum< 0.00001:
				# 	local+=1
				# 	if local>50:
				# 		r.stop()
				# 		print( "WorkingTime: {} sec".format(time.time()-start))
				# 		rospy.sleep(10)
				# 		exit()
				
				return 'finished' # goal>2 && next_point>0.4
			else:
				return 'finished2' # goal<=2
		
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
	sub=rospy.Subscriber('/get_Gbest',Pose,callback)
	global l
	global r
	global next_point
	global goal
	global obst_d
	global pub
	global speed
	global obstacle
	global obst_dist
	l=Laser_ClosestPoint(robotname)
	r=robot(robotname)

	# define_goal
	speed=Twist()
	goal=Point()
	obst_dist=Float32()
	goal.x,goal.y=get_goal()
	Pbest=10000
	next_point=Point()
	obstacle = rospy.Publisher("/{}/obst".format(robotname),Float32, queue_size=10)
	pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
	rate = rospy.Rate(1)
	rate.sleep()
	
			
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
