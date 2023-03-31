#!/usr/bin/env python
import rospy
import sys
import math
from math import atan2
import roslib; roslib.load_manifest('swarm')
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

args=rospy.myargv(argv=sys.argv)
robotname= args[1]

pose_x=0.0
pose_y=0.0
pose_theta=0.0
x2=0.0
y2=0.0
distance2=0

class robot():
    def __init__(self, pose_x, pose_y, pose_theta):

        self.pose_x = pose_x
        self.pose_y = pose_y
        self.pose_theta = pose_theta

    def go(self, distance):
        speed.linear.x = 0.05*distance
        speed.angular.z = 0.0

    def turn_left(self, distance, angledif):
        speed.linear.x = 0.05*distance
        speed.angular.z = -0.8*angledif
    
    def turn_right(self,distance,angledif):
        speed.linear.x=0.05*distance
        speed.angular.z=0.8*angledif

    def stop(self):
        speed.linear.x=0.0
        speed.angular.z=0.0
    
    def avoid_obstacle(self):
        speed.linear.x=0.1
        speed.angular.z=-0.1

 # callback from the subscription to /robot_1/odom topic
def callback1(msg):
    global x2
    global y2
    x2=msg.pose.pose.position.x
    y2=msg.pose.pose.position.y
    # rospy.loginfo('x:{} , y:{},'.format(x2,y

def callback(msg):

    global pose_x
    global pose_y
    global pose_theta
    
# getting the pose values of the robot by subscribing to the /odom topic
    pose_x = msg.pose.pose.position.x
    pose_y = msg.pose.pose.position.y
    rot_q= msg.pose.pose.orientation
    (roll,pitch,pose_theta)= euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w]) 

# Initialize class Robot instance
rospy.init_node('swarm_control', anonymous=False)
robot_1=robot(pose_x,pose_y,pose_theta)
sub = rospy.Subscriber("/{}/odom".format(robotname), Odometry , callback)
sub2=rospy.Subscriber("/robot_1/odom", Odometry , callback1)
    # rospy.loginfo('x:{} , y:{}, theta:{}'.format(x,y,theta)) 
    # rospy.loginfo('robot_name is: %s ', robotname)
pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=1)

rate = rospy.Rate(4)
speed = Twist()
goal = Point()
goal.x = 10
goal.y = 10
landmark = Point()
goal.x = 6
goal.y = 6

while not rospy.is_shutdown():
    inc_x = goal.x - pose_x
    inc_y = goal.y - pose_y
    angle_to_goal = atan2(inc_y, inc_x)
    distance = math.sqrt((goal.x - pose_x)**2 + (goal.y - pose_y)**2) # distance from robot to goal
    distance2=math.sqrt((pose_x-x2)**2+(pose_y-y2)**2) # distance from robot to robot_1
    angledif = angle_to_goal - pose_theta
    if abs(distance>1.5) : 
        if abs(angledif)>=0.2 :
            if angledif < 0:
                robot_1.turn_left(distance,angledif)
            
            else:
                robot_1.turn_right(distance,angledif)
            
        else:
            robot_1.go(distance)
    
    else:
        robot_1.avoid_obstacle()

    
    pub.publish(speed)
    rate.sleep