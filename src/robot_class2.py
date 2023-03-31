#!/usr/bin/env python
import  rospy
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from    math    import sqrt,pow,atan2,pi,cos,sin
from    tf.transformations import euler_from_quaternion
from    random import *
import  numpy


class robot():
    def __init__(self,robotname):
        self.test=Point()
        self.test.x=10
        self.test.y=10
        self.robotname=robotname
        global speed 
        global pub
        self.robot_velocity = 0

        self.Pbest_point=Point()
        self.next_point=Point()
        self.apf_point=Point()
        speed=Twist()
        self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.callback)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.rate.sleep()
        # current vel weight
        global w 
        w=0.6 #weight
        # local best
        global c1 #cognitive acceleration
        c1=1.5 #
        # global best
        global c2 #social acceleration
        c2=0.5
        global c3 
        c3=0
        self.Pbest_point.x=self.robot_pose_x
        self.Pbest_point.y=self.robot_pose_y
    # callback from the subscriber

    def callback(self,msg):
        # getting the pose values of the robot by subscribing to the /odom topic              
        self.robot_pose_x=msg.pose.pose.position.x
        self.robot_pose_y=msg.pose.pose.position.y
        self.rot_q= msg.pose.pose.orientation
        (roll,pitch,self.yaw)= euler_from_quaternion([self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w]) 

    def get_Pbest(self,goal_point):
        
        if self.euclidean_distance(goal_point)<self.euclidean_distance(self.Pbest_point): 
            self.Pbest_point.x=self.robot_pose_x
            self.Pbest_point.y=self.robot_pose_y
        return self.Pbest_point

    def get_next_point(self,Gbest,Pbest):
        # self.next_point.x=w*(self.robot_pose_x)+c1*numpy.random.uniform(0,1)*(Pbest.x-self.robot_pose_x)+c2*numpy.random.uniform(0,1)*(Gbest.x-self.robot_pose_x)
        # self.next_velocity = w*(self.robot_velocity)+c1*numpy.random.uniform(0,1)*(Pbest.x-self.robot_pose_x)+c2*numpy.random.uniform(0,1)*(Gbest.x-self.robot_pose_x)
        # self.next_point.x = self.robot_pose_x + self.next_velocity
        # self.next_point.y = self.robot_pose_y + self.next_velocity
        self.next_point.y=w*(self.robot_pose_y)+c1*numpy.random.uniform(0,1)*(Pbest.y-self.robot_pose_y)+c2*numpy.random.uniform(0,1)*(Gbest.y-self.robot_pose_y)
        rospy.loginfo('--------------%s' , self.robotname )
        rospy.loginfo('next_point X: %s Y:%s' , self.next_point.x,self.next_point.y )
        return self.next_point

# -------------------------------------------------------------------------  
    def stop(self):
        speed.linear.x = 0
        speed.angular.z = 0
        self.pub.publish(speed)


# Gets the distance to the  point given as argument
# -------------------------------------------------------------------------      
    def euclidean_distance(self, goal_point):
        distance= sqrt(pow((goal_point.x - self.robot_pose_x), 2) +
                    pow((goal_point.y - self.robot_pose_y), 2))

        return distance

# -------------------------------------------------------------------------  
    def linear_vel(self,goal_point, constant=0.2):
        if self.euclidean_distance(goal_point)>5:
            return 0.8
        else:
            return 0.4

# -------------------------------------------------------------------------  
    def angle (self,goal_point):
        desired_angle_goal=atan2(goal_point.y- self.robot_pose_y,goal_point.x- self.robot_pose_x)

        return desired_angle_goal

    def angle_deg (self, goal_point):
        desired_angle_goal = atan2(goal_point.y- self.robot_pose_y,goal_point.x- self.robot_pose_x)
        desired_angle_goal = round(desired_angle_goal * (180 / pi), 4)
        return desired_angle_goal

# -------------------------------------------------------------------------  
    def angular_vel(self, goal_point, constant=5):
        angle_diff=self.angle(goal_point) - self.yaw
        if abs(angle_diff)>0.1:
            if angle_diff != 0 :
                speed.angular.z=angular_vel= -constant * (angle_diff)
            else:
                speed.angular.z=angular_vel= constant * (angle_diff)
        
        return speed.angular.z

    def angular_vel_deg(self, goal_point, constant=0.04):
        yaw_deg = round(self.yaw * (180 / pi), 4)
        angle_diff = self.angle_deg(goal_point) - yaw_deg

        if abs(angle_diff )> 0.1 :
            if abs(angle_diff) < 180: 
                speed.angular.z = -constant * (angle_diff)
            else:
                speed.angular.z = constant * (angle_diff)
           

       
        return speed.angular.z

def get_goal():
    goal_x=0
    goal_y=0
    return goal_x,goal_y
        
