#! /usr/bin/env python2.7
import  rospy
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from    math    import sqrt,pow,atan2,pi,cos,sin
from    tf.transformations import euler_from_quaternion
from    Laser_Class import Laser_ClosestPoint
from    random import *
import  numpy


class robot():
    def __init__(self,robotname):
        self.robotname=robotname
        global speed 
        global pub
        global l
        global obstacle
        l=Laser_ClosestPoint(robotname)

        self.Pbest_point=Point()
        self.next_point=Point()

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
        c1=1.5
        # global best
        global c2 #social acceleration
        c2=0.6
        global c3 #obstacle acceleration
        self.Pbest_point.x=self.robot_pose_x
        self.Pbest_point.y=self.robot_pose_y
        
    # callback from the subscriber
    def pbest_dist(self,pb_dist,Pbest):
        self.pb_dist.x=Pbest.x-self.robot_pose_x
        self.pb_dist.y=Pbest.y-self.robot_pose_y
        return pb_dist
    
    def gbest_dist(self,gb_dist,Gbest):
        self.gb_dist.x=Gbest.x-self.robot_pose_x
        self.gb_dist.y=Gbest.y-self.robot_pose_y
        return gb_dist

    def callback(self,msg):
        # getting the pose values of the robot by subscribing to the /odom topic              
        self.robot_pose_x=msg.pose.pose.position.x
        self.robot_pose_y=msg.pose.pose.position.y
        
        self.rot_q= msg.pose.pose.orientation
        (self.roll,self.pitch,self.yaw)= euler_from_quaternion([self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w]) 
    
    def closest_point(self):
        # call the Laser Class and geting the closest point
        point_returned=l.closest_point()
        
        closest_point=Point()
        closest_point.x=point_returned.x
        closest_point.y=point_returned.y
        return closest_point

    def get_Pbest(self,goal_point):
        if self.euclidean_distance(goal_point)<self.euclidean_distance(self.Pbest_point): 
            self.Pbest_point.x=self.robot_pose_x
            self.Pbest_point.y=self.robot_pose_y
        return self.Pbest_point

    def get_next_point(self,Pbest,Gbest,obst,obst_d,pb_dist,gb_dist):
        #Define obstacle acceleration with obstacle dist
        c3=0
        if obst_d < 2 and obst_d <= 1:
            c3=0.2
        if obst_d < 1 and obst_d >= 0.3:
            c3=0.4
        elif 0.01 <= obst_d < 0.3:
            c3=0.6

        #Compute next point
        self.next_point.x=w*(self.robot_pose_x)+c1*numpy.random.uniform(0,1)*pb_dist.x+c2*numpy.random.uniform(0,1)*gb_dist.x -c3*numpy.random.uniform(0,1)*(obst.x-self.robot_pose_x)
        self.next_point.y=w*(self.robot_pose_y)+c1*numpy.random.uniform(0,1)*pb_dist.y+c2*numpy.random.uniform(0,1)*gb_dist.y -c3*numpy.random.uniform(0,1)*(obst.x-self.robot_pose_y)

        rospy.loginfo('--------------%s' , self.robotname )
        rospy.loginfo('next_point X: %s Y:%s' , self.next_point.x,self.next_point.y )
        return self.next_point

    def stop(self):
        speed.linear.x = 0
        speed.angular.z = 0
        self.pub.publish(speed)

    # Gets the distance to the  point given as argument
    def euclidean_distance(self, goal_point):
        distance= sqrt(pow((goal_point.x - self.robot_pose_x), 2) +
                    pow((goal_point.y - self.robot_pose_y), 2))

        return distance

    #Define linear velocity for robot
    def linear_vel(self,goal_point, constant=0.2):
        if self.euclidean_distance(goal_point)>5:
            return 1
        else:
            return 0.6

    #Calculate the angle between robot's pose and the goal point
    def angle (self,goal_point):
        desired_angle_goal=atan2(goal_point.y- self.robot_pose_y,goal_point.x- self.robot_pose_x)
        return desired_angle_goal

    #Define angular velocity for robot
    def angular_vel(self, goal_point, constant=5):
        angle_diff=self.angle(goal_point) - self.yaw
        if abs(angle_diff)>0.1:
            if angle_diff != 0 :
                speed.angular.z=angular_vel= -constant * (angle_diff)
            else:
                speed.angular.z=angular_vel= constant * (angle_diff)
        return speed.angular.z

#Define goal point 
def get_goal():
    goal_x=0
    goal_y=0
    return goal_x,goal_y
        
