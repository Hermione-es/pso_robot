#! /usr/bin/env python2.7
import  rospy
from    geometry_msgs.msg import Twist,Point,Pose
#from    std_msgs.msg import Float32
from    nav_msgs.msg import Odometry
from    math    import sqrt,pow,atan2,pi,cos,sin
from    tf.transformations import euler_from_quaternion
from    Laser_Class_origin import Laser_ClosestPoint
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
        global l
        global obstacle
        l=Laser_ClosestPoint(robotname)

        self.Pbest_point=Point()
        self.next_point=Point()
        self.apf_point=Point()
        speed=Twist()
        self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.callback)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
        #self.obstacle = rospy.Publisher("/{}/obst".format(robotname),Float32, queue_size=10)
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
        c2=0.6
        global c3 
        #c3=0.5
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

    def get_next_point(self,Gbest,Pbest,obst,pb_dist,gb_dist):
        obst=self.closest_point()
        obst_d=self.euclidean_distance(obst)
        c3=0
        if obst_d < 2 and obst_d <= 1:
            c3=0.2
        if obst_d < 1 and obst_d >= 0.3:
            c3=0.4
        elif 0.01 <= obst_d < 0.3:
            c3=0.6
        self.next_point.x=w*(self.robot_pose_x)+c1*numpy.random.uniform(0,1)*pb_dist.x+c2*numpy.random.uniform(0,1)*gb_dist.x -c3*numpy.random.uniform(0,1)*(obst.x-self.robot_pose_x)
        self.next_point.y=w*(self.robot_pose_y)+c1*numpy.random.uniform(0,1)*pb_dist.y+c2*numpy.random.uniform(0,1)*gb_dist.y -c3*numpy.random.uniform(0,1)*(obst.x-self.robot_pose_y)
        # if obst_d < 0.5:
        #     self.next_point.x *= (-1)
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
            return 1
        
        else:
            return 0.6


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

    def get_apf_vel(self,goal,safety_radius=2.5):
        # goal_point=Point()
        goal_point=goal

        obst=self.closest_point()
        obst_d=self.euclidean_distance(obst)
        
        yaw_deg=round(self.yaw * (180 / pi), 4)
        angle_obst=self.angle_deg(obst)-yaw_deg
        angle_goal=self.angle_deg(goal_point)-yaw_deg

        



        if (obst_d<safety_radius) & (obst_d>0.3):
            obst_ang_speed=0

            # obst_lin_speed_scaler=1/(safety_radius/(obst_d+0.001))
                    # angle_goal
            if  (angle_obst<60) & (angle_obst>-60):
                # if abs(angle_goal)>0 :
                            # angle_goal
                if angle_goal-angle_obst>0 :
                    obst_ang_speed=-6*(safety_radius/obst_d)

                        
                else:
                    obst_ang_speed=6*(safety_radius/obst_d)

                   
            return obst_ang_speed 
            # ,obst_lin_speed_scaler
        if (obst_d<0.5) & (obst_d>0.3):
            if abs(angle_obst)>0 :
                if angle_goal-angle_obst>0 :
                    obst_ang_speed=-1000*(safety_radius/obst_d)
                    print(obst_ang_speed)
                    
                else:
                    obst_ang_speed=1000*(safety_radius/obst_d)
                    print(obst_ang_speed)
            return obst_ang_speed

        else:
            return 0
            # ,1.0

def get_goal():
    goal_x=0
    goal_y=0
    return goal_x,goal_y
        
