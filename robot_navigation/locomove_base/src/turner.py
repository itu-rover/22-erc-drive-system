#!/usr/bin/env python


from re import S
import rospy
from geometry_msgs.msg import Pose2D,PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from math import sqrt,pi
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from locomove_base.srv import *

class turner_node:
    def __init__(self):
        rospy.init_node("turner_approach")
        self.XY_GOAL_TOLERANCE = rospy.get_param("/locomove_base/DWBLocalPlanner/xy_goal_tolerance")
        self.YAW_GOAL_TOLERANCE = rospy.get_param("/locomove_base/DWBLocalPlanner/yaw_goal_tolerance") - 0.1

        self.goal_X = None       
        self.goal_y = None
        self.goal_theta = None
        self.x = None
        self.y = None
        self.theta = None
        rospy.Subscriber("/locomove_base/DWBLocalPlanner/goal", Pose2D, self.take_goal) # In locomobase it will be Pose2D
        #rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.take_goal_sim) # In locomobase it will be Pose2D
        #rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, self.odom_cb)
        self.odom_sub = rospy.Subscriber("/odometry/filtered",Odometry, self.odom_cb)
        self.publisher = rospy.Publisher("/cmd_hand_drive", Twist, queue_size=10)
        self.stop_pub = rospy.Publisher('/stop_command', Bool, queue_size=1)
        self.stop = rospy.ServiceProxy('/locomove_base/stopper_service', osci)
        
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0

        self.twist.linear.x = 0
        self.twist.linear.y = 0

        self.reached = False
        self.goal_received = False
        self.next_goal_flag = False
        self.rate = rospy.Rate(100)

        self.run()

    def take_goal(self,data):
        self.goal_x = data.x
        self.goal_y = data.y
        self.goal_theta = data.theta
        self.goal_received = True

    def take_goal_sim(self,data):
        if(self.goal_received == True):
            self.next_goal = data
            self.next_goal_flag = True
        else:
            self.goal_x = data.pose.position.x
            self.goal_y = data.pose.position.y
            q_x = data.pose.orientation.x
            q_y = data.pose.orientation.y
            q_z = data.pose.orientation.z
            q_w = data.pose.orientation.w

        self.goal_theta = euler_from_quaternion((q_x,q_y,q_z,q_w))[2]
        
        self.goal_received = True

    def odom_cb(self,data):
        self.x = data.pose.pose.position.x 
        self.y = data.pose.pose.position.y
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w

        self.theta = euler_from_quaternion((q_x,q_y,q_z,q_w))[2]

    def l2_norm(self,pose1,pose2):
        x1,y1= pose1
        x2,y2 = pose2
        return sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def run(self):
        distance = 0
        while not rospy.is_shutdown():
            # if not (self.theta == None or self.goal_theta == None or self.YAW_GOAL_TOLERANCE == None):
            #     print("dis:",distance < self.XY_GOAL_TOLERANCE + 0.3)
            #     print("abs:",abs(self.theta - self.goal_theta) < self.YAW_GOAL_TOLERANCE)
            if(self.reached == False and self.goal_received == True ):
                distance = self.l2_norm((self.x,self.y),(self.goal_x,self.goal_y))
                if(distance < self.XY_GOAL_TOLERANCE + 0.1 and abs(self.theta - self.goal_theta) > self.YAW_GOAL_TOLERANCE):
                    if self.find_direction(self.theta,self.goal_theta) == "left":
                        self.twist.angular.z = pi/3
                    else:
                        self.twist.angular.z = -pi/3
                    print(self.twist)
                    self.publisher.publish(self.twist)
                    print("donuyoz")
                elif (distance < self.XY_GOAL_TOLERANCE + 0.9 and abs(self.theta - self.goal_theta) < self.YAW_GOAL_TOLERANCE):
                    self.reached = True 
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0   
                    self.publisher.publish(self.twist)
                    truth = Bool()
                    truth.data = True
                    self.stop_pub.publish(truth)
            elif self.reached ==True and self.goal_received == True :
                self.stop(1)
                self.reached = False
                self.goal_received = False
                
            elif self.goal_received == False and self.next_goal_flag == True and self.reached == False :
                self.next_goal_flag = False
                self.goal_x = self.next_goal.pose.position.x
                self.goal_y = self.next_goal.pose.position.y
                q_x = self.next_goal.pose.orientation.x
                q_y = self.next_goal.pose.orientation.y
                q_z = self.next_goal.pose.orientation.z
                q_w = self.next_goal.pose.orientation.w
                self.goal_theta = euler_from_quaternion((q_x,q_y,q_z,q_w))[2]
                self.goal_received = True

                
            self.rate.sleep()


    def find_direction(self,current_yaw,goal_yaw):
        current_yaw += 2*pi
        goal_yaw += 2*pi

        difference = goal_yaw - current_yaw

        if(current_yaw < goal_yaw and abs(difference) <= pi):
            return "left"
        elif(current_yaw < goal_yaw and abs(difference) > pi):
            return "right"
        elif(current_yaw > goal_yaw and abs(difference) <= pi):
            return "right"
        else:    
            return "left"    


if __name__ == "__main__":

    node = turner_node()