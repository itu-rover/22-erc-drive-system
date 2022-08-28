#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import FiducialTransformArray
from nav_msgs.msg import Odometry
import time

class localisationNode():
    def __init__(self):
        rospy.init_node('initial_pose_node')
        self.odom_sub = rospy.Subscriber('/odometry/wheel', Odometry, self.odom_callback)
        self.tag_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.tag_callback)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/initial_pose', PoseStamped, queue_size=10)
        
        self.pos = {
            "R_0": [0,0],
            "Tag": [4,-0.75],
            "R_n": [0,0]
        }
        self.dist = {
            "Tag_to_R_0": 0,
            "Tag_to_R_n": 0,
            "R_0_to_R_n": 0,
        }
        self.angle_alpha = 0
        self.angle_beta = 0
        self.yaw = 0

    def init_calc(self):
        self.dist['Tag_to_R_0'] = math.sqrt(abs(self.pos['R_0'][0] - self.pos['Tag'][0])**2 + abs(self.pos['R_0'][1] - self.pos['Tag'][1])**2)
        self.angle_beta = math.atan(abs(self.pos['R_0'][0]-self.pos['Tag'][0])/abs(self.pos['R_0'][1]-self.pos['Tag'][1]))
        print("Distance btw. tag and rover start position: ", self.dist['Tag_to_R_0'])
    
    def move_forward(self):
        # Create a goal to move forward then publish it. Call odometry callback to calculate precise movement distance.
        goal = PoseStamped()
        goal.pose.position.x = 2
        goal.pose.position.y = 0
        goal.pose.position.z = 0
        goal.pose.orientation.w=1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"
        self.goal_pub.publish(goal)
        print("Goal pose: ", goal)
        rospy.sleep(10)
        print("10 seconds passed after pose publishing.")
        

    def last_calc(self):
        print("Distance btw. tag and rover start position: ", self.dist['Tag_to_R_0'])
        print("Distance btw. tag and rover current position: ",self.dist['Tag_to_R_n'])
        print("Distance btw. rover start and current position: ", self.dist['R_0_to_R_n'])
        self.angle_alpha = math.acos((self.dist['R_0_to_R_n']**2 + self.dist['Tag_to_R_0']**2 - self.dist['Tag_to_R_n']**2)/2*self.dist['R_0_to_R_n']*self.dist['Tag_to_R_0'])
        total_angle = self.angle_alpha + self.angle_beta
        dist_T = math.sqrt(self.pos['Tag'][0]**2 + self.pos['Tag'][1]**2)
        dist_R0 = math.sqrt(self.pos['R_0'][0]**2 + self.pos['R_0'][1]**2)
        if dist_R0 < dist_T:
            self.yaw = -total_angle
        else:
            self.yaw = math.pi/2 + total_angle

    def tag_callback(self, data):
        # for now, only considering the first tag.
        # self.pos['R_1'] = [data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y]
        self.dist['Tag_to_R_n'] = math.sqrt(data.transforms[0].transform.translation.x**2 + data.transforms[0].transform.translation.y**2)
        print("Tag data: ", data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y)
        print("Distance btw. tag and current position of the rover:", self.dist['Tag_to_R_n'])

    def odom_callback(self, data):
        # calculate the actual movement distance (dist_R0_R1) with odometry information.
        self.dist['R_0_to_R_n'] = math.sqrt(data.pose.pose.position.x**2 + data.pose.pose.position.y**2)
        self.pos['R_n'] = [data.pose.pose.position.x, data.pose.pose.position.y]
        print("Distance btw. start and current position of the rover:", self.dist['R_0_to_R_n'])
    
    def publish_result(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            result = PoseStamped()
            result.header.stamp = rospy.Time.now()
            result.pose.position.x = self.pos['R_n'][0]
            result.pose.position.y = self.pos['R_n'][1]
            result.pose.orientation.z = [self.yaw]
            self.pose_pub.publish(result)
            print("Current pose of the rover: ", result)
            rate.sleep()


if __name__ == "__main__":
    obj = localisationNode()
    obj.init_calc()
    obj.move_forward()
    obj.last_calc()
    obj.publish_result()