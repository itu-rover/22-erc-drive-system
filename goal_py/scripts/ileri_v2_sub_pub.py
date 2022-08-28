#!/usr/bin/env python

import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatusArray

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher31', anonymous=False)
        
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.turner_sub = rospy.Subscriber('/stop_command', Bool, self.turner_cb)

        if(rospy.get_param('/rover/environment') == "sim"):
            self.sub = rospy.Subscriber('/locomove_base/status',GoalStatusArray,self.sim_cb) 
        else:                     
            self.sub = rospy.Subscriber('/move_base/status',Bool,self.outdoor_cb)
        
        self.check = None
        self.task = rospy.get_param('/rover/task')
        self.start = rospy.get_param('/rover/start_coordinates')

    def sim_cb(self,data):
        self.check=data.status_list[0].status

    def outdoor_cb(self,data):
        self.check=data.data

    def turner_cb(self, data):
        self.check = data.data
    
    def send_goal(self, goal):
        rospy.loginfo("Publishing goal...")
        goal.header.stamp = rospy.Time.now()
        time.sleep(1)
        self.pub.publish(goal)
        rospy.loginfo("Goal published.")
        rate = rospy.Rate(10)
        start = rospy.Time.now().secs
        end = rospy.Time.now().secs
        while(self.check==None and (end - start) < 100):
            rate.sleep()
            end = rospy.Time.now().secs
        if(self.check==None):
            return False
        else:
            self.check = None
            return True
    
    def convert_coordinates(self, waypoint_array):
        converted = []
        
        if self.task == "navigation":
            converted = waypoint_array
            converted[1] = -1*converted[1]
        elif self.task == "probing":
            # Probing start coordinates: (x', y').
            # Goal coordinates in nav centric system: (x, y).
            # Converted: (x' - x, y' + y).
            # Yaw change is not calculated for now.
            converted = [self.start[0] - waypoint_array[0], - self.start[1] + waypoint_array[1], 1]
        print(converted)
        return converted
        
    def create_waypoints(self):

        """ 
            Description: 
                Takes waypoints from yaml file and publishes goals one by one.
        """ 

        parameters = rospy.get_param('/rover/waypoints')
        keys = parameters.keys()
        
        for count, key in enumerate(keys):
            rospy.loginfo("Executing goal: %s", key)
            for goal in reversed(list(parameters[key])):
                print(goal)
                converted_waypoints = self.convert_coordinates([parameters[key][goal]['x'], parameters[key][goal]['y'], parameters[key][goal]['theta']])

                goal = PoseStamped()
                goal.pose.position.x = converted_waypoints[0]
                goal.pose.position.y = converted_waypoints[1]
                euler_yaw = converted_waypoints[2]
                quat = quaternion_from_euler(0,0, euler_yaw)
                goal.pose.orientation.x = quat[0]
                goal.pose.orientation.y = quat[1]
                goal.pose.orientation.z = quat[2]
                goal.pose.orientation.w = quat[3]
                goal.header.frame_id = 'odom'
                #

                result = self.send_goal(goal)

                if(result):
                    rospy.loginfo("Goal execution done!")
                else:
                    rospy.loginfo("Goal execution FAILED.")
                    break

            rospy.loginfo("%d. waypoint has been reached.", count)
            input ("Press any key to continue to next waypoint: ")

if __name__ == '__main__':
    try:
        gp = GoalPublisher()
        gp.create_waypoints()
    except KeyboardInterrupt:
        rospy.signal_shutdown()