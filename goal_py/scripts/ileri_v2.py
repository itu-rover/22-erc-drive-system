#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher', anonymous=False)
        
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.task = rospy.get_param('/rover/task')
        self.start = rospy.get_param('/rover/start_coordinates')
    
    def send_goal(self, goal):

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
    
    def convert_coordinates(self, waypoint_array):
        converted = []
        
        if self.task == "navigation":
            converted = waypoint_array
        elif self.task == "probing":
            # Probing start coordinates: (x', y').
            # Goal coordinates in nav centric system: (x, y).
            # Converted: (x' - x, y' + y).
            # Yaw change is not calculated for now.
            converted = [self.start[0] - waypoint_array[0], - self.start[1] + waypoint_array[1], 1]
        
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
            for goal in parameters[key]:

                converted_waypoints = self.convert_coordinates([parameters[key][goal]['x'], parameters[key][goal]['y'], parameters[key][goal]['theta']])

                pose = MoveBaseGoal()
                pose.target_pose.pose.position.x = converted_waypoints[0]
                pose.target_pose.pose.position.y = converted_waypoints[1]
                euler_yaw = converted_waypoints[2]
                quat = quaternion_from_euler(0,0, euler_yaw)
                pose.target_pose.pose.orientation.x = quat[0]
                pose.target_pose.pose.orientation.y = quat[1]
                pose.target_pose.pose.orientation.z = quat[2]
                pose.target_pose.pose.orientation.w = quat[3]
                pose.target_pose.header.frame_id = 'odom'
                pose.target_pose.header.stamp = rospy.Time.now()

                result = self.send_goal(pose)
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
        pass