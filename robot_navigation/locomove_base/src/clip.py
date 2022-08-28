#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class clipper():
    def __init__(self):
        rospy.init_node("clipper")
        self.sub = rospy.Subscriber("/nav_vel", Twist, self.nav_cb)
        self.pub = rospy.Publisher("/clip_vel", Twist, queue_size=10)
        self.tvist = Twist()
        rospy.spin()
        
    def nav_cb(self, data):
        self.tvist = data
        if(data.angular.z > 0.07):
            self.tvist.angular.z = 0.07
        self.pub.publish(self.tvist)
        
if __name__ == "__main__":
    try:
        clipper()
    except KeyboardInterrupt:
        rospy.signal_shutdown("killme")