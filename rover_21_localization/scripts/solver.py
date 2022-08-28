#!/usr/bin/env python

import rospy
import numpy as np
import math                     
from numpy.linalg import norm                   
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseStamped

class find_pose():
    def __init__(self):
        rospy.init_node('init_pose_node')
        self.tag_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.tag_callback)
        self.pose_pub = rospy.Publisher('/initial_pose', PoseStamped, queue_size=10)

        self.tag1 = [2.47, 6.35, 0]
        self.tag2 = [5.11, 3.17, 0]
        self.base = [5.94, 7.1] # find rover to camera distance.
        self.cam = [0,0]
        self.yaw = 0
        rospy.sleep(2)
        #self.trilaterate()
        #self.find_yaw()
        self.find_cam()
        self.publish_pose()

    def tag_callback(self, data):
        # calculate two tag distances.
        if data.transforms[0].fiducial_id != 1: 
            temp = self.tag1
            self.tag1 = self.tag2
            self.tag2 = temp
        
        self.tag1[2] = math.sqrt(data.transforms[0].transform.translation.x**2 + data.transforms[0].transform.translation.y**2) #  + data.transforms[0].transform.translation.z**2
        self.tag2[2] = math.sqrt(data.transforms[1].transform.translation.x**2 + data.transforms[1].transform.translation.y**2) # + data.transforms[1].transform.translation.z**2  
        
        #print(data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y)
        #print(data.transforms[1].transform.translation.x, data.transforms[1].transform.translation.y)
        #print("A tag position & distance", self.tag1)
        #print("Another tag position & distance", self.tag2)


    def trilaterate(self):
        x1 = self.tag1[0]
        x2 = self.tag2[0]
        x3 = self.base[0]
        y1 = self.tag1[1]
        y2 = self.tag2[1]
        y3 = self.base[1]
        r1 = self.tag1[2]
        r2 = self.tag2[2]
        r3 = self.base[2]

        A = (-2*x1 + 2*x2)
        B = (-2*y1 + 2*y2)
        C = (r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2)
        D = (-2*x2 + 2*x3)
        E = (-2*y2 + 2*y3)
        F = (r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2)

        x = (C*E - F*B) / (E*A - B*D)
        y = (C*D - A*F) / (B*D - A*E)

        self.cam = [x,y]
        print("X, Y and distance of camera: ", self.cam)

    def find_yaw(self):
        dist = math.sqrt((self.base[0] - self.cam[0])**2 + (self.base[1] - self.cam[1])**2)
        print(dist)
        rov_vec = [self.cam[0]-self.base[0], self.cam[1]-self.base[1]]
        x_proj = [abs(self.cam[0] - self.base[0]), 0]
        self.yaw = math.acos(np.dot(x_proj, rov_vec)/(x_proj[0]*dist))
        print("Yaw angle of the rover: ", self.yaw)

    def get_intersections(self):
        print("A tag position & distance", self.tag1)
        print("Another tag position & distance", self.tag2)        

        x0 = self.tag1[0]
        y0 = self.tag1[1]
        r0 = self.tag1[2]

        x1 = self.tag2[0]
        y1 = self.tag2[1]
        r1 = self.tag2[2]

        # circle 1: (x0, y0), radius r0
        # circle 2: (x1, y1), radius r1

        d=math.sqrt((x1-x0)**2 + (y1-y0)**2)

        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)

        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        print("Candidate 1: ", x3, y3)
        print("Candidate 2: ", x4, y4)
        return (x3, y3, x4, y4)

    def find_cam(self):
        x0 = self.base[0]
        y0 = self.base[1]

        x1, y1, x2, y2 = self.get_intersections()
        
        d1 = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        d2 = math.sqrt((x2-x0)**2 + (y2-y0)**2)

        if d1 > d2:
            self.cam[0] = x2
            self.cam[1] = y2
        else:
            self.cam[0] = x1
            self.cam[1] = y1
        print(self.cam)
        self.find_yaw()


    def publish_pose(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            result = PoseStamped()
            result.header.stamp = rospy.Time.now()
            result.pose.position.x = self.base[0]
            result.pose.position.y = self.base[1]
            result.pose.orientation.z = self.yaw
            result.pose.orientation.w = 1
            self.pose_pub.publish(result)
            #print("Current pose of the rover: ", result)
            rate.sleep()
        rospy.spin()

if __name__ == "__main__":
    init_pose = find_pose()
        
    """ self.p1 = np.array([0,0,0]) # 4,5,0 rover start
        self.p2 = np.array([3,0.75,0]) # 7,2,0 tag 1
        self.p3 = np.array([4,-0.75,0]) # 3,2,0 tag 2
        self.r1 = 0.25 # camera to rover center distance x:0.25, y:0, z:0.2
        self.r2 = 0 # tag1 to camera distance
        self.r3 = 0 # tag2 to camera distance
        self.p4 = np.array([0,0,0]) # camera position to be filled """

    
    """ def tril(self):                      
        temp1 = self.p2-self.p1                                        
        e_x = temp1/norm(temp1)                              
        temp2 = self.p3-self.p1                                        
        i = np.dot(e_x,temp2)                                   
        temp3 = temp2 - i*e_x                                
        e_y = temp3/norm(temp3)                              
        e_z = np.cross(e_x,e_y)                                 
        d = norm(self.p2-self.p1)                                      
        j = np.dot(e_y,temp2)                                   
        x = (self.r1**2 - self.r2**2 + d**2) / (2*d)                    
        y = (self.r1**2 - self.r3**2 -2*i*x + i**2 + j**2) / (2*j)       
        temp4 = self.r1**2 - x**2 - y**2                            
        if temp4<0:                                          
            temp4 = 0
            #raise Exception("The three spheres do not intersect!")
        z = np.sqrt(temp4)                                      
        p_12_a = self.p1 + x*e_x + y*e_y + z*e_z                  
        p_12_b = self.p1 + x*e_x + y*e_y - z*e_z                  
        self.p4 = p_12_b
        print(p_12_a)
        self.yaw = math.atan(self.p4[0]-self.p1[0]/self.p4[1]-self.p1[1])

    def my_way(self):
        # Tag1: Leftmost tag according to camera. 
        x0 = self.p2[0]
        y0 = self.p2[1]
        r0 = self.r2
        # Tag1: Rightmost tag according to camera.
        x1 = self.p3[0]
        y1 = self.p3[1]
        r1 = self.r3

        d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d

        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 
        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        cand1 = [x3,y3]
        cand2 = [x4,y4]

        dist1 = math.sqrt((cand1[0]-self.p1[0])**2 + (cand1[1]-self.p1[1]**2))
        dist2 = math.sqrt((cand2[0]-self.p1[0])**2 + (cand2[1]-self.p1[1]**2))

        if dist1 < dist2:
            self.p4 = cand1
        else:
            self.p4 = cand2
        print(cand1, cand2)
        self.yaw = math.atan(self.p4[0]-self.p1[0]/self.p4[1]-self.p1[1]) 
        print(self.yaw) """



        
"""  # non intersecting
        if d > r0 + r1 :
            print("non-intersecting")
            return None
        # One circle within other
        if d < abs(r0-r1):
            print("One circle within other")
            return None
        # coincident circles
        if d == 0 and r0 == r1:
            print("coincident circles")
            return None
        else: """