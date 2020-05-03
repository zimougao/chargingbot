#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import geometry_msgs.msg
import tf

from geometry_msgs.msg import Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion 
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import pow, atan2, sqrt

speed = Twist()
carrot = Point()
cake = Quaternion()
Euc_Dis = 0.845
yaw = pitch = roll = 0
x = (0, 0, 0)
y = [0, 0, 0]

def getCarrot(msg):
    global carrot
    if len(msg.markers) == 1:
       carrot = msg.markers[0].pose.pose.position
    else:
       carrot = Point()



def getCake(msg):
    global cake
    global x
    if len(msg.markers) == 1:
       cake = msg.markers[0].pose.pose.orientation
       cake_list = [cake.x, cake.y, cake.z, cake.w]
       x = (roll, pitch, yaw) = euler_from_quaternion (cake_list)
    else:
       cake = Quaternion()

rospy.init_node ("carrot_navigation")
marker = rospy.Subscriber ("/ar_pose_marker", AlvarMarkers, getCarrot)
move = rospy.Publisher ("/husky_velocity_controller/cmd_vel", Twist, queue_size=10)
marker001 = rospy.Subscriber ("/ar_pose_marker", AlvarMarkers, getCake)

r = rospy.Rate(100)


while not rospy.is_shutdown():
    y = x[1]

    speed.linear.x = 0.04*(sqrt(pow((carrot.x), 2) + 
                                pow((carrot.y), 2)))
    speed.angular.z = 0.1 * carrot.y * carrot.x + 0.4 * y
    Euc_Dis = (sqrt(pow((carrot.x), 2) + 
                pow((carrot.y), 2)))


#    print speed.linear.x
#    print cake
    print x
#    print speed
#    print y/3.14*360
#    print Euc_Dis

    if carrot.x >= 1.1:
           move.publish(speed)

    r.sleep()
