#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from copy import deepcopy

import geometry_msgs.msg
import moveit_msgs.msg
import tf
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import argparse
import actionlib
import os

def getCarrot(msg):
    global carrot
    if len(msg.markers) == 1:
       carrot = msg.markers[0].pose.pose.position
    else:
       carrot = Point()


class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 是否需要使用笛卡尔空间的运动规划
        cartesian = rospy.get_param('~cartesian', True)
                      
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.1)
        arm.set_goal_orientation_tolerance(0.01)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.1)
        arm.set_max_velocity_scaling_factor(0.1)


        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        print(end_effector_link)

        # 控制机械臂先回到初始化位置
                                               
        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
              

        print(start_pose) 

        # 将初始位姿加入路点列表

        rospy.sleep(1)
        test_pose = arm.get_current_pose(end_effector_link).pose
            
        print(test_pose) 

        ######
        marker = rospy.Subscriber ("/ar_pose_marker", AlvarMarkers, getCarrot)
        listener = tf.TransformListener()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/base_link', '/ar_marker_4', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


        x = euler_from_quaternion(rot)
        quaternion = quaternion_from_euler(x[0]+1.5707, x[1], x[2]+3.1415926)
        
        print x


        arm.set_named_target('test5')
#        arm.go()
        rospy.sleep(1)
        ######

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]

        pose_target.position.x = 0.33
        pose_target.position.y = -0.134
        pose_target.position.z = 1.0484

        arm.set_start_state_to_current_state()
        arm.set_pose_target(pose_target,end_effector_link)

        #_, traj, _, _ = arm_group.plan()
        traj= arm.plan()
        plan1 = arm.execute(traj)
        # 设置路点数据，并加入路点列表


        # 控制机械臂先回到初始化位置
#        arm.set_named_target('test1')
#        arm.go()
#        rospy.sleep(1)
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
