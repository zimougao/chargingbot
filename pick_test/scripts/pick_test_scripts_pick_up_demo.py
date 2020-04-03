#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

import argparse
import actionlib
import os



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_up',anonymous = True)
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander('manipulator')

reference_frame = 'base_link'

arm_group.set_pose_reference_frame(reference_frame)
end_effector_link = arm_group.get_end_effector_link()
arm_group.allow_replanning(True)



# open the gripper
os.system("rosrun pick_test send_gripper.py --value 0.0")

# get close to the target1
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = -0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = 0.5
pose_target.orientation.w = -0.5

pose_target.position.x = 0.6
pose_target.position.y = -0.2
pose_target.position.z = 0.45

arm_group.set_start_state_to_current_state()
arm_group.set_pose_target(pose_target,end_effector_link)

#_, traj, _, _ = arm_group.plan()
traj= arm_group.plan()
plan1 = arm_group.execute(traj)

# get close to the target1
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = -0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = 0.5
pose_target.orientation.w = -0.5

pose_target.position.x = 0.6
pose_target.position.y = -0.2
pose_target.position.z = 0.35

arm_group.set_start_state_to_current_state()
arm_group.set_pose_target(pose_target,end_effector_link)

#_, traj, _, _ = arm_group.plan()
traj= arm_group.plan()
plan1 = arm_group.execute(traj)


# close the gripper
os.system("rosrun pick_test send_gripper.py --value 0.413") #0.418
# os.system("rosrun pick_test send_gripper.py --value 0.7")
# pick up
pose_target = geometry_msgs.msg.Pose()

pose_target.orientation.x = -0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = 0.5
pose_target.orientation.w = -0.5

pose_target.position.x = 0.6
pose_target.position.y = -0.2
pose_target.position.z = 0.5


arm_group.set_start_state_to_current_state()
arm_group.set_pose_target(pose_target,end_effector_link)

#_, traj, _, _ = arm_group.plan()
traj= arm_group.plan()
plan1 = arm_group.execute(traj)
# open the gripper
os.system("rosrun pick_test send_gripper.py --value 0")

rospy.sleep(1)
moveit_commander.roscpp_shutdown()
