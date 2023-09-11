#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
from time import sleep
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi  
import numpy as np
import tf
import math

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander('arm')
gripper = moveit_commander.MoveGroupCommander('gripper')
end_effector_link = arm.get_end_effector_link()

def main(): #进行初始化操作
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_controller', anonymous=True)
    
    arm_demo()


def demo():
    
    home_pose()
    gripper_move(True)
    sleep(5)
    move_to_target()
    sleep(5)
    gripper_move(False)
    sleep(2)
    move_to_target2()
    sleep(5)
    gripper_move(True)
    sleep(3)
    home_pose()
    sleep(5)

def arm_demo():
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.2
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.326
    #pose_goal.orientation=normolize(pose_goal.orientation)
    print("arm target",pose_goal)
    arm_move_to_goal(pose_goal)

def home_pose():
    joint_goal = arm.get_current_joint_values()
    print("jc:",joint_goal)
    joint_goal[0] = 0.0 #home pose
    joint_goal[1] = -0.99
    joint_goal[2] = 0.3
    joint_goal[3] = 0.7
    arm.go(joint_goal,wait=True)
    #arm.stop()

    # gripper_goal = gripper.get_current_joint_values() #初始化夹子
    # print("gc",gripper_goal)
    # gripper_goal[0]=0.010 #0.01 打开夹子 -0.01 关上夹子
    # gripper.go(gripper_goal,wait=True)
    #gripper.stop()

def gripper_move(gripper_open):
    if(gripper_open==True):
        gripper_goal = gripper.get_current_joint_values() 
        gripper_goal[0]=0.010 #0.01 打开夹子 -0.01 关上夹子
        gripper.go(gripper_goal,wait=True)
        print("open gripper")
    else:
        gripper_goal = gripper.get_current_joint_values() 
        gripper_goal[0]=-0.01 #0.01 打开夹子 -0.01 关上夹子
        gripper.go(gripper_goal,wait=True)
        print("close gripper")

def move_to_target():
    joint_goal = arm.get_current_joint_values()
    print("jc:",joint_goal)
    # joint_goal[0] = 0.0 #grap pose
    # joint_goal[1] = 0.5
    # joint_goal[2] = 0.6
    # joint_goal[3] = -0.9

    joint_goal[0] = 0.0
    joint_goal[1] = 0.0
    joint_goal[2] = 0.0
    joint_goal[3] = 0.35
    arm.go(joint_goal,wait=True)

def move_to_target2():
    joint_goal = arm.get_current_joint_values()
    print("jc:",joint_goal)
    # joint_goal[0] = 0.0 #grap pose
    # joint_goal[1] = 0.5
    # joint_goal[2] = 0.6
    # joint_goal[3] = -0.9

    joint_goal[0] = -1.57
    joint_goal[1] = -0.44
    joint_goal[2] = 0.26
    joint_goal[3] = 0.17
    arm.go(joint_goal,wait=True)

def arm_move_to_goal(pose_goal):
    move_group = arm
        
    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(pose_goal)
        
    plan = move_group.go(wait=True)        
    move_group.stop()
    move_group.clear_pose_targets()

def normolize(goal):
    sum2=goal.x*goal.x+goal.y*goal.y+goal.z*goal.z+goal.w*goal.w
    goal.x=math.sqrt(goal.x/sum2)
    goal.y=math.sqrt(goal.y/sum2)
    goal.z=math.sqrt(goal.z/sum2)
    goal.w=math.sqrt(goal.w/sum2)
    return goal

if __name__=="__main__":
    main()



